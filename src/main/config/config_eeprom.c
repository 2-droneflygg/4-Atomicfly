/*********************************************************************************
 提供EEPROM配置相关API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/crc.h"
#include "common/utils.h"

#include "config/config_eeprom.h"
#include "config/config_streamer.h"
#include "pg/pg.h"
#include "config/config.h"

#include "drivers/system.h"

// EEPROM配置大小
static uint16_t eepromConfigSize;

/* --------------------------配置记录标志枚举-------------------------- */	
typedef enum {
    CR_CLASSICATION_SYSTEM   = 0,						   		 // 系统类
} configRecordFlags_e;

#define CR_CLASSIFICATION_MASK  (0x3)					  		 // CR分类掩码
#define CRC_START_VALUE         0xFFFF					  		 // CRC起始值
#define CRC_CHECK_VALUE         0x1D0F  				  		 // 预先计算的CRC值

/* ----------------------------配置头结构体---------------------------- */	
typedef struct {
    uint8_t eepromConfigVersion;						  		 // EEEPROM配置版本
    uint8_t magic_be;                   				  		 // magic number, 0xBE
} PG_PACKED configHeader_t;

/* ---------------------------配置描述结构体--------------------------- */	
// PG寄存器信息块
typedef struct {
    uint16_t size;	   			   	    				  		 // 大小
    pgn_t pgn;		   									  		 // 参数组号 
    uint8_t version;   									  		 // 版本
    // 较低的2位用来指示系统或概要文件编号，参见CR_CLASSIFICATION_MASK
    uint8_t flags;										  		 // 标志
    uint8_t pg[];					    				  		 // 参数组
} PG_PACKED configRecord_t;

/* ----------------------------配置尾结构体---------------------------- */	
// 校验和紧随配置尾之后
typedef struct {
    uint16_t terminator;			    				  		 // 终止符
} PG_PACKED configFooter_t;

/* ----------------------------包测试结构体---------------------------- */	
// 用于在构建时检查编译器打包。
typedef struct {
    uint8_t byte;                       				  		 // 字节
    uint32_t word;										  	     // 字
} PG_PACKED packingTest_t;

/**********************************************************************
函数名称：initEEPROM
函数功能：初始化EEPROM
函数形参：None
函数返回值：None
函数描述：
	用于验证编译环境，编译失败则输出错误信息。
**********************************************************************/
void initEEPROM(void)
{
    // 验证此体系结构是否按预期打包 - 编译校验，校验条件失败，则输出错误信息
    // offsetof函数：
    // 		在一个常数整数 size_t 类型是一个结构成员的结构从一开始的字节偏移
    // 		格式：ffsetof(type, member-designator)
    // 		type -- 这个类成员指示符类型是一个有效的成员指示符
    // 		member-designator -- 这是类类型成员指示符
    STATIC_ASSERT(offsetof(packingTest_t, byte) == 0, byte_packing_test_failed);
    STATIC_ASSERT(offsetof(packingTest_t, word) == 1, word_packing_test_failed);
    STATIC_ASSERT(sizeof(packingTest_t) == 5, overall_packing_test_failed);
    STATIC_ASSERT(sizeof(configFooter_t) == 2, footer_size_failed);
    STATIC_ASSERT(sizeof(configRecord_t) == 6, record_size_failed);
}

/**********************************************************************
函数名称：isEEPROMVersionValid
函数功能：EEPROM版本是否有效
函数形参：None
函数返回值：如果版本有效则返回true
函数描述：None
**********************************************************************/
bool isEEPROMVersionValid(void)
{
    const uint8_t *p = &__config_start;
    const configHeader_t *header = (const configHeader_t *)p; 	 // 读取版本号
    if (header->eepromConfigVersion != EEPROM_CONF_VERSION) { 	 // 验证版本号是否正确
        return false;
    }
    return true;
}

/**********************************************************************
函数名称：isEEPROMVersionValid
函数功能：扫描EEPROM配置是否有效
函数形参：None
函数返回值：如果配置有效则返回true
函数描述：None
**********************************************************************/
bool isEEPROMStructureValid(void)
{
	// 获取配置起始地址 - F405: ORIGIN = 0x08004000, LENGTH = 16K
    const uint8_t *p = &__config_start;  
    const configHeader_t *header = (const configHeader_t *)p; 	 // 读取配置头
	// 检查合法性
    if (header->magic_be != 0xBE) {
        return false;
    }
	// 初始化CRC起始值 - 进行迭代校验
    uint16_t crc = CRC_START_VALUE;
    crc = crc16_ccitt_update(crc, header, sizeof(*header));   	 // 校验配置头
    p += sizeof(*header);  								     	 // 起始地址偏移配置头大小

    for (;;) {
		// 读取配置记录
        const configRecord_t *record = (const configRecord_t *)p;
		// 判断是否读取完毕
        if (record->size == 0) {								 
            break;
        }
		// 检查配置大小合法性
        if (p + record->size >= &__config_end || record->size < sizeof(*record)) {
            // 配置太大或太小 
            return false;
        }
		// 校验数据
        crc = crc16_ccitt_update(crc, p, record->size);
		// 偏移地址
        p += record->size;
    }

    const configFooter_t *footer = (const configFooter_t *)p; 	 // 读取配置尾
    crc = crc16_ccitt_update(crc, footer, sizeof(*footer));
    p += sizeof(*footer);

    // 在CRC计算中包含存储的CRC
    const uint16_t *storedCrc = (const uint16_t *)p;			 // 读取CRC
    crc = crc16_ccitt_update(crc, storedCrc, sizeof(*storedCrc));	 
    p += sizeof(storedCrc);
	// 获取EEPROM配置大小
    eepromConfigSize = p - &__config_start;

    // CRC有这样一种属性:如果CRC6+本身包含在计算中，那么得出的CRC将具有恒定值
    return crc == CRC_CHECK_VALUE; 						      	 // 判断CRC是否校验正确
}

/**********************************************************************
函数名称：findEEPROM
函数功能：在EEPROM中查找寄存器 + 分类的配置记录
函数形参：寄存器，分类
函数返回值：找到记录则返回配置地址，当没有找到记录时返回NULL
函数描述：
	此函数假定EEPROM内容有效。
**********************************************************************/
static const configRecord_t *findEEPROM(const pgRegistry_t *reg, configRecordFlags_e classification)
{
    const uint8_t *p = &__config_start;      					 // EEPROM起始地址
    p += sizeof(configHeader_t);             					 // EEPROM起始地址+配置头
    // 遍历操作
    while (true) {
        const configRecord_t *record = (const configRecord_t *)p;// 起始地址
        // 配置过大或过小
        if (record->size == 0 || p + record->size >= &__config_end || record->size < sizeof(*record))
            break;
		// 找到记录则返回配置地址
        if (pgN(reg) == record->pgn && (record->flags & CR_CLASSIFICATION_MASK) == classification)
            return record;
        p += record->size;
    }
    // 记录没有找到
    return NULL;
}

/**********************************************************************
函数名称：loadEEPROM
函数功能：初始化EEPROM中所有PG记录
函数形参：reg，classification
函数返回值：当没有找到记录时返回NULL
函数描述：
	这个函数按顺序处理所有PGs，扫描每个PGs的EEPROM
	但是每个PG只被加载/初始化一次，并且按照定义的顺序。
**********************************************************************/
bool loadEEPROM(void)
{
    bool success = true;
	// 遍历PG寄存器
    PG_FOREACH(reg) {
    	// 在EEPROM中查找寄存器 + 系统类的配置记录
        const configRecord_t *rec = findEEPROM(reg, CR_CLASSICATION_SYSTEM);
        if (rec) {
            // pg加载 - 配置从EEPROM可用，使用它初始化PG.
            // 处理配置版本不匹配 - 重置为默认值
            if (!pgLoad(reg, rec->pg, rec->size - offsetof(configRecord_t, pg), rec->version)) {
                success = false;
            }
        } else {
        	// 复位所有配置
            pgReset(reg);
            success = false;
        }
    }
    return success;
}

/**********************************************************************
函数名称：writeSettingsToEEPROM
函数功能：写入设置到EEPROM
函数形参：None
函数返回值：状态
函数描述：None
**********************************************************************/
static bool writeSettingsToEEPROM(void)
{
	// 定义配置流结构体变量
    config_streamer_t streamer;
	// -----------------------------------------------------------------------配置流初始化 -清空
    config_streamer_init(&streamer);
	// -----------------------------------------------------------------------配置流开始
    config_streamer_start(&streamer, (uintptr_t)&__config_start, &__config_end - &__config_start);
	// -----------------------------------------------------------------------初始化配置头
    configHeader_t header = {
        .eepromConfigVersion =  EEPROM_CONF_VERSION,
        .magic_be =             0xBE,
    };
	// -----------------------------------------------------------------------配置流写入 - 配置头
    config_streamer_write(&streamer, (uint8_t *)&header, sizeof(header));
	// 计算CRC值
    uint16_t crc = CRC_START_VALUE;
    crc = crc16_ccitt_update(crc, (uint8_t *)&header, sizeof(header));
	// -----------------------------------------------------------------------遍历PG寄存器 - 写入寄存器+配置记录
    PG_FOREACH(reg) {
    	// 相关初始化
        const uint16_t regSize = pgSize(reg);
        configRecord_t record = {
            .size = sizeof(configRecord_t) + regSize,
            .pgn = pgN(reg),
            .version = pgVersion(reg),
            .flags = 0
        };
		// 初始化标志位
        record.flags |= CR_CLASSICATION_SYSTEM;
		// 配置流写入 - 配置记录
        config_streamer_write(&streamer, (uint8_t *)&record, sizeof(record));
		// 计算CRC值
        crc = crc16_ccitt_update(crc, (uint8_t *)&record, sizeof(record));
		// 配置流写入 - 寄存器
        config_streamer_write(&streamer, reg->address, regSize);
		// 计算CRC值
        crc = crc16_ccitt_update(crc, reg->address, regSize);
    }
	// -----------------------------------------------------------------------初始化配置尾 - 终止符
    configFooter_t footer = {
        .terminator = 0,
    };
	// -----------------------------------------------------------------------配置流写入 - 配置尾
    config_streamer_write(&streamer, (uint8_t *)&footer, sizeof(footer));
    crc = crc16_ccitt_update(crc, (uint8_t *)&footer, sizeof(footer));
    // 在CRC中包含大端格式的CRC
    const uint16_t invertedBigEndianCrc = ~(((crc & 0xFF) << 8) | (crc >> 8));
	// 配置流写入 - CRC
    config_streamer_write(&streamer, (uint8_t *)&invertedBigEndianCrc, sizeof(crc));
	// -----------------------------------------------------------------------配置流刷新
    config_streamer_flush(&streamer);
	// -----------------------------------------------------------------------配置流完成
    const bool success = config_streamer_finish(&streamer) == 0;
    return success;
}

/**********************************************************************
函数名称：writeConfigToEEPROM
函数功能：写入配置到EEPROM
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void writeConfigToEEPROM(void)
{
    bool success = false;
    for (int attempt = 0; attempt < 3 && !success; attempt++) {
		// 写入设置到EEPROM
        if (writeSettingsToEEPROM()) {
            success = true;
        }
    }

    if (success && isEEPROMVersionValid() && isEEPROMStructureValid()) {
		// FLASH写入成功
        return;
    }

    // Flash写入失败
    failureMode(FAILURE_CONFIG_STORE_FAILURE);
}

