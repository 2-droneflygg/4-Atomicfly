/**********************************************************************
 参数组：
 	用于存储固件配置数据：
 		参数组合在逻辑组中(一个特定设备/功能的所有参数为一个参数组)，这些组在固件中被建模为数据结构。
 参数组规则：
	1. 每个特性或相关特性组，硬件驱动程序或相关硬件驱动程序组应该使用自己的参数组。
	2. 任何给定ID和版本的参数组在所有目标和构建配置中都应该是相同的 
		- 这意味着参数组中直接或间接包含的任何东西都不能是有条件的，给定目标上可用参数的最佳粒度是是否支持给定参数组。
	3. 唯一的例外是2，为参数组阵列。对于形参组数组，允许有条件地定义数组的长度。
	    因为定义数组元素的结构体的长度对于任何给定版本都是固定的，数组的整体大小与数组一起存储，所以在读取数组时可以计算数组的长度。
	    这个考虑只适用于用' PG_DECLARE_ARRAY '声明的参数组数组以及声明中提供的长度。对于形参组结构中的任何数组的长度(直接或嵌套)。
	4. 由于1和2，配置特性或驱动程序所需的所有参数都应该包含在这个参数组中。
	    如果一个参数需要一个以上的特性或驱动程序，它应该被“向上”移动到一个通用参数组中，
	    所有需要该参数的特性或驱动程序都依赖于这个通用参数组-如果需要，必须创建这样一个参数组。
	5. 新的参数应该始终附加在参数组的末尾。如果这是对参数组的唯一更改，则不需要进一步更改。仍然可以读取该参数组中先前存储的数据版本。
	    在这种情况下，新元素将被初始化为0。如果0是新参数的无效值，则必须按照6中描述的方式处理。
	    以确保形参被初始化为一个有效值。另外，为了正确处理结构中的更改，如
	    果添加新形参的形参组是用' PG_DECLARE_ARRAY '声明的形参组数组，则形参组的版本必须像6中所述的那样递增。
	6. 5中未涉及的对参数组的任何更改。(例如删除元素,改变元素)的类型、
		版本的参数组中定义的(如“PG_REGISTER…”)必须增加以使代码处理参数组知道的格式参数组改变了以前存储的版本现在无效。
	7. 所有对形参组数组(使用' PG_DECLARE_ARRAY() '声明)或包含在形参组结构中的数组的更改都要求形参组的版本
		(如' PG_REGISTER..'也必须增加，和6一样的考虑因素)。
	8. 当创建一个新的形参组时，' PG_DECLARE() ' / ' PG_DECLARE_ARRAY() '应该放在代码中作为形参组类型的结构体的定义之后。
**********************************************************************/
#include <stddef.h>
#include <string.h>
#include <stdint.h>

#include "platform.h"

#include "common/maths.h"

#include "pg.h"

/**********************************************************************
函数名称：pgOffset
函数功能：pg偏移
函数形参：reg
函数返回值：reg->address
函数描述：None
**********************************************************************/
static uint8_t *pgOffset(const pgRegistry_t* reg)
{
	// 获取参数组在RAM中的地址
    return reg->address;
}

/**********************************************************************
函数名称：pgResetInstance
函数功能：复位pg寄存器实例 - 调用PG复位函数
函数形参：pgn，base
函数返回值：None
函数描述：None
**********************************************************************/
void pgResetInstance(const pgRegistry_t *reg, uint8_t *base)
{
	// 获取寄存器大小
    const uint16_t regSize = pgSize(reg);   
	// 清空RAM中寄存器内存
    memset(base, 0, regSize);				
    // __pg_resetdata_start和__pg_resetdata_end在FLASH链接文件中定义
    if (reg->reset.ptr >= (void*)__pg_resetdata_start && reg->reset.ptr < (void*)__pg_resetdata_end) {
        // 指针指向resetdata节，指向它的是数据模板
        memcpy(base, reg->reset.ptr, regSize);
    } else if (reg->reset.fn) {
        // reset函数，调用它
        reg->reset.fn(base);
    }
}

/**********************************************************************
函数名称：pgLoad
函数功能：加载pg参数组
函数形参：reg，from，size，version
函数返回值：状态
函数描述：None
**********************************************************************/
bool pgLoad(const pgRegistry_t* reg, const void *from, int size, int version)
{
	// 复位pg实例 - 调用PG复位函数
    pgResetInstance(reg, pgOffset(reg));
    // 仅恢复匹配版本，否则保持默认值
    if (version == pgVersion(reg)) {
        const int take = MIN(size, pgSize(reg));
        memcpy(pgOffset(reg), from, take);
        return true;
    }
    return false;
}

/**********************************************************************
函数名称：pgReset
函数功能：复位pg寄存器
函数形参：pgn
函数返回值：None
函数描述：None
**********************************************************************/
void pgReset(const pgRegistry_t* reg)
{
	// 复位pg实例 - 调用PG复位函数
    pgResetInstance(reg, pgOffset(reg));
}

/**********************************************************************
函数名称：pgResetAll
函数功能：复位全部pg寄存器
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void pgResetAll(void)
{
	// 遍历所有PG寄存器
    PG_FOREACH(reg) {
    	// 复位pg寄存器
        pgReset(reg);
    }
}
