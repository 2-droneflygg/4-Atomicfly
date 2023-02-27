/*********************************************************************************
 提供SPI预初始化操作API。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_SPI
#include "drivers/io.h"
#include "drivers/resource.h"
#include "drivers/system.h"
#include "drivers/max7456.h"

#include "pg/max7456.h"

/* ---------------------------SPI预初始化信息结构体---------------------------- */	
typedef struct spiPreinit_s {
    ioTag_t iotag;						// IO引脚
    uint8_t iocfg;						// IO配置
    bool init;							// 初始化状态
} spiPreinit_t;
// ---------------------------------------------------------定义SPI预初始化信息数组
static spiPreinit_t spiPreinitArray[SPI_PREINIT_COUNT];
// ---------------------------------------------------------SPI预初始化数量
static int spiPreinitCount = 0;

/**********************************************************************
函数名称：spiPreinitRegister
函数功能：SPI预初始化注册表
函数形参：IO引脚，IO配置，初始化状态
函数返回值：None
函数描述：None
**********************************************************************/
void spiPreinitRegister(ioTag_t iotag, uint8_t iocfg, bool init)
{
	// 判断合法性
    if (!iotag) {
        return;
    }
	// 判断合法性
    if (spiPreinitCount == SPI_PREINIT_COUNT) {
		// 蜂鸣器通知失败
        indicateFailure(FAILURE_DEVELOPER, 5);
        return;
    }
	// 注册信息
    spiPreinitArray[spiPreinitCount].iotag = iotag;
    spiPreinitArray[spiPreinitCount].iocfg = iocfg;
    spiPreinitArray[spiPreinitCount].init = init;
    ++spiPreinitCount;
}

/**********************************************************************
函数名称：spiPreinitPin
函数功能：SPI引脚预初始化
函数形参：preinit，index
函数返回值：None
函数描述：None
**********************************************************************/
static void spiPreinitPin(spiPreinit_t *preinit, int index)
{
	// 获取IO
    IO_t io = IOGetByTag(preinit->iotag);
	// IO初始化
    IOInit(io, OWNER_PREINIT, RESOURCE_INDEX(index));
	// IO配置
    IOConfigGPIO(io, preinit->iocfg);
	// 判断初始化状态
    if (preinit->init) {
        IOHi(io);
    } else {
        IOLo(io);
    }
}

/**********************************************************************
函数名称：spiPreinit
函数功能：SPI预初始化
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void spiPreinit(void)
{
	// max7456预初始化
#ifdef USE_MAX7456
    max7456PreInit(max7456Config());
#endif

	// spi引脚预初始化
    for (int i = 0; i < spiPreinitCount; i++) {
        spiPreinitPin(&spiPreinitArray[i], i);
    }
}

/**********************************************************************
函数名称：spiPreinitByIO
函数功能：通过IO预初始化SPI
函数形参：io
函数返回值：None
函数描述：None
**********************************************************************/
void spiPreinitByIO(IO_t io)
{
	// 遍历所有SPI预初始化引脚
    for (int i = 0; i < spiPreinitCount; i++) {
        if (io == IOGetByTag(spiPreinitArray[i].iotag)) {
            spiPreinitPin(&spiPreinitArray[i], i);
            return;
        }
    }
}

/**********************************************************************
函数名称：spiPreinitByTag
函数功能：通过IO标签预初始化SPI
函数形参：tag
函数返回值：None
函数描述：None
**********************************************************************/
void spiPreinitByTag(ioTag_t tag)
{
	// 通过IO预初始化SPI
    spiPreinitByIO(IOGetByTag(tag));
}
#endif

