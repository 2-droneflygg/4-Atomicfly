/*********************************************************************************
 内部集成电路 (I2C) 接口[硬件IIC]：
 	采用中断方式传输，提高系统实时性。
*********************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#if defined(USE_I2C) 
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/rcc.h"

#include "drivers/bus_i2c.h"
#include "drivers/bus_i2c_impl.h"

// ---------------------------------------------------------IIC引脚上下拉配置（复用开漏输出）
// 必须设置为开漏输出，实现IIC的线与逻辑
#define IOCFG_I2C_PU IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_OD, GPIO_PuPd_UP)
#define IOCFG_I2C    IO_CONFIG(GPIO_Mode_AF, 0, GPIO_OType_OD, GPIO_PuPd_NOPULL)

// ---------------------------------------------------------预定义IIC硬件配置信息存储数组
const i2cHardware_t i2cHardware[I2CDEV_COUNT] = {
#ifdef USE_I2C_DEVICE_2
    {
        .device = I2CDEV_2,
        .reg = I2C2,
        .sclPins = {
            I2CPINDEF(PB10, GPIO_AF_I2C2),
        },
        .sdaPins = {
            I2CPINDEF(PB11, GPIO_AF_I2C2),
        },
        .rcc = RCC_APB1(I2C2),
        .ev_irq = I2C2_EV_IRQn,
        .er_irq = I2C2_ER_IRQn,
    },
#endif
};

// ---------------------------------------------------------IIC设备信息存储数组
i2cDevice_t i2cDevice[I2CDEV_COUNT];


/**********************************************************************
函数名称：i2c_er_handler
函数功能：IIC错误处理
函数形参：device
函数返回值：None
函数描述：
	中途检测到应答失败，会进入异常中断I2C1_ER_IRQHandler。
**********************************************************************/
static void i2c_er_handler(I2CDevice device) 
{
	// 获取IIC设备
    I2C_TypeDef *I2Cx = i2cDevice[device].hardware->reg;
	// 获取IIC状态
    i2cState_t *state = &i2cDevice[device].state;

    // 预定义IIC状态寄存器
    volatile uint32_t SR1Register = I2Cx->SR1;

	// 判断总线错误状态（总线错误 | 仲裁丢失（主模式）| 应答失败 | 上溢/下溢）
    if (SR1Register & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR)) 
        state->error = true;

    // 判断总线错误状态（总线错误 | 仲裁丢失（主模式）| 应答失败）
    if (SR1Register & (I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF)) {
		// 读取 I2C_SR1 后再读取 I2C_SR2 可将 ADDR 标志清零
        (void)I2Cx->SR2;                                  
		// 禁用RXNE/TXE中断 - 防止发生错误中断
        I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);          
		// 确保没有仲裁丢失（主模式）和发送停止
        if (!(SR1Register & I2C_SR1_ARLO) && !(I2Cx->CR1 & I2C_CR1_STOP)) {    
			// 判断是否有起始信号
            if (I2Cx->CR1 & I2C_CR1_START) {               
				// 等待发送成功
                while (I2Cx->CR1 & I2C_CR1_START) {; }             
				// 发送stop以完成总线事务
                I2C_GenerateSTOP(I2Cx, ENABLE);     
				// 等待stop完成发送
                while (I2Cx->CR1 & I2C_CR1_STOP) {; }       
				// 重新初始化IIC
                i2cInit(device);                                                
            }
            else {
				// 发送stop以完成总线事务
                I2C_GenerateSTOP(I2Cx, ENABLE);          
				// 当总线未激活时禁用事件和错误中断
                I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);           
            }
        }
    }
	// 重置所有错误标志位以清除中断
    I2Cx->SR1 &= ~(I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_AF | I2C_SR1_OVR);    
	// 总线空闲
    state->busy = 0;
}

/**********************************************************************
函数名称：i2c_ev_handler
函数功能：IIC事件处理
函数形参：device
函数返回值：None
函数描述：None
**********************************************************************/
void i2c_ev_handler(I2CDevice device) 
{
	// 获取IIC设备
    I2C_TypeDef *I2Cx = i2cDevice[device].hardware->reg;
	// 获取IIC状态
    i2cState_t *state = &i2cDevice[device].state;

	// 地址发送状态，停止位发送状态
    static uint8_t subaddress_sent, final_stop;        
	// index is signed -1 == 已发送子地址
    static int8_t index;                                                        
    // 预定义IIC状态寄存器
    uint8_t SReg_1 = I2Cx->SR1;                                                 

	// ---------------------------------------------------1.起始信号完成
    if (SReg_1 & I2C_SR1_SB) {            
		// 重置POS位，使ACK/NACK应用于当前字节
        I2Cx->CR1 &= ~I2C_CR1_POS;                 
		// 确保ACK为使能状态
        I2C_AcknowledgeConfig(I2Cx, ENABLE);        
		// 重置索引
        index = 0;          
		// -------------------------接收数据
        if (state->reading && (subaddress_sent || 0xFF == state->reg)) {        
			// 发送地址标志位置1
            subaddress_sent = 1;   
			// 发送2字节 - 10位寻址模式
            if (state->bytes == 2)
				// 置位POS位，ACK位控制移位寄存器中要接收的下一个字节的 (N)ACK
                I2Cx->CR1 |= I2C_CR1_POS;                                  
			// 发送地址并设置为接收模式
            I2C_Send7bitAddress(I2Cx, state->addr, I2C_Direction_Receiver);     
        }
		// -------------------------发送数据
        else {                    
			// 发送地址并设置为传输模式
            I2C_Send7bitAddress(I2Cx, state->addr, I2C_Direction_Transmitter);  
			// 判断寄存器地址是否为0xFF
            if (state->reg != 0xFF)                                             
                index = -1;                                                    
        }
    }
	// ---------------------------------------------------2.地址已发送 - 7位寻址模式接收到地址字节的ACK后该位置1
	// 注意： 收到NACK后ADDR位不会置1
    else if (SReg_1 & I2C_SR1_ADDR) {                                         
        // 数据存储器屏障，确保在新的存储器访问开始之前，所有的存储器访问已经完成。
        __DMB();                
		// 判断要接收的字节（注册传输信息的数据长度）
		// -------------------------接收1个字节
        if (state->bytes == 1 && state->reading && subaddress_sent) {      
			// 关闭ACK
            I2C_AcknowledgeConfig(I2Cx, DISABLE);        
			// 数据存储器屏障，确保在新的存储器访问开始之前，所有的存储器访问已经完成。
            __DMB();
			// 清除ADDR标志位
            (void)I2Cx->SR2;                                                    
			// 发送stop以完成总线事务
            I2C_GenerateSTOP(I2Cx, ENABLE);                                     
            final_stop = 1;
			// 使能RXNE/TXE中断
            I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                   
        }	
		// -------------------------接收多个字节
        else {                 
			// 清除ADDR标志位
            (void)I2Cx->SR2;     
			// 数据存储器屏障，确保在新的存储器访问开始之前，所有的存储器访问已经完成。
            __DMB();
			// ---------------接收2个字节
            if (state->bytes == 2 && state->reading && subaddress_sent) {   
				// 关闭ACK
                I2C_AcknowledgeConfig(I2Cx, DISABLE);               
				// 禁用RXNE/TXE中断
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                        
            }
			// ---------------接收3个字节
            else if (state->bytes == 3 && state->reading && subaddress_sent) {   
				// 禁用RXNE/TXE中断
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);  
			}
			// ---------------接收多个字节
            else {                                  
				// 使能RXNE/TXE中断
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);
            }
        }
    }
	// ---------------------------------------------------完成数据字节传输 - 由硬件置 1
	// 接收过程中接收到一个新字节（包括 ACK 脉冲）但尚未读取DR寄存器 (RxNE=1)
	// 发送过程中将发送一个新字节但尚未向DR寄存器写入数据 (TxE=1)
    else if (SReg_1 & I2C_SR1_BTF) {                                   		   
        final_stop = 1;
		// -------------------------接收数据
        if (state->reading && subaddress_sent) {     
			// 判断接收多少字节
            if (state->bytes > 2) {            
				// 关闭ACK
                I2C_AcknowledgeConfig(I2Cx, DISABLE);    
				// 读取数据
                state->read_p[index++] = (uint8_t)I2Cx->DR;   
				// 发送stop以完成总线事务
                I2C_GenerateSTOP(I2Cx, ENABLE);                         	
                final_stop = 1;                  
				// 读取数据
                state->read_p[index++] = (uint8_t)I2Cx->DR;    
				// 使能RXNE/TXE中断
                I2C_ITConfig(I2Cx, I2C_IT_BUF, ENABLE);                 	
            }
            else {                                                      	
                if (final_stop)
					// 发送stop以完成总线事务
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     	
                else
					// 生成IIC通信起始条件
                    I2C_GenerateSTART(I2Cx, ENABLE);    
				// 读取数据
                state->read_p[index++] = (uint8_t)I2Cx->DR;             	
                state->read_p[index++] = (uint8_t)I2Cx->DR;             		
                index++;                                                	
            }
        }
		// -------------------------发送数据
        else {           
            if (subaddress_sent || (state->writing)) {
                if (final_stop)
					// 发送stop以完成总线事务
                    I2C_GenerateSTOP(I2Cx, ENABLE);                     
                else
					// 重新生成起始条件
                    I2C_GenerateSTART(I2Cx, ENABLE);                    		
                index++;                                                	
            }
            else {   
				// 重新生成起始条件
                I2C_GenerateSTART(I2Cx, ENABLE);                        		
                subaddress_sent = 1;                                    	
            }
        }
        // 等待清除标志位，得到BTF
        while (I2Cx->CR1 & I2C_CR1_START) {; }
    }
	// ---------------------------------------------------接收缓冲区非空
	// 接收模式下数据寄存器非空时置1（RxNE不会在地址阶段置1）
	// 由软件读取或写入DR寄存器来清零，或在PE=0 时由硬件清零
	// 发生 ARLO 事件时 RxNE 不会置 1
	// 注意： BTF置1时无法通过读取数据将RxNE清零，因为此时数据寄存器仍为满
    else if (SReg_1 & I2C_SR1_RXNE) {                 
		// 读取数据
        state->read_p[index++] = (uint8_t)I2Cx->DR;
        if (state->bytes == (index + 3))
			// 禁用TXE以允许缓冲区刷新
            I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                    	
        if (state->bytes == index)                                      	
            index++;                                                    		
    }
	// ---------------------------------------------------发送缓冲区为空
	// 发送过程中DR为空时该位置1（TxE 不会在地址阶段置 1）
	// 由软件写入DR寄存器来清零，或在出现起始、停止位或者PE=0时由硬件清零
	// 如果接收到 NACK 或要发送的下一个字节为PEC (PEC=1)， TxE将不会置1
	// 注意：写入第一个要发送的数据或在BTF置1时写入数据都无法将TxE清零，
	// 		因为这两种情况下数据寄存器仍为空。
    else if (SReg_1 & I2C_SR1_TXE) {                                    
        if (index != -1) {                                              
			// 发送数据
            I2Cx->DR = state->write_p[index++];
			// 发送完成
            if (state->bytes == index)                                  		
            	// 禁用TXE以允许缓冲区刷新
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                		
        }
        else {
            index++;
			// 发送寄存器地址
            I2Cx->DR = state->reg;                            
			// 如果接收或发送0字节，立即刷新
            if (state->reading || !(state->bytes))                 
				// 禁用TXE以允许缓冲区刷新
                I2C_ITConfig(I2Cx, I2C_IT_BUF, DISABLE);                		
        }
    }
	// 完成传输事务
    if (index == state->bytes + 1) {              
		// 重置
        subaddress_sent = 0;  
		// 如果有一个停止位，总线是不工作的，禁用中断以防止BTF
        if (final_stop)                
			// 禁用EVT和ERR中断
            I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);   
		// 总线空闲
        state->busy = 0;
    }
}

/**********************************************************************
函数名称：I2C1_ER_IRQHandler
函数功能：IIC1错误中断服务函数
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void I2C1_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_1);
}

/**********************************************************************
函数名称：I2C1_EV_IRQHandler
函数功能：IIC1事件中断服务函数
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void I2C1_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_1);
}

/**********************************************************************
函数名称：I2C2_ER_IRQHandler
函数功能：IIC2错误中断服务函数
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void I2C2_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_2);
}

/**********************************************************************
函数名称：I2C2_EV_IRQHandler
函数功能：IIC2事件中断服务函数
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void I2C2_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_2);
}

/**********************************************************************
函数名称：I2C3_ER_IRQHandler
函数功能：IIC3错误中断服务函数
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void I2C3_ER_IRQHandler(void) {
    i2c_er_handler(I2CDEV_3);
}

/**********************************************************************
函数名称：I2C3_EV_IRQHandler
函数功能：IIC3事件中断服务函数
函数形参：None
函数返回值：None
函数描述：None
**********************************************************************/
void I2C3_EV_IRQHandler(void) {
    i2c_ev_handler(I2CDEV_3);
}

/**********************************************************************
函数名称：i2cHandleHardwareFailure
函数功能：IIC硬件错误处理
函数形参：device
函数返回值：false
函数描述：None
**********************************************************************/
static bool i2cHandleHardwareFailure(I2CDevice device)
{
    // 重新初始化IIC
    i2cInit(device);
    return false;
}

/**********************************************************************
函数名称：i2cWait
函数功能：IIC等待
函数形参：IIC设备
函数返回值：状态
函数描述：None
**********************************************************************/
bool i2cWait(I2CDevice device)
{
	// 获取IIC状态信息
    i2cState_t *state = &i2cDevice[device].state;
	// 获取IIC超时时间
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;
	// 等待IIC不忙碌或者超时
    while (state->busy && --timeout > 0) {; }
	// 如果超时 - 进行IIC硬件错误处理 && 回调函数本身（IIC继续等待）
    if (timeout == 0)
        return i2cHandleHardwareFailure(device) && i2cWait(device);
    return !(state->error);
}

/**********************************************************************
函数名称：i2cBusy
函数功能：判断IIC是否空闲 
函数形参：IIC设备，获取错误状态
函数返回值：IIC设备是否忙碌
函数描述：None
**********************************************************************/
bool i2cBusy(I2CDevice device, bool *error)
{
	// 获取IIC状态信息
    i2cState_t *state = &i2cDevice[device].state;
	// 判断是否获取错误状态
    if (error) {
        *error = state->error;
    }
    return state->busy;
}

/**********************************************************************
函数名称：i2cWriteBuffer
函数功能：IIC写缓存
函数形参：IIC设备，器件地址，寄存器地址，数据长度，数据
函数返回值：状态
函数描述：None
**********************************************************************/
bool i2cWriteBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data)
{
	// 判断IIC设备合法性
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }
	// 获取IIC设备
    I2C_TypeDef *I2Cx = i2cDevice[device].reg;
    if (!I2Cx) {
        return false;
    }
	// 获取IIC状态
    i2cState_t *state = &i2cDevice[device].state;
    if (state->busy) {
        return false;
    }
	// 获取IIC超时时间
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;
	// 创建传输信息
    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 1;
    state->reading = 0;
    state->write_p = data;
    state->read_p = data;
    state->bytes = len_;
    state->busy = 1;
    state->error = false;
	// 确保没有事件中断
    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                                 
    	// 确保总线没有正在传输
        if (!(I2Cx->CR1 & I2C_CR1_START)) {             
			// 确保总线没有正在传输 - 等待传输完成
            while (I2Cx->CR1 & I2C_CR1_STOP && --timeout > 0) {; }      
			// 如果超时则进行硬件错误处理
            if (timeout == 0)
                return i2cHandleHardwareFailure(device);
			// 生成IIC通信起始信号
            I2C_GenerateSTART(I2Cx, ENABLE);                            
        }
		// 允许中断再次触发 - 在中断服务函数进行发送
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            
    }
    return true;
}

/**********************************************************************
函数名称：i2cReadBuffer
函数功能：IIC读缓存
函数形参：IIC设备，器件地址，寄存器地址，数据长度，缓存地址
函数返回值：状态
函数描述：None
**********************************************************************/
bool i2cReadBuffer(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
	// 判断IIC设备合法性
    if (device == I2CINVALID || device > I2CDEV_COUNT) {
        return false;
    }
	// 获取IIC设备
    I2C_TypeDef *I2Cx = i2cDevice[device].reg;
    if (!I2Cx) {
        return false;
    }
	// 获取IIC状态
    i2cState_t *state = &i2cDevice[device].state;
    if (state->busy) {
        return false;
    }
	// 获取IIC超时时间
    uint32_t timeout = I2C_DEFAULT_TIMEOUT;
	// 创建传输信息
    state->addr = addr_ << 1;
    state->reg = reg_;
    state->writing = 0;
    state->reading = 1;
    state->read_p = buf;
    state->write_p = buf;
    state->bytes = len;
    state->busy = 1;
    state->error = false;
	// 确保没有事件中断
    if (!(I2Cx->CR2 & I2C_IT_EVT)) {                    
		// 确保总线没有正在传输
        if (!(I2Cx->CR1 & I2C_CR1_START)) {      
			// 确保总线没有正在传输 - 等待传输完成
            while (I2Cx->CR1 & I2C_CR1_STOP && --timeout > 0) {; }   
			// 如果超时则进行硬件错误处理
            if (timeout == 0)
                return i2cHandleHardwareFailure(device);
			// 生成IIC通信起始信号 
            I2C_GenerateSTART(I2Cx, ENABLE);                          
        }
		// 允许中断再次触发 - 在中断服务函数进行接收
        I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, ENABLE);            
    }
    return true;
}

/**********************************************************************
函数名称：i2cWrite
函数功能：IIC写
函数形参：IIC设备，地址，IICx，长度，数据
函数返回值：状态
函数描述：None
**********************************************************************/
bool i2cWrite(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t data)
{
    return i2cWriteBuffer(device, addr_, reg_, 1, &data) && i2cWait(device);
}

/**********************************************************************
函数名称：i2cRead
函数功能：IIC读
函数形参：IIC设备，地址，IICx，长度，缓存地址
函数返回值：状态
函数描述：None
**********************************************************************/
bool i2cRead(I2CDevice device, uint8_t addr_, uint8_t reg_, uint8_t len, uint8_t* buf)
{
    return i2cReadBuffer(device, addr_, reg_, len, buf) && i2cWait(device);
}

/**********************************************************************
函数名称：i2cInit
函数功能：IIC初始化
函数形参：device
函数返回值：None
函数描述：None
**********************************************************************/
void i2cInit(I2CDevice device)
{
	NVIC_InitTypeDef nvic;
    I2C_InitTypeDef i2cInit;
	
	// 判断是否有IIC设备
    if (device == I2CINVALID)
        return;
	// 获取IIC设备信息
    i2cDevice_t *pDev = &i2cDevice[device];    
	// 获取IIC硬件信息
    const i2cHardware_t *hw = pDev->hardware;  
	// 获取总线IO
    const IO_t scl = pDev->scl;				   
    const IO_t sda = pDev->sda;
	// 判断合法性
    if (!hw || IOGetOwner(scl) || IOGetOwner(sda)) {
        return;
    }
	// 获取IIC设备
    I2C_TypeDef *I2Cx = hw->reg;
	// 复位IIC设备状态
    memset(&pDev->state, 0, sizeof(pDev->state));

	// 注册IO_pin，设置所有者和资源
    IOInit(scl, OWNER_I2C_SCL, RESOURCE_INDEX(device));
    IOInit(sda, OWNER_I2C_SDA, RESOURCE_INDEX(device));

    // 使能IIC时钟
    RCC_ClockCmd(hw->rcc, ENABLE);
	
	// IIC中断配置
    I2C_ITConfig(I2Cx, I2C_IT_EVT | I2C_IT_ERR, DISABLE);

    // IO配置初始化 
    IOConfigGPIOAF(scl, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sclAF);
    IOConfigGPIOAF(sda, pDev->pullUp ? IOCFG_I2C_PU : IOCFG_I2C, pDev->sdaAF);

	// 初始化IIC为默认值
    I2C_DeInit(I2Cx);
	// 将IIC结构体初始化为默认值
    I2C_StructInit(&i2cInit);

	// 禁用EVT和ERR中断 - 它们在第一个请求时启用
    I2C_ITConfig(I2C2, I2C_IT_EVT | I2C_IT_ERR, DISABLE);    
	// 配置IIC
    i2cInit.I2C_Mode = I2C_Mode_I2C;									// IIC模式
    i2cInit.I2C_DutyCycle = I2C_DutyCycle_2;							// 快速模式工作周期
    i2cInit.I2C_OwnAddress1 = 0;										// 指定第一个设备地址
    i2cInit.I2C_Ack = I2C_Ack_Enable;									// 启用IIC应答
    i2cInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;		// 7位应答
    i2cInit.I2C_ClockSpeed = 800000;                                    // IIC时钟速度 - 测试了800khz的最高速度，没有问题
	// 使能IIC
    I2C_Cmd(I2C2, ENABLE);
	// 初始化IIC
    I2C_Init(I2C2, &i2cInit);
	// 使能IIC时钟扩展
    I2C_StretchClockCmd(I2C2, ENABLE);

    // 初始化IIC错误中断
    nvic.NVIC_IRQChannel = I2C2_ER_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_ER);
    nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_ER);
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    // 初始化IIC事件中断
    nvic.NVIC_IRQChannel = I2C2_EV_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_I2C_EV);
    nvic.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_I2C_EV);
    NVIC_Init(&nvic);
}
#endif

