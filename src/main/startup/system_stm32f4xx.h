#ifndef __SYSTEM_STM32F4XX_H
#define __SYSTEM_STM32F4XX_H

/*!< System Clock Frequency (Core Clock) */
extern uint32_t SystemCoreClock;     
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
extern void OverclockRebootIfNecessary(uint32_t overclockLevel);
extern void systemClockSetHSEValue(uint32_t frequency);
extern int SystemSYSCLKSource(void);
extern int SystemPLLSource(void);
#endif /*__SYSTEM_STM32F4XX_H */

