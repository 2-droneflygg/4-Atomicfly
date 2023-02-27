/******************************************************************************/
/* STM32F4xx外设中断处理程序 */
/* 在这里添加中断处理程序的使用外设(PPP) */
/* 为可用的外部中断处理程序的名称，请参考启动文件(startup_stm32f4xx.s)。*/
/******************************************************************************/
#include "platform.h"

#include "stm32f4xx_it.h"
#include "stm32f4xx_conf.h"

#include "usb_core.h"
#include "usbd_core.h"
#include "usbd_cdc_core.h"

extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern USB_OTG_CORE_HANDLE USB_OTG_dev;

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#endif

// This function handles NMI exception.
void NMI_Handler(void)
{
}

// This function handles SVCall exception.
void SVC_Handler(void)
{
}

// This function handles Debug Monitor exception.
void DebugMon_Handler(void)
{
}

// This function handles PendSVC exception.
void PendSV_Handler(void)
{
}

// OTG 文件系统唤醒中断服务函数
#ifdef USE_USB_OTG_FS
void OTG_FS_WKUP_IRQHandler(void)
{
	  if (USB_OTG_dev.cfg.low_power)
	  {
	    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
	    SystemInit();
	    USB_OTG_UngateClock(&USB_OTG_dev);
	  }
	  EXTI_ClearITPendingBit(EXTI_Line18);
}
#endif

// This function handles EXTI15_10_IRQ Handler.
#ifdef USE_USB_OTG_HS
void OTG_HS_WKUP_IRQHandler(void)
{
	  if (USB_OTG_dev.cfg.low_power)
	  {
	    *(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
	    SystemInit();
	    USB_OTG_UngateClock(&USB_OTG_dev);
	  }
	  EXTI_ClearITPendingBit(EXTI_Line20);
}
#endif

// This function handles OTG_HS Handler.
#ifdef USE_USB_OTG_HS
void OTG_HS_IRQHandler(void)
#else
void OTG_FS_IRQHandler(void)
#endif
{
  	USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
// This function handles EP1_IN Handler.
void OTG_HS_EP1_IN_IRQHandler(void)
{
  	USBD_OTG_EP1IN_ISR_Handler (&USB_OTG_dev);
}

// This function handles EP1_OUT Handler.
void OTG_HS_EP1_OUT_IRQHandler(void)
{
  	USBD_OTG_EP1OUT_ISR_Handler (&USB_OTG_dev);
}
#endif

