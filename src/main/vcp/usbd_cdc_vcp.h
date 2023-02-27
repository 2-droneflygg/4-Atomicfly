#ifndef __USBD_CDC_VCP_H
#define __USBD_CDC_VCP_H

#include "stm32f4xx_conf.h"

#include "usbd_cdc_core.h"
#include "usbd_conf.h"
#include <stdint.h>

#include "usbd_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

extern USB_OTG_CORE_HANDLE USB_OTG_dev;

uint32_t CDC_Send_DATA(const uint8_t *ptrBuffer, uint32_t sendLength);
uint32_t CDC_Send_FreeBytes(void);
uint32_t CDC_Receive_DATA(uint8_t* recvBuf, uint32_t len);       // HJI
uint32_t CDC_Receive_BytesAvailable(void);

uint8_t usbIsConfigured(void);  // HJI
uint8_t usbIsConnected(void);   // HJI
uint32_t CDC_BaudRate(void);
void CDC_SetCtrlLineStateCb(void (*cb)(void *context, uint16_t ctrlLineState), void *context);
void CDC_SetBaudRateCb(void (*cb)(void *context, uint32_t baud), void *context);

/* External variables --------------------------------------------------------*/
extern __IO uint32_t bDeviceState; /* USB device status */

typedef enum _DEVICE_STATE {
    UNCONNECTED,
    ATTACHED,
    POWERED,
    SUSPENDED,
    ADDRESSED,
    CONFIGURED
} DEVICE_STATE;

/* Exported typef ------------------------------------------------------------*/
/* The following structures groups all needed parameters to be configured for the
   ComPort. These parameters can modified on the fly by the host through CDC class
   command class requests. */
typedef struct __attribute__ ((packed))
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
} LINE_CODING;


#endif /* __USBD_CDC_VCP_H */

