/******************************************************************************
  * @file    usbd_desc.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   header file for the usbd_desc.c file
******************************************************************************/
#ifndef __USB_DESC_H
#define __USB_DESC_H

#include "usbd_def.h"


#define USB_DEVICE_DESCRIPTOR_TYPE              0x01
#define USB_CONFIGURATION_DESCRIPTOR_TYPE       0x02
#define USB_STRING_DESCRIPTOR_TYPE              0x03
#define USB_INTERFACE_DESCRIPTOR_TYPE           0x04
#define USB_ENDPOINT_DESCRIPTOR_TYPE            0x05
#define USB_SIZ_DEVICE_DESC                     18
#define USB_SIZ_STRING_LANGID                   4

extern  uint8_t USBD_DeviceDesc  [USB_SIZ_DEVICE_DESC];
extern  uint8_t USBD_StrDesc[USB_MAX_STR_DESC_SIZ];
extern  uint8_t USBD_OtherSpeedCfgDesc[USB_LEN_CFG_DESC];
extern  uint8_t USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC];
extern  uint8_t USBD_LangIDDesc[USB_SIZ_STRING_LANGID];
extern  USBD_DEVICE USR_desc;

extern  uint8_t USBD_DeviceDesc_MSC  [USB_SIZ_DEVICE_DESC];
extern  uint8_t USBD_StrDesc_MSC[USB_MAX_STR_DESC_SIZ];
extern  uint8_t USBD_OtherSpeedCfgDesc_MSC[USB_LEN_CFG_DESC];
extern  uint8_t USBD_DeviceQualifierDesc_MSC[USB_LEN_DEV_QUALIFIER_DESC];
extern  uint8_t USBD_LangIDDesc_MSC[USB_SIZ_STRING_LANGID];
extern  USBD_DEVICE MSC_desc;

uint8_t *     USBD_USR_DeviceDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_LangIDStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_ManufacturerStrDescriptor ( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_ProductStrDescriptor ( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_SerialStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_ConfigStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_USR_InterfaceStrDescriptor( uint8_t speed , uint16_t *length);

uint8_t *     USBD_MSC_DeviceDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_MSC_LangIDStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_MSC_ManufacturerStrDescriptor ( uint8_t speed , uint16_t *length);
uint8_t *     USBD_MSC_ProductStrDescriptor ( uint8_t speed , uint16_t *length);
uint8_t *     USBD_MSC_SerialStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_MSC_ConfigStrDescriptor( uint8_t speed , uint16_t *length);
uint8_t *     USBD_MSC_InterfaceStrDescriptor( uint8_t speed , uint16_t *length);

#ifdef USB_SUPPORT_USER_STRING_DESC
uint8_t *     USBD_USR_USRStringDesc (uint8_t speed, uint8_t idx , uint16_t *length);
#endif /* USB_SUPPORT_USER_STRING_DESC */
#endif /* __USB_DESC_H */

