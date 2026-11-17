/**
  ******************************************************************************
  * @file    usbd_cdc_vcp.h
  * @brief   Header for usbd_cdc_vcp.c file - USB Bulk interface
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_VCP_H
#define __USBD_CDC_VCP_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_core.h"
#include "usbd_conf.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

void USB_REC_IQRHandler(uint8_t * Buf, uint32_t Len);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_VCP_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
