/**
  ******************************************************************************
  * @file    usbd_cdc_core.c
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    17-March-2018
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB CDC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                CDC Class Driver Description
  *          =================================================================== 
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus 
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class

  *           @note
  *             For the Abstract Control Model, this core allows only transmitting the requests to
  *             lower layer dispatcher (ie. usbd_cdc_vcp.c/.h) which should manage each request and
  *             perform relative actions.
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *      
  *  @endverbatim
  *                                  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      <http://www.st.com/SLA0044>
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_core.h"
#include "usbd_desc.h"
#include "usbd_req.h"
#include "usb_dcd.h"


/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup usbd_cdc 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup usbd_cdc_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_cdc_Private_Defines
  * @{
  */ 

#define USB_BULK_IDLE         0
#define USB_BULK_BUSY         1

/**
  * @}
  */ 


/** @defgroup usbd_cdc_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_cdc_Private_FunctionPrototypes
  * @{
  */

/*********************************************
   Bulk Device library callbacks
 *********************************************/
uint8_t  usbd_bulk_Init        (void  *pdev, uint8_t cfgidx);
uint8_t  usbd_bulk_DeInit      (void  *pdev, uint8_t cfgidx);
uint8_t  usbd_bulk_Setup       (void  *pdev, USB_SETUP_REQ *req);
uint8_t  usbd_bulk_DataIn      (void *pdev, uint8_t epnum);
uint8_t  usbd_bulk_DataOut     (void *pdev, uint8_t epnum);
static uint8_t  usbd_bulk_IsoIncomplete (void *pdev);

/*********************************************
   Bulk specific management functions
 *********************************************/
static uint8_t  *USBD_bulk_GetCfgDesc (uint8_t speed, uint16_t *length);
#ifdef USE_USB_OTG_HS
static uint8_t  *USBD_bulk_GetOtherCfgDesc (uint8_t speed, uint16_t *length);
#endif
/**
  * @}
  */ 

/** @defgroup usbd_cdc_Private_Variables
  * @{
  */ 
extern CDC_IF_Prop_TypeDef  APP_FOPS;
extern uint8_t USBD_DeviceDesc   [USB_SIZ_DEVICE_DESC];

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t usbd_bulk_CfgDesc  [USB_BULK_CONFIG_DESC_SIZ] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN static __IO uint32_t  usbd_bulk_AltSet  __ALIGN_END = 0;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t USB_Rx_Buffer   [CDC_DATA_MAX_PACKET_SIZE] __ALIGN_END ;

/* Transmit buffer and state for direct bulk transfer */
__ALIGN_BEGIN uint8_t USB_Tx_Buffer   [APP_TX_DATA_SIZE] __ALIGN_END ;
uint32_t USB_Tx_Length = 0;
uint8_t  USB_Tx_State = USB_BULK_IDLE;

/* Bulk interface class callbacks structure */
USBD_Class_cb_TypeDef  USBD_CDC_cb =
{
  usbd_bulk_Init,
  usbd_bulk_DeInit,
  usbd_bulk_Setup,
  NULL,                 /* EP0_TxSent, */
  NULL,                 /* EP0_RxReady, */
  usbd_bulk_DataIn,
  usbd_bulk_DataOut,
  NULL,                 /* SOF - no polling needed for bulk (NULL-checked in usbd_core.c) */
  usbd_bulk_IsoIncomplete,   /* IsoINIncomplete - must not be NULL! */
  usbd_bulk_IsoIncomplete,   /* IsoOUTIncomplete - must not be NULL! */
  USBD_bulk_GetCfgDesc,
#ifdef USE_USB_OTG_HS
  USBD_bulk_GetOtherCfgDesc,
#endif /* USE_USB_OTG_HS  */
};

/* USB Bulk device Configuration Descriptor */
/* 1 interface, 2 bulk endpoints, vendor-specific class */
__ALIGN_BEGIN uint8_t usbd_bulk_CfgDesc[USB_BULK_CONFIG_DESC_SIZ]  __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_CONFIGURATION_DESCRIPTOR_TYPE,      /* bDescriptorType: Configuration */
  USB_BULK_CONFIG_DESC_SIZ,               /* wTotalLength */
  0x00,
  0x01,   /* bNumInterfaces: 1 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 100 mA */

  /*---------------------------------------------------------------------------*/

  /*Interface Descriptor - Vendor Specific Bulk */
  0x09,   /* bLength: Interface Descriptor size */
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0xFF,   /* bInterfaceClass: Vendor Specific */
  0x00,   /* bInterfaceSubClass */
  0x00,   /* bInterfaceProtocol */
  0x00,   /* iInterface: */

  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */

  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType: Endpoint */
  CDC_IN_EP,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_MAX_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
} ;

#ifdef USE_USB_OTG_HS
__ALIGN_BEGIN uint8_t usbd_bulk_OtherCfgDesc[USB_BULK_CONFIG_DESC_SIZ]  __ALIGN_END =
{
  0x09,   /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION,
  USB_BULK_CONFIG_DESC_SIZ,
  0x00,
  0x01,   /* bNumInterfaces: 1 interface */
  0x01,   /* bConfigurationValue: */
  0x04,   /* iConfiguration: */
  0xC0,   /* bmAttributes: */
  0x32,   /* MaxPower 100 mA */

  /*Interface Descriptor - Vendor Specific Bulk */
  0x09,   /* bLength: Interface Descriptor size */
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0xFF,   /* bInterfaceClass: Vendor Specific */
  0x00,   /* bInterfaceSubClass */
  0x00,   /* bInterfaceProtocol */
  0x00,   /* iInterface: */

  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  0x40,                              /* wMaxPacketSize: */
  0x00,
  0x00,                              /* bInterval: ignore for Bulk transfer */

  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,     /* bDescriptorType: Endpoint */
  CDC_IN_EP,                        /* bEndpointAddress */
  0x02,                             /* bmAttributes: Bulk */
  0x40,                             /* wMaxPacketSize: */
  0x00,
  0x00                              /* bInterval */
};
#endif /* USE_USB_OTG_HS  */

/**
  * @}
  */ 

/** @defgroup usbd_cdc_Private_Functions
  * @{
  */ 

/**
  * @brief  usbd_bulk_Init
  *         Initialize the Bulk interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
uint8_t  usbd_bulk_Init (void  *pdev,
                               uint8_t cfgidx)
{
  /* Open EP IN */
  DCD_EP_Open(pdev,
              CDC_IN_EP,
              CDC_DATA_IN_PACKET_SIZE,
              USB_OTG_EP_BULK);

  /* Open EP OUT */
  DCD_EP_Open(pdev,
              CDC_OUT_EP,
              CDC_DATA_OUT_PACKET_SIZE,
              USB_OTG_EP_BULK);

  /* Initialize the Interface physical components */
  APP_FOPS.pIf_Init();

  /* Prepare Out endpoint to receive next packet */
  DCD_EP_PrepareRx(pdev,
                   CDC_OUT_EP,
                   (uint8_t*)(USB_Rx_Buffer),
                   CDC_DATA_OUT_PACKET_SIZE);

  return USBD_OK;
}

/**
  * @brief  usbd_bulk_DeInit
  *         DeInitialize the Bulk layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
uint8_t  usbd_bulk_DeInit (void  *pdev,
                                 uint8_t cfgidx)
{
  /* Close EP IN */
  DCD_EP_Close(pdev, CDC_IN_EP);

  /* Close EP OUT */
  DCD_EP_Close(pdev, CDC_OUT_EP);

  /* Restore default state of the Interface physical components */
  APP_FOPS.pIf_DeInit();

  return USBD_OK;
}

/**
  * @brief  usbd_bulk_Setup
  *         Handle the Bulk specific requests (standard only, no CDC class requests)
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
uint8_t  usbd_bulk_Setup (void  *pdev,
                                USB_SETUP_REQ *req)
{
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        (uint8_t *)&usbd_bulk_AltSet,
                        1);
      break;

    case USB_REQ_SET_INTERFACE :
      if ((uint8_t)(req->wValue) < USBD_ITF_MAX_NUM)
      {
        usbd_bulk_AltSet = (uint8_t)(req->wValue);
      }
      else
      {
        USBD_CtlError (pdev, req);
      }
      break;

    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL;
    }
    break;

  default:
    USBD_CtlError (pdev, req);
    return USBD_FAIL;
  }
  return USBD_OK;
}

/**
  * @brief  usbd_bulk_DataIn
  *         Data sent on non-control IN endpoint (bulk transfer complete)
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  usbd_bulk_DataIn (void *pdev, uint8_t epnum)
{
  USB_Tx_State = USB_BULK_IDLE;
  return USBD_OK;
}

/**
  * @brief  usbd_bulk_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
uint8_t  usbd_bulk_DataOut (void *pdev, uint8_t epnum)
{
  uint16_t USB_Rx_Cnt;

  /* Get the received data buffer and update the counter */
  USB_Rx_Cnt = ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].xfer_count;

  /* USB data will be immediately processed, this allow next USB traffic being
  NAKed till the end of the application Xfer */
  APP_FOPS.pIf_DataRx(USB_Rx_Buffer, USB_Rx_Cnt);

  /* Prepare Out endpoint to receive next packet */
  DCD_EP_PrepareRx(pdev,
                   CDC_OUT_EP,
                   (uint8_t*)(USB_Rx_Buffer),
                   CDC_DATA_OUT_PACKET_SIZE);

  return USBD_OK;
}

/**
  * @brief  USBD_Bulk_TransmitPacket
  *         Direct bulk transmit - send data immediately on IN endpoint
  *         Similar to rmcs_slave's USBD_CDC_TransmitPacket approach
  * @param  pdev: device instance
  * @param  buf: data buffer
  * @param  len: data length
  * @retval status (0 = OK, 1 = BUSY)
  */
uint8_t USBD_Bulk_TransmitPacket(void *pdev, uint8_t *buf, uint32_t len)
{
  /* Must check device is configured - endpoint not open before SET_CONFIGURATION */
  if (((USB_OTG_CORE_HANDLE*)pdev)->dev.device_status != USB_OTG_CONFIGURED)
    return 1;  /* not ready */

  if (USB_Tx_State != USB_BULK_IDLE)
    return 1;  /* busy */

  USB_Tx_State = USB_BULK_BUSY;

  DCD_EP_Tx(pdev,
            CDC_IN_EP,
            buf,
            len);

  return 0;
}

/**
  * @brief  USBD_cdc_GetCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_bulk_GetCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (usbd_bulk_CfgDesc);
  return usbd_bulk_CfgDesc;
}

#ifdef USE_USB_OTG_HS
static uint8_t  *USBD_bulk_GetOtherCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (usbd_bulk_OtherCfgDesc);
  return usbd_bulk_OtherCfgDesc;
}
#endif

/**
  * @brief  usbd_bulk_IsoIncomplete
  *         Dummy handler - usbd_core.c calls IsoINIncomplete/IsoOUTIncomplete
  *         without NULL check, so we must provide a valid function pointer.
  */
static uint8_t usbd_bulk_IsoIncomplete(void *pdev)
{
  return USBD_OK;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
