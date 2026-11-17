
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
#pragma     data_alignment = 4
#endif                          /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

/* Includes ------------------------------------------------------------------ */
#include "usbd_cdc_vcp.h"

/* USB OTG device handle - defined in vision.cpp */
extern USB_OTG_CORE_HANDLE USB_OTG_dev;

extern uint8_t USB_Tx_State;

/* Private function prototypes ----------------------------------------------- */
static uint16_t VCP_Init(void);
static uint16_t VCP_DeInit(void);
static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t * Buf, uint32_t Len);
static uint16_t VCP_DataTx(uint8_t* Buf, uint32_t Len);
static uint16_t VCP_DataRx(uint8_t * Buf, uint32_t Len);

CDC_IF_Prop_TypeDef VCP_fops = {
  VCP_Init,
  VCP_DeInit,
  VCP_Ctrl,
  VCP_DataTx,
  VCP_DataRx
};

/* Private functions --------------------------------------------------------- */

static uint16_t VCP_Init(void)
{
  return USBD_OK;
}

static uint16_t VCP_DeInit(void)
{
  return USBD_OK;
}

static uint16_t VCP_Ctrl(uint32_t Cmd, uint8_t * Buf, uint32_t Len)
{
  return USBD_OK;
}

/**
  * @brief  VCP_DataTx
  *         Direct bulk transmit - send data immediately via bulk IN endpoint
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else VCP_FAIL
  */
static uint16_t VCP_DataTx(uint8_t* Buf, uint32_t Len)
{
  /* Direct bulk transmit - non-blocking, drop data if busy or not ready */
  return USBD_Bulk_TransmitPacket(&USB_OTG_dev, Buf, Len);
}

/**
  * @brief  VCP_DataRx
  *         Data received over USB OUT endpoint are sent to application
  *         through this function.
  * @param  Buf: Buffer of data received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else VCP_FAIL
  */
static uint16_t VCP_DataRx(uint8_t * Buf, uint32_t Len)
{
  USB_REC_IQRHandler(Buf, Len);
  return USBD_OK;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
