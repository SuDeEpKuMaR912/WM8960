/**
  ******************************************************************************
  * @file    usbd_audio.c
  * @author  MCD Application Team
  * @brief   This file provides the Audio core functions.
  *
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * @verbatim
  *
  *          ===================================================================
  *                                AUDIO Class  Description
  *          ===================================================================
  *           This driver manages the Audio Class 1.0 following the "USB Device Class Definition for
  *           Audio Devices V1.0 Mar 18, 98".
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Standard AC Interface Descriptor management
  *             - 1 Audio Streaming Interface (with single channel, PCM, Stereo mode)
  *             - 1 Audio Streaming Endpoint
  *             - 1 Audio Terminal Input (1 channel)
  *             - Audio Class-Specific AC Interfaces
  *             - Audio Class-Specific AS Interfaces
  *             - AudioControl Requests: only SET_CUR and GET_CUR requests are supported (for Mute)
  *             - Audio Feature Unit (limited to Mute control)
  *             - Audio Synchronization type: Asynchronous
  *             - Single fixed audio sampling rate (configurable in usbd_conf.h file)
  *          The current audio class version supports the following audio features:
  *             - Pulse Coded Modulation (PCM) format
  *             - sampling rate: 48KHz.
  *             - Bit resolution: 16
  *             - Number of channels: 2
  *             - No volume control
  *             - Mute/Unmute capability
  *             - Asynchronous Endpoints
  *
  * @note     In HS mode and when the DMA is used, all variables and data structures
  *           dealing with the DMA during the transaction process should be 32-bit aligned.
  *
  *
  *  @endverbatim
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
- "stm32xxxxx_{eval}{discovery}_audio.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio.h"
#include "usbd_ctlreq.h"


/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_AUDIO
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_AUDIO_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_Macros
  * @{
  */
#define AUDIO_SAMPLE_FREQ(frq) \
  (uint8_t)(frq), (uint8_t)((frq >> 8)), (uint8_t)((frq >> 16))

#define AUDIO_PACKET_SZE(frq) \
  (uint8_t)(((frq * 2U * 2U) / 1000U) & 0xFFU), (uint8_t)((((frq * 2U * 2U) / 1000U) >> 8) & 0xFFU)

#ifdef USE_USBD_COMPOSITE
#define AUDIO_PACKET_SZE_WORD(frq)     (uint32_t)((((frq) * 2U * 2U)/1000U))
#endif /* USE_USBD_COMPOSITE  */
/**
  * @}
  */


/** @defgroup USBD_AUDIO_Private_FunctionPrototypes
  * @{
  */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);

static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req);
#ifndef USE_USBD_COMPOSITE
static uint8_t *USBD_AUDIO_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length);
#endif /* USE_USBD_COMPOSITE  */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev);
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev);

static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum);
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
static void *USBD_AUDIO_GetAudioHeaderDesc(uint8_t *pConfDesc);

/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Variables
  * @{
  */

USBD_ClassTypeDef USBD_AUDIO =
{
  USBD_AUDIO_Init,
  USBD_AUDIO_DeInit,
  USBD_AUDIO_Setup,
  USBD_AUDIO_EP0_TxReady,
  USBD_AUDIO_EP0_RxReady,
  USBD_AUDIO_DataIn,
  USBD_AUDIO_DataOut,
  USBD_AUDIO_SOF,
  USBD_AUDIO_IsoINIncomplete,
  USBD_AUDIO_IsoOutIncomplete,
#ifdef USE_USBD_COMPOSITE
  NULL,
  NULL,
  NULL,
  NULL,
#else
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetCfgDesc,
  USBD_AUDIO_GetDeviceQualifierDesc,
#endif /* USE_USBD_COMPOSITE  */
};

#ifndef USE_USBD_COMPOSITE
/* USB AUDIO device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_CfgDesc[USB_AUDIO_CONFIG_DESC_SIZ] __ALIGN_END =
{
  /* Configuration 1 */
  0x09,
  USB_DESC_TYPE_CONFIGURATION,
  LOBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
  HIBYTE(USB_AUDIO_CONFIG_DESC_SIZ),
  0x03,                                 /* bNumInterfaces: AC + AS-speaker + AS-mic (was 0x02) */
  0x01,
  0x00,
#if (USBD_SELF_POWERED == 1U)
  0xC0,
#else
  0x80,
#endif
  USBD_MAX_POWER,
  /* 09 byte */

  /* Interface 0: Audio Control, standard */
  AUDIO_INTERFACE_DESC_SIZE,
  USB_DESC_TYPE_INTERFACE,
  0x00, 0x00, 0x00,
  USB_DEVICE_CLASS_AUDIO,
  AUDIO_SUBCLASS_AUDIOCONTROL,
  AUDIO_PROTOCOL_UNDEFINED,
  0x00,
  /* 09 byte */

  /* Class-specific AC Interface Header -- now covers 2 streaming interfaces */
  0x0A,                                 /* bLength: 8 + bInCollection(2) (was 0x09) */
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,
  AUDIO_CONTROL_HEADER,
  0x00, 0x01,                           /* bcdADC 1.00 */
  0x46, 0x00,                           /* wTotalLength = 70 = AC-only descriptors below (was 0x27) */
  0x02,                                 /* bInCollection = 2 streaming interfaces (was 0x01) */
  0x01, 0x02,                           /* baInterfaceNr: IF1, IF2 (was just 0x01) */
  /* 10 byte */

  /* Input Terminal 1: USB streaming -> feeds the speaker path (unchanged) */
  AUDIO_INPUT_TERMINAL_DESC_SIZE,
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,
  AUDIO_CONTROL_INPUT_TERMINAL,
  0x01,
  0x01, 0x01,                           /* wTerminalType 0x0101 USB streaming */
  0x00,
  0x02,
  0x03, 0x00,                           /* L+R */
  0x00, 0x00,
  /* 12 byte */

  /* Feature Unit 2: playback mute, source = Terminal 1 (unchanged) */
  0x09,
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,
  AUDIO_CONTROL_FEATURE_UNIT,
  AUDIO_OUT_STREAMING_CTRL,             /* bUnitID = 2 */
  0x01,                                 /* bSourceID = 1 */
  0x01,
  AUDIO_CONTROL_MUTE, 0,
  0x00,
  /* 09 byte */

  /* Output Terminal 3: Speaker, source = Feature Unit 2 (unchanged) */
  0x09,
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,
  AUDIO_CONTROL_OUTPUT_TERMINAL,
  0x03,
  0x01, 0x03,                           /* wTerminalType 0x0301 Speaker */
  0x00,
  0x02,                                 /* bSourceID = 2 */
  0x00,
  /* 09 byte */

  /* --- NEW: Input Terminal 4: Microphone --- */
  AUDIO_INPUT_TERMINAL_DESC_SIZE,
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,
  AUDIO_CONTROL_INPUT_TERMINAL,
  0x04,
  0x01, 0x02,                           /* wTerminalType 0x0201 Microphone */
  0x00,
  0x02,                                 /* stereo, matches rx_buf's L/R layout */
  0x03, 0x00,
  0x00, 0x00,
  /* 12 byte */

  /* --- NEW: Feature Unit 5: mic mute, source = Terminal 4 --- */
  0x09,
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,
  AUDIO_CONTROL_FEATURE_UNIT,
  AUDIO_IN_STREAMING_CTRL,              /* bUnitID = 5 */
  0x04,                                 /* bSourceID = 4 */
  0x01,
  AUDIO_CONTROL_MUTE, 0,
  0x00,
  /* 09 byte */

  /* --- NEW: Output Terminal 6: USB streaming, captured mic data to host --- */
  0x09,
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,
  AUDIO_CONTROL_OUTPUT_TERMINAL,
  0x06,
  0x01, 0x01,                           /* wTerminalType 0x0101 USB streaming */
  0x00,
  0x05,                                 /* bSourceID = 5 */
  0x00,
  /* 09 byte */

  /* Interface 1, Alt 0 - zero bandwidth (unchanged) */
  AUDIO_INTERFACE_DESC_SIZE,
  USB_DESC_TYPE_INTERFACE,
  0x01, 0x00, 0x00,
  USB_DEVICE_CLASS_AUDIO,
  AUDIO_SUBCLASS_AUDIOSTREAMING,
  AUDIO_PROTOCOL_UNDEFINED,
  0x00,
  /* 09 byte */

  /* Interface 1, Alt 1 - operational (unchanged) */
  AUDIO_INTERFACE_DESC_SIZE,
  USB_DESC_TYPE_INTERFACE,
  0x01, 0x01, 0x01,
  USB_DEVICE_CLASS_AUDIO,
  AUDIO_SUBCLASS_AUDIOSTREAMING,
  AUDIO_PROTOCOL_UNDEFINED,
  0x00,
  /* 09 byte */

  /* Class-specific AS General, speaker (unchanged) */
  AUDIO_STREAMING_INTERFACE_DESC_SIZE,
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,
  AUDIO_STREAMING_GENERAL,
  0x01,                                 /* bTerminalLink = Input Terminal 1 */
  0x01,
  0x01, 0x00,
  /* 07 byte */

  /* Format Type I, speaker (unchanged) */
  0x0B,
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,
  AUDIO_STREAMING_FORMAT_TYPE,
  AUDIO_FORMAT_TYPE_I,
  0x02, 0x02, 16, 0x01,
  AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ),
  /* 11 byte */

  /* Endpoint 1 OUT (unchanged) */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,
  USB_DESC_TYPE_ENDPOINT,
  AUDIO_OUT_EP,
  USBD_EP_TYPE_ISOC,
  AUDIO_PACKET_SZE(USBD_AUDIO_FREQ),
  AUDIO_FS_BINTERVAL,
  0x00, 0x00,
  /* 09 byte */

  /* Endpoint - Audio Streaming, OUT (unchanged) */
  AUDIO_STREAMING_ENDPOINT_DESC_SIZE,
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,
  AUDIO_ENDPOINT_GENERAL,
  0x00, 0x00, 0x00, 0x00,
  /* 07 byte */

  /* --- NEW: Interface 2, Alt 0 - zero bandwidth (mic) --- */
  AUDIO_INTERFACE_DESC_SIZE,
  USB_DESC_TYPE_INTERFACE,
  0x02, 0x00, 0x00,
  USB_DEVICE_CLASS_AUDIO,
  AUDIO_SUBCLASS_AUDIOSTREAMING,
  AUDIO_PROTOCOL_UNDEFINED,
  0x00,
  /* 09 byte */

  /* --- NEW: Interface 2, Alt 1 - operational (mic) --- */
  AUDIO_INTERFACE_DESC_SIZE,
  USB_DESC_TYPE_INTERFACE,
  0x02, 0x01, 0x01,
  USB_DEVICE_CLASS_AUDIO,
  AUDIO_SUBCLASS_AUDIOSTREAMING,
  AUDIO_PROTOCOL_UNDEFINED,
  0x00,
  /* 09 byte */

  /* --- NEW: Class-specific AS General, mic --- */
  AUDIO_STREAMING_INTERFACE_DESC_SIZE,
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,
  AUDIO_STREAMING_GENERAL,
  0x06,                                 /* bTerminalLink = Output Terminal 6 */
  0x01,
  0x01, 0x00,
  /* 07 byte */

  /* --- NEW: Format Type I, mic -- same 48 kHz / 16-bit / stereo as rx_buf --- */
  0x0B,
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,
  AUDIO_STREAMING_FORMAT_TYPE,
  AUDIO_FORMAT_TYPE_I,
  0x02, 0x02, 16, 0x01,
  AUDIO_SAMPLE_FREQ(USBD_AUDIO_FREQ),
  /* 11 byte */

  /* --- NEW: Endpoint 1 IN (mic), reuses the TX FIFO already reserved for EP1 --- */
  AUDIO_STANDARD_ENDPOINT_DESC_SIZE,
  USB_DESC_TYPE_ENDPOINT,
  AUDIO_IN_EP,
  USBD_EP_TYPE_ISOC,
  AUDIO_PACKET_SZE(USBD_AUDIO_FREQ),
  AUDIO_FS_BINTERVAL,
  0x00, 0x00,
  /* 09 byte */

  /* --- NEW: Endpoint - Audio Streaming, IN --- */
  AUDIO_STREAMING_ENDPOINT_DESC_SIZE,
  AUDIO_ENDPOINT_DESCRIPTOR_TYPE,
  AUDIO_ENDPOINT_GENERAL,
  0x00, 0x00, 0x00, 0x00,
  /* 07 byte */
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_AUDIO_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};
#endif /* USE_USBD_COMPOSITE  */

static uint8_t AUDIOOutEpAdd = AUDIO_OUT_EP;
static uint8_t AUDIOInEpAdd  = AUDIO_IN_EP;      /* new */
/**
  * @}
  */

/** @defgroup USBD_AUDIO_Private_Functions
  * @{
  */

/**
  * @brief  USBD_AUDIO_Init
  *         Initialize the AUDIO interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);
  USBD_AUDIO_HandleTypeDef *haudio;

  /* Allocate Audio structure */
  haudio = (USBD_AUDIO_HandleTypeDef *)USBD_malloc(sizeof(USBD_AUDIO_HandleTypeDef));

  if (haudio == NULL)
  {
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    return (uint8_t)USBD_EMEM;
  }

  pdev->pClassDataCmsit[pdev->classId] = (void *)haudio;
  pdev->pClassData = pdev->pClassDataCmsit[pdev->classId];

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    pdev->ep_out[AUDIOOutEpAdd & 0xFU].bInterval = AUDIO_HS_BINTERVAL;
  }
  else   /* LOW and FULL-speed endpoints */
  {
    pdev->ep_out[AUDIOOutEpAdd & 0xFU].bInterval = AUDIO_FS_BINTERVAL;
  }

  /* Open EP OUT */
  (void)USBD_LL_OpenEP(pdev, AUDIOOutEpAdd, USBD_EP_TYPE_ISOC, AUDIO_OUT_PACKET);
  pdev->ep_out[AUDIOOutEpAdd & 0xFU].is_used = 1U;

  haudio->alt_setting = 0U;
  haudio->offset = AUDIO_OFFSET_UNKNOWN;
  haudio->wr_ptr = 0U;
  haudio->rd_ptr = 0U;
  haudio->rd_enable = 0U;

  /* Initialize the Audio output Hardware layer */
  if (((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->Init(USBD_AUDIO_FREQ,
                                                                      AUDIO_DEFAULT_VOLUME,
                                                                      0U) != 0U)
  {
    return (uint8_t)USBD_FAIL;
  }

  /* Prepare Out endpoint to receive 1st packet */
  (void)USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd, haudio->buffer,
                               AUDIO_OUT_PACKET);

  /* --- new: open the mic IN endpoint --- */
  if (pdev->dev_speed == USBD_SPEED_HIGH)
  {
    pdev->ep_in[AUDIOInEpAdd & 0xFU].bInterval = AUDIO_HS_BINTERVAL;
  }
  else
  {
    pdev->ep_in[AUDIOInEpAdd & 0xFU].bInterval = AUDIO_FS_BINTERVAL;
  }

  (void)USBD_LL_OpenEP(pdev, AUDIOInEpAdd, USBD_EP_TYPE_ISOC, AUDIO_IN_PACKET);
  pdev->ep_in[AUDIOInEpAdd & 0xFU].is_used = 1U;

  haudio->alt_setting_in = 0U;
  haudio->in_rd_ptr = 0U;
  haudio->in_wr_ptr = 0U;
  memset(haudio->buffer_in, 0, sizeof(haudio->buffer_in));

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Init
  *         DeInitialize the AUDIO layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t USBD_AUDIO_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
  UNUSED(cfgidx);

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  /* Open EP OUT */
  (void)USBD_LL_CloseEP(pdev, AUDIOOutEpAdd);
  pdev->ep_out[AUDIOOutEpAdd & 0xFU].is_used = 0U;
  pdev->ep_out[AUDIOOutEpAdd & 0xFU].bInterval = 0U;

  /* DeInit  physical Interface components */
  if (pdev->pClassDataCmsit[pdev->classId] != NULL)
  {
    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->DeInit(0U);
    (void)USBD_free(pdev->pClassDataCmsit[pdev->classId]);
    pdev->pClassDataCmsit[pdev->classId] = NULL;
    pdev->pClassData = NULL;
  }

  (void)USBD_LL_CloseEP(pdev, AUDIOInEpAdd);
  pdev->ep_in[AUDIOInEpAdd & 0xFU].is_used = 0U;
  pdev->ep_in[AUDIOInEpAdd & 0xFU].bInterval = 0U;

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_Setup
  *         Handle the AUDIO specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t USBD_AUDIO_Setup(USBD_HandleTypeDef *pdev,
                                USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  uint16_t len;
  uint8_t *pbuf;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    case USB_REQ_TYPE_CLASS:
      switch (req->bRequest)
      {
        case AUDIO_REQ_GET_CUR:
          AUDIO_REQ_GetCurrent(pdev, req);
          break;

        case AUDIO_REQ_SET_CUR:
          AUDIO_REQ_SetCurrent(pdev, req);
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;

    case USB_REQ_TYPE_STANDARD:
      switch (req->bRequest)
      {
        case USB_REQ_GET_STATUS:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            (void)USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_GET_DESCRIPTOR:
          if ((req->wValue >> 8) == AUDIO_DESCRIPTOR_TYPE)
          {
            pbuf = (uint8_t *)USBD_AUDIO_GetAudioHeaderDesc(pdev->pConfDesc);
            if (pbuf != NULL)
            {
              len = MIN(USB_AUDIO_DESC_SIZ, req->wLength);
              (void)USBD_CtlSendData(pdev, pbuf, len);
            }
            else
            {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          break;

        case USB_REQ_GET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            if (req->wIndex == 0x02U)
            {
              (void)USBD_CtlSendData(pdev, (uint8_t *)&haudio->alt_setting_in, 1U);
            }
            else
            {
              (void)USBD_CtlSendData(pdev, (uint8_t *)&haudio->alt_setting, 1U);
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_SET_INTERFACE:
          if (pdev->dev_state == USBD_STATE_CONFIGURED)
          {
            if (req->wIndex == 0x02U)                 /* Interface 2: microphone */
            {
              haudio->alt_setting_in = (uint8_t)(req->wValue);

              if (haudio->alt_setting_in == 1U)
              {
                /* Host just opened the mic stream: arm the first IN packet */
                haudio->in_rd_ptr = 0U;
                (void)USBD_LL_Transmit(pdev, AUDIOInEpAdd, &haudio->buffer_in[0], AUDIO_IN_PACKET);
              }
              else
              {
                (void)USBD_LL_FlushEP(pdev, AUDIOInEpAdd);
              }
            }
            else if ((uint8_t)(req->wValue) <= USBD_MAX_NUM_INTERFACES)  /* Interface 1: speaker */
            {
              haudio->alt_setting = (uint8_t)(req->wValue);
            }
            else
            {
              USBD_CtlError(pdev, req);
              ret = USBD_FAIL;
            }
          }
          else
          {
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
          }
          break;

        case USB_REQ_CLEAR_FEATURE:
          break;

        default:
          USBD_CtlError(pdev, req);
          ret = USBD_FAIL;
          break;
      }
      break;
    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
  }

  return (uint8_t)ret;
}

#ifndef USE_USBD_COMPOSITE
/**
  * @brief  USBD_AUDIO_GetCfgDesc
  *         return configuration descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_AUDIO_GetCfgDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_AUDIO_CfgDesc);

  return USBD_AUDIO_CfgDesc;
}
#endif /* USE_USBD_COMPOSITE  */
/**
  * @brief  USBD_AUDIO_DataIn
  *         handle data IN Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef *haudio;

  if (epnum != (AUDIOInEpAdd & 0x7FU))
  {
    return (uint8_t)USBD_OK;
  }

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];
  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (haudio->alt_setting_in == 1U)
  {
    (void)USBD_LL_Transmit(pdev, AUDIOInEpAdd,
                           &haudio->buffer_in[haudio->in_rd_ptr], AUDIO_IN_PACKET);

    haudio->in_rd_ptr += AUDIO_IN_PACKET;
    if (haudio->in_rd_ptr >= AUDIO_IN_TOTAL_BUF_SIZE)
    {
      haudio->in_rd_ptr = 0U;
    }
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_EP0_RxReady
  *         handle EP0 Rx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (haudio->control.cmd == AUDIO_REQ_SET_CUR)
  {
    /* In this driver, to simplify code, only SET_CUR request is managed */

    if (haudio->control.unit == AUDIO_OUT_STREAMING_CTRL)
    {
      ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->MuteCtl(haudio->control.data[0]);
      haudio->control.cmd = 0U;
      haudio->control.len = 0U;
    }
  }

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_EP0_TxReady
  *         handle EP0 TRx Ready event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_EP0_TxReady(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);

  /* Only OUT control data are processed */
  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @retval status
  */
static uint8_t USBD_AUDIO_SOF(USBD_HandleTypeDef *pdev)
{
  UNUSED(pdev);

  return (uint8_t)USBD_OK;
}

/**
  * @brief  USBD_AUDIO_SOF
  *         handle SOF event
  * @param  pdev: device instance
  * @param  offset: audio offset
  * @retval status
  */
void USBD_AUDIO_Sync(USBD_HandleTypeDef *pdev, AUDIO_OffsetTypeDef offset)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  uint32_t BufferSize = AUDIO_TOTAL_BUF_SIZE / 2U;

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return;
  }

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  haudio->offset = offset;

  if (haudio->rd_enable == 1U)
  {
    haudio->rd_ptr += (uint16_t)BufferSize;

    if (haudio->rd_ptr == AUDIO_TOTAL_BUF_SIZE)
    {
      /* roll back */
      haudio->rd_ptr = 0U;
    }
  }

  if (haudio->rd_ptr > haudio->wr_ptr)
  {
    if ((haudio->rd_ptr - haudio->wr_ptr) < AUDIO_OUT_PACKET)
    {
      BufferSize += 4U;
    }
    else
    {
      if ((haudio->rd_ptr - haudio->wr_ptr) > (AUDIO_TOTAL_BUF_SIZE - AUDIO_OUT_PACKET))
      {
        BufferSize -= 4U;
      }
    }
  }
  else
  {
    if ((haudio->wr_ptr - haudio->rd_ptr) < AUDIO_OUT_PACKET)
    {
      BufferSize -= 4U;
    }
    else
    {
      if ((haudio->wr_ptr - haudio->rd_ptr) > (AUDIO_TOTAL_BUF_SIZE - AUDIO_OUT_PACKET))
      {
        BufferSize += 4U;
      }
    }
  }

  if (haudio->offset == AUDIO_OFFSET_FULL)
  {
    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->AudioCmd(&haudio->buffer[0],
                                                                        BufferSize, AUDIO_CMD_PLAY);
    haudio->offset = AUDIO_OFFSET_NONE;
  }
}

/**
  * @brief  Pushes freshly-captured PCM data (from the SAI RX DMA callback) into
  *         the ring buffer that USBD_AUDIO_DataIn() drains toward the host.
  * @param  pdev: device instance
  * @param  psrc: pointer to the newly-available PCM bytes
  * @param  size: number of bytes
  * @retval None
  */
void USBD_AUDIO_Record_Push(USBD_HandleTypeDef *pdev, uint8_t *psrc, uint32_t size)
{
  USBD_AUDIO_HandleTypeDef *haudio;

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return;
  }

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if ((haudio->in_wr_ptr + size) <= AUDIO_IN_TOTAL_BUF_SIZE)
  {
    memcpy(&haudio->buffer_in[haudio->in_wr_ptr], psrc, size);
  }
  else
  {
    uint32_t first = AUDIO_IN_TOTAL_BUF_SIZE - haudio->in_wr_ptr;
    memcpy(&haudio->buffer_in[haudio->in_wr_ptr], psrc, first);
    memcpy(&haudio->buffer_in[0], psrc + first, size - first);
  }

  haudio->in_wr_ptr += (uint16_t)size;
  if (haudio->in_wr_ptr >= AUDIO_IN_TOTAL_BUF_SIZE)
  {
    haudio->in_wr_ptr -= AUDIO_IN_TOTAL_BUF_SIZE;
  }
}

/**
  * @brief  USBD_AUDIO_IsoINIncomplete
  *         handle data ISO IN Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoINIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  if (epnum == (AUDIOInEpAdd & 0x7FU))
  {
    (void)USBD_LL_FlushEP(pdev, AUDIOInEpAdd);
  }
  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_IsoOutIncomplete
  *         handle data ISO OUT Incomplete event
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_IsoOutIncomplete(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  USBD_AUDIO_HandleTypeDef *haudio;

  if (pdev->pClassDataCmsit[pdev->classId] == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  /* Prepare Out endpoint to receive next audio packet */
  (void)USBD_LL_PrepareReceive(pdev, epnum,
                               &haudio->buffer[haudio->wr_ptr],
                               AUDIO_OUT_PACKET);

  return (uint8_t)USBD_OK;
}
/**
  * @brief  USBD_AUDIO_DataOut
  *         handle data OUT Stage
  * @param  pdev: device instance
  * @param  epnum: endpoint index
  * @retval status
  */
static uint8_t USBD_AUDIO_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  uint16_t PacketSize;
  USBD_AUDIO_HandleTypeDef *haudio;

#ifdef USE_USBD_COMPOSITE
  /* Get the Endpoints addresses allocated for this class instance */
  AUDIOOutEpAdd = USBD_CoreGetEPAdd(pdev, USBD_EP_OUT, USBD_EP_TYPE_ISOC, (uint8_t)pdev->classId);
#endif /* USE_USBD_COMPOSITE */

  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  if (epnum == AUDIOOutEpAdd)
  {
    /* Get received data packet length */
    PacketSize = (uint16_t)USBD_LL_GetRxDataSize(pdev, epnum);

    /* Packet received Callback */
    ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->PeriodicTC(&haudio->buffer[haudio->wr_ptr],
                                                                          PacketSize, AUDIO_OUT_TC);

    /* Increment the Buffer pointer or roll it back when all buffers are full */
    haudio->wr_ptr += PacketSize;

    if (haudio->wr_ptr >= AUDIO_TOTAL_BUF_SIZE)
    {
      /* All buffers are full: roll back */
      haudio->wr_ptr = 0U;

      if (haudio->offset == AUDIO_OFFSET_UNKNOWN)
      {
        ((USBD_AUDIO_ItfTypeDef *)pdev->pUserData[pdev->classId])->AudioCmd(&haudio->buffer[0],
                                                                            AUDIO_TOTAL_BUF_SIZE / 2U,
                                                                            AUDIO_CMD_START);
        haudio->offset = AUDIO_OFFSET_NONE;
      }
    }

    if (haudio->rd_enable == 0U)
    {
      if (haudio->wr_ptr == (AUDIO_TOTAL_BUF_SIZE / 2U))
      {
        haudio->rd_enable = 1U;
      }
    }

    /* Prepare Out endpoint to receive next audio packet */
    (void)USBD_LL_PrepareReceive(pdev, AUDIOOutEpAdd,
                                 &haudio->buffer[haudio->wr_ptr],
                                 AUDIO_OUT_PACKET);
  }

  return (uint8_t)USBD_OK;
}

/**
  * @brief  AUDIO_Req_GetCurrent
  *         Handles the GET_CUR Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_GetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  (void)USBD_memset(haudio->control.data, 0, USB_MAX_EP0_SIZE);

  /* Send the current mute state */
  (void)USBD_CtlSendData(pdev, haudio->control.data,
                         MIN(req->wLength, USB_MAX_EP0_SIZE));
}

/**
  * @brief  AUDIO_Req_SetCurrent
  *         Handles the SET_CUR Audio control request.
  * @param  pdev: device instance
  * @param  req: setup class request
  * @retval status
  */
static void AUDIO_REQ_SetCurrent(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
  USBD_AUDIO_HandleTypeDef *haudio;
  haudio = (USBD_AUDIO_HandleTypeDef *)pdev->pClassDataCmsit[pdev->classId];

  if (haudio == NULL)
  {
    return;
  }

  if (req->wLength != 0U)
  {
    haudio->control.cmd = AUDIO_REQ_SET_CUR;     /* Set the request value */
    haudio->control.len = (uint8_t)MIN(req->wLength, USB_MAX_EP0_SIZE);  /* Set the request data length */
    haudio->control.unit = HIBYTE(req->wIndex);  /* Set the request target unit */

    /* Prepare the reception of the buffer over EP0 */
    (void)USBD_CtlPrepareRx(pdev, haudio->control.data, haudio->control.len);
  }
}

#ifndef USE_USBD_COMPOSITE
/**
  * @brief  DeviceQualifierDescriptor
  *         return Device Qualifier descriptor
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t *USBD_AUDIO_GetDeviceQualifierDesc(uint16_t *length)
{
  *length = (uint16_t)sizeof(USBD_AUDIO_DeviceQualifierDesc);

  return USBD_AUDIO_DeviceQualifierDesc;
}
#endif /* USE_USBD_COMPOSITE  */
/**
  * @brief  USBD_AUDIO_RegisterInterface
  * @param  pdev: device instance
  * @param  fops: Audio interface callback
  * @retval status
  */
uint8_t USBD_AUDIO_RegisterInterface(USBD_HandleTypeDef *pdev,
                                     USBD_AUDIO_ItfTypeDef *fops)
{
  if (fops == NULL)
  {
    return (uint8_t)USBD_FAIL;
  }

  pdev->pUserData[pdev->classId] = fops;

  return (uint8_t)USBD_OK;
}

#ifdef USE_USBD_COMPOSITE
/**
  * @brief  USBD_AUDIO_GetEpPcktSze
  * @param  pdev: device instance (reserved for future use)
  * @param  If: Interface number (reserved for future use)
  * @param  Ep: Endpoint number (reserved for future use)
  * @retval status
  */
uint32_t USBD_AUDIO_GetEpPcktSze(USBD_HandleTypeDef *pdev, uint8_t If, uint8_t Ep)
{
  uint32_t mps;

  UNUSED(pdev);
  UNUSED(If);
  UNUSED(Ep);

  mps = AUDIO_PACKET_SZE_WORD(USBD_AUDIO_FREQ);

  /* Return the wMaxPacketSize value in Bytes (Freq(Samples)*2(Stereo)*2(HalfWord)) */
  return mps;
}
#endif /* USE_USBD_COMPOSITE */

/**
  * @brief  USBD_AUDIO_GetAudioHeaderDesc
  *         This function return the Audio descriptor
  * @param  pdev: device instance
  * @param  pConfDesc:  pointer to Bos descriptor
  * @retval pointer to the Audio AC Header descriptor
  */
static void *USBD_AUDIO_GetAudioHeaderDesc(uint8_t *pConfDesc)
{
  USBD_ConfigDescTypeDef *desc = (USBD_ConfigDescTypeDef *)(void *)pConfDesc;
  USBD_DescHeaderTypeDef *pdesc = (USBD_DescHeaderTypeDef *)(void *)pConfDesc;
  uint8_t *pAudioDesc =  NULL;
  uint16_t ptr;

  if (desc->wTotalLength > desc->bLength)
  {
    ptr = desc->bLength;

    while (ptr < desc->wTotalLength)
    {
      pdesc = USBD_GetNextDesc((uint8_t *)pdesc, &ptr);
      if ((pdesc->bDescriptorType == AUDIO_INTERFACE_DESCRIPTOR_TYPE) &&
          (pdesc->bDescriptorSubType == AUDIO_CONTROL_HEADER))
      {
        pAudioDesc = (uint8_t *)pdesc;
        break;
      }
    }
  }
  return pAudioDesc;
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
