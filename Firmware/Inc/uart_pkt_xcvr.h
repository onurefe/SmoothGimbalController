/**
  ******************************************************************************
  * @file    uart_pkt_xcvr.h
  * @author  Onur Efe
  * @date    05.05.2016
  * @brief   UART packet transceiver class interface.
  ******************************************************************************
  *
  * Copyright (c) 2016 MEDIALAB TECHNOLOGY
  *
  * This program is free software; you can redistribute it and/or modify it 
  * under the terms of the GNU General Public License as published by the Free 
  * Software Foundation; either version 2 of the License, or (at your option) 
  * any later version. This program is distributed in the hope that it will be 
  * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General 
  * Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_PKT_XCVR_H
#define __UART_PKT_XCVR_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "generic.h"

/* Exported constants */
#define UART_PKT_XCVR_BUFFER_SIZE               14U
#define UART_PKT_XCVR_PACKET_SIZE               12U

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  UART_PKT_XCVR_LOCK_STATE_LOCKED               = 0x00,                 /* Module is locked */
  UART_PKT_XCVR_LOCK_STATE_UNLOCKED             = 0x01                  /* Unlocked */
} UartPktXcvr_LockState_t;                                              /* Lock State typedef */

typedef enum
{
  UART_PKT_XCVR_STATE_UNINIT                    = 0x00,                 /* Module is not initialized */
  UART_PKT_XCVR_STATE_READY                     = 0x01,                 /* Ready */
  UART_PKT_XCVR_STATE_OPERATING                 = 0x02,                 /* Operating */
  UART_PKT_XCVR_STATE_ERROR                     = 0x03                  /* Error */
} UartPktXcvr_State_t;                                                  /* State typedef */

typedef enum
{
  UART_PKT_XCVR_ERR_CODE_NONE                   = 0x00,                 /* None */
  UART_PKT_XCVR_ERR_CODE_INVALID_PARAMETER      = 0x01,                 /* Invalid parameter */
  UART_PKT_XCVR_ERR_CODE_INCOMPATIBLE_STATE     = 0x02,                 /* Incompatible state */
  UART_PKT_XCVR_ERR_CODE_HAL_UART_ERROR         = 0x03,                 /* HAL UART error */
  UART_PKT_XCVR_ERR_CODE_RACE_CONDITION         = 0x04                  /* Race condition occured */
} UartPktXcvr_ErrCode_t;                                                /* Error Code typedef */
 
typedef enum
{
  UART_PKT_XCVR_OP_RES_SUCCESS                  = 0x00,                 /* Success */
  UART_PKT_XCVR_OP_RES_FAILURE                  = 0x01                  /* Failure */
} UartPktXcvr_OpRes_t;                                                  /* Operation result typedef */

typedef struct
{
  UART_HandleTypeDef                            *huart;                 /* Uart object handle */
} UartPktXcvr_Init_t;                                                   /* Init struct typedef */

typedef struct
{
  uint8_t                                       buffer[UART_PKT_XCVR_BUFFER_SIZE];      /* Buffer */
  uint8_t                                       receiverState;          /* Receiver state */
  uint8_t                                       callbackType;           /* Callback type */
  uint8_t                                       unhandledIntFlag;       /* Unhandled interrupt flag */
} UartPktXcvr_Statics_t;                                                /* Statics typedef */

typedef struct
{
  UartPktXcvr_LockState_t                       lockState;              /* Lock state */
  UartPktXcvr_State_t                           state;                  /* State */
  UartPktXcvr_ErrCode_t                         errCode;                /* Error Code */
  UartPktXcvr_Init_t                            init;                   /* Init */
  UartPktXcvr_Statics_t                         statics;                /* Static variables */
} UartPktXcvr_Handle_t;                                                 /* Uart packet transceiver handle typedef */

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Initializes module.
  *
  * @param  pHandle: Pointer to the module's handle.
  * @param  pInitStruct: Struct containing initializing variables.
  *
  * @retval UartPktXcvr_OpRes_t.
  */
UartPktXcvr_OpRes_t UartPktXcvr_Init(UartPktXcvr_Handle_t *pHandle,
                                     UartPktXcvr_Init_t *pInitStruct);

/**
  * @brief  Starts module.
  *
  * @param  pHandle: Pointer to the module's handle.
  *
  * @retval UartPktXcvr_OpRes_t.
  */
UartPktXcvr_OpRes_t UartPktXcvr_Start(UartPktXcvr_Handle_t *pHandle);

/**
  * @brief  Stops operating module.
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval UartPktXcvr_OpRes_t.
  */
UartPktXcvr_OpRes_t UartPktXcvr_Stop(UartPktXcvr_Handle_t *pHandle);

/**
  * @brief  Reset module to the uninitialized state.
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval UartPktXcvr_OpRes_t.
  */
UartPktXcvr_OpRes_t UartPktXcvr_Reset(UartPktXcvr_Handle_t *pHandle);

/**
  * @brief  Callback which is triggered when a packet is received.
  *
  * @param  pHandle: Pointer to module's handle.
  * @param  pPacket: Pointer to received packet.
  *
  * @retval None.
  */
void UartPktXcvr_PacketReceivedCallback(UartPktXcvr_Handle_t *pHandle,
                                        uint8_t *pPacket);
/**
  * @brief  Callback which is triggered when error occured.
  *
  * @param  pHandle: Pointer to module's handle.
  */
void UartPktXcvr_ErrorOccuredCallback(UartPktXcvr_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif

#endif