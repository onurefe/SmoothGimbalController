/**
  ******************************************************************************
  * @file    uart_pkt_xcvr.c
  * @author  Onur Efe
  * @date    05.05.2016
  * @brief   USART packet transceiver class implementation.
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
/* Includes ------------------------------------------------------------------*/
#include "uart_pkt_xcvr.h"

/* Private constants ---------------------------------------------------------*/
#define MAX_MODULE_COUNT                        5U 

#define RECEIVER_STATE_AWAITING_START_CONDITION 0x01
#define RECEIVER_STATE_RECEIVING_PACKET         0x02

#define RX_COMPLETED_CB                         0x00
#define ERROR_CB                                0x01

#define START_CONDITION_VALUE                   0xFF
#define CORRECTION_BITMAP_INDEX                 12U

/* Private macros ------------------------------------------------------------*/
#define __ERROR(_HANDLE_, _ERR_CODE_)   \
                                          do {                                                                  \
                                                (_HANDLE_)->state = (UART_PKT_XCVR_STATE_ERROR);                \
                                                (_HANDLE_)->errCode = (_ERR_CODE_);                             \
                                                (_HANDLE_)->lockState = (UART_PKT_XCVR_LOCK_STATE_UNLOCKED);    \
                                                return (UART_PKT_XCVR_OP_RES_FAILURE);                          \
                                          } while(0)

#define __ERROR_INT(_HANDLE_, _ERR_CODE_)       \
                                          do {                                                                  \
                                                (_HANDLE_)->state = (UART_PKT_XCVR_STATE_ERROR);                \
                                                (_HANDLE_)->errCode = (_ERR_CODE_);                             \
                                                (_HANDLE_)->lockState = (UART_PKT_XCVR_LOCK_STATE_UNLOCKED);    \
                                                UartPktXcvr_ErrorOccurredCallback((_HANDLE_));                  \
                                                return;                                                         \
                                          } while(0)

#define __LOCK(_HANDLE_)                        ((_HANDLE_)->lockState = (UART_PKT_XCVR_LOCK_STATE_LOCKED))

#define __UNLOCK(_HANDLE_)                      ((_HANDLE_)->lockState = (UART_PKT_XCVR_LOCK_STATE_UNLOCKED))

#define IS_LOCKED(_HANDLE_)                     ((_HANDLE_)->lockState == (UART_PKT_XCVR_LOCK_STATE_LOCKED))

/* Private variables ---------------------------------------------------------*/
static UartPktXcvr_Handle_t *RegisteredModules[MAX_MODULE_COUNT];
static uint8_t NumOfModules = 0;

/* Private functions ---------------------------------------------------------*/
static void UartCBHandler(UART_HandleTypeDef *huart, uint8_t UartCBType);

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
                                     UartPktXcvr_Init_t *pInitStruct)
{
  /* Check if the module is at error state */
  if (pHandle->state == UART_PKT_XCVR_STATE_ERROR)
  {
    return UART_PKT_XCVR_OP_RES_FAILURE;
  }
  
  __LOCK(pHandle);
  
  /* Check if the module's state is compatible with the operation */
  if (pHandle->state != UART_PKT_XCVR_STATE_UNINIT)
  {
    __ERROR(pHandle, UART_PKT_XCVR_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Check if the uart module has been initialized */
  if (pInitStruct->huart->gState == HAL_UART_STATE_RESET)
  {
    __ERROR(pHandle, UART_PKT_XCVR_ERR_CODE_HAL_UART_ERROR);
  }
  
  /* Register handle */
  RegisteredModules[NumOfModules++] = pHandle;

  /* Initialize variables */
  pHandle->init.huart = pInitStruct->huart;
  
  /* Set module's state to ready */
  pHandle->state = UART_PKT_XCVR_STATE_READY;
  
  __UNLOCK(pHandle);

  return UART_PKT_XCVR_OP_RES_SUCCESS;
}

/**
  * @brief  Starts module.
  *
  * @param  pHandle: Pointer to the module's handle.
  *
  * @retval UartPktXcvr_OpRes_t.
  */
UartPktXcvr_OpRes_t UartPktXcvr_Start(UartPktXcvr_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == UART_PKT_XCVR_STATE_ERROR)
  {
    return UART_PKT_XCVR_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, UART_PKT_XCVR_ERR_CODE_RACE_CONDITION);
  }

  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != UART_PKT_XCVR_STATE_READY)
  {
    __ERROR(pHandle, UART_PKT_XCVR_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Set static variables to their initial values */
  pHandle->statics.receiverState = RECEIVER_STATE_AWAITING_START_CONDITION;
  pHandle->statics.unhandledIntFlag = FALSE;
  
  /* Set module's state to operating */
  pHandle->state = UART_PKT_XCVR_STATE_OPERATING;
  
  /* Ensure that the peripheral's buffer is empty */
  uint8_t retval;
  retval = HAL_UART_Receive(pHandle->init.huart, pHandle->statics.buffer, 
                            UART_PKT_XCVR_BUFFER_SIZE, 250);
  
  if ((retval != HAL_OK) && (retval != HAL_TIMEOUT))
  {
    __ERROR(pHandle, UART_PKT_XCVR_ERR_CODE_HAL_UART_ERROR);
  }
  
  /* Trigger first interrupt reception */
  if (HAL_UART_Receive_DMA(pHandle->init.huart, pHandle->statics.buffer,
                           sizeof(uint8_t)) != HAL_OK)
  {
    __ERROR(pHandle, UART_PKT_XCVR_ERR_CODE_HAL_UART_ERROR);
  }
  
  __UNLOCK(pHandle);
  
  return UART_PKT_XCVR_OP_RES_SUCCESS;
}

/**
  * @brief  Stops operating module.
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval UartPktXcvr_OpRes_t.
  */
UartPktXcvr_OpRes_t UartPktXcvr_Stop(UartPktXcvr_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == UART_PKT_XCVR_STATE_ERROR)
  {
    return UART_PKT_XCVR_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, UART_PKT_XCVR_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check if the module's state is compatible with the operation */
  if (pHandle->state != UART_PKT_XCVR_STATE_OPERATING)
  {
    __ERROR(pHandle, UART_PKT_XCVR_ERR_CODE_INCOMPATIBLE_STATE);
  }

  /* Set module's state to ready */
  pHandle->state = UART_PKT_XCVR_STATE_READY;
  
  __UNLOCK(pHandle);
 
  return UART_PKT_XCVR_OP_RES_SUCCESS;
}

/**
  * @brief  Reset module to the uninitialized state.
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval UartPktXcvr_OpRes_t.
  */
UartPktXcvr_OpRes_t UartPktXcvr_Reset(UartPktXcvr_Handle_t *pHandle)
{
  __LOCK(pHandle);
  
  /* Set module's state to reset */
  pHandle->state = UART_PKT_XCVR_STATE_UNINIT;
  
  __UNLOCK(pHandle);
  
  return UART_PKT_XCVR_OP_RES_SUCCESS;
}

/**
  * @brief  Callback which is triggered when a packet is received.
  *
  * @param  pHandle: Pointer to module's handle.
  * @param  pData: Pointer to received data.
  *
  * @retval None.
  */
__weak void UartPktXcvr_PacketReceivedCallback(UartPktXcvr_Handle_t *pHandle,
                                               uint8_t *pData)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pHandle);
  UNUSED(pData);
}

/**
  * @brief  Callback which is triggered when error occured.
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval None.
  */
__weak void UartPktXcvr_ErrorOccurredCallback(UartPktXcvr_Handle_t *pHandle)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(pHandle);
}

/**
  * @brief  HAL UART module rx callback implementation.
  *
  * @param  huart: UART module handle.
  *
  * @retval None.
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UartCBHandler(huart, RX_COMPLETED_CB);
}

/**
  * @brief  HAL UART module error callback implementation.
  *
  * @param  huart: UART module handle.
  *
  * @retval None.
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  UartCBHandler(huart, ERROR_CB);
}

/**
  * @brief  HAL UART module callback handler.
  *
  * @param  huart: Pointer to UART module handle.
  * @param  UartCBType: UART callback type.
  *
  * @retval None.
  */
static void UartCBHandler(UART_HandleTypeDef *huart, uint8_t UartCBType)
{
  /* If the handle is invalid, discard the process */
  if (huart == NULL)
  {
    return;
  }
  
  uint8_t module_found = FALSE;
  
  UartPktXcvr_Handle_t *pHandle;
  
  /* Parse HAL UART handle, if it's registered */
  for (uint8_t i = 0; i < NumOfModules; i++)
  {
    if (RegisteredModules[i]->init.huart == huart)
    {
      module_found = TRUE;
      
      pHandle = RegisteredModules[i];
      break;
    }
  }
  
  /* Handle is not registered, discard the process */
  if (module_found == FALSE)
  {
    return;
  }
  
  /* Check if the module is at error state */
  if (pHandle->state == UART_PKT_XCVR_STATE_ERROR)
  {
    return;
  }
  
  /* If the module is locked, that means an error has occured */
  if (IS_LOCKED(pHandle))
  {
    __ERROR_INT(pHandle, UART_PKT_XCVR_ERR_CODE_RACE_CONDITION);
  }
  
  /* Lock module */
  __LOCK(pHandle);
  
  /* Check for the state compatibility */
  if (pHandle->state != UART_PKT_XCVR_STATE_OPERATING)
  {
    __ERROR_INT(pHandle, UART_PKT_XCVR_ERR_CODE_INCOMPATIBLE_STATE);
  }

  /* If the handled callback is RX Completed callback */
  if (UartCBType == RX_COMPLETED_CB)
  {
    /* If the receiver state is awaiting start condition */
    if (pHandle->statics.receiverState == RECEIVER_STATE_AWAITING_START_CONDITION)
    {
      /* If the start condition has been sent */
      if (pHandle->statics.buffer[0] == START_CONDITION_VALUE)
      {
        /* Make UART peripheral ready to receive data */
        if (HAL_UART_Receive_DMA(pHandle->init.huart, pHandle->statics.buffer,
                                 UART_PKT_XCVR_BUFFER_SIZE) != HAL_OK)
        {
          __ERROR_INT(pHandle, UART_PKT_XCVR_ERR_CODE_HAL_UART_ERROR);
        }
        
        /* Update the receiver state */
        pHandle->statics.receiverState = RECEIVER_STATE_RECEIVING_PACKET;
        
      }
      else
      {
        /* Make UART peripheral ready to receive data */
        if (HAL_UART_Receive_DMA(pHandle->init.huart, pHandle->statics.buffer,
                                 sizeof(uint8_t)) != HAL_OK)
        {
          __ERROR_INT(pHandle, UART_PKT_XCVR_ERR_CODE_HAL_UART_ERROR);
        }
      }
    }
    else
    {
      uint16_t correction_bitmap = 0;
      
      correction_bitmap |= pHandle->statics.buffer[CORRECTION_BITMAP_INDEX];
      correction_bitmap |= (pHandle->statics.buffer[CORRECTION_BITMAP_INDEX + 1] << 8);
      
      /* Post process the received packet */
      for (uint8_t i = 0; i < UART_PKT_XCVR_PACKET_SIZE; i++)
      {
        uint16_t mask;
        mask = 0x01 << i;
        
        /* If the corresponding bit in the flags index is set, then indexed byte is
          equal to start condition value */
        if ((correction_bitmap & mask) != 0)
        {
          pHandle->statics.buffer[i] = START_CONDITION_VALUE;
        }
      }
      
      UartPktXcvr_PacketReceivedCallback(pHandle, pHandle->statics.buffer);
      
      /* Wait for the start condition */
      if (HAL_UART_Receive_DMA(pHandle->init.huart,
                               pHandle->statics.buffer,
                               sizeof(uint8_t)) != HAL_OK)
      {
        __ERROR_INT(pHandle, UART_PKT_XCVR_ERR_CODE_HAL_UART_ERROR);
      }
      
      /* Update the receiver state */
      pHandle->statics.receiverState = RECEIVER_STATE_AWAITING_START_CONDITION;
    }
  }
  
  else if (UartCBType == ERROR_CB)
  {
    __ERROR_INT(pHandle, UART_PKT_XCVR_ERR_CODE_HAL_UART_ERROR);
  }
  
  /* Unlock the module */
  __UNLOCK(pHandle);
  
  return;
}