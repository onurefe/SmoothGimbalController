/**
  ******************************************************************************
  * @file    i2c_manager.h
  * @author  Onur Efe
  * @date    03.06.2016
  * @brief   I2C manager module interface.
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
#ifndef __I2C_MANAGER_H
#define __I2C_MANAGER_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "generic.h"

/* Defines -------------------------------------------------------------------*/
#define I2C_MANAGER_REQ_QUEUE_SIZE              12

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  I2C_MANAGER_LOCK_STATE_LOCKED                 = 0x00,         /* Module is locked */
  I2C_MANAGER_LOCK_STATE_UNLOCKED               = 0x01          /* IModule is unlocked */
} I2CManager_LockState_t;                                       /* Lock State typedef */

typedef enum
{
  I2C_MANAGER_STATE_UNINIT                      = 0x00,         /* Module is not initialized */
  I2C_MANAGER_STATE_READY                       = 0x01,         /* Module is initialized and ready to be started */
  I2C_MANAGER_STATE_OPERATING                   = 0x02,         /* Module is operating */
  I2C_MANAGER_STATE_ERROR                       = 0x03          /* Error has occured. Reset the module. */
} I2CManager_State_t;                                           /* State typedef */

typedef enum
{
  I2C_MANAGER_OP_RES_SUCCESS                    = 0x00,         /* Operation successful */
  I2C_MANAGER_OP_RES_FAILURE                    = 0x01          /* Error occured during operation */
} I2CManager_OpRes_t;                                           /* Operation result typedef */

typedef enum
{
  I2C_MANAGER_ERR_CODE_NONE                     = 0x00,         /* No error */
  I2C_MANAGER_ERR_CODE_INCOMPATIBLE_STATE       = 0x01,         /* State is incompatible with the operation */
  I2C_MANAGER_ERR_CODE_INVALID_PARAMETER        = 0x02,         /* Invalid parameter */
  I2C_MANAGER_ERR_CODE_REG_OBJ_LIM_EXCEEDED     = 0x03,         /* Maximum registered object limit exceeded */
  I2C_MANAGER_ERR_CODE_REQ_QUEUE_IS_FULL        = 0x04,         /* Request queue is full */
  I2C_MANAGER_ERR_CODE_HAL_I2C_ERROR            = 0x05,         /* HAL I2C module related error */
  I2C_MANAGER_ERR_CODE_RACE_CONDITION           = 0x06          /* Race condition occured */
} I2CManager_ErrCode_t;                                         /* Error code typedef */

typedef enum
{
  I2C_MANAGER_REQ_TYPE_READ                     = 0x00,         /* Read request */
  I2C_MANAGER_REQ_TYPE_WRITE                    = 0x01          /* Write request */
} I2CManager_ReqType_t;                                         /* Request type typedef */

typedef enum
{
  I2C_MANAGER_REQ_STATUS_PENDING                = 0x00,         /* Request status pending */
  I2C_MANAGER_REQ_STATUS_PROCESSING             = 0x01,         /* Processing */
  I2C_MANAGER_REQ_STATUS_COMPLETED              = 0x02,         /* Completed */
  I2C_MANAGER_REQ_STATUS_CANCELED               = 0x03,         /* Request is cancelled by the requesting module */
  I2C_MANAGER_REQ_STATUS_ERROR                  = 0x04          /* Error occured during processing request */
} I2CManager_ReqStatus_t;                                       /* Request status typedef */

typedef struct
{
  I2CManager_ReqType_t          type;                   /* Request type */
  uint8_t                       devAddr;                /* Device address */
  uint8_t                       memAddr;                /* Memory address */
  uint8_t                       *pData;                 /* Pointer of the data */
  uint8_t                       size;                   /* Size of the data */
  I2CManager_ReqStatus_t        *pReqStatusFlag;        /* Request status flag */
} I2CManager_Request_t;                                 /* Request typedef */ 

typedef struct
{
  I2CManager_Request_t          elements[I2C_MANAGER_REQ_QUEUE_SIZE];   /* Request buffer */
  uint8_t                       front;                  /* Front of the queue */
  uint8_t                       rear;                   /* Rear of the queue */
  uint8_t                       size;                   /* Size of the queue */
} I2CManager_ReqQueue_t;                                /* Request queue typedef */

typedef struct 
{
  I2CManager_ReqQueue_t         reqQueue;               /* Request queue */
  uint8_t                       unhandledIntFlag;       /* Unhandled interrupt flag */
  uint8_t                       callbackType;           /* Callback type */
} I2CManager_Statics_t;                                 /* Statics typedef */

typedef struct
{
  I2C_HandleTypeDef             *hi2c;                  /* I2C handle */
} I2CManager_Init_t;                                    /* Init typedef */

typedef struct
{
  I2CManager_LockState_t        lockState;              /* Lock State */
  I2CManager_State_t            state;                  /* State */
  I2CManager_ErrCode_t          errCode;                /* Error Code */
  I2CManager_Init_t             init;                   /* Init parameters */
  I2CManager_Statics_t          statics;                /* Static variables */
} I2CManager_Handle_t;                                  /* Handle typedef */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes module. Module's state is switched from uninitialized state
  *         to ready.
  *
  * @param  pHandle: Pointer to module's handle.
  * @param  pInitStruct: Pointer to module's init struct.
  *
  * @retval None.
  */
I2CManager_OpRes_t I2CManager_Init(I2CManager_Handle_t *pHandle, 
                                   I2CManager_Init_t *pInitStruct);

/**
  * @brief  Switches module to operating state(when the module is in ready state).
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval None.
  */
I2CManager_OpRes_t I2CManager_Start(I2CManager_Handle_t *pHandle);

/**
  * @brief  Stops operating module. 
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval None.
  */
I2CManager_OpRes_t I2CManager_Stop(I2CManager_Handle_t *pHandle);

/**
  * @brief  Resets module to uninitialized state. 
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval None.
  */
I2CManager_OpRes_t I2CManager_Reset(I2CManager_Handle_t *pHandle);

/**
  * @brief  Requests I2C read/write operation from the manager module.
  *
  * @param  pHandle: Pointer to module's handle.
  * @param  pRequest: Pointer to request parameters.
  *
  * @retval None.
  */
I2CManager_OpRes_t I2CManager_Request(I2CManager_Handle_t *pHandle, 
                                      I2CManager_Request_t *pRequest);
                      
/**
  * @brief  Cancels requests.
  *
  * @param  pHandle: Pointer to module's handle.
  * @param  pRequest: Pointer to request parameters.
  *
  * @retval None.
  */
I2CManager_OpRes_t I2CManager_CancelRequest(I2CManager_Handle_t *pHandle, 
                                            I2CManager_Request_t *pRequest);

/**
  * @brief  Error callback.
  *
  * @param  pHandle: Pointer to module's handle.
  *
  * @retval None.
  */
void I2CManager_ErrorCallback(I2CManager_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif

#endif