/**
  ******************************************************************************
  * @file    bldc_drv.h
  * @author  Onur Efe
  * @date    18.05.2016
  * @brief   Brushless DC driver class interface.
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
#ifndef __BLDC_DRV_H
#define __BLDC_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "generic.h"

/* Exported contants */
#define BLDC_DRV_INITIAL_DUTY_RATIO     0.0f            /* Initial duty ratio */
#define BLDC_DRV_INITIAL_POSITION       0.0f            /* Initial position */
   
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  BLDC_DRV_LOCK_STATE_LOCKED            = 0x00,         /* Module is locked */
  BLDC_DRV_LOCK_STATE_UNLOCKED          = 0x01          /* Module is unlocked */
} BldcDrv_LockState_t;                                  /* Lock State typedef */
   
typedef enum 
{
  BLDC_DRV_STATE_UNINIT                 = 0x00,         /* Module is not initialized */
  BLDC_DRV_STATE_READY                  = 0x01,         /* Module is ready for the operation */
  BLDC_DRV_STATE_OPERATING              = 0x02,         /* Module is operating */
  BLDC_DRV_STATE_ERROR                  = 0x03          /* Error occured */
} BldcDrv_State_t;                                      /* State typedef */

typedef enum
{
  BLDC_DRV_ERR_CODE_NONE                = 0x00,         /* None */
  BLDC_DRV_ERR_CODE_INVALID_PARAMETER   = 0x01,         /* Invalid parameter */
  BLDC_DRV_ERR_CODE_INCOMPATIBLE_STATE  = 0x02,         /* Incompatible state */
  BLDC_DRV_ERR_CODE_HAL_TIM_ERROR       = 0x03,         /* HAL TIM error */
  BLDC_DRV_ERR_CODE_DRV8313_ERROR       = 0x04,         /* DRV8313 fault */
  BLDC_DRV_ERR_CODE_RACE_CONDITION      = 0x05          /* Race condition occured */
} BldcDrv_ErrCode_t;                                    /* Error Code typedef */

typedef enum 
{
  BLDC_DRV_OP_RES_SUCCESS               = 0x00,         /* Operation successful */
  BLDC_DRV_OP_RES_FAILURE               = 0x01          /* Failure */
} BldcDrv_OpRes_t;                                      /* Operation result typedef */

typedef enum
{
  BLDC_DRV_ROT_DIR_POSITIVE             = 0x00,         /* Positive rotation in right handed coordinate system */
  BLDC_DRV_ROT_DIR_NEGATIVE             = 0x01,         /* Negative rotation in right handed coordinate system */
} BldcDrv_Direction_t;                                  /* Direction typedef */                  

typedef struct 
{
  uint32_t                              inRChannel;     /* The channel which controls the R phase of BLDC */
  uint32_t                              inSChannel;     /* The channel which controls the S phase of BLDC */
  uint32_t                              inTChannel;     /* The channel which controls the T phase of BLDC */
  GPIO_TypeDef                          *nResetPort;    /* Port of the nRESET pin */
  GPIO_TypeDef                          *nFaultPort;    /* Port of the nFAULT pin */
  GPIO_TypeDef                          *nSleepPort;    /* Port of the nSLEEP pin */
  uint16_t                              nResetPin;      /* Pin of the nRESET pin */
  uint16_t                              nFaultPin;      /* Pin of the nFAULT pin */
  uint16_t                              nSleepPin;      /* Pin of the nSLEEP pin */
} BldcDrv_PinMap_t;                                     /* DRV8313 IC pin map */

typedef struct
{
  uint8_t                               poleCount;      /* Motor pole count */
  BldcDrv_PinMap_t                      pinMap;         /* Pin map of the DRV8313 IC */
  TIM_HandleTypeDef                     *hTim;          /* Handle of the timer peripheral which is used for PWM generation */
} BldcDrv_Init_t;                                       /* Init typedef */

typedef struct
{
  float                                 offset;         /* Electrical position to real position offset */
  BldcDrv_Direction_t                   direction;      /* Direction */
} BldcDrv_Statics_t;                                    /* Statics typedef */

typedef struct
{
  BldcDrv_LockState_t                   lockState;      /* Lock state */
  BldcDrv_State_t                       state;          /* State */
  BldcDrv_ErrCode_t                     errCode;        /* Error Code */
  BldcDrv_Init_t                        init;           /* Init */
  BldcDrv_Statics_t                     statics;        /* Statics */
} BldcDrv_Handle_t;                                     /* Handle typedef */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes module.
  *
  * @param  pHandle: Pointer of the handle.
  * @param  pInitStruct: Pointer to initializer object.
  *
  * @retval Operation result.
  */
BldcDrv_OpRes_t BldcDrv_Init(BldcDrv_Handle_t *pHandle, BldcDrv_Init_t *pInitStruct);

/**
  * @brief  Starts module.
  *
  * @param  pHandle: Pointer of the handle.
  *
  * @retval Operation result.
  */
BldcDrv_OpRes_t BldcDrv_Start(BldcDrv_Handle_t *pHandle);

/**
  * @brief  Executes handle. Should be called periodically 
  *         for driving the brushless motor properly.
  *
  * @param  pHandle: Handle.
  * @param  position: Destinated rotor position of the motor.
  * @param  dutyRatio: Controls the duty cycle.
  * @param  shiftRatio: Controls the phase shift between electrical angle and 
  *         rotor position.
  *      
  * @retval Operation result.
  */
BldcDrv_OpRes_t BldcDrv_Execute(BldcDrv_Handle_t *pHandle, float position, 
                                float dutyRatio, float shiftRatio);

/**
  * @brief  Sets direction of the BLDC motor.
  *
  * @param  direction: Positive or negative rotation direction in right handed 
  *         coordinate system.
  * @param  offset: Angular difference in radians from electrical position and 
  *         real position.
  *     
  * @retval Operation result.
  */
BldcDrv_OpRes_t BldcDrv_SetParameters(BldcDrv_Handle_t *pHandle, 
                                      BldcDrv_Direction_t direction, float offset);
/**
  * @brief  Stops the module.
  *
  * @param  pHandle: Pointer of the handle.
  *
  * @retval Operation result.
  */
BldcDrv_OpRes_t BldcDrv_Stop(BldcDrv_Handle_t *pHandle);

/**
  * @brief  Resets the module. Switches to uninitialized state.
  *         
  * @param  pHandle: Pointer of the handle.
  *
  * @retval Operation result.
  */
BldcDrv_OpRes_t BldcDrv_Reset(BldcDrv_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif

#endif