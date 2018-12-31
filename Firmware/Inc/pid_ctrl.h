/**
  ******************************************************************************
  * @file    pid_ctrl.h
  * @author  Onur Efe
  * @date    17.05.2016
  * @brief   PID Controller class interface.
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
#ifndef __PID_CTRL_H
#define __PID_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "generic.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  PID_CTRL_LOCK_STATE_LOCKED            = 0x00,                 /* Module is locked */
  PID_CTRL_LOCK_STATE_UNLOCKED          = 0x01                  /* Module is unlocked */
} PidCtrl_LockState_t;                                          /* Lock State typedef */

typedef enum 
{
  PID_CTRL_STATE_UNINIT                 = 0x00,                 /* Module is not initialized */
  PID_CTRL_STATE_READY                  = 0x01,                 /* Module is ready to operate */
  PID_CTRL_STATE_OPERATING              = 0x02,                 /* Module is operating */
  PID_CTRL_STATE_ERROR                  = 0x03,                 /* Module is at error state */
} PidCtrl_State_t;                                              /* State typedef */

typedef enum
{
  PID_CTRL_ERR_CODE_NONE                = 0x00,                 /* None */
  PID_CTRL_ERR_CODE_INVALID_PARAMETER   = 0x01,                 /* Invalid parameter */
  PID_CTRL_ERR_CODE_INCOMPATIBLE_STATE  = 0x02,                 /* Incompatible state */
  PID_CTRL_ERR_CODE_RACE_CONDITION      = 0x03                  /* Race condition */
} PidCtrl_ErrCode_t;                                            /* Error Code typedef */

typedef enum 
{
  PID_CTRL_OP_RES_SUCCESS               = 0x00,                 /* Success */
  PID_CTRL_OP_RES_FAILURE               = 0x01                  /* Failure */
} PidCtrl_OpRes_t;                                              /* Operation result typedef */

typedef struct
{
  float                                 gain;                   /* PID Gain */
  float                                 integralTC;             /* Integral block time constant */
  float                                 derivativeTC;           /* Derivative block time constant */
  float                                 filterTC;               /* Time constant of the derivative filter */
  float                                 lowLimit;               /* Output limiter low limit */
  float                                 highLimit;              /* Output limiter high limit */
} PidCtrl_Init_t;                                               /* PID Controller Init typedef */

typedef struct
{
  float                                 filterMultiplier;       /* Filter multiplier is a value related with EMA filter */
  float                                 lastIntOutput;          /* Last integrated input signal */
  float                                 lastInput;              /* Last input signal */
  float                                 lastFiltOutput;         /* Last filter output */
} PidCtrl_Statics_t;                                            /* Statics typedef */

typedef struct 
{
  PidCtrl_LockState_t                   lockState;              /* Lock state */
  PidCtrl_State_t                       state;                  /* State */
  PidCtrl_ErrCode_t                     errCode;                /* Error Code */
  PidCtrl_Init_t                        init;                   /* Init */
  PidCtrl_Statics_t                     statics;                /* Statics */
} PidCtrl_Handle_t;                                             /* Handle typedef */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes module. 
  *
  * @param  pHandle: Pointer to module handle.
  * @param  pInitStruct: Struct which contains parameters used with initializing the module.
  *
  * @retval Operation result.
  */
PidCtrl_OpRes_t PidCtrl_Init(PidCtrl_Handle_t *pHandle, PidCtrl_Init_t *pInitStruct);

/**
  * @brief  Starts module. 
  *
  * @param  pHandle: Pointer to module handle.
  *
  * @retval Operation result.
  */
PidCtrl_OpRes_t PidCtrl_Start(PidCtrl_Handle_t *pHandle);

/**
  * @brief  Module executer. Should be called periodically for proper
  *         operation.
  *
  * @param  pHandle: Pointer to the module's handle.
  * @param  input: Input signal of the PID controller.
  * @param  pOutput: Pointer to store output signal.
  *
  * @retval Operation result.
  */
PidCtrl_OpRes_t PidCtrl_Execute(PidCtrl_Handle_t *pHandle, float input, 
                                float *pOutput);

/**
  * @brief  Stops operating module.
  * 
  * @param  pHandle: Pointer to the module handle.
  *
  * @retval Operation result.
  */
PidCtrl_OpRes_t PidCtrl_Stop(PidCtrl_Handle_t *pHandle);

/**
  * @brief  Resets module.
  *
  * @param  pHandle: Pointer to the module's handle.
  *
  * @retval Operation result.
  */
PidCtrl_OpRes_t PidCtrl_Reset(PidCtrl_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif

#endif