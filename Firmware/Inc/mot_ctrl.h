/**
  ******************************************************************************
  * @file    mot_ctrl.h
  * @author  Onur Efe
  * @date    18.02.2016
  * @brief   Motor controller module header file.
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
#ifndef __MOT_CTRL_H
#define __MOT_CTRL_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "generic.h"
#include <math.h>
#include "pos_sens_drv.h"
#include "pid_ctrl.h"
#include "bldc_drv.h"
#include "eeprom_emulator.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  MOT_CTRL_LOCK_STATE_LOCKED                    = 0x00,                 /* Module is locked */
  MOT_CTRL_LOCK_STATE_UNLOCKED                  = 0x01                  /* Module is unlocked */
} MotCtrl_LockState_t;                                                  /* Lock State typedef */

typedef enum 
{
  MOT_CTRL_STATE_UNINIT                         = 0x00,                 /* Module is not initialized */
  MOT_CTRL_STATE_READY                          = 0x01,                 /* Module is ready to operate */
  MOT_CTRL_STATE_OPERATING                      = 0x02,                 /* Operating properly */
  MOT_CTRL_STATE_ERROR                          = 0x03                  /* Error */
} MotCtrl_State_t;                                                      /* State typedef */

typedef enum
{
  MOT_CTRL_ERR_CODE_NONE                        = 0x00,                 /* None */
  MOT_CTRL_ERR_CODE_INVALID_PARAMETER           = 0x01,                 /* Invalid parameter */
  MOT_CTRL_ERR_CODE_INCOMPATIBLE_STATE          = 0x02,                 /* Incompatible state error */
  MOT_CTRL_ERR_CODE_POS_SENS_DRV_ERROR          = 0x03,                 /* Position Sensor Driver error */
  MOT_CTRL_ERR_CODE_PID_CTRL_ERROR              = 0x04,                 /* PID Controller error */
  MOT_CTRL_ERR_CODE_BLDC_DRV_ERROR              = 0x05,                 /* BLDC Driver error */
  MOT_CTRL_ERR_CODE_EEPROM_EMULATOR_ERROR       = 0x06,                 /* Eeprom Emulator error */
  MOT_CTRL_ERR_CODE_RACE_CONDITION              = 0x07                  /* Race condition occured */
} MotCtrl_ErrCode_t;                                                    /* Error Code typedef */

typedef enum 
{
  MOT_CTRL_OP_RES_SUCCESS                       = 0x00,                 /* Success */
  MOT_CTRL_OP_RES_FAILURE                       = 0x01                  /* Failure */
} MotCtrl_OpRes_t;                                                      /* Operation Result typedef */

typedef struct 
{
  PosSensDrv_Init_t                             posSensDrvInit;         /* Position Sensor Driver init struct */
  PidCtrl_Init_t                                pidCtrlInit;            /* PID Controller init struct */
  BldcDrv_Init_t                                bldcDrvInit;            /* BLDC Driver init struct */
  uint8_t                                       eeEmulObjId;            /* Eeprom Emulator object id */
} MotCtrl_Init_t;                                                       /* Init typedef */

typedef struct
{
  uint8_t                                       eeEmulObjId;            /* Eeprom Emulator object id */
} MotCtrl_Init2_t;

typedef struct
{
  uint8_t                                       validityFlag;           /* Indicates if the data is valid(TRUE) */
  float                                         offset;                 /* Offset value */
  BldcDrv_Direction_t                           direction;              /* Direction */
} MotCtrl_NV_t;                                                         /* Non-Volatile typedef */

typedef struct
{
  PosSensDrv_Handle_t                           posSensDrvHandle;       /* Position Sensor Driver handle */
  PidCtrl_Handle_t                              pidCtrlHandle;          /* PID Controller handle */
  BldcDrv_Handle_t                              bldcDrvHandle;          /* BLDC Driver handle */
} MotCtrl_SubModules_t;                                                 /* Sub Modules typedef */         

typedef struct
{
  MotCtrl_LockState_t                           lockState;              /* Lock State */
  MotCtrl_State_t                               state;                  /* State */
  MotCtrl_ErrCode_t                             errCode;                /* Error Code */
  MotCtrl_Init2_t                                init;                  /* Init */
  MotCtrl_SubModules_t                          subModules;             /* Sub Modules */
} MotCtrl_Handle_t;                                                     /* Handle typedef */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Motor controller module initializer. 
  *
  * @param  pHandle: Pointer to module handle.
  * @param  pInitStruct: Pointer to init struct.
  *
  * @retval Operation result.
  */
MotCtrl_OpRes_t MotCtrl_Init(MotCtrl_Handle_t *pHandle, MotCtrl_Init_t *pInitStruct);

/**
  * @brief  Motor controller module initializer. 
  *
  * @param  pHandle: Pointer to module handle.
  *
  * @retval Operation result.
  */
MotCtrl_OpRes_t MotCtrl_Start(MotCtrl_Handle_t *pHandle);

/**
  * @brief  Module executer. Should be called periodicaly for proper
  *         operation.
  *
  * @param  pHandle: Pointer to motor controller module handle.
  * @param  destinationPosition: Destination position.
  *
  * @retval Operation result.
  */
MotCtrl_OpRes_t MotCtrl_Execute(MotCtrl_Handle_t *pHandle, float destinationPosition);

/**
  * @brief  Stops module's operation. 
  *
  * @param  pHandle: Pointer to module handle.
  *
  * @retval Operation result.
  */
MotCtrl_OpRes_t MotCtrl_Stop(MotCtrl_Handle_t *pHandle);
    
/**
  * @brief  Resets module. 
  *
  * @param  pHandle: Pointer to module handle.
  *
  * @retval Operation result.
  */
MotCtrl_OpRes_t MotCtrl_Reset(MotCtrl_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif

#endif