/**
  ******************************************************************************
  * @file    orient_sens_drv.h
  * @author  Onur Efe
  * @date    24.03.2016
  * @brief   BNO055 orientation sensor driver class interface.
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
#ifndef __ORIENT_SENS_DRV_H
#define __ORIENT_SENS_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "generic.h"
#include "quaternion.h"
#include "i2c_manager.h"
#include "eeprom_emulator.h"

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  ORIENT_SENS_DRV_STATE_UNINIT                          = 0x00, /* Module is not initialized */
  ORIENT_SENS_DRV_STATE_READY                           = 0x01, /* Module is ready */
  ORIENT_SENS_DRV_STATE_OPERATING                       = 0x02, /* Sensor driver is at operating state */
  ORIENT_SENS_DRV_STATE_ERROR                           = 0x03  /* Error has occured */
} OrientSensDrv_State_t;                                        /* States typedef */

typedef enum
{
  ORIENT_SENS_DRV_LOCK_STATE_LOCKED                     = 0x00, /* Locked */
  ORIENT_SENS_DRV_LOCK_STATE_UNLOCKED                   = 0x01  /* Unlocked */
} OrientSensDrv_LockState_t;                                    /* Lock State typedef */

typedef enum 
{
  ORIENT_SENS_DRV_ERR_CODE_NONE                         = 0x00, /* None */
  ORIENT_SENS_DRV_ERR_CODE_INCOMPATIBLE_STATE           = 0x01, /* Incompatible State */
  ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR            = 0x02, /* I2C Manager Error */
  ORIENT_SENS_DRV_ERR_CODE_BNO055_ERROR                 = 0x03, /* BNO055 Error */
  ORIENT_SENS_DRV_ERR_CODE_EEPROM_EMULATOR_ERROR        = 0x04, /* Eeprom Emulator Error */
  ORIENT_SENS_DRV_ERR_CODE_RACE_CONDITION               = 0x05  /* Race condition occured */
} OrientSensDrv_ErrCode_t;                                      /* Error Code typedef */

typedef enum
{
  ORIENT_SENS_DRV_OP_RES_SUCCESS                        = 0x00, /* Success */
  ORIENT_SENS_DRV_OP_RES_FAILURE                        = 0x01  /* Failure */
} OrientSensDrv_OpRes_t;                                        /* Operation Results typedef */

typedef struct 
{
  uint8_t                               eeEmulObjId;            /* Eeprom Emulator object id */
  uint8_t                               devAddr;                /* Device address */
  I2CManager_Handle_t                   *pI2CManagerHandle;     /* Pointer to I2C Manager handle */
} OrientSensDrv_Init_t;                                         /* Init typedef */

typedef struct
{
  uint8_t                               quaData[8];             /* Quaternion data */
  I2CManager_ReqStatus_t                quaDataReadReqStatusFlag;       /* Quaternion data read request flag */
  I2CManager_Request_t                  quaDataReadReq;         /* Quaternion data read request */
} OrientSensDrv_Statics_t;                                      /* Statics typedef */

typedef struct
{
  OrientSensDrv_LockState_t             lockState;              /* Lock State */
  OrientSensDrv_State_t                 state;                  /* State */
  OrientSensDrv_ErrCode_t               errCode;                /* Error Code */
  OrientSensDrv_Init_t                  init;                   /* Init */
  OrientSensDrv_Statics_t               statics;                /* Statics */
} OrientSensDrv_Handle_t;                                       /* Handle struct typedef */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Orientation sensor driver module initializer.
  *
  * @param  pHandle: Pointer to orientation sensor driver module handle.
  * @param  pInitStruct: Pointer of the init struct.
  *
  * @retval OrientSensDrv_OpRes_t.
  */
OrientSensDrv_OpRes_t OrientSensDrv_Init(OrientSensDrv_Handle_t *pHandle,
                                         OrientSensDrv_Init_t *pInitStruct);

/**
  * @brief  Starts operation.
  *
  * @param  pHandle: Pointer to orientation sensor driver module handle.
  *
  * @retval OrientSensDrv_OpRes_t.
  */
OrientSensDrv_OpRes_t OrientSensDrv_Start(OrientSensDrv_Handle_t *pHandle);

/**
  * @brief  Orientation sensor driver module executer. 
  *
  * @param  pHandle: Pointer to orientation sensor driver module handle.
  * @param  pOrientation: Pointer to return orientation data. 
  *
  * @retval OrientSensDrv_OpRes_t.
  */
OrientSensDrv_OpRes_t OrientSensDrv_Execute(OrientSensDrv_Handle_t *pHandle,
                                            Quaternion_Quaternion_t *pOrientation);

/**
  * @brief  Stops operation.
  *
  * @param  pHandle: Pointer to orientation sensor driver module handle.
  *
  * @retval OrientSensDrv_OpRes_t.
  */
OrientSensDrv_OpRes_t OrientSensDrv_Stop(OrientSensDrv_Handle_t *pHandle);

/**
  * @brief  Resets orientation sensor driver module.
  *
  * @param  pHandle: Pointer to orientation sensor driver module handle.
  *
  * @retval OrientSensDrv_OpRes_t.
  */
OrientSensDrv_OpRes_t OrientSensDrv_Reset(OrientSensDrv_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif

#endif