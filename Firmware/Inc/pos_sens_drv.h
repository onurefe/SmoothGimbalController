/**
  ******************************************************************************
  * @file    pos_sens_drv.h
  * @author  Onur Efe
  * @date    18.05.2016
  * @brief   AMS-AS5048B Position sensor driver class interface.
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
#ifndef __POS_SENS_DRV_H
#define __POS_SENS_DRV_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "i2c_manager.h"
#include "generic.h"

/* Exported types ------------------------------------------------------------*/
typedef enum 
{
  POS_SENS_DRV_STATE_UNINIT                     = 0x00, /* Module is not uninitialized */
  POS_SENS_DRV_STATE_READY                      = 0x01, /* Module is ready for operation */
  POS_SENS_DRV_STATE_OPERATING                  = 0x02, /* Module is operating */
  POS_SENS_DRV_STATE_ERROR                      = 0x03  /* Error has occured */
} PosSensDrv_State_t;                                   /* State typedef */

typedef enum
{
  POS_SENS_DRV_LOCK_STATE_LOCKED                = 0x00, /* Module is locked */
  POS_SENS_DRV_LOCK_STATE_UNLOCKED              = 0x01  /* Module is unlocked */
} PosSensDrv_LockState_t;                               /* Lock State typedef */

typedef enum
{
  POS_SENS_DRV_OP_RES_SUCCESS                   = 0x00, /* Success */
  POS_SENS_DRV_OP_RES_FAILURE                   = 0x01  /* Failure */
} PosSensDrv_OpRes_t;                                   /* Operation Result typedef */

typedef enum 
{
  POS_SENS_DRV_ERR_CODE_NONE                    = 0x00, /* Operation successful */
  POS_SENS_DRV_ERR_CODE_INVALID_POINTER         = 0x01, /* Invalid pointer */
  POS_SENS_DRV_ERR_CODE_INVALID_PARAMETER       = 0x02, /* Invalid parameter value */
  POS_SENS_DRV_ERR_CODE_INCOMPATIBLE_STATE      = 0x03, /* Position sensor driver object state is incompatible for the operation */
  POS_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR       = 0x04, /* I2C Manager error */
  POS_SENS_DRV_ERR_CODE_RACE_CONDITION          = 0x06  /* Race condition */
} PosSensDrv_ErrCode_t;                                 /* Error Code typedef */

typedef struct
{
  uint8_t                       devAddr;                /* Device address */
  I2CManager_Handle_t           *pI2CManagerHandle;     /* Pointer to I2C manager handle */
  float                         zeroPosition;           /* Zero position of the sensor */
  float                         filterTC;               /* Filter time constant */
} PosSensDrv_Init_t;                                    /* Init typedef */

typedef struct
{                 
  uint8_t                       angleReg[2];            /* Angle register */
  I2CManager_ReqStatus_t        angleRegReadReqStatusFlag;      /* Angle register read request flag */
  I2CManager_Request_t          angleRegReadReq;        /* Angle register read request */
  float                         filterCoefficient;      /* Filter coefficient */
  float                         expAvr;                 /* Exponential average value */
} PosSensDrv_Statics_t;                                 /* Statics typedef */

typedef struct
{
  PosSensDrv_LockState_t        lockState;              /* Lock State */
  PosSensDrv_State_t            state;                  /* State */
  PosSensDrv_ErrCode_t          errCode;                /* Error Code */
  PosSensDrv_Statics_t          statics;                /* Statics */
  PosSensDrv_Init_t             init;                   /* Init struct */
} PosSensDrv_Handle_t;                                  /* Handle typedef */


/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Position sensor driver module initializer.
  *
  * @param  pHandle: Pointer to position sensor driver module handle.
  * @param  pInitStruct: Position sensor driver init struct.
  *
  * @retval Operation result.
  */
PosSensDrv_OpRes_t PosSensDrv_Init(PosSensDrv_Handle_t *pHandle, PosSensDrv_Init_t *pInitStruct);

/**
  * @brief  Starts module's operation.
  *
  * @param  pHandle: Pointer to position sensor driver module handle.
  *
  * @retval Operation result.
  */
PosSensDrv_OpRes_t PosSensDrv_Start(PosSensDrv_Handle_t *pHandle);

/**
  * @brief  Position sensor driver module executer. This function should be called
  *         periodically to perform correctly.
  *
  * @param  pHandle: Pointer to position sensor driver module handle.
  * @param  pPosition: Pointer to store the read position data.
  *
  * @retval Operation result.
  */
PosSensDrv_OpRes_t PosSensDrv_Execute(PosSensDrv_Handle_t *pHandle, float *pPosition);

/**
  * @brief  Stops module's operation.
  *
  * @param  pHandle: Pointer to position sensor driver module handle.
  *
  * @retval Operation result.
  */
PosSensDrv_OpRes_t PosSensDrv_Stop(PosSensDrv_Handle_t *pHandle);

/**
  * @brief  Resets module to uninitialized state.
  *
  * @param  pHandle: Pointer to position sensor driver module handle.
  *
  * @retval Operation result.
  */
PosSensDrv_OpRes_t PosSensDrv_Reset(PosSensDrv_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif

#endif