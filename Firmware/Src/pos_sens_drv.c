/**
  ******************************************************************************
  * @file    pos_sens_drv.c
  * @author  Onur Efe
  * @date    18.05.2016
  * @brief   AMS-AS5048B Position sensor driver class implementation.
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
#include "pos_sens_drv.h"
#include <math.h>

/* Private constants ---------------------------------------------------------*/
#define CONTROL_PERIOD                          (1.0f / CONTROL_FREQ)   /* Control period in units of seconds */
#define AS5048B_RESOLUTION                      16384U                  /* 14 bit resolution */
#define AS5048B_DIAGNOSTICS_REG_ADDR            0xFB                    /* Diagnostic flags(8 bit) */
#define AS5048B_ANGLE_REG_ADDR                  0xFE                    /* Angle value(14 bit) register address */

#define AS5048B_DIAGNOSTICS_OCF_BIT             0x01                    /* Logic high indicates the device finished Offset Compensation Algorithm */
#define AS5048B_DIAGNOSTICS_COF_BIT             0x02                    /* Logic high indicates an out of range error in the CORDIC part */
#define AS5048B_DIAGNOSTICS_COMP_L_BIT          0x04                    /* Logic high indicates low magnetic field */
#define AS5048B_DIAGNOSTICS_COMP_H_BIT          0x08                    /* Logic high indicates high magnetic field */
#define AS5048B_MEM_ADDRESS_SIZE                1U                      /* AS5048B memory address size */

#define STARTUP_TIME                            100                     /* Start-up time in units of miliseconds */

/* Private macros ------------------------------------------------------------*/
#define __ERROR(_HANDLE_, _ERR_CODE_)   \
                                          do {                                                                          \
                                                (_HANDLE_)->state = (POS_SENS_DRV_STATE_ERROR);                         \
                                                (_HANDLE_)->errCode = (_ERR_CODE_);                                     \
                                                (_HANDLE_)->lockState = (POS_SENS_DRV_LOCK_STATE_UNLOCKED);    \
                                                return (POS_SENS_DRV_OP_RES_FAILURE);                                   \
                                          } while(0)

#define __LOCK(_HANDLE_)                        ((_HANDLE_)->lockState = (POS_SENS_DRV_LOCK_STATE_LOCKED)) 
                                          
#define __UNLOCK(_HANDLE_)                      ((_HANDLE_)->lockState = (POS_SENS_DRV_LOCK_STATE_UNLOCKED))
                                          
#define IS_LOCKED(_HANDLE_)                     ((_HANDLE_)->lockState == (POS_SENS_DRV_LOCK_STATE_LOCKED))

#define HI_UINT16(_VAL_)                        (((uint8_t *)&(_VAL_))[1])

#define LO_UINT16(_VAL_)                        (((uint8_t *)&(_VAL_))[0])

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Position sensor driver module initializer.
  *
  * @param  pHandle: Pointer to position sensor driver module handle.
  * @param  pInitStruct: Position sensor driver init struct.
  *
  * @retval Operation result.
  */
PosSensDrv_OpRes_t PosSensDrv_Init(PosSensDrv_Handle_t *pHandle, PosSensDrv_Init_t *pInitStruct)
{
  /* Check if the module is at error state */
  if (pHandle->state == POS_SENS_DRV_STATE_ERROR)
  {
    return POS_SENS_DRV_OP_RES_FAILURE;
  }
  
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != POS_SENS_DRV_STATE_UNINIT)
  {
    __ERROR(pHandle, POS_SENS_DRV_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Set init parameters */
  pHandle->init.devAddr = pInitStruct->devAddr;
  pHandle->init.pI2CManagerHandle = pInitStruct->pI2CManagerHandle;
  pHandle->init.filterTC = pInitStruct->filterTC;
  pHandle->init.zeroPosition = pInitStruct->zeroPosition;
  
  /* Set error code to none */
  pHandle->errCode = POS_SENS_DRV_ERR_CODE_NONE;
  
  /* Set module's state to ready */
  pHandle->state = POS_SENS_DRV_STATE_READY;
  
  __UNLOCK(pHandle);
  
  return POS_SENS_DRV_OP_RES_SUCCESS;
}

/**
  * @brief  Starts module's operation.
  *
  * @param  pHandle: Pointer to position sensor driver module handle.
  *
  * @retval Operation result.
  */
PosSensDrv_OpRes_t PosSensDrv_Start(PosSensDrv_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == POS_SENS_DRV_STATE_ERROR)
  {
    return POS_SENS_DRV_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, POS_SENS_DRV_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != POS_SENS_DRV_STATE_READY)
  {
    __ERROR(pHandle, POS_SENS_DRV_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Set static members */
  pHandle->statics.angleRegReadReq.type = I2C_MANAGER_REQ_TYPE_READ;
  pHandle->statics.angleRegReadReq.devAddr = pHandle->init.devAddr;
  pHandle->statics.angleRegReadReq.memAddr = AS5048B_ANGLE_REG_ADDR;
  pHandle->statics.angleRegReadReq.pData = pHandle->statics.angleReg;
  pHandle->statics.angleRegReadReq.pReqStatusFlag = &(pHandle->statics.angleRegReadReqStatusFlag);
  pHandle->statics.angleRegReadReq.size = sizeof(uint16_t);
  pHandle->statics.filterCoefficient = CONTROL_PERIOD / (pHandle->init.filterTC + CONTROL_PERIOD); 
  pHandle->statics.expAvr = 0.0f;
  
  /* Delay for the startup time */
  WaitUntil(STARTUP_TIME);

  /* Request first angle register read */
  if (I2CManager_Request(pHandle->init.pI2CManagerHandle,
                         &(pHandle->statics.angleRegReadReq)) != I2C_MANAGER_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, POS_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
  }
  
  /* Set module's state to operating */
  pHandle->state = POS_SENS_DRV_STATE_OPERATING;
  
  __UNLOCK(pHandle);
  
  return POS_SENS_DRV_OP_RES_SUCCESS;
}

/**
  * @brief  Position sensor driver module executer. This function should be called
  *         periodically to perform correctly.
  *
  * @param  pHandle: Pointer to position sensor driver module handle.
  * @param  pPosition: Pointer to store the read position data.
  *
  * @retval Operation result.
  */
PosSensDrv_OpRes_t PosSensDrv_Execute(PosSensDrv_Handle_t *pHandle, float *pPosition)
{
  /* Check if the module is at error state */
  if (pHandle->state == POS_SENS_DRV_STATE_ERROR)
  {
    return POS_SENS_DRV_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, POS_SENS_DRV_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != POS_SENS_DRV_STATE_OPERATING)
  {
    __ERROR(pHandle, POS_SENS_DRV_ERR_CODE_INCOMPATIBLE_STATE);
  }

  /* If the angle register read request isn't completed, it means that there is 
    an error */
  if (pHandle->statics.angleRegReadReqStatusFlag != I2C_MANAGER_REQ_STATUS_COMPLETED)
  {
    __ERROR(pHandle, POS_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
  }
  
  uint16_t angle;
  float new_position;
  
  /* Calculate angle value from the raw data */
  angle = ((uint16_t)pHandle->statics.angleReg[0] << 6) + \
          ((uint16_t)pHandle->statics.angleReg[1]);
          
  /* Calculate new position in units of radians */
  new_position = (2.0f * PI * ((float)angle / AS5048B_RESOLUTION)) - \
                 pHandle->init.zeroPosition;
  
  /* Apply linearization depending on the most probable way methode*/
  while ((new_position - pHandle->statics.expAvr) > PI)
  {
    new_position -= (2.0f * PI);
  }
    
  while ((new_position - pHandle->statics.expAvr) < -PI)
  {
    new_position += (2.0f * PI);
  }
  
  /* Update exponential average */
  pHandle->statics.expAvr = pHandle->statics.filterCoefficient * new_position + \
                            (1.0f - pHandle->statics.filterCoefficient) * pHandle->statics.expAvr;

  /* Set position */
  *pPosition = pHandle->statics.expAvr;

  /* Request angle register read */
  if (I2CManager_Request(pHandle->init.pI2CManagerHandle,
                         &(pHandle->statics.angleRegReadReq)) != I2C_MANAGER_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, POS_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
  }
  
  __UNLOCK(pHandle);
  
  return POS_SENS_DRV_OP_RES_SUCCESS;
}

/**
  * @brief  Stops module's operation.
  *
  * @param  pHandle: Pointer to position sensor driver module handle.
  *
  * @retval Operation result.
  */
PosSensDrv_OpRes_t PosSensDrv_Stop(PosSensDrv_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == POS_SENS_DRV_STATE_ERROR)
  {
    return POS_SENS_DRV_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, POS_SENS_DRV_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != POS_SENS_DRV_STATE_OPERATING)
  {
    __ERROR(pHandle, POS_SENS_DRV_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  if (I2CManager_CancelRequest(pHandle->init.pI2CManagerHandle, 
                               &(pHandle->statics.angleRegReadReq)) != I2C_MANAGER_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, POS_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
  }
  
  /* Set module's state to ready */
  pHandle->state = POS_SENS_DRV_STATE_READY;
  
  __UNLOCK(pHandle);
  
  return POS_SENS_DRV_OP_RES_SUCCESS;
}

/**
  * @brief  Resets module to uninitialized state.
  *
  * @param  pHandle: Pointer to position sensor driver module handle.
  *
  * @retval Operation result.
  */
PosSensDrv_OpRes_t PosSensDrv_Reset(PosSensDrv_Handle_t *pHandle)
{
  __LOCK(pHandle);
  
  /* Set module's state to uninitialized */
  pHandle->state = POS_SENS_DRV_STATE_UNINIT;
  
  __UNLOCK(pHandle);
  
  return POS_SENS_DRV_OP_RES_SUCCESS;
}