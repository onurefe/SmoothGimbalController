/**
  ******************************************************************************
  * @file    orient_sens_drv.c
  * @author  Onur Efe
  * @date    24.03.2016
  * @brief   BNO055 orientation sensor driver class implementation.
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
#include <math.h>
#include "orient_sens_drv.h"

/* Private constants ---------------------------------------------------------*/
/* Page ID register */
#define BNO055_PAGE_ID_REG_ADDR                                 0x07
#define BNO055_PAGE_ID_REG_LEN                                  0x01
#define BNO055_PAGE_ID_REG_PAGE0_VALUE                          0x00
#define BNO055_PAGE_ID_REG_PAGE1_VALUE                          0x01

/* System Status register */
#define BNO055_SYS_STATUS_REG_ADDR                              0x39
#define BNO055_SYS_STATUS_REG_LEN                               1U
#define BNO055_SYS_STATUS_REG_SYS_ERROR_VALUE                   0x01

/* Power Mode register */
#define BNO055_PWR_MODE_REG_ADDR                                0x3E
#define BNO055_PWR_MODE_REG_LEN                                 1U
#define BNO055_PWR_MODE_REG_NORMAL_VALUE                        0x00

/* Operation Mode register */
#define BNO055_OPR_MODE_REG_ADDR                                0x3D
#define BNO055_OPR_MODE_REG_LEN                                 1U
#define BNO055_OPR_MODE_REG_NDOF_VALUE                          0x0C
#define BNO055_OPR_MODE_REG_CONFIG_VALUE                        0x00

/* Calibration Status register */
#define BNO055_CALIB_STAT_REG_ADDR                              0x35
#define BNO055_CALIB_STAT_REG_LEN                               1U
#define BNO055_CALIB_STAT_SYS_CALIB_STATUS_MSK                  0xC0
#define BNO055_CALIB_STAT_SYS_CALIB_STATUS_CALIBRATED_VALUE     0xC0

/* Calibration Profile */
#define BNO055_CALIBRATION_PROFILE_ADDR                         0x55
#define BNO055_CALIBRATION_PROFILE_LEN                          22U

/* Quaternion Data memory region */
#define BNO055_QUA_DATA_ADDR                                    0x20
#define BNO055_QUA_DATA_LEN                                     8U

/* Start-up delay in units of miliseconds */
#define STARTUP_DELAY                                           1000UL

/* Private macros ------------------------------------------------------------*/
#define __ERROR(_HANDLE_, _ERR_CODE_)   \
                                          do {                                                                          \
                                                (_HANDLE_)->state = (ORIENT_SENS_DRV_STATE_ERROR);                      \
                                                (_HANDLE_)->errCode = (_ERR_CODE_);                                     \
                                                (_HANDLE_)->lockState = (ORIENT_SENS_DRV_LOCK_STATE_UNLOCKED);          \
                                                return (ORIENT_SENS_DRV_OP_RES_FAILURE);                                \
                                          } while (0)

#define __LOCK(_HANDLE_)                        ((_HANDLE_)->lockState = (ORIENT_SENS_DRV_LOCK_STATE_LOCKED)) 
                                          
#define __UNLOCK(_HANDLE_)                      ((_HANDLE_)->lockState = (ORIENT_SENS_DRV_LOCK_STATE_UNLOCKED))
                                          
#define IS_LOCKED(_HANDLE_)                     ((_HANDLE_)->lockState == (ORIENT_SENS_DRV_LOCK_STATE_LOCKED))
                                            
/* Private functions ---------------------------------------------------------*/
static ErrorStatus readBuffer(OrientSensDrv_Handle_t *pHandle, uint8_t memAddr, 
                              uint8_t *pData, uint8_t length);

static ErrorStatus writeBuffer(OrientSensDrv_Handle_t *pHandle, uint8_t memAddr, 
                               uint8_t *pData, uint8_t length);

/* Private variables ---------------------------------------------------------*/
uint8_t temp[22];                       // Temporary variable for sensor control and initialization.

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
                                         OrientSensDrv_Init_t *pInitStruct)
{
  /* Check if the module is at error state */
  if (pHandle->state == ORIENT_SENS_DRV_STATE_ERROR)
  {
    return ORIENT_SENS_DRV_OP_RES_FAILURE;
  }
  
  __LOCK(pHandle);
  
  /* Check if the i2c module has been initialized */
  if (pInitStruct->pI2CManagerHandle->state != I2C_MANAGER_STATE_OPERATING)
  {
    __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
  }
  
  /* Set parameters */
  pHandle->init.eeEmulObjId = pInitStruct->eeEmulObjId;
  pHandle->init.devAddr = pInitStruct->devAddr;
  pHandle->init.pI2CManagerHandle = pInitStruct->pI2CManagerHandle;
  
  /* Wait until sensor startup is completed */
  WaitUntil(STARTUP_DELAY);
  
  /* Set sensor active page to Page 0, all operations are executed on Page 0 so 
     there won't be any switching of the active page from now on */
  {
    temp[0] = BNO055_PAGE_ID_REG_PAGE0_VALUE;
    
    /* Set page id register to PAGE0 value */
    if (writeBuffer(pHandle, BNO055_PAGE_ID_REG_ADDR, temp, 
                    BNO055_PAGE_ID_REG_LEN) != SUCCESS)
    {
      __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
    }
  }
  
  /* Read System Status register and check if any error occured during initialization */
  {
    /* Read System Status register */
    if (readBuffer(pHandle, BNO055_SYS_STATUS_REG_ADDR, temp, 
                   BNO055_SYS_STATUS_REG_LEN) != SUCCESS)
    {
      __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
    }
    
    /* Check for error value */
    if (temp[0] == BNO055_SYS_STATUS_REG_SYS_ERROR_VALUE)
    {
      __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
    }
  }
  
  /* Set sensor power mode to normal */
  {
    temp[0] = BNO055_PWR_MODE_REG_NORMAL_VALUE;
    
    /* Write to power mode register */
    if (writeBuffer(pHandle, BNO055_PWR_MODE_REG_ADDR, temp, 
                    BNO055_PWR_MODE_REG_LEN) != SUCCESS)
    {
      __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
    }
    
    /* Delay for 10ms to ensure power mode switching completed */
    DelayMs(10);
  }
  
  /* Ensure that the sensor is in config mode */
  {
    temp[0] = BNO055_OPR_MODE_REG_CONFIG_VALUE;
    
    /* Write to operation mode register */
    if (writeBuffer(pHandle, BNO055_OPR_MODE_REG_ADDR, temp,
                    BNO055_OPR_MODE_REG_LEN) != SUCCESS)
    {
      __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
    }
    
    /* Delay for 20ms to ensure operation mode switching completed */
    DelayMs(20);
  }
  
  /* Set module's error code to none */
  pHandle->errCode = ORIENT_SENS_DRV_ERR_CODE_NONE;
  
  /* Set module's state to ready */
  pHandle->state = ORIENT_SENS_DRV_STATE_READY;
  
  __UNLOCK(pHandle);
  
  return ORIENT_SENS_DRV_OP_RES_SUCCESS;
}

/**
  * @brief  Starts operation.
  *
  * @param  pHandle: Pointer to orientation sensor driver module handle.
  *
  * @retval OrientSensDrv_OpRes_t.
  */
OrientSensDrv_OpRes_t OrientSensDrv_Start(OrientSensDrv_Handle_t *pHandle)
{
  uint8_t calibrated_flag;
  uint8_t calibration_data_length;
  
  /* Check if the module is at error state */
  if (pHandle->state == ORIENT_SENS_DRV_STATE_ERROR)
  {
    return ORIENT_SENS_DRV_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check if the module's state is compatible with the operation */
  if (pHandle->state != ORIENT_SENS_DRV_STATE_READY)
  {
    __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Read calibration profile if it exists in the non-volatile memory and save it to
    the device */
  {
    uint8_t ret_val;
    
    ret_val = EepromEmulator_ReadObject(pHandle->init.eeEmulObjId, 
                                        BNO055_CALIBRATION_PROFILE_LEN,
                                        &calibration_data_length, temp);
    
    /* Take action according to return value */
    if (ret_val == EEPROM_EMULATOR_OP_RES_SUCCESS)
    {
      calibrated_flag = TRUE;
      
      /* Write calibration data to the buffer */
      if (writeBuffer(pHandle, BNO055_CALIBRATION_PROFILE_ADDR, temp,
                      BNO055_CALIBRATION_PROFILE_LEN) != SUCCESS)
      {
        __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
      }
    }
    else if (ret_val == EEPROM_EMULATOR_OP_RES_OBJ_NOT_FOUND)
    {
      calibrated_flag = FALSE;
    }
    else
    {
      __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_EEPROM_EMULATOR_ERROR);
    }
  }

  /* Switch operation mode to 9-axis sensor fusion mode */
  {
    temp[0] = BNO055_OPR_MODE_REG_NDOF_VALUE;
    
    /* Set operation mode to NDOF mode */
    if (writeBuffer(pHandle, BNO055_OPR_MODE_REG_ADDR, temp,
                    BNO055_OPR_MODE_REG_LEN) != SUCCESS)
    {
      __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
    }
    
    /* Delay for 1s to ensure operation mode switching and first conversion is completed */
    DelayMs(1000);
  }
  
  if (calibrated_flag == FALSE)
  {
    /* Wasn't calibrated. So wait until calib_stat value is set, 
    and store the calibration data after that */
    do
    {
      /* Read calibration status register */
      if (readBuffer(pHandle, BNO055_CALIB_STAT_REG_ADDR,
                     temp, BNO055_CALIB_STAT_REG_LEN) == ERROR)
      {
        __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
      }
    }
    while ((temp[0] & BNO055_CALIB_STAT_SYS_CALIB_STATUS_MSK) != \
           BNO055_CALIB_STAT_SYS_CALIB_STATUS_CALIBRATED_VALUE);

    /* Switch config mode to obtain the calibration profile */
    {
      temp[0] = BNO055_OPR_MODE_REG_CONFIG_VALUE;

      /* Write to operation mode register */
      if (writeBuffer(pHandle, BNO055_OPR_MODE_REG_ADDR, temp,
                    BNO055_OPR_MODE_REG_LEN) != SUCCESS)
      {
        __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
      }
    
      /* Delay for 20ms to ensure operation mode switching completed */
      DelayMs(20);
    }
  
    /* Read calibration data */
    if (readBuffer(pHandle, BNO055_CALIBRATION_PROFILE_ADDR,
                   temp, BNO055_CALIBRATION_PROFILE_LEN) == ERROR)
    {
      __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
    }

    /* Write calibration profile */
    if (EepromEmulator_WriteObject(pHandle->init.eeEmulObjId,
                                   BNO055_CALIBRATION_PROFILE_LEN,
                                   temp) != EEPROM_EMULATOR_OP_RES_SUCCESS)
    {
      __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_EEPROM_EMULATOR_ERROR);
    }

    /* Switch operation mode to 9-axis sensor fusion mode */
    {
      temp[0] = BNO055_OPR_MODE_REG_NDOF_VALUE;

      /* Set operation mode to NDOF mode */
      if (writeBuffer(pHandle, BNO055_OPR_MODE_REG_ADDR, temp,
                    BNO055_OPR_MODE_REG_LEN) != SUCCESS)
      {
        __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
      }

      /* Delay for 1s to ensure operation mode switching and first conversion is completed */
      DelayMs(1000);
    }
  }

  /* Initialize static variables */
  pHandle->statics.quaDataReadReq.devAddr = pHandle->init.devAddr;
  pHandle->statics.quaDataReadReq.memAddr = BNO055_QUA_DATA_ADDR;
  pHandle->statics.quaDataReadReq.pData = pHandle->statics.quaData;
  pHandle->statics.quaDataReadReq.pReqStatusFlag = &(pHandle->statics.quaDataReadReqStatusFlag);
  pHandle->statics.quaDataReadReq.size = 8;
  pHandle->statics.quaDataReadReq.type = I2C_MANAGER_REQ_TYPE_READ;
  
  /* Do the first quaternion data request */
  if (I2CManager_Request(pHandle->init.pI2CManagerHandle,
                         &(pHandle->statics.quaDataReadReq)) != I2C_MANAGER_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
  }
      
  /* Start procedure is completed, set module's state to operating */
  pHandle->state = ORIENT_SENS_DRV_STATE_OPERATING;
  
  __UNLOCK(pHandle);
  
  return ORIENT_SENS_DRV_OP_RES_SUCCESS;
}

/**
  * @brief  Orientation sensor driver module executer. 
  *
  * @param  pHandle: Pointer to orientation sensor driver module handle.
  * @param  pOrientation: Pointer to return orientation data. 
  *
  * @retval OrientSensDrv_OpRes_t.
  */
OrientSensDrv_OpRes_t OrientSensDrv_Execute(OrientSensDrv_Handle_t *pHandle, 
                                            Quaternion_Quaternion_t *pOrientation)
{
  /* Check if the module is at error state */
  if (pHandle->state == ORIENT_SENS_DRV_STATE_ERROR)
  {
    return ORIENT_SENS_DRV_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check if the module's state is compatible with the operation */
  if (pHandle->state != ORIENT_SENS_DRV_STATE_OPERATING)
  {
    __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_INCOMPATIBLE_STATE);
  }

  /* Check if the read request is completed */
  if (pHandle->statics.quaDataReadReqStatusFlag != I2C_MANAGER_REQ_STATUS_COMPLETED)
  {
    __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
  }

  /* Cast signed integers to floating pointer variables
   Turn the MSB and LSB into a signed 16-bit value */
  pOrientation->w = (float)((int16_t)(pHandle->statics.quaData[1] << 8) | \
                                      pHandle->statics.quaData[0]);
  pOrientation->x = (float)((int16_t)(pHandle->statics.quaData[3] << 8) | \
                                      pHandle->statics.quaData[2]);
  pOrientation->y = (float)((int16_t)(pHandle->statics.quaData[5] << 8) | \
                                      pHandle->statics.quaData[4]);
  pOrientation->z = (float)((int16_t)(pHandle->statics.quaData[7] << 8) | \
                                      pHandle->statics.quaData[6]);

  /* Request quaternion data read for the next cycle */
  if (I2CManager_Request(pHandle->init.pI2CManagerHandle,
                         &(pHandle->statics.quaDataReadReq)) != I2C_MANAGER_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
  }
      
  /* Set driver state to operating */
  pHandle->state = ORIENT_SENS_DRV_STATE_OPERATING;
  
  __UNLOCK(pHandle);
  
  return ORIENT_SENS_DRV_OP_RES_SUCCESS;
}

/**
  * @brief  Stops operation.
  *
  * @param  pHandle: Pointer to orientation sensor driver module handle.
  *
  * @retval OrientSensDrv_OpRes_t.
  */
OrientSensDrv_OpRes_t OrientSensDrv_Stop(OrientSensDrv_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == ORIENT_SENS_DRV_STATE_ERROR)
  {
    return ORIENT_SENS_DRV_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != ORIENT_SENS_DRV_STATE_OPERATING)
  {
    __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Switch operation mode to config mode */
  {
    temp[0] = BNO055_OPR_MODE_REG_CONFIG_VALUE;
    
    /* Set operation mode to config mode */
    if (writeBuffer(pHandle, BNO055_OPR_MODE_REG_ADDR, temp,
                    BNO055_OPR_MODE_REG_LEN) != SUCCESS)
    {
      __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
    }
    
    /* Delay for 20ms to ensure operation mode switching completed */
    DelayMs(20);
  }
  
  /* Set driver state to ready */
  pHandle->state = ORIENT_SENS_DRV_STATE_READY;
  
  __UNLOCK(pHandle);
  
  return ORIENT_SENS_DRV_OP_RES_SUCCESS;
}
                                         
/**
  * @brief  Resets orientation sensor driver module.
  *
  * @param  pHandle: Pointer to orientation sensor driver module handle.
  *
  * @retval OrientSensDrv_OpRes_t.
  */
OrientSensDrv_OpRes_t OrientSensDrv_Reset(OrientSensDrv_Handle_t *pHandle)
{
  __LOCK(pHandle);
  
  /* Switch operation mode to config mode */
  {
    temp[0] = BNO055_OPR_MODE_REG_CONFIG_VALUE;
    
    /* Set operation mode to config mode */
    if (writeBuffer(pHandle, BNO055_OPR_MODE_REG_ADDR, temp,
                    BNO055_OPR_MODE_REG_LEN) != SUCCESS)
    {
      __ERROR(pHandle, ORIENT_SENS_DRV_ERR_CODE_I2C_MANAGER_ERROR);
    }
    
    /* Delay for 20ms to ensure operation mode switching completed */
    DelayMs(20);
  }
  
  /* Set driver state to reset */
  pHandle->state = ORIENT_SENS_DRV_STATE_UNINIT;
  
  __UNLOCK(pHandle);
  
  return ORIENT_SENS_DRV_OP_RES_SUCCESS;
}

/**
  * @brief  Writes block of data to the sensor memory in blocking mode.
  *
  * @param  pHandle: Pointer to orientation sensor driver module.
  * @param  memAddr: Data memory address.
  * @param  pData: Pointer to data block.
  * @param  length: Length of the data which is to be written.
  *
  * @retval SUCCESS or ERROR.
  */
static ErrorStatus writeBuffer(OrientSensDrv_Handle_t *pHandle, uint8_t memAddr,
                               uint8_t *pData, uint8_t length)
{
  I2CManager_ReqStatus_t request_status;
  I2CManager_Request_t  request;
  
  /* Set request parameters */
  request.devAddr = pHandle->init.devAddr;
  request.memAddr = memAddr;
  request.pData = pData;
  request.pReqStatusFlag = &request_status;
  request.size = sizeof(uint8_t);
  request.type = I2C_MANAGER_REQ_TYPE_WRITE;
  
  /* Write all the elements one by one */ 
  /* Write process needed to be like these because BNO055 sensor doesn't support 
     multiple byte write */ 
  for (uint8_t i = 0; i < length; i++)
  {
    /* Request */
    if (I2CManager_Request(pHandle->init.pI2CManagerHandle, 
                           &request) != I2C_MANAGER_OP_RES_SUCCESS)
    {
      return ERROR;
    }
    
    /* Wait while the request is being processed */
    while ((request_status == I2C_MANAGER_REQ_STATUS_PENDING) || \
           (request_status == I2C_MANAGER_REQ_STATUS_PROCESSING));
    
    /* Check if the request handled without errors */
    if (request_status == I2C_MANAGER_REQ_STATUS_ERROR)
    {
      return ERROR;
    }
    
    request.memAddr++;
    request.pData++;
  }
  
  return SUCCESS;
}

/**
  * @brief  Reads block of data from the sensor memory in blocking mode.
  *
  * @param  pHandle: Pointer to orientation sensor driver module.
  * @param  memAddr: Data memory address.
  * @param  pData: Pointer to data block.
  * @param  length: Length of the data which is to be read.
  *
  * @retval SUCCESS or ERROR.
  */
static ErrorStatus readBuffer(OrientSensDrv_Handle_t *pHandle, uint8_t memAddr, 
                              uint8_t *pData, uint8_t length)
{
  I2CManager_ReqStatus_t request_status;
  I2CManager_Request_t  request;
  
  /* Set request parameters */
  request.devAddr = pHandle->init.devAddr;
  request.memAddr = memAddr;
  request.pData = pData;
  request.pReqStatusFlag = &request_status;
  request.size = length;
  request.type = I2C_MANAGER_REQ_TYPE_READ;
  
  /* Request */
  if (I2CManager_Request(pHandle->init.pI2CManagerHandle, 
                         &request) != I2C_MANAGER_OP_RES_SUCCESS)
  {
    return ERROR;
  }
    
  /* Wait while the request is being processed */
  while ((request_status == I2C_MANAGER_REQ_STATUS_PENDING) || \
         (request_status == I2C_MANAGER_REQ_STATUS_PROCESSING));
    
  /* Check if the request handled without errors */
  if (request_status == I2C_MANAGER_REQ_STATUS_ERROR)
  {
    return ERROR;
  }
  
  return SUCCESS;
}