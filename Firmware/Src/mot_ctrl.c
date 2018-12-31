/**
  ******************************************************************************
  * @file    mot_ctrl.c
  * @author  Onur Efe
  * @date    18.02.2016
  * @brief   Motor controller class implementation. 
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
#include "mot_ctrl.h"

/* Private constants ---------------------------------------------------------*/
#define CONTROL_PERIOD                          (1.0f / CONTROL_FREQ)
#define BLDC_RECOGNITION_PERIOD_IN_MS           8192U
#define BLDC_RECOGNITION_STEP_COUNT             512U

/* Private macros ------------------------------------------------------------*/
#define __ERROR(_HANDLE_, _ERR_CODE_)   \
                                          do {                                                                  \
                                                (_HANDLE_)->state = (MOT_CTRL_STATE_ERROR);                     \
                                                (_HANDLE_)->errCode = (_ERR_CODE_);                             \
                                                (_HANDLE_)->lockState = (MOT_CTRL_LOCK_STATE_UNLOCKED);         \
                                                return (MOT_CTRL_OP_RES_FAILURE);                               \
                                          } while(0)

#define __LOCK(_HANDLE_)                        ((_HANDLE_)->lockState = (MOT_CTRL_LOCK_STATE_LOCKED))
                                          
#define __UNLOCK(_HANDLE_)                      ((_HANDLE_)->lockState = (MOT_CTRL_LOCK_STATE_UNLOCKED))
                                          
#define IS_LOCKED(_HANDLE_)                     ((_HANDLE_)->lockState == (MOT_CTRL_LOCK_STATE_LOCKED))

/* Exported functions --------------------------------------------------------*/
/**
  * @brief  Motor controller module initializer. 
  *
  * @param  pHandle: Pointer to module handle.
  * @param  pInitStruct: Pointer to init struct.
  *
  * @retval Operation result.
  */
MotCtrl_OpRes_t MotCtrl_Init(MotCtrl_Handle_t *pHandle, MotCtrl_Init_t *pInitStruct)
{
  /* Check if the module is at error state */
  if (pHandle->state == MOT_CTRL_STATE_ERROR)
  {
    return MOT_CTRL_OP_RES_FAILURE;
  }
  
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != MOT_CTRL_STATE_UNINIT)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_INCOMPATIBLE_STATE);
  }
 
  /* Initialize Position Sensor Driver module */
  if (PosSensDrv_Init(&(pHandle->subModules.posSensDrvHandle), 
                      &(pInitStruct->posSensDrvInit)) != POS_SENS_DRV_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_POS_SENS_DRV_ERROR);
  }
  
  /* Initialize PID Controller modules */
  if ((PidCtrl_Init(&(pHandle->subModules.pidCtrlHandle), 
                    &(pInitStruct->pidCtrlInit)) != PID_CTRL_OP_RES_SUCCESS))
              
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_PID_CTRL_ERROR);
  }
  
  /* Initialize Brushless DC Driver module */
  if (BldcDrv_Init(&(pHandle->subModules.bldcDrvHandle), 
                   &(pInitStruct->bldcDrvInit)) != BLDC_DRV_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_BLDC_DRV_ERROR);
  }

  /* Set eeprom emulator object id */
  pHandle->init.eeEmulObjId = pInitStruct->eeEmulObjId;
  
  /* Set module's error code to none */
  pHandle->errCode = MOT_CTRL_ERR_CODE_NONE;
  
  /* Set module's state to ready */
  pHandle->state = MOT_CTRL_STATE_READY;
  
  __UNLOCK(pHandle);
  
  return MOT_CTRL_OP_RES_SUCCESS;
}

/**
  * @brief  Motor controller module starter.
  *
  * @param  pHandle: Pointer to module handle.
  *
  * @retval Operation result.
  */
MotCtrl_OpRes_t MotCtrl_Start(MotCtrl_Handle_t *pHandle)
{
  uint8_t temp;
  uint8_t recognized_flag;
  MotCtrl_NV_t  non_volatile_obj;
  
  /* Check if the module is at error state */
  if (pHandle->state == MOT_CTRL_STATE_ERROR)
  {
    return MOT_CTRL_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_RACE_CONDITION);
  }

  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != MOT_CTRL_STATE_READY)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Start Position Sensor Driver module */
  if (PosSensDrv_Start(&(pHandle->subModules.posSensDrvHandle)) != POS_SENS_DRV_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_POS_SENS_DRV_ERROR);
  }
  
  /* Start PID Controller module */
  if (PidCtrl_Start(&(pHandle->subModules.pidCtrlHandle)) != PID_CTRL_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_PID_CTRL_ERROR);
  }
  
  /* Start Brushless DC Driver module */
  if (BldcDrv_Start(&(pHandle->subModules.bldcDrvHandle)) != BLDC_DRV_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_BLDC_DRV_ERROR);
  }

  /* Read calibration profile if it exists in the non-volatile memory and save it to
    the device */
  {
    uint8_t ret_val;
    
    ret_val = EepromEmulator_ReadObject(pHandle->init.eeEmulObjId, 
                                        sizeof(MotCtrl_NV_t),
                                        &temp, (uint8_t *)&non_volatile_obj);
    recognized_flag = FALSE;
    
    /* Take action according to return value */
    if (ret_val == EEPROM_EMULATOR_OP_RES_SUCCESS)
    {
      if (non_volatile_obj.validityFlag == TRUE)
      {
        /* Set parameters */
        if (BldcDrv_SetParameters(&pHandle->subModules.bldcDrvHandle, 
                                  non_volatile_obj.direction, 
                                  non_volatile_obj.offset) != BLDC_DRV_OP_RES_SUCCESS)
        {
          __ERROR(pHandle, MOT_CTRL_ERR_CODE_BLDC_DRV_ERROR);
        }
        
        recognized_flag = TRUE;
      }
    }
    else if (ret_val == EEPROM_EMULATOR_OP_RES_OBJ_NOT_FOUND)
    {
      /* If the object is not found, add instructions if necessary */
    }
    else
    {
      __ERROR(pHandle, MOT_CTRL_ERR_CODE_EEPROM_EMULATOR_ERROR);
    }
  }
  
  /* Recognize BLDC motors, if they are not recognized. */
  if (recognized_flag != TRUE)
  {
    float temp;
    float electrical_position;
    float encoder_position[BLDC_RECOGNITION_STEP_COUNT];
    float offset;
    BldcDrv_Direction_t direction;
    
    /* Detect BLDC response for full rotation */
    for (uint16_t i = 0; i < BLDC_RECOGNITION_STEP_COUNT; i++)
    {
      electrical_position = (float)i * 2.0f * PI / BLDC_RECOGNITION_STEP_COUNT;
      
      /* Set electrical angle */
      if (BldcDrv_Execute(&pHandle->subModules.bldcDrvHandle, 
                          electrical_position, 1.0f, 0.0f) != BLDC_DRV_OP_RES_SUCCESS)
      {
        __ERROR(pHandle, MOT_CTRL_ERR_CODE_BLDC_DRV_ERROR);
      }
      
      if (i == 0)
      {
        /* This is the first step, it will take a while for the system to be stabilized */
        DelayMs(3000);
        
        /* Execute position sensor driver to get encoder data(which is to be discarded) */
        if (PosSensDrv_Execute(&pHandle->subModules.posSensDrvHandle, 
                               &temp) != POS_SENS_DRV_OP_RES_SUCCESS)
        {
          __ERROR(pHandle, MOT_CTRL_ERR_CODE_POS_SENS_DRV_ERROR);
        }
        
        /* Delay for some time, because encoder read process should be completed */
        DelayMs(2);
      }
      else
      {
        /* Delay for some time, for the system to get into new position */
        DelayMs(BLDC_RECOGNITION_PERIOD_IN_MS / BLDC_RECOGNITION_STEP_COUNT);
      }
      
      /* Execute position sensor driver to get encoder data */
      if (PosSensDrv_Execute(&pHandle->subModules.posSensDrvHandle, 
                             &encoder_position[i]) != POS_SENS_DRV_OP_RES_SUCCESS)
      {
        __ERROR(pHandle, MOT_CTRL_ERR_CODE_POS_SENS_DRV_ERROR);
      }
    }
    
    /* Reset BLDC */
    if (BldcDrv_Execute(&pHandle->subModules.bldcDrvHandle,
                        BLDC_DRV_INITIAL_POSITION,
                        BLDC_DRV_INITIAL_DUTY_RATIO, 0.0f) != BLDC_DRV_OP_RES_SUCCESS)
    {
      __ERROR(pHandle, MOT_CTRL_ERR_CODE_BLDC_DRV_ERROR);
    }
    
    /* Determine the direction */
    if ((encoder_position[BLDC_RECOGNITION_STEP_COUNT - 1] - encoder_position[0]) >= 0.0f)
    {
      direction = BLDC_DRV_ROT_DIR_POSITIVE;
    }
    else
    {
      direction = BLDC_DRV_ROT_DIR_NEGATIVE;
    }
    
    temp = 0.0f;
    
    /* Calculate the offset */
    for (uint16_t i = 1; i < BLDC_RECOGNITION_STEP_COUNT; i++)
    {
      if (direction == BLDC_DRV_ROT_DIR_POSITIVE)
      {
        temp += ((float)i * 2.0f * PI / BLDC_RECOGNITION_STEP_COUNT) - encoder_position[i];
      }
      else
      {
        temp += ((float)i * 2.0f * PI / BLDC_RECOGNITION_STEP_COUNT) + encoder_position[i];
      }
    }
    
    offset = temp / BLDC_RECOGNITION_STEP_COUNT;
    
    /* Set parameters */
    if (BldcDrv_SetParameters(&pHandle->subModules.bldcDrvHandle,
                              direction, offset) != BLDC_DRV_OP_RES_SUCCESS)
    {
      __ERROR(pHandle, MOT_CTRL_ERR_CODE_BLDC_DRV_ERROR);
    }
    
    /* Set non-volatile object and write it to the internal flash via eeprom emulator module */
    non_volatile_obj.validityFlag = TRUE;
    non_volatile_obj.direction = direction;
    non_volatile_obj.offset = offset;
    
    if (EepromEmulator_WriteObject(pHandle->init.eeEmulObjId, sizeof(MotCtrl_NV_t),
                                   (uint8_t *)&non_volatile_obj) != EEPROM_EMULATOR_OP_RES_SUCCESS)
    {
      __ERROR(pHandle, MOT_CTRL_ERR_CODE_EEPROM_EMULATOR_ERROR);
    }
  }
  
  /* Set module's state to operating */
  pHandle->state = MOT_CTRL_STATE_OPERATING;
  
  __UNLOCK(pHandle);

  return MOT_CTRL_OP_RES_SUCCESS;
}

/**
  * @brief  Module executer. Should be called periodicaly for proper
  *         operation.
  *
  * @param  pHandle: Pointer to motor controller module handle.
  * @param  destinationPosition: Destination position.
  *
  * @retval Operation result.
  */
MotCtrl_OpRes_t MotCtrl_Execute(MotCtrl_Handle_t *pHandle, float destinationPosition)
{
  /* Check if the module is at error state */
  if (pHandle->state == MOT_CTRL_STATE_ERROR)
  {
    return MOT_CTRL_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);

  /* Check for state compatibility */
  if (pHandle->state != MOT_CTRL_STATE_OPERATING)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_INCOMPATIBLE_STATE);
  }

  float real_position;
  float position_error;
  float pid_response;
  float duty_ratio;
  float shift_ratio;
  
  /* Execute Position Sensor Driver module and get position */
  if (PosSensDrv_Execute(&(pHandle->subModules.posSensDrvHandle), 
                         &real_position) != POS_SENS_DRV_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_POS_SENS_DRV_ERROR);
  }
  
  /* Calculate position error */
  position_error = destinationPosition - real_position;
  
  /* Execute PID Controller module */
  if (PidCtrl_Execute(&(pHandle->subModules.pidCtrlHandle), position_error,
                      &pid_response) != PID_CTRL_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_PID_CTRL_ERROR);
  }

  /* Set duty and shift ratio according to pid response */
  if (pid_response > 0.0f)
  {
    duty_ratio = pid_response;
    shift_ratio = 1.0f;
  }
  else
  {
    duty_ratio = -pid_response;
    shift_ratio = -1.0f;
  }
  
  /* Execute BLDC Driver module */
  if (BldcDrv_Execute(&(pHandle->subModules.bldcDrvHandle), 
                      real_position, duty_ratio, shift_ratio) != BLDC_DRV_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_BLDC_DRV_ERROR);
  }
  
  __UNLOCK(pHandle);
  
  return MOT_CTRL_OP_RES_SUCCESS;
}

/**
  * @brief  Stops module's operation. 
  *
  * @param  pHandle: Pointer to module handle.
  *
  * @retval Operation result.
  */
MotCtrl_OpRes_t MotCtrl_Stop(MotCtrl_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == MOT_CTRL_STATE_ERROR)
  {
    return MOT_CTRL_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != MOT_CTRL_STATE_OPERATING)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_INCOMPATIBLE_STATE);
  }

  /* Stop Position Sensor Driver module */
  if (PosSensDrv_Stop(&(pHandle->subModules.posSensDrvHandle)) != POS_SENS_DRV_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_POS_SENS_DRV_ERROR);
  }
  
  /* Stop PID Controller module */
  if (PidCtrl_Stop(&(pHandle->subModules.pidCtrlHandle)) != PID_CTRL_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_PID_CTRL_ERROR);
  }
  
  /* Stop BLDC Driver module */
  if (BldcDrv_Stop(&(pHandle->subModules.bldcDrvHandle)) != BLDC_DRV_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_BLDC_DRV_ERROR);
  }
  
  /* Set module's state to ready */
  pHandle->state = MOT_CTRL_STATE_READY;
  
  __UNLOCK(pHandle);
  
  return MOT_CTRL_OP_RES_SUCCESS;
}
    
/**
  * @brief  Resets module. 
  *
  * @param  pHandle: Pointer to module handle.
  *
  * @retval Operation result.
  */
MotCtrl_OpRes_t MotCtrl_Reset(MotCtrl_Handle_t *pHandle)
{
  __LOCK(pHandle);

  /* Reset Position Sensor Driver module */
  if (PosSensDrv_Reset(&(pHandle->subModules.posSensDrvHandle)) != POS_SENS_DRV_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_POS_SENS_DRV_ERROR);
  }
  
  /* Reset PID Controller module */
  if (PidCtrl_Reset(&(pHandle->subModules.pidCtrlHandle)) != PID_CTRL_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_PID_CTRL_ERROR);
  }
  
  /* Reset BLDC Driver module */
  if (BldcDrv_Reset(&(pHandle->subModules.bldcDrvHandle)) != BLDC_DRV_OP_RES_SUCCESS)
  {
    __ERROR(pHandle, MOT_CTRL_ERR_CODE_BLDC_DRV_ERROR);
  }
  
  /* Set module's state to uninit */
  pHandle->state = MOT_CTRL_STATE_UNINIT;
  
  __UNLOCK(pHandle);
  
  return MOT_CTRL_OP_RES_SUCCESS;
}