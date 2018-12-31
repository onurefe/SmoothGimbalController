/**
  ******************************************************************************
  * @file    vector_int.c
  * @author  Onur Efe
  * @date    13.07.2016
  * @brief   Vector interpolater module.
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
#include "vector_int.h"

/* Privates constants --------------------------------------------------------*/
#define CONTROL_PERIOD                          (1.0f / CONTROL_FREQ)
    
/* Private macros ------------------------------------------------------------*/
#define __ERROR(_HANDLE_, _ERR_CODE_)   \
                                          do {                                                                  \
                                                (_HANDLE_)->state = (VECTOR_INT_STATE_ERROR);                   \
                                                (_HANDLE_)->errCode = (_ERR_CODE_);                             \
                                                (_HANDLE_)->lockState = (VECTOR_INT_LOCK_STATE_UNLOCKED);       \
                                                return (VECTOR_INT_OP_RES_FAILURE);                             \
                                          } while(0)

#define __LOCK(_HANDLE_)                        ((_HANDLE_)->lockState = (VECTOR_INT_LOCK_STATE_LOCKED))
                                          
#define __UNLOCK(_HANDLE_)                      ((_HANDLE_)->lockState = (VECTOR_INT_LOCK_STATE_UNLOCKED))
                                          
#define IS_LOCKED(_HANDLE_)                     ((_HANDLE_)->lockState == (VECTOR_INT_LOCK_STATE_LOCKED))

/* Private functions ---------------------------------------------------------*/
static void put(VectorInt_CircBuff_t *C, VectorInt_InstVector_t *pElement);
static void read(VectorInt_CircBuff_t *C, VectorInt_InstVector_t *pElement, uint8_t index);

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes a module.
  *
  * @params pHandle: Pointer to module's handle.
  *
  * @retval Operation result.
  */
VectorInt_OpRes_t VectorInt_Init(VectorInt_Handle_t *pHandle, VectorInt_Init_t *pInitStruct)
{
  /* Check if the module is at error state */
  if (pHandle->state == VECTOR_INT_STATE_ERROR)
  {
    return VECTOR_INT_OP_RES_FAILURE;
  }
  
  __LOCK(pHandle);
  
  /* Validate parameters */
  if ((pInitStruct->filterTC == 0.0f) || \
      (pInitStruct->circBufferSize > VECTOR_INT_MAX_CIRC_BUFFER_SIZE) || \
      (pInitStruct->vectorSize > VECTOR_INT_MAX_VECTOR_SIZE))
  {
    __ERROR(pHandle, VECTOR_INT_ERR_CODE_INVALID_PARAMETER);
  }
  
  /* Check for state compatibility */
  if (pHandle->state != VECTOR_INT_STATE_UNINIT)
  {
    __ERROR(pHandle, VECTOR_INT_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Set init variables */
  pHandle->init.filterTC = pInitStruct->filterTC;
  pHandle->init.timeShift = pInitStruct->timeShift;
  pHandle->init.circBufferSize = pInitStruct->circBufferSize;
  pHandle->init.vectorSize = pInitStruct->vectorSize;
  
  for (uint8_t i = 0; i < pInitStruct->vectorSize; i++)
  {
    pHandle->init.unitVector[i] = pInitStruct->unitVector[i];
  }
  
  /* Set module's state to ready */
  pHandle->state = VECTOR_INT_STATE_READY;
  
  __UNLOCK(pHandle);
  
  return VECTOR_INT_OP_RES_SUCCESS;
}

/**
  * @brief  Starts a module.
  *
  * @params pHandle: Pointer to module's handle.
  *
  * @retval Operation result.
  */
VectorInt_OpRes_t VectorInt_Start(VectorInt_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == VECTOR_INT_STATE_ERROR)
  {
    return VECTOR_INT_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, VECTOR_INT_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != VECTOR_INT_STATE_READY)
  {
    __ERROR(pHandle, VECTOR_INT_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Set static variables to their initial values */
  pHandle->statics.circBuffer.vectorSize = pHandle->init.vectorSize;
  pHandle->statics.circBuffer.start = 0U;
  pHandle->statics.circBuffer.size = 0U;
  pHandle->statics.circBuffer.capacity = pHandle->init.circBufferSize;
  pHandle->statics.filterMultiplier = (float)CONTROL_PERIOD / \
                                             (pHandle->init.filterTC + CONTROL_PERIOD);
  pHandle->statics.tsInTicks = (uint32_t)(round(CONTROL_FREQ * pHandle->init.timeShift));
  pHandle->statics.newInputFlag = FALSE;
  
  /* Set exponential average vector to it's initial value */
  for (uint8_t i = 0; i < pHandle->init.vectorSize; i++)
  {
    pHandle->statics.expAvr[i] = pHandle->init.unitVector[i];
  }
  
  /* Set module's state to operating */
  pHandle->state = VECTOR_INT_STATE_OPERATING;
  
  __UNLOCK(pHandle);
  
  return VECTOR_INT_OP_RES_SUCCESS;
}

/**
  * @brief  Executes the vector interpolater module.
  *
  * @params pHandle: Pointer to module's handle.
  * @params pVector: Pointer to interpolation result.
  * @params tick: Current tick value.
  *
  * @retval Operation result.
  */
VectorInt_OpRes_t VectorInt_Execute(VectorInt_Handle_t *pHandle, float *pVector,
                                    uint32_t tick)
{
  /* Check if the module is at error state */
  if (pHandle->state == VECTOR_INT_STATE_ERROR)
  {
    return VECTOR_INT_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, VECTOR_INT_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != VECTOR_INT_STATE_OPERATING)
  {
    __ERROR(pHandle, VECTOR_INT_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Check for new input element */
  if (pHandle->statics.newInputFlag == TRUE)
  {
    /* If there is an unprocessed input, put it into the circular buffer if it's tick 
      value is larger than last element's and set newInputFlag to false */
    put(&(pHandle->statics.circBuffer), &(pHandle->statics.inputElement));
    pHandle->statics.newInputFlag = FALSE;
  }
  
  uint8_t elements_found = FALSE;
  
  VectorInt_InstVector_t lower_element;
  VectorInt_InstVector_t upper_element;
  
  float relative_difference;
  float interpolation_result[VECTOR_INT_MAX_VECTOR_SIZE];
  
  /* Find interpolation elements */
  for (uint8_t i = 0; i < pHandle->statics.circBuffer.size; i++)
  {
    /* Read circular buffer into upper element */
    read(&(pHandle->statics.circBuffer), &upper_element, i);
    
    /* If shifted tick value is greater or equal to the tick value, upper element is found */
    if ((upper_element.tick + pHandle->statics.tsInTicks) >= tick)
    {
      /* If there is a previous element */
      if (i > 0)
      {
        /* Read circular buffer into lower element */
        read(&(pHandle->statics.circBuffer), &lower_element, (i - 1));
        
        elements_found = TRUE;
      }
      
      break;
    }
  }
  
  /* If both of the elements have been found, apply linear interpolation */ 
  if (elements_found == TRUE)
  {
    /* Calculate relative difference */
    relative_difference = (float)tick - (lower_element.tick + pHandle->statics.tsInTicks);
    relative_difference /= (upper_element.tick - lower_element.tick);
      
    /* Apply for each component of the vector */
    for (uint8_t i = 0; i < pHandle->init.vectorSize; i++)
    {
      interpolation_result[i] = lower_element.vector[i] + \
                                ((upper_element.vector[i] - lower_element.vector[i]) * relative_difference);
    }
  }
  
  /* If there is no upper element, set result to the last element */
  else if (pHandle->statics.circBuffer.size > 0)
  {
    /* Read circular buffer into the lower element */
    read(&(pHandle->statics.circBuffer), &lower_element, (pHandle->statics.circBuffer.size - 1));
    
    for (uint8_t i = 0; i < pHandle->init.vectorSize; i++)
    {
      interpolation_result[i] = lower_element.vector[i];
    }
  }
  
  /* Else set result to the unit vector */
  else
  {
    for (uint8_t i = 0; i < pHandle->init.vectorSize; i++)
    {
      interpolation_result[i] = pHandle->init.unitVector[i];
    }
  }
  
  /* Apply low pass filter(exponential average) to interpolation result and set the 
    return vector */
  for (uint8_t i = 0; i < pHandle->init.vectorSize; i++)
  {
    pHandle->statics.expAvr[i] *= (1.0f - pHandle->statics.filterMultiplier);
    pHandle->statics.expAvr[i] += pHandle->statics.filterMultiplier * interpolation_result[i];
    pVector[i] = pHandle->statics.expAvr[i];
  }
  
  __UNLOCK(pHandle);

  return VECTOR_INT_OP_RES_SUCCESS;
}

/**
  * @brief  Stops the operating module.
  *
  * @params pHandle: Pointer to module's handle.
  *
  * @retval Operation result.
  */
VectorInt_OpRes_t VectorInt_Stop(VectorInt_Handle_t *pHandle)
{
  /* Check if the module is at error state */
  if (pHandle->state == VECTOR_INT_STATE_ERROR)
  {
    return VECTOR_INT_OP_RES_FAILURE;
  }
  
  if (IS_LOCKED(pHandle))
  {
    __ERROR(pHandle, VECTOR_INT_ERR_CODE_RACE_CONDITION);
  }
  
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != VECTOR_INT_STATE_OPERATING)
  {
    __ERROR(pHandle, VECTOR_INT_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Set module's state to ready */
  pHandle->state = VECTOR_INT_STATE_READY;
  
  __UNLOCK(pHandle);
  
  return VECTOR_INT_OP_RES_SUCCESS;
}

/**
  * @brief  Resets the module.
  *
  * @params pHandle: Pointer to module's handle.
  *
  * @retval Operation result.
  */
VectorInt_OpRes_t VectorInt_Reset(VectorInt_Handle_t *pHandle)
{
  __LOCK(pHandle);
  
  /* Check for state compatibility */
  if (pHandle->state != VECTOR_INT_STATE_READY)
  {
    __ERROR(pHandle, VECTOR_INT_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Set module's state to uninit */
  pHandle->state = VECTOR_INT_STATE_UNINIT;
  
  __UNLOCK(pHandle);
  
  return VECTOR_INT_OP_RES_SUCCESS;
}

/**
  * @brief  Input value for interpolation to the module.
  *
  * @params pHandle: Pointer to module's handle.
  * @params pVector: Pointer to the input vector.
  * @params tick: Interpolater tick.
  *
  * @retval Operation result.
  */
VectorInt_OpRes_t VectorInt_Input(VectorInt_Handle_t *pHandle, float *pVector, uint32_t tick)
{
  /* Check if the module is at error state */
  if (pHandle->state == VECTOR_INT_STATE_ERROR)
  {
    return VECTOR_INT_OP_RES_FAILURE;
  }
  
  /* Check for state compatibility */
  if (pHandle->state != VECTOR_INT_STATE_OPERATING)
  {
    __ERROR(pHandle, VECTOR_INT_ERR_CODE_INCOMPATIBLE_STATE);
  }
  
  /* Set element according to input */
  for (uint8_t i = 0; i < pHandle->statics.circBuffer.vectorSize; i++)
  {
    pHandle->statics.inputElement.vector[i] = pVector[i];
  }
  
  /* Set tick value of the element */
  pHandle->statics.inputElement.tick = tick;
  
  /* Set new input flag */
  pHandle->statics.newInputFlag = TRUE;

  return VECTOR_INT_OP_RES_SUCCESS;
}

static void read(VectorInt_CircBuff_t *C, VectorInt_InstVector_t *pElement, uint8_t index)
{
  uint8_t read_index;

  /* Calculate read index */
  read_index = C->start + index;
  while (read_index >= C->capacity)
  {
    read_index -= C->capacity;
  }
  
  if (read_index >= C->size)
  { 
    return;
  }
  
  /* Set return pointer elements */
  /* Set the vector part */
  for (uint8_t i = 0; i < C->vectorSize; i++)
  {
    pElement->vector[i] = C->elements[read_index].vector[i];
  }
  
  /* Set tick value */
  pElement->tick = C->elements[read_index].tick;

  return;
}

static void put(VectorInt_CircBuff_t *C, VectorInt_InstVector_t *pElement)
{
  uint8_t write_index;
  
  /* Calculate write index */
  write_index = C->start + C->size;
  
  if (write_index >= C->capacity)
  {
    write_index -= C->capacity;
  }
  
  /* Copy the vector part */
  for (uint8_t i = 0; i < C->vectorSize; i++)
  {
    C->elements[write_index].vector[i] = pElement->vector[i];
  }
  
  /* Copy tick value */
  C->elements[write_index].tick = pElement->tick;
    
  if (C->size < C->capacity)
  {
    ++C->size;
  }
  else
  {
    if (++C->start >= C->capacity)
    {
      C->start -= C->capacity;
    }
  }
}