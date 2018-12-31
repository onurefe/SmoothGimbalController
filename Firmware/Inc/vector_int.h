/**
  ******************************************************************************
  * @file    vector_int.h
  * @author  Onur Efe
  * @date    13.07.2016
  * @brief   Vector interpolator module header file.
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
#ifndef __VECTOR_INT_H
#define __VECTOR_INT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "generic.h"
 
/* Exported constants --------------------------------------------------------*/
#define VECTOR_INT_MAX_CIRC_BUFFER_SIZE         15
#define VECTOR_INT_MAX_VECTOR_SIZE              4
   
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  VECTOR_INT_LOCK_STATE_LOCKED                  = 0x00,         /* Module is locked */
  VECTOR_INT_LOCK_STATE_UNLOCKED                = 0x01          /* Module is unlocked */
} VectorInt_LockState_t;                                        /* Lock State typedef */

typedef enum
{
  VECTOR_INT_STATE_UNINIT                       = 0x00,         /* Module is not initialized */
  VECTOR_INT_STATE_READY                        = 0x01,         /* Module is ready to operate */
  VECTOR_INT_STATE_OPERATING                    = 0x02,         /* Operating */
  VECTOR_INT_STATE_ERROR                        = 0x03          /* Error occured */
} VectorInt_State_t;                                            /* State typedef */

typedef enum
{
  VECTOR_INT_ERR_CODE_NONE                      = 0x00,         /* None */
  VECTOR_INT_ERR_CODE_INVALID_PARAMETER         = 0x01,         /* Invalid parameter */
  VECTOR_INT_ERR_CODE_INCOMPATIBLE_STATE        = 0x02,         /* Incompatible state */
  VECTOR_INT_ERR_CODE_RACE_CONDITION            = 0x03          /* Race condition */
} VectorInt_ErrCode_t;                                          /* Error Code typedef */

typedef enum
{
  VECTOR_INT_OP_RES_SUCCESS                     = 0x00,         /* Success */
  VECTOR_INT_OP_RES_FAILURE                     = 0x01          /* Failure */
} VectorInt_OpRes_t;                                            /* Operation Result typedef */

typedef struct
{
  float                 vector[VECTOR_INT_MAX_VECTOR_SIZE];     /* Vector */
  uint32_t              tick;                                   /* Tick */
} VectorInt_InstVector_t;                                       /* Instant vector typedef */

typedef struct
{
  VectorInt_InstVector_t        elements[VECTOR_INT_MAX_CIRC_BUFFER_SIZE];  /* Elements */
  uint8_t               vectorSize;                             /* Vector size */
  uint8_t               start;                                  /* Start index */
  uint8_t               size;                                   /* Size */
  uint8_t               capacity;                               /* Capacity */
} VectorInt_CircBuff_t;                                         /* Circular Buffer typedef */

typedef struct
{
  float                 unitVector[VECTOR_INT_MAX_VECTOR_SIZE]; /* Unit vector */
  float                 filterTC;                               /* Exponential filter time constant */
  float                 timeShift;                              /* Time shift */
  uint8_t               circBufferSize;                         /* Circular buffer size */
  uint8_t               vectorSize;                             /* Vector size */
} VectorInt_Init_t;                                             /* Init typedef */

typedef struct
{
  VectorInt_CircBuff_t  circBuffer;                             /* Circular buffer */
  VectorInt_InstVector_t        inputElement;                   /* Input element */
  float                 filterMultiplier;                       /* Filter multiplier */
  float                 expAvr[VECTOR_INT_MAX_VECTOR_SIZE];     /* Exponential average */
  uint32_t              tsInTicks;                              /* Time shift in ticks */
  uint8_t               newInputFlag;                           /* There is a new input */
} VectorInt_Statics_t;                                          /* Statics typedef */

typedef struct
{
  VectorInt_LockState_t lockState;                              /* Lock State */
  VectorInt_State_t     state;                                  /* State */
  VectorInt_ErrCode_t   errCode;                                /* Error Code */
  VectorInt_Init_t      init;                                   /* Init */
  VectorInt_Statics_t   statics;                                /* Statics */
} VectorInt_Handle_t;                                           /* Handle typedef */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Initializes a module.
  *
  * @params pHandle: Pointer to module's handle.
  *
  * @retval Operation result.
  */
VectorInt_OpRes_t VectorInt_Init(VectorInt_Handle_t *pHandle, VectorInt_Init_t *pInitStruct);

/**
  * @brief  Starts a module.
  *
  * @params pHandle: Pointer to module's handle.
  *
  * @retval Operation result.
  */
VectorInt_OpRes_t VectorInt_Start(VectorInt_Handle_t *pHandle);

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
                                    uint32_t tick);

/**
  * @brief  Stops the operating module.
  *
  * @params pHandle: Pointer to module's handle.
  *
  * @retval Operation result.
  */
VectorInt_OpRes_t VectorInt_Stop(VectorInt_Handle_t *pHandle);

/**
  * @brief  Resets the module.
  *
  * @params pHandle: Pointer to module's handle.
  *
  * @retval Operation result.
  */
VectorInt_OpRes_t VectorInt_Reset(VectorInt_Handle_t *pHandle);

/**
  * @brief  Input value for interpolation to the module.
  *
  * @params pHandle: Pointer to module's handle.
  * @params pVector: Pointer to the input vector.
  * @params tick: Interpolater tick.
  *
  * @retval Operation result.
  */
VectorInt_OpRes_t VectorInt_Input(VectorInt_Handle_t *pHandle, float *pVector, 
                                  uint32_t tick);

#ifdef __cplusplus
}
#endif

#endif