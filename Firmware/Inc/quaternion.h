/**
  ******************************************************************************
  * @file    quaternion.h
  * @author  Onur Efe
  * @date    21.04.2016
  * @brief   Quaternion class interface. Class includes functions related with 
  *          quaternions. Functions are for the normalized quaternions! So the user
  *          should first normalize a quaternion before making any mathematical operation
  *          with it.
  *
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
#ifndef __QUATERNION_H
#define __QUATERNION_H

#ifdef __cplusplus
 extern "C" {
#endif
   
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "generic.h"
   
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  QUATERNION_OP_RES_SUCCESS             = 0x00,         /* Operation successfull */
  QUATERNION_OP_RES_INVALID_POINTER     = 0x01,         /* Invalid pointer input */
  QUATERNION_OP_RES_INVALID_PARAMETER   = 0x02,         /* Invalid parameter input */
  QUATERNION_OP_RES_GIMBAL_LOCK_FAILURE = 0x03          /* Gimbal lock occured */
} Quaternion_OpRes_t;                                   /* Operation result typedef */

typedef struct 
{
  float first;                                          /* First(in units of radians) */
  float second;                                         /* Second(in units of radians) */
  float third;                                          /* Third(in units of radians) */
} Quaternion_EulerAngles_t;                             /* Euler angles typedef */

typedef struct
{
  float w;                                              /* w component of a quaternion */
  float x;                                              /* x component of a quaternion */
  float y;                                              /* y component of a quaternion */
  float z;                                              /* z component of a quaternion */
} Quaternion_Quaternion_t;                              /* Quaternion typedef */

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Rotates given quaternion according to the given quaternion. 
  *
  * @param  pTerm: Quaternion which is to be rotated.
  * @param  pRotQuat: Pointer of the rotation quaternion.
  * @param  pResult: Pointer to return the result.
  *
  * @retval Quaternion_OpRes_t.
  */
Quaternion_OpRes_t Quaternion_Rotate(Quaternion_Quaternion_t *pTerm, 
                                     Quaternion_Quaternion_t *pRotQuat, 
                                     Quaternion_Quaternion_t *pResult);

/**
  * @brief  Multiplies terms. 
  *
  * @param  pMultiplicand: Pointer of the multiplicand.
  * @param  pMultiplier: Pointer of the multiplier.
  * @param  pResult: Pointer to return the result.
  *
  * @retval Quaternion_OpRes_t.
  */
Quaternion_OpRes_t Quaternion_Multiply(Quaternion_Quaternion_t *pMultiplicand, 
                                       Quaternion_Quaternion_t *pMultiplier, 
                                       Quaternion_Quaternion_t *pResult);

/**
  * @brief  Divides terms.  
  *
  * @param  pDividend: Pointer of the dividend.
  * @param  pDivisor: Pointer of the divisor.
  * @param  pResult: Pointer to return the result.
  *
  * @retval Quaternion_OpRes_t.
  */
Quaternion_OpRes_t Quaternion_Divide(Quaternion_Quaternion_t *pDividend, 
                                     Quaternion_Quaternion_t *pDivisor, 
                                     Quaternion_Quaternion_t *pResult);

  /**
  * @brief  Normalizes the term. 
  *
  * @param  pTerm: Pointer of the term.
  * @param  pResult: Pointer to return the result.
  *
  * @retval Quaternion_OpRes_t.
  */
Quaternion_OpRes_t Quaternion_NormalizeQuaternion(Quaternion_Quaternion_t *pTerm, 
                                                  Quaternion_Quaternion_t *pResult);

/**
  * @brief  Gets euler angle equivalent of the quaternion which represents orientation. 
  *
  * @param  pOrientation: Pointer of the orientation.
  * @param  pLastEulerAngles: Pointer to the last euler angles.
  * @param  pResult: Pointer to return the result.
  *
  * @retval Quaternion_OpRes_t.
  */
Quaternion_OpRes_t Quaternion_CalculateEulerAngles(Quaternion_Quaternion_t *pOrientation,
                                                   Quaternion_EulerAngles_t *pLastEulerAngles,
                                                   Quaternion_EulerAngles_t *pResult);
/**
  * @brief  Calculates inverse of a term. 
  *
  * @param  pTerm: Pointer of the term.
  * @param  pResult: Pointer to return the inverse.
  *
  * @retval Quaternion_OpRes_t.
  */
Quaternion_OpRes_t Quaternion_CalculateInverse(Quaternion_Quaternion_t *pTerm, 
                                               Quaternion_Quaternion_t *pResult);

#ifdef __cplusplus
}
#endif

#endif