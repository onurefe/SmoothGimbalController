/**
  ******************************************************************************
  * @file    quaternion.c
  * @author  Onur Efe
  * @date    21.04.2016
  * @brief   Quaternion class interface. Class includes functions related with 
  *          quaternions.
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
/* Includes ------------------------------------------------------------------*/
#include "quaternion.h"
#include <math.h>

/* Private constants ---------------------------------------------------------*/
#define CONTROL_PERIOD                  (1.0f / CONTROL_FREQ)
    
#define CONVERGENCE_TO_ZERO_VALUE       0.0001f

/* Private functions ---------------------------------------------------------*/
static float EffAtan2(float y, float x);

static float EffAsin(float value);

static void multiply(Quaternion_Quaternion_t *pMultiplicand,
                     Quaternion_Quaternion_t *pMultiplier,
                     Quaternion_Quaternion_t *pResult);

static void conjugate(Quaternion_Quaternion_t *pQuaternion,
                      Quaternion_Quaternion_t *pResult);

static void norm(Quaternion_Quaternion_t *pQuaternion, float *pResult);

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
                                     Quaternion_Quaternion_t *pResult)
{
  Quaternion_Quaternion_t inverted_rot_quat;
  Quaternion_Quaternion_t result;
  
  /* Calculate the inverse of unit rotation quaternion */
  conjugate(pRotQuat, &inverted_rot_quat);
  
  /* Calculate the rotated vector */
  multiply(pTerm, pRotQuat, &result);
  multiply(&result, &inverted_rot_quat, &result);
  
  /* Set result vector */
  pResult->w = result.w;
  pResult->x = result.x;
  pResult->y = result.y;
  pResult->z = result.z;
  
  return QUATERNION_OP_RES_SUCCESS;
}

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
                                       Quaternion_Quaternion_t *pResult)
{
  /* Multiply the multiplicand with the multiplier */
  multiply(pMultiplicand, pMultiplier, pResult);

  return QUATERNION_OP_RES_SUCCESS;
}

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
                                     Quaternion_Quaternion_t *pResult)
{
  Quaternion_Quaternion_t inverse_of_divisor;
  
  /* Invert the divisor */
  conjugate(pDivisor, &inverse_of_divisor);

  /* Multiply the dividend with the inverse of divisor */
  multiply(pDividend, &inverse_of_divisor, pResult); 
  
  return QUATERNION_OP_RES_SUCCESS;
}

/**
  * @brief  Normalizes the term. 
  *
  * @param  pTerm: Pointer of the term.
  * @param  pResult: Pointer to return the result.
  *
  * @retval Quaternion_OpRes_t.
  */
Quaternion_OpRes_t Quaternion_NormalizeQuaternion(Quaternion_Quaternion_t *pTerm, 
                                                  Quaternion_Quaternion_t *pResult)
{
  float norm_of_term;
  
  /* Calculate norm of the term */
  norm(pTerm, &norm_of_term);
  
  /* Divide term with norm of it */
  pResult->w = pTerm->w / norm_of_term;
  pResult->x = pTerm->x / norm_of_term;
  pResult->y = pTerm->y / norm_of_term;
  pResult->z = pTerm->z / norm_of_term;
  
  return QUATERNION_OP_RES_SUCCESS;
}

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
                                                   Quaternion_EulerAngles_t *pResult)
{
  Quaternion_EulerAngles_t euler_angles;
  
  float p0, p1, p2, p3, esign;
  
  /* Set p0, p1, p2, p3 */
  p0 = pOrientation->w;
  p1 = pOrientation->x;
  p2 = pOrientation->y;
  p3 = pOrientation->z;
  esign = -1.0f;

  /* Calculate euler angles */
  euler_angles.first = EffAtan2((2.0f * (p0 * p3 - esign * p1 * p2)),
                                (1.0f - 2.0f * (p2 * p2 + p3 * p3)));
      
  euler_angles.second = EffAsin(2.0f * (p0 * p2 + (esign * p1 * p3)));
      
  euler_angles.third = EffAtan2((2.0f * (p0 * p1 - esign * p2 * p3)),
                                (1.0f - 2.0f * (p1 * p1 + p2 * p2)));
  
  /* Apply linearization and some limitations on euler angles */
  for (uint8_t i = 0; i < 3; i++)
  {
    while ((((float *)&euler_angles)[i] - ((float *)pLastEulerAngles)[i]) > PI)
    {
      ((float *)&euler_angles)[i] -= (2.0f * PI); 
    }
    
    while ((((float *)&euler_angles)[i] - ((float *)pLastEulerAngles)[i]) < -PI)
    {
      ((float *)&euler_angles)[i] += (2.0f * PI);
    }
  }

  /* Set result */
  pResult->first = euler_angles.first;
  pResult->second = euler_angles.second;
  pResult->third = euler_angles.third;
  
  return QUATERNION_OP_RES_SUCCESS;
}

/**
  * @brief  Calculates inverse of a term. 
  *
  * @param  pTerm: Pointer of the term.
  * @param  pResult: Pointer to return the inverse.
  *
  * @retval Quaternion_OpRes_t.
  */
Quaternion_OpRes_t Quaternion_CalculateInverse(Quaternion_Quaternion_t *pTerm, 
                                               Quaternion_Quaternion_t *pResult)
{
  /* Calculate the conjugate of term */
  conjugate(pTerm, pResult);

  return QUATERNION_OP_RES_SUCCESS;
}

/**
  * @brief  Effective arctan2 approximation.
  * 
  * @param  y: Y component.
  * @param  x: X component.
  *
  * @retval Result.
  */
static float EffAtan2(float y, float x)
{
  float r;
  float r2;
  float offset_value;
  float result;
  
  const float c1 = 1.6867629106f;
  const float c2 = 0.4378497304f;
  const float c3 = 1.6867633134f;
  const float tansixthpi = 0.5773502691f;
  const float tantwelfthpi = 0.2679491924f;
  
  uint16_t sign = FALSE;
  uint16_t offset = FALSE;
  uint16_t region = FALSE;
  uint16_t complement = FALSE;
  
  /* If the x value is equal to zero, return PI/2 or -PI/2 */
  if (fabs(x) < CONVERGENCE_TO_ZERO_VALUE)
  {
    if (y >= 0.0f)
    {
      return (PI / 2.0f);
    }
    else 
    {
      return (-PI / 2.0f);
    }
  }
  
  /* If the y value is equal to zero, return 0 or PI */
  if (y == 0.0f)
  {
    if (x > 0.0f)
    {
      return 0.0f;
    }
    else
    {
      return PI;
    }
  }
  
  /* Check if the quadrant is second or third, to extend the expansion */
  if ((y > 0.0f) && (x < 0.0f))
  {
    offset = TRUE;
    offset_value = PI;
  }
  
  if ((y < 0.0f) && (x < 0.0f))
  {
    offset = TRUE;
    offset_value = -PI;
  }

  r = y / x;
  
  /* Negate r, if it's negative */
  if (r < 0.0f)
  {
    r = -r;
    sign = TRUE;
  }
  
  /* Take r's multiplicative complement, if its bigger than 1 */
  if (r > 1.0f)
  {
    r = 1.0f / r;
    complement = TRUE;
  }
    
  /* Reduce argument under PI/12 */
  if (r > tantwelfthpi)
  {
    r = (r - tansixthpi) / (1.0f + tansixthpi * r);
    region = TRUE;
  }
  
  /* Calculate result */
  r2 = r * r;
  result = r * ((c1 + r2 * c2) / (c3 + r2));
  
  /* Apply corrections */
  if (region == TRUE)
  {
    result += (PI / 6.0f);
  }
  
  if (complement == TRUE)
  {
    result = (PI / 2.0f) - result;
  }
  
  if (sign == TRUE)
  {
    result = -result;
  }
  
  if (offset == TRUE)
  {
    result += offset_value;
  }
  
  return result;
}

/**
  * @brief  Effective arcsin approximation.
  * 
  * @param  x: Term of the arcsin function.
  *
  * @retval Result.
  */
static float EffAsin(float x)
{
  float result;
  float x2, x3;
  uint16_t sign = FALSE;
  
  /* Calculate required powers of x */
  x2 = x * x;
  x3 = x2 * x;
  
  /* If x is a negative number, make sign flag true and negate the x */ 
  if (x < 0.0f)
  {
    sign = TRUE;
    x = -x;
  }
  
  /* If absolute value of x is greater than 1, return */
  if (x > 1.0f)
  {
    return 0.0f;
  }
  
  /* Calculate result */
  result = (PI / 2.0f) - (sqrt(1.0f - x) * (1.5707288f + (-0.2121144f * x) + \
                                            (0.0742610f * x2) + (-0.0187293f * x3)));
  
  /* If sign flag is set, negate the result */
  if (sign == TRUE)
  {
    result = -result;
  }
  
  return result;
}

/**
  * @brief  Multiplies terms. 
  *
  * @param  pMultiplicand: Pointer of the multiplicand.
  * @param  pMultiplier: Pointer of the multiplier.
  * @param  pResult: Pointer to return the result.
  *
  * @retval None.
  */
static void multiply(Quaternion_Quaternion_t *pMultiplicand, Quaternion_Quaternion_t *pMultiplier, 
                     Quaternion_Quaternion_t *pResult)
{
  /* pResult can be equal to pMultiplicand, so use a temporary
     result quaternion to prevent possible errors */
  Quaternion_Quaternion_t result;
  
  /* Calculate product */
  result.w = (pMultiplicand->w * pMultiplier->w) + (-(pMultiplicand->x * pMultiplier->x)) + \
             (-(pMultiplicand->y * pMultiplier->y)) + (-(pMultiplicand->z * pMultiplier->z));
  
  result.x = (pMultiplicand->w * pMultiplier->x) + (pMultiplicand->x * pMultiplier->w) + \
             (pMultiplicand->y * pMultiplier->z) + (-(pMultiplicand->z * pMultiplier->y));
  
  result.y = (pMultiplicand->w * pMultiplier->y) + (-(pMultiplicand->x * pMultiplier->z)) + \
             (pMultiplicand->y * pMultiplier->w) + (pMultiplicand->z * pMultiplier->x);
  
  result.z = (pMultiplicand->w * pMultiplier->z) + (pMultiplicand->x * pMultiplier->y) + \
             (-(pMultiplicand->y * pMultiplier->x)) + (pMultiplicand->z * pMultiplier->w);
  
  /* Store it to the register pointed with pResult pointer */
  pResult->w = result.w;
  pResult->x = result.x;
  pResult->y = result.y;
  pResult->z = result.z;
}

/**
  * @brief  Calculates conjugate of a term. 
  *
  * @param  pTerm: Pointer of the term.
  * @param  pResult: Pointer to return the result.
  *
  * @retval None.
  */
static void conjugate(Quaternion_Quaternion_t *pTerm, Quaternion_Quaternion_t *pResult)
{
  /* Store the conjugated quaternion to result variable */
  pResult->w = pTerm->w;
  pResult->x = (-pTerm->x);
  pResult->y = (-pTerm->y);
  pResult->z = (-pTerm->z);
}

/**
  * @brief  Calculates norm of a term. 
  *
  * @param  pTerm: Pointer of the term.
  * @param  pResult: Pointer to return the result.
  *
  * @retval None.
  */
static void norm(Quaternion_Quaternion_t *pTerm, float *pResult)
{
  float sum_of_squared_components;
  
  /* Norm squared */
  sum_of_squared_components = (pTerm->w * pTerm->w);
  sum_of_squared_components += (pTerm->x * pTerm->x);
  sum_of_squared_components += (pTerm->y * pTerm->y);
  sum_of_squared_components += (pTerm->z * pTerm->z);
  
  *pResult = (sqrt(sum_of_squared_components));
}