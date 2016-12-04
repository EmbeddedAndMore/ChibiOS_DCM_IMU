/*
 * DP_Math_Vector2f.h
 *
 *  Created on: Apr 5, 2015
 *      Author: Mohamad Armoun
 */

#ifndef DP_MATH_VECTOR2F_H_
#define DP_MATH_VECTOR2F_H_
#include "ch.h"

typedef struct VECTOR1{
  float x;
  float y;
}Vector2f;


float pythagorous2(float a, float b);
float Vector2f_Lenght(Vector2f v);
void Vector2f_normalize(Vector2f *v);
bool_t Vector2f_isinf(Vector2f v);
float Vector2f_dot_prod(Vector2f first,Vector2f sec);
float Vector2f_Cross_prod(Vector2f first,Vector2f sec);

#endif /* DP_MATH_VECTOR2F_H_ */
