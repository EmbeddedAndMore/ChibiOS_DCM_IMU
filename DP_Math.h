/*
 * DP_Math.h
 *
 *  Created on: Apr 1, 2015
 *      Author: Mohamad Armoun
 */

#ifndef DP_MATH_H_
#define DP_MATH_H_
#include "ch.h"

typedef struct VECTOR{
  volatile float x;
  volatile float y;
  volatile float z;
}Vector3f;



//____ functions ____
float safe_sqrt(float v);
float safe_asin(float v);


Vector3f Vector3f_Sum(Vector3f first ,Vector3f sec );
Vector3f Vector3f_Sub(Vector3f first ,Vector3f sec);
float Vector3f_dot_prod(Vector3f first ,Vector3f sec);
Vector3f Vector3f_cross_prod(Vector3f first ,Vector3f sec);
//void Vector_Mult(Vector3f *dist ,Vector3f first ,Vector3f sec );
Vector3f Vector3f_Mult_to_float(Vector3f first ,float sec);
void Vector3f_zero(Vector3f *vect);
float Vector3f_lenght(Vector3f v);
float pythagorous3(float a, float b ,float c);
void Vector3f_normalize(Vector3f *v);
bool_t Vector3f_isnan(Vector3f v);
bool_t Vector3f_isinf(Vector3f v);
bool_t Vector3f_iszero(Vector3f v);
Vector3f  Vector3f_div_to_float(Vector3f v, float f);
//_____________________________ other _______________________________
float constrain_float(float amt, float low, float high);
#endif /* DP_MATH_H_ */
