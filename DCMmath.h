/*
 * DCMmath.h
 *
 *  Created on: Apr 1, 2015
 *      Author: Mohamad Armoun
 */


#ifndef DCMMATH_H_
#define DCMMATH_H_
#include "DP_Math.h"
#include "DP_Math_Vector2f.h"

typedef struct MATRIX{

  Vector3f a;
  Vector3f b;
  Vector3f c;

}Matrix3f;



void from_euler(Matrix3f *mat,float roll,float pitch,float yaw);
void to_euler(Matrix3f mat,float *roll, float *pitch, float *yaw);
void Mat_rotate(Matrix3f *mat,Vector3f g);
void Mat_rotateXY(Matrix3f *mat,Vector3f g);
Vector2f mat_mulXY(Matrix3f mat,Vector3f v);
Vector3f mul_Mat3f_to_Vec3f(Matrix3f mat,Vector3f v);
void Mat_rotateXYinv(Matrix3f *mat,Vector3f g);
Vector3f mul_transpose(Matrix3f mat,Vector3f v);
Matrix3f mult_Matrix3f(Matrix3f fm,Matrix3f sm);
Matrix3f transposed(void);
void zero(Matrix3f *m);
Vector3f matrix_colX(Matrix3f mat);
Vector3f matrix_colY(Matrix3f mat);
Vector3f matrix_colZ(Matrix3f mat);
bool_t matrix_isnan(Matrix3f m);

void matrix_identity(Matrix3f *m);
#endif /* DCMMATH_H_ */
