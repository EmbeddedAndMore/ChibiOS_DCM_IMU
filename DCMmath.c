/*
 * DCMmath.c
 *
 *  Created on: Apr 1, 2015
 *      Author: Mohamad Armoun
 */

#include "ch.h"
#include "hal.h"
#include "DCMmath.h"
#include "DP_Math.h"
#include "DP_Math_Vector2f.h"
#include <math.h>


Vector3f a,b,c;

void from_euler(Matrix3f *mat,float roll,float pitch,float yaw)
{

    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float sr = sinf(roll);
    float cr = cosf(roll);
    float sy = sinf(yaw);
    float cy = cosf(yaw);

    mat->a.x = cp * cy;
    mat->a.y = (sr * sp * cy) - (cr * sy);
    mat->a.z = (cr * sp * cy) + (sr * sy);
    mat->b.x = cp * sy;
    mat->b.y = (sr * sp * sy) + (cr * cy);
    mat->b.z = (cr * sp * sy) - (sr * cy);
    mat->c.x = -sp;
    mat->c.y = sr * cp;
    mat->c.z = cr * cp;
}

void to_euler(Matrix3f mat,float *roll, float *pitch, float *yaw)
{
      if (pitch != NULL) {
          *pitch = -safe_asin(mat.c.x);
      }
      if (roll != NULL) {
          *roll = atan2f(mat.c.y, mat.c.z);

      }
      if (yaw != NULL) {
          *yaw = atan2f(mat.b.x, mat.a.x);
      }
}


// apply an additional rotation from a body frame gyro vector
// to a rotation matrix.
// Eqn. 10
void Mat_rotate(Matrix3f *mat,Vector3f g)
{
    //Matrix3f temp_matrix;
    mat->a.x += (mat->a.y * g.z) - (mat->a.z * g.y);
    mat->a.y += (mat->a.z * g.x) - (mat->a.x * g.z);
    mat->a.z += (mat->a.x * g.y) - (mat->a.y * g.x);
    mat->b.x += (mat->b.y * g.z) - (mat->b.z * g.y);
    mat->b.y += (mat->b.z * g.x) - (mat->b.x * g.z);
    mat->b.z += (mat->b.x * g.y) - (mat->b.y * g.x);
    mat->c.x += (mat->c.y * g.z) - (mat->c.z * g.y);
    mat->c.y += (mat->c.z * g.x) - (mat->c.x * g.z);
    mat->c.z += (mat->c.x * g.y) - (mat->c.y * g.x);

}



// apply an additional rotation from a body frame gyro vector
// to a rotation matrix.
//Eqn 10.
void Mat_rotateXY(Matrix3f *mat,Vector3f g)
{
    mat->a.x += -mat->a.z * g.y;
    mat->a.y += mat->a.z * g.x;
    mat->a.z += mat->a.x * g.y - mat->a.y * g.x;
    mat->b.x += -mat->b.z * g.y;
    mat->b.y += mat->b.z * g.x;
    mat->b.z += mat->b.x * g.y - mat->b.y * g.x;
    mat->c.x += -mat->c.z * g.y;
    mat->c.y += mat->c.z * g.x;
    mat->c.z += mat->c.x * g.y - mat->c.y * g.x;
}

void Mat_rotateXYinv(Matrix3f *mat,Vector3f g)
{
    mat->a.x =   mat->a.z * g.y;
    mat->a.y = - mat->a.z * g.x;
    mat->a.z = - mat->a.x * g.y + mat->a.y * g.x;
    mat->b.x =   mat->b.z * g.y;
    mat->b.y = - mat->b.z * g.x;
    mat->b.z = - mat->b.x * g.y + mat->b.y * g.x;
    mat->c.x =   mat->c.z * g.y;
    mat->c.y = - mat->c.z * g.x;
    mat->c.z = - mat->c.x * g.y + mat->c.y * g.x;
}



// multiplication by a vector
Vector3f mul_Mat3f_to_Vec3f(Matrix3f mat,Vector3f v)
{
    Vector3f tmp_vect;
    tmp_vect.x = mat.a.x *v.x + mat.a.y*v.y + mat.a.z*v.z;
    tmp_vect.y = mat.b.x *v.x + mat.b.y*v.y + mat.b.z*v.z;
    tmp_vect.z = mat.c.x *v.x + mat.c.y*v.y + mat.c.z*v.z;
    return tmp_vect;
}

// multiplication by a vector, extracting only the xy components
Vector2f mat_mulXY(Matrix3f mat,Vector3f v)
{
    Vector2f tmp_vect;
    tmp_vect.x  = mat.a.x * v.x + mat.a.y * v.y + mat.a.z * v.z;
    tmp_vect.y  = mat.b.x * v.x + mat.b.y * v.y + mat.b.z * v.z;
    return tmp_vect;
}

// multiplication of transpose by a vector
Vector3f mul_transpose(Matrix3f mat,Vector3f v)
{
    Vector3f tmp_vect;
    tmp_vect.x = mat.a.x * v.x + mat.b.x * v.y + mat.c.x * v.z;
    tmp_vect.y = mat.a.y * v.x + mat.b.y * v.y + mat.c.y * v.z;
    tmp_vect.z = mat.a.z * v.x + mat.b.z * v.y + mat.c.z * v.z;
    return tmp_vect;
}

// multiplication by another Matrix3f
Matrix3f mult_Matrix3f(Matrix3f fm,Matrix3f sm)
{
     Matrix3f tmp_matrix;
     tmp_matrix.a.x = fm.a.x * sm.a.x + fm.a.y * sm.b.x + fm.a.z * sm.c.x;
     tmp_matrix.a.y = fm.a.x * sm.a.y + fm.a.y * sm.b.y + fm.a.z * sm.c.y;
     tmp_matrix.a.z = fm.a.x * sm.a.z + fm.a.y * sm.b.z + fm.a.z * sm.c.z;

     tmp_matrix.b.x = fm.b.x * sm.a.x + fm.b.y * sm.b.x + fm.b.z * sm.c.x;
     tmp_matrix.b.y = fm.b.x * sm.a.y + fm.b.y * sm.b.y + fm.b.z * sm.c.y;
     tmp_matrix.b.z = fm.b.x * sm.a.z + fm.b.y * sm.b.z + fm.b.z * sm.c.z;

     tmp_matrix.c.x = fm.c.x * sm.a.x + fm.c.y * sm.b.x + fm.c.z * sm.c.x;
     tmp_matrix.c.y = fm.c.x * sm.a.y + fm.c.y * sm.b.y + fm.c.z * sm.c.y;
     tmp_matrix.c.z = fm.c.x * sm.a.z + fm.c.y * sm.b.z + fm.c.z * sm.c.z;

     return tmp_matrix;

}


Matrix3f transposed(void)
{
    Matrix3f tmp_mat;
    tmp_mat.a.x = a.x; tmp_mat.a.y = b.x ; tmp_mat.a.z = c.x ;
    tmp_mat.b.x = a.y; tmp_mat.b.y = b.y ; tmp_mat.b.z = c.y ;
    tmp_mat.c.x = a.z; tmp_mat.c.y = b.z ; tmp_mat.c.z = c.z ;
    return tmp_mat;
}

void zero(Matrix3f *m)
{
    m->a.x = 0 ; m->a.y = 0 ; m->a.z = 0;
    m->b.x = 0 ; m->b.y = 0 ; m->b.z = 0;
    m->c.x = 0 ; m->c.y = 0 ; m->c.z = 0;
}

// extract x column
Vector3f matrix_colX(Matrix3f mat)
{
  Vector3f tmp_vect = {mat.a.x,mat.b.x,mat.c.x};
  return tmp_vect;
}

// extract y column
Vector3f matrix_colY(Matrix3f mat)
{
    Vector3f tmp_vect = {mat.a.y,mat.b.y,mat.c.y};
    return tmp_vect;
}

// extract z column
Vector3f matrix_colZ(Matrix3f mat)
{
  Vector3f tmp_vect = {mat.a.y,mat.b.y,mat.c.y};
    return tmp_vect;
}
bool_t matrix_isnan(Matrix3f m)
{
  return Vector3f_isnan(m.a) || Vector3f_isnan(m.b) || Vector3f_isnan(m.c);
}

void matrix_identity(Matrix3f *m)
{
          m->a.x = m->b.y = m->c.z = 1;
          m->a.y = m->a.z = 0;
          m->b.x = m->b.z = 0;
          m->c.x = m->c.y = 0;
}


