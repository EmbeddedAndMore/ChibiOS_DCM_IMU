/*
 * DP_Math.c
 *
 *  Created on: Apr 1, 2015
 *      Author: Mohamad Armoun
 */

#include "DP_Math.h"
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "chprintf.h"


float safe_asin(float v)
{
    if(isnan(v)){
        return 0.0;
    }

    if (v >= 1.0f){
        return M_PI_2;
    }

    if (v <= -1.0f) {
        return -M_PI_2;
    }

    return asinf(v);
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
    float ret = sqrtf(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}



Vector3f Vector3f_Sum(Vector3f first ,Vector3f sec)
{
    Vector3f tmp;
    tmp.x = first.x + sec.x;
    tmp.y = first.y + sec.y;
    tmp.z = first.z + sec.z;
    return tmp;
}

Vector3f Vector3f_Sub(Vector3f first ,Vector3f sec)
{
    Vector3f dist;
    dist.x = first.x - sec.x;
    dist.y = first.y - sec.y;
    dist.z = first.z - sec.z;
    return dist;
}

// vector dot product
float Vector3f_dot_prod(Vector3f first ,Vector3f sec)
{
  float temp = (float)((first.x * sec.x) + (first.y * sec.y) + (first.z * sec.z));
  //chprintf((BaseSequentialStream *)&SD6,"%f\n",temp);// eg.18
  return temp;
}

// vector cross product
Vector3f Vector3f_cross_prod(Vector3f first ,Vector3f sec)
{
  Vector3f tmp= {(first.y*sec.z)-(first.z*sec.y),(first.z*sec.x)-(first.x*sec.z),(first.x*sec.y)-(first.y*sec.x)};
  return tmp;
}

Vector3f Vector3f_Mult_to_float(Vector3f first ,float sec)
{
    Vector3f tmp;
    tmp.x = first.x * sec;
    tmp.y = first.y * sec;
    tmp.z = first.z * sec;
    return tmp;
}
bool_t Vector3f_isnan(Vector3f v)
{
    return isnan(v.x) || isnan(v.y) || isnan(v.z);
}
bool_t Vector3f_isinf(Vector3f v)
{
    return isinf(v.x) || isinf(v.y) || isinf(v.z);
}

void Vector3f_zero(Vector3f *vect)
{
    vect->x = vect->y = vect->z = 0;
}

bool_t Vector3f_iszero(Vector3f v)
{
  return v.x == 0 && v.y == 0 && v.z == 0;
}


float Vector3f_lenght(Vector3f v)
{
  return pythagorous3(v.x , v.y , v.z);
}

// 3D vector length
float pythagorous3(float a, float b, float c) {
    return sqrtf(powf(a,2) + powf(b,2)+powf(c,2));
}

void Vector3f_normalize(Vector3f *v)
{
  float l = Vector3f_lenght((Vector3f)*v);
  v->x /= l; v->y /=l ; v->z/=l;
}

Vector3f  Vector3f_div_to_float(Vector3f v, float f)
{
    Vector3f temp;
    temp.x = v.x / f;
    temp.y = v.y / f;
    temp.z = v.z / f;
    return temp;
}



//__________________________ other _____________________________
// constrain a value
float constrain_float(float amt, float low, float high)
{
    // the check for NaN as a float prevents propogation of
    // floating point errors through any function that uses
    // constrain_float(). The normal float semantics already handle -Inf
    // and +Inf
    if (isnan(amt)) {
        return (low+high)*0.5f;
    }
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
