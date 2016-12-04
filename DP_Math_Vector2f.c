/*
 * DP_Math_Vector2f.c
 *
 *  Created on: Apr 5, 2015
 *      Author: Mohamad Armoun
 */

#include "ch.h"
#include "DP_Math_Vector2f.h"
#include <math.h>

// 2D vector length
float pythagorous2(float a, float b) {

    return sqrtf(powf(a,2) + powf(b,2));
}


float Vector2f_Lenght(Vector2f v)
{
   return pythagorous2(v.x,v.y);
}


// normalizes this vector
void Vector2f_normalize(Vector2f *v)
{
     float f = Vector2f_Lenght(*v);
     v->x /= f;
     v->y /= f;
}

bool_t Vector2f_isinf(Vector2f v)
{
  return isinf(v.x) || isinf(v.y);
}

float Vector2f_dot_prod(Vector2f first,Vector2f sec)
{
  return first.x*sec.x + first.y*sec.y;
}

float Vector2f_Cross_prod(Vector2f first,Vector2f sec)
{
  return first.x*sec.y - first.y*sec.x;
}
