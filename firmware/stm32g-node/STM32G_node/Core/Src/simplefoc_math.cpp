#include "simplefoc_math.h"

#include <math.h>

float SimpleFoc_NormalizeAngle(float angle)
{
  float normalized = fmodf(angle, SIMPLEFOC_2PI);

  if (normalized < 0.0f)
  {
    normalized += SIMPLEFOC_2PI;
  }

  return normalized;
}

void SimpleFoc_SinCos(float angle, float *sine_value, float *cosine_value)
{
  if (sine_value != 0)
  {
    *sine_value = sinf(angle);
  }

  if (cosine_value != 0)
  {
    *cosine_value = cosf(angle);
  }
}

float SimpleFoc_Constrain(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }

  if (value > max_value)
  {
    return max_value;
  }

  return value;
}
