#ifndef SIMPLEFOC_MATH_H
#define SIMPLEFOC_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

#define SIMPLEFOC_PI 3.14159265359f
#define SIMPLEFOC_PI_2 1.57079632679f
#define SIMPLEFOC_PI_6 0.52359877559f
#define SIMPLEFOC_2PI 6.28318530718f
#define SIMPLEFOC_SQRT3_2 0.86602540378f
#define SIMPLEFOC_RPM_TO_RADS 0.10471975512f

float SimpleFoc_NormalizeAngle(float angle);
void SimpleFoc_SinCos(float angle, float *sine_value, float *cosine_value);
float SimpleFoc_Constrain(float value, float min_value, float max_value);

#ifdef __cplusplus
}
#endif

#endif
