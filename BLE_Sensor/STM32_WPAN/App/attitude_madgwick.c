#include "attitude_madgwick.h"

#include <math.h>
#include <stddef.h>

#define MADGWICK_PI       (3.14159265358979323846f)
#define MADGWICK_EPSILON  (1.0e-9f)

static float inv_sqrt(float value)
{
  if (value <= MADGWICK_EPSILON)
  {
    return 0.0f;
  }

  return 1.0f / sqrtf(value);
}

void Madgwick_Init(Madgwick_t *filter, float beta)
{
  if (filter == NULL)
  {
    return;
  }

  filter->beta = beta;
  Madgwick_Reset(filter);
}

void Madgwick_Reset(Madgwick_t *filter)
{
  if (filter == NULL)
  {
    return;
  }

  filter->q0 = 1.0f;
  filter->q1 = 0.0f;
  filter->q2 = 0.0f;
  filter->q3 = 0.0f;
}

void Madgwick_UpdateIMU(Madgwick_t *filter,
                        float gx,
                        float gy,
                        float gz,
                        float ax,
                        float ay,
                        float az,
                        float dt)
{
  float q0;
  float q1;
  float q2;
  float q3;
  float recip_norm;
  float s0;
  float s1;
  float s2;
  float s3;
  float q_dot1;
  float q_dot2;
  float q_dot3;
  float q_dot4;
  float two_q0;
  float two_q1;
  float two_q2;
  float two_q3;
  float four_q0;
  float four_q1;
  float four_q2;
  float eight_q1;
  float eight_q2;
  float q0q0;
  float q1q1;
  float q2q2;
  float q3q3;

  if ((filter == NULL) || (dt <= 0.0f))
  {
    return;
  }

  q0 = filter->q0;
  q1 = filter->q1;
  q2 = filter->q2;
  q3 = filter->q3;

  q_dot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  q_dot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  q_dot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  q_dot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  if ((ax != 0.0f) || (ay != 0.0f) || (az != 0.0f))
  {
    recip_norm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= recip_norm;
    ay *= recip_norm;
    az *= recip_norm;

    two_q0 = 2.0f * q0;
    two_q1 = 2.0f * q1;
    two_q2 = 2.0f * q2;
    two_q3 = 2.0f * q3;
    four_q0 = 4.0f * q0;
    four_q1 = 4.0f * q1;
    four_q2 = 4.0f * q2;
    eight_q1 = 8.0f * q1;
    eight_q2 = 8.0f * q2;
    q0q0 = q0 * q0;
    q1q1 = q1 * q1;
    q2q2 = q2 * q2;
    q3q3 = q3 * q3;

    s0 = four_q0 * q2q2 + two_q2 * ax + four_q0 * q1q1 - two_q1 * ay;
    s1 = four_q1 * q3q3 - two_q3 * ax + 4.0f * q0q0 * q1 -
         two_q0 * ay - four_q1 + eight_q1 * q1q1 + eight_q1 * q2q2 +
         four_q1 * az;
    s2 = 4.0f * q0q0 * q2 + two_q0 * ax + four_q2 * q3q3 -
         two_q3 * ay - four_q2 + eight_q2 * q1q1 + eight_q2 * q2q2 +
         four_q2 * az;
    s3 = 4.0f * q1q1 * q3 - two_q1 * ax + 4.0f * q2q2 * q3 -
         two_q2 * ay;

    recip_norm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recip_norm;
    s1 *= recip_norm;
    s2 *= recip_norm;
    s3 *= recip_norm;

    q_dot1 -= filter->beta * s0;
    q_dot2 -= filter->beta * s1;
    q_dot3 -= filter->beta * s2;
    q_dot4 -= filter->beta * s3;
  }

  q0 += q_dot1 * dt;
  q1 += q_dot2 * dt;
  q2 += q_dot3 * dt;
  q3 += q_dot4 * dt;

  recip_norm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  filter->q0 = q0 * recip_norm;
  filter->q1 = q1 * recip_norm;
  filter->q2 = q2 * recip_norm;
  filter->q3 = q3 * recip_norm;
}

void Madgwick_GetEuler(const Madgwick_t *filter,
                       float *roll_deg,
                       float *pitch_deg,
                       float *yaw_deg)
{
  if (filter == NULL)
  {
    return;
  }

  if (roll_deg != NULL)
  {
    *roll_deg = atan2f(2.0f * (filter->q0 * filter->q1 + filter->q2 * filter->q3),
                       1.0f - 2.0f * (filter->q1 * filter->q1 + filter->q2 * filter->q2)) *
                (180.0f / MADGWICK_PI);
  }

  if (pitch_deg != NULL)
  {
    float value = 2.0f * (filter->q0 * filter->q2 - filter->q3 * filter->q1);

    if (value > 1.0f)
    {
      value = 1.0f;
    }
    else if (value < -1.0f)
    {
      value = -1.0f;
    }

    *pitch_deg = asinf(value) * (180.0f / MADGWICK_PI);
  }

  if (yaw_deg != NULL)
  {
    *yaw_deg = atan2f(2.0f * (filter->q0 * filter->q3 + filter->q1 * filter->q2),
                      1.0f - 2.0f * (filter->q2 * filter->q2 + filter->q3 * filter->q3)) *
               (180.0f / MADGWICK_PI);
  }
}
