#ifndef ATTITUDE_MADGWICK_H
#define ATTITUDE_MADGWICK_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
  float beta;
} Madgwick_t;

void Madgwick_Init(Madgwick_t *filter, float beta);
void Madgwick_Reset(Madgwick_t *filter);
void Madgwick_UpdateIMU(Madgwick_t *filter,
                        float gx,
                        float gy,
                        float gz,
                        float ax,
                        float ay,
                        float az,
                        float dt);
void Madgwick_GetEuler(const Madgwick_t *filter,
                       float *roll_deg,
                       float *pitch_deg,
                       float *yaw_deg);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_MADGWICK_H */
