#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  SENSOR_CAL_IDLE = 0,
  SENSOR_CAL_ACCEL_COLLECT,
  SENSOR_CAL_ACCEL_WAIT_NEXT,
  SENSOR_CAL_GYRO_COLLECT,
  SENSOR_CAL_COMPLETE
} SensorCalibrationState_t;

typedef struct
{
  SensorCalibrationState_t state;
  uint8_t face_index;
  uint16_t sample_count;
  uint16_t samples_per_face;
  uint16_t gyro_samples;
  float accel_mean[6][3];
  float accel_bias[3];
  float accel_scale[3];
  float gyro_bias[3];
  float accumulator[3];
} SensorCalibration_t;

void SensorCalibration_Init(SensorCalibration_t *calibration);
SensorCalibrationState_t SensorCalibration_Button(SensorCalibration_t *calibration);
void SensorCalibration_ProcessSample(SensorCalibration_t *calibration,
                                     const float acceleration_g[3],
                                     const float gyro_dps[3]);
void SensorCalibration_Apply(const SensorCalibration_t *calibration,
                             const float acceleration_g[3],
                             const float gyro_dps[3],
                             float corrected_acceleration_g[3],
                             float corrected_gyro_dps[3]);
uint8_t SensorCalibration_IsComplete(const SensorCalibration_t *calibration);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_CALIBRATION_H */
