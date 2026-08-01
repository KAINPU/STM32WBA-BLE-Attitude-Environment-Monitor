#include "sensor_calibration.h"

#include <math.h>
#include <string.h>

#define ACCELERATION_G                 (1.0f)
#define ACCEL_SAMPLES_PER_FACE         (64U)
#define GYRO_CALIBRATION_SAMPLES       (500U)
#define CALIBRATION_EPSILON             (1.0e-6f)

static void reset_accumulator(SensorCalibration_t *calibration)
{
  calibration->sample_count = 0U;
  calibration->accumulator[0] = 0.0f;
  calibration->accumulator[1] = 0.0f;
  calibration->accumulator[2] = 0.0f;
}

void SensorCalibration_Init(SensorCalibration_t *calibration)
{
  if (calibration == NULL)
  {
    return;
  }

  memset(calibration, 0, sizeof(*calibration));
  calibration->state = SENSOR_CAL_IDLE;
  calibration->samples_per_face = ACCEL_SAMPLES_PER_FACE;
  calibration->gyro_samples = GYRO_CALIBRATION_SAMPLES;
  calibration->accel_scale[0] = 1.0f;
  calibration->accel_scale[1] = 1.0f;
  calibration->accel_scale[2] = 1.0f;
}

SensorCalibrationState_t SensorCalibration_Button(SensorCalibration_t *calibration)
{
  if (calibration == NULL)
  {
    return SENSOR_CAL_IDLE;
  }

  if (calibration->state == SENSOR_CAL_IDLE)
  {
    calibration->face_index = 0U;
    reset_accumulator(calibration);
    calibration->state = SENSOR_CAL_ACCEL_COLLECT;
  }
  else if (calibration->state == SENSOR_CAL_ACCEL_WAIT_NEXT)
  {
    reset_accumulator(calibration);
    calibration->face_index++;
    if (calibration->face_index < 6U)
    {
      calibration->state = SENSOR_CAL_ACCEL_COLLECT;
    }
    else
    {
      float positive;
      float negative;
      uint8_t axis;

      for (axis = 0U; axis < 3U; axis++)
      {
        positive = calibration->accel_mean[axis * 2U][axis];
        negative = calibration->accel_mean[axis * 2U + 1U][axis];
        calibration->accel_bias[axis] = (positive + negative) * 0.5f;
        if (fabsf(positive - negative) > CALIBRATION_EPSILON)
        {
          calibration->accel_scale[axis] = (2.0f * ACCELERATION_G) / (positive - negative);
        }
        else
        {
          calibration->accel_scale[axis] = 1.0f;
        }
      }
      reset_accumulator(calibration);
      calibration->state = SENSOR_CAL_GYRO_COLLECT;
    }
  }
  else if (calibration->state == SENSOR_CAL_COMPLETE)
  {
    SensorCalibration_Init(calibration);
  }

  return calibration->state;
}

void SensorCalibration_ProcessSample(SensorCalibration_t *calibration,
                                     const float acceleration_g[3],
                                     const float gyro_dps[3])
{
  uint8_t axis;

  if ((calibration == NULL) || (acceleration_g == NULL) || (gyro_dps == NULL))
  {
    return;
  }

  if (calibration->state == SENSOR_CAL_ACCEL_COLLECT)
  {
    for (axis = 0U; axis < 3U; axis++)
    {
      calibration->accumulator[axis] += acceleration_g[axis];
    }
    calibration->sample_count++;
    if (calibration->sample_count >= calibration->samples_per_face)
    {
      for (axis = 0U; axis < 3U; axis++)
      {
        calibration->accel_mean[calibration->face_index][axis] =
            calibration->accumulator[axis] / (float)calibration->sample_count;
      }
      calibration->state = SENSOR_CAL_ACCEL_WAIT_NEXT;
    }
  }
  else if (calibration->state == SENSOR_CAL_GYRO_COLLECT)
  {
    for (axis = 0U; axis < 3U; axis++)
    {
      calibration->accumulator[axis] += gyro_dps[axis];
    }
    calibration->sample_count++;
    if (calibration->sample_count >= calibration->gyro_samples)
    {
      for (axis = 0U; axis < 3U; axis++)
      {
        calibration->gyro_bias[axis] =
            calibration->accumulator[axis] / (float)calibration->sample_count;
      }
      calibration->state = SENSOR_CAL_COMPLETE;
    }
  }
}

void SensorCalibration_Apply(const SensorCalibration_t *calibration,
                             const float acceleration_g[3],
                             const float gyro_dps[3],
                             float corrected_acceleration_g[3],
                             float corrected_gyro_dps[3])
{
  uint8_t axis;

  if ((calibration == NULL) || (acceleration_g == NULL) || (gyro_dps == NULL) ||
      (corrected_acceleration_g == NULL) || (corrected_gyro_dps == NULL))
  {
    return;
  }

  for (axis = 0U; axis < 3U; axis++)
  {
    corrected_acceleration_g[axis] =
        (acceleration_g[axis] - calibration->accel_bias[axis]) * calibration->accel_scale[axis];
    corrected_gyro_dps[axis] = gyro_dps[axis] - calibration->gyro_bias[axis];
  }
}

uint8_t SensorCalibration_IsComplete(const SensorCalibration_t *calibration)
{
  return (calibration != NULL) && (calibration->state == SENSOR_CAL_COMPLETE);
}
