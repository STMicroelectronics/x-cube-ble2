/**
  ******************************************************************************
  * @file    st_data_parser.c
  * @author  SRA Application Team
  * @brief   Contains utilities for parsing data from ST Custom BLE
  *          characteristics.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "st_data_parser.h"

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/**
 * @brief  Get the timestamp value from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The timestamp value
 */
uint16_t Get_Timestamp(uint8_t data_length, uint8_t* value)
{
  uint16_t timestamp = 0; /* no value */

  if (data_length > 1) {
    timestamp = (value[1]<<8) | value[0];
  }

  return timestamp;
}

/**
 * @brief  Get the pressure value from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The pressure value [mBar]
 */
uint32_t Get_Pressure(uint8_t data_length, uint8_t* value)
{
  uint32_t  pressure = 0; /* no value */

  if ((data_length == ENV_DATA_LEN_LONG)  ||
      (data_length == ENV_DATA_LEN_SHORT) ||
      (data_length == PRESS_DATA_LEN))
  {
    pressure = (value[5]<<24) | (value[4]<<16) | (value[3]<<8) | value[2];
  }

  return pressure;
}

/**
 * @brief  Get the humidity value from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The humidity value [%]
 */
uint16_t Get_Humidity(uint8_t data_length, uint8_t* value)
{
  uint16_t humidity = 0; /* no value */

  if ((data_length == ENV_DATA_LEN_LONG) || (data_length == ENV_DATA_LEN_SHORT)) {
    humidity = (value[7]<<8) | value[6];
  }

  if (data_length == HUM_DATA_LEN) {
    humidity = (value[3]<<8) | value[2];
  }

  return humidity;
}

/**
 * @brief  Get the temperature value from the BLE data received
 * @param  The data length
 * @param  The received data
 * @param  The sensor from which the data is sent (1 or 2)
 * @retval The temperature value [Celtius degree]
 */
uint16_t Get_Temperature(uint8_t data_length, uint8_t* value, uint8_t sensor)
{
  uint16_t temperature = 0; /* no value */

  switch (data_length)
  {
  case ENV_DATA_LEN_LONG:
    if (sensor == 1) {
      temperature = (value[11]<<8) | value[10];
    }
    else if (sensor == 2) {
      temperature = (value[9]<<8) | value[8];
    }
    break;
  case ENV_DATA_LEN_SHORT:
    temperature = (value[7]<<8) | value[6];
    break;
  case TEMP_DATA_LEN:
    temperature = (value[3]<<8) | value[2];
    break;
  default:
    break;
  }

  return temperature;
}

/**
 * @brief  Get the CO value from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The CO value [%]
 */
uint32_t Get_CO(uint8_t data_length, uint8_t* value)
{
  uint32_t  co = 0; /* no value */

  if (data_length == CO_DATA_LEN) {
    co = (value[5]<<24) | (value[4]<<16) | (value[3]<<8) | value[2];
  }

  return co;
}

/**
 * @brief  Get the LED status from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The LED status [0=off, 1=on]
 */
uint8_t Get_LED_Status(uint8_t data_length, uint8_t* value)
{
  uint8_t  led_status = 0;

  if (data_length == LED_DATA_LEN) {
    led_status = value[2];
  }

  return led_status;
}

/**
 * @brief  Get the acceleration event type from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The acceleration event type (value, name)
 */
accEvent_t Get_Acc_Event(uint8_t data_length, uint8_t* value)
{
  accEvent_t acc_event;

  acc_event.type_val = 0;

  if (data_length == ACC_EVENT_DATA_LEN_SHORT) {
    acc_event.type_val = (value[3]<<8) | value[2];
  }
  else if (data_length == ACC_EVENT_DATA_LEN_LONG) {
    /* in this case value[2] is always 0 (with no meaning) */
    acc_event.type_val = (value[4]<<8) | value[3];
  }

  switch (acc_event.type_val)
  {
  case (ACC_NOT_USED):
    acc_event.type_name = "NOT_USED";
    break;
  case (ACC_6D_OR_TOP):
    acc_event.type_name = "6D_OR_TOP";
    break;
  case (ACC_6D_OR_LEFT):
    acc_event.type_name = "6D_OR_LEFT";
    break;
  case (ACC_6D_OR_BOTTOM):
    acc_event.type_name = "6D_OR_BOTTOM";
    break;
  case (ACC_6D_OR_RIGHT):
    acc_event.type_name = "6D_OR_RIGHT";
    break;
  case (ACC_6D_OR_UP):
    acc_event.type_name = "6D_OR_UP";
    break;
  case (ACC_6D_OR_DOWN):
    acc_event.type_name = "6D_OR_DOWN";
    break;
  case (ACC_TILT):
    acc_event.type_name = "TILT";
    break;
  case (ACC_FREE_FALL):
    acc_event.type_name = "FREE_FALL";
    break;
  case (ACC_SINGLE_TAP):
    acc_event.type_name = "SINGLE_TAP";
    break;
  case (ACC_DOUBLE_TAP):
    acc_event.type_name = "DOUBLE_TAP";
    break;
  case (ACC_WAKE_UP):
    acc_event.type_name = "WAKE_UP";
    break;
  default:
    acc_event.type_name = "NOT_RECOGNIZED";
    break;
  }

  return acc_event;
}

/**
 * @brief  Get the microphone audio level from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The microphone audio level [db]
 */
uint8_t Get_Mic_Audio_Level(uint8_t data_length, uint8_t* value)
{
  uint8_t mic_audio_level = 0;

  if (data_length == MIC_DATA_LEN) {
    mic_audio_level = value[2];
  }

  return mic_audio_level;
}

/**
 * @brief  Get the proximity value from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The proximity value [mm]
 */
uint16_t Get_Proximity(uint8_t data_length, uint8_t* value)
{
  uint16_t proximity = 0; /* no value */
  uint16_t data;

  if (data_length == PROX_DATA_LEN) {
    data = (value[3]<<8) | value[2];
    /* if first bit == 1 => high range value (i.e. PRX data from 53L0A1 or Bluecoin)*/
    if ((data & INT16_FIRST_BIT_MASK) >> 15 == 1) {
      data = (data & ~INT16_FIRST_BIT_MASK);
      if (data > HIGH_RANGE_DATA_MAX) {
        proximity = OUT_OF_RANGE_VALUE;
      }
      else {
        proximity = data;
      }
    }
    else {
      proximity = data;
    }
  }

  return proximity;
}

/**
 * @brief  Get the lux level value from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The lux level value
 */
uint16_t Get_Lux_Level(uint8_t data_length, uint8_t* value)
{
  uint16_t lux_level = 0; /* no value */

  if (data_length == LUX_DATA_LEN) {
    lux_level = (value[3]<<8) | value[2];
  }

  return lux_level;
}

/**
 * @brief  Get the ECompass type from the BLE data received
 *         (angle To Magnetic North in cents of degree [0.00 -> 359,99])
 * @param  The data length
 * @param  The received data
 * @retval The E-Compass value [0.00 -> 359,99]
 */
uint16_t Get_ECompass(uint8_t data_length, uint8_t* value)
{
  uint16_t e_compass = 0; /* no value */

  if (data_length == ECOMPASS_DATA_LEN) {
    e_compass = (value[3]<<8) | value[2];
  }

  return e_compass;
}

/**
 * @brief  Get the activity type from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The activity type (value, name)
 */
activityRec_t Get_Activity_Recognition(uint8_t data_length, uint8_t* value)
{
  activityRec_t act_rec;
  /* activity recognition types */
  char *MAR_TYPES[7] = {"NOACTIVITY",
                        "STATIONARY",
                        "WALKING",
                        "FASTWALKING",
                        "JOGGING",
                        "BIKING",
                        "DRIVING"};

  act_rec.type_val = 0;  /* no value */

  if (data_length == ACT_REC_DATA_LEN) {
    act_rec.type_val  = value[2];
  }
  act_rec.type_name = MAR_TYPES[act_rec.type_val];

  return act_rec;
}

/**
 * @brief  Get the carry position type from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The carry position type (value, name)
 */
carryPosition_t Get_Carry_Pos_Recognition(uint8_t data_length, uint8_t* value)
{
  carryPosition_t carry_pos;
  /* carry position types */
  char *MCP_TYPES[8] = {"UNKNOWN",
                        "ONDESK",
                        "INHAND",
                        "NEARHEAD",
                        "SHIRTPOCKET",
                        "TROUSERPOCKET",
                        "ARMSWING",
                        "JACKETPOCKET"};

  carry_pos.type_val = 0; /* no value */

  if (data_length == CARRY_POS_REC_DATA_LEN) {
    carry_pos.type_val = value[2];
  }
  carry_pos.type_name = MCP_TYPES[carry_pos.type_val];

  return carry_pos;
}

/**
 * @brief  Get the gesture type from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The gesture type (value, name)
 */
gestureRec_t Get_Gesture_Recognition(uint8_t data_length, uint8_t* value)
{
  gestureRec_t gest_rec;
  /* gesture types */
  char *MGR_TYPES[4] = {"NOGESTURE",
                        "PICKUP",
                        "GLANCE",
                        "WAKEUP"};
  gest_rec.type_val = 0; /* no value */

  if (data_length == GESTURE_REC_DATA_LEN) {
    gest_rec.type_val = value[2];
  }
  gest_rec.type_name = MGR_TYPES[gest_rec.type_val];

  return gest_rec;
}

/**
 * @brief  Get the motion intensity from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The motion intensity (value, name)
 */
intensityDet_t Get_Motion_Intensity(uint8_t data_length, uint8_t* value)
{
  intensityDet_t int_det;
  /* intensity detection types */
  char *MID_TYPES[11] = {"ON_DESK",
                         "BED_COUCH_PILLOW",
                         "LIGHT_MOVEMENTS",
                         "BIKING",
                         "TYPING_WRITING",
                         "TYPING_SLOW_WALKING",
                         "WASHING_HANDS_WALKING",
                         "FWALKING",
                         "FWALKING_JOGGING",
                         "FJOGGING_BRUSHING",
                         "SPRINTING"};

  int_det.type_val = 0; /* no value */

  if (data_length == MOT_INTENSITY_DATA_LEN) {
    int_det.type_val = value[2];
  }
  int_det.type_name = MID_TYPES[int_det.type_val];

  return int_det;
}

/**
 * @brief  Get the pedometer information from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The pedometer information
 */
pedometerInfo_t Get_Pedometer_Info(uint8_t data_length, uint8_t* value)
{
  pedometerInfo_t pedo_info;

  if (data_length == PEDOMETER_DATA_LEN) {
    pedo_info.steps = (value[5]<<24) | (value[4]<<16) | (value[3]<<8) | value[2];
    pedo_info.steps_min = (value[7]<<8) | value[6];
  }

  return pedo_info;
}

/**
 * @brief  Get the motion data (accelerometer, gyroscope and magnetometer)
 *         from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The motion data information (accelerometer [mg], gyroscope [dps] and
           magnetometer [mGa])
 */
motionData_t Get_Motion_Data(uint8_t data_length, uint8_t* value)
{
  motionData_t mot_data;

  if (data_length == ACCGYRMAG_DATA_LEN) {
    /* Accelerometer */
    mot_data.acc.x.val = ((int8_t)value[3]<<8) | value[2];
    mot_data.acc.x.is_neg = (mot_data.acc.x.val < 0) ? 1 : 0;
    mot_data.acc.x.val = (mot_data.acc.x.val < 0) ? (-mot_data.acc.x.val) : (mot_data.acc.x.val);

    mot_data.acc.y.val = ((int8_t)value[5]<<8) | value[4];
    mot_data.acc.y.is_neg = (mot_data.acc.y.val < 0) ? 1 : 0;
    mot_data.acc.y.val = (mot_data.acc.y.val < 0) ? (-mot_data.acc.y.val) : (mot_data.acc.y.val);

    mot_data.acc.z.val = ((int8_t)value[7]<<8) | value[6];
    mot_data.acc.z.is_neg = (mot_data.acc.z.val < 0) ? 1 : 0;
    mot_data.acc.z.val = (mot_data.acc.z.val < 0) ? (-mot_data.acc.z.val) : (mot_data.acc.z.val);

    /* Gyroscope */
    mot_data.gyr.x.val = ((int8_t)value[9]<<8) | value[8];
    mot_data.gyr.x.is_neg = (mot_data.gyr.x.val < 0) ? 1 : 0;
    mot_data.gyr.x.val = (mot_data.gyr.x.val < 0) ? (-mot_data.gyr.x.val) : (mot_data.gyr.x.val);

    mot_data.gyr.y.val = ((int8_t)value[11]<<8) | value[10];
    mot_data.gyr.y.is_neg = (mot_data.gyr.y.val < 0) ? 1 : 0;
    mot_data.gyr.y.val = (mot_data.gyr.y.val < 0) ? (-mot_data.gyr.y.val) : (mot_data.gyr.y.val);

    mot_data.gyr.z.val = ((int8_t)value[13]<<8) | value[12];
    mot_data.gyr.z.is_neg = (mot_data.gyr.z.val < 0) ? 1 : 0;
    mot_data.gyr.z.val = (mot_data.gyr.z.val < 0) ? (-mot_data.gyr.z.val) : (mot_data.gyr.z.val);

    /* Magnetometer */
    mot_data.mag.x.val = ((int8_t)value[15]<<8) | value[14];
    mot_data.mag.x.is_neg = (mot_data.mag.x.val < 0) ? 1 : 0;
    mot_data.mag.x.val = (mot_data.mag.x.val < 0) ? (-mot_data.mag.x.val) : (mot_data.mag.x.val);

    mot_data.mag.y.val = ((int8_t)value[17]<<8) | value[16];
    mot_data.mag.y.is_neg = (mot_data.mag.y.val < 0) ? 1 : 0;
    mot_data.mag.y.val = (mot_data.mag.y.val < 0) ? (-mot_data.mag.y.val) : (mot_data.mag.y.val);

    mot_data.mag.z.val = ((int8_t)value[19]<<8) | value[18];
    mot_data.mag.z.is_neg = (mot_data.mag.z.val < 0) ? 1 : 0;
    mot_data.mag.z.val = (mot_data.mag.z.val < 0) ? (-mot_data.mag.z.val) : (mot_data.mag.z.val);
  }

  return mot_data;
}

/**
 * @brief  Get the quaternions data from the BLE data received
 * @param  The data length
 * @param  The received data
 * @retval The quaternions data
 */
quatData_t Get_Quaternions_Data(uint8_t data_length, uint8_t* value)
{
  quatData_t quat_data;

  /* Quaternions 1 */
  if ((data_length == QUATERNIONS_1_DATA_LEN) ||
      (data_length == QUATERNIONS_2_DATA_LEN) ||
      (data_length == QUATERNIONS_3_DATA_LEN)) {
    quat_data.q1.x.val = ((int8_t)value[3]<<8) | value[2];
    quat_data.q1.x.is_neg = (quat_data.q1.x.val < 0) ? 1 : 0;
    quat_data.q1.x.val = (quat_data.q1.x.val < 0) ? (-quat_data.q1.x.val) : (quat_data.q1.x.val);

    quat_data.q1.y.val = ((int8_t)value[5]<<8) | value[4];
    quat_data.q1.y.is_neg = (quat_data.q1.y.val < 0) ? 1 : 0;
    quat_data.q1.y.val = (quat_data.q1.y.val < 0) ? (-quat_data.q1.y.val) : (quat_data.q1.y.val);

    quat_data.q1.z.val = ((int8_t)value[7]<<8) | value[6];
    quat_data.q1.z.is_neg = (quat_data.q1.z.val < 0) ? 1 : 0;
    quat_data.q1.z.val = (quat_data.q1.z.val < 0) ? (-quat_data.q1.z.val) : (quat_data.q1.z.val);
  }
  if ((data_length == QUATERNIONS_2_DATA_LEN) ||
      (data_length == QUATERNIONS_3_DATA_LEN)) {
    quat_data.q2.x.val = ((int8_t)value[9]<<8) | value[8];
    quat_data.q2.x.is_neg = (quat_data.q2.x.val < 0) ? 1 : 0;
    quat_data.q2.x.val = (quat_data.q2.x.val < 0) ? (-quat_data.q2.x.val) : (quat_data.q2.x.val);

    quat_data.q2.y.val = ((int8_t)value[11]<<8) | value[10];
    quat_data.q2.y.is_neg = (quat_data.q2.y.val < 0) ? 1 : 0;
    quat_data.q2.y.val = (quat_data.q2.y.val < 0) ? (-quat_data.q2.y.val) : (quat_data.q2.y.val);

    quat_data.q2.z.val = ((int8_t)value[13]<<8) | value[12];
    quat_data.q2.z.is_neg = (quat_data.q2.z.val < 0) ? 1 : 0;
    quat_data.q2.z.val = (quat_data.q2.z.val < 0) ? (-quat_data.q2.z.val) : (quat_data.q2.z.val);
  }
  if (data_length == QUATERNIONS_3_DATA_LEN) {
    quat_data.q3.x.val = ((int8_t)value[15]<<8) | value[14];
    quat_data.q3.x.is_neg = (quat_data.q3.x.val < 0) ? 1 : 0;
    quat_data.q3.x.val = (quat_data.q3.x.val < 0) ? (-quat_data.q3.x.val) : (quat_data.q3.x.val);

    quat_data.q3.y.val = ((int8_t)value[17]<<8) | value[16];
    quat_data.q3.y.is_neg = (quat_data.q3.y.val < 0) ? 1 : 0;
    quat_data.q3.y.val = (quat_data.q3.y.val < 0) ? (-quat_data.q3.y.val) : (quat_data.q3.y.val);

    quat_data.q3.z.val = ((int8_t)value[19]<<8) | value[18];
    quat_data.q3.z.is_neg = (quat_data.q3.z.val < 0) ? 1 : 0;
    quat_data.q3.z.val = (quat_data.q3.z.val < 0) ? (-quat_data.q3.z.val) : (quat_data.q3.z.val);
  }

  return quat_data;
}
