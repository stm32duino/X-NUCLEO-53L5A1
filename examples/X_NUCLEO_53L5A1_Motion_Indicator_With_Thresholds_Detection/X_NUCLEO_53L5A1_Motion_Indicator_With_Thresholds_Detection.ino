/**
 ******************************************************************************
 * @file    X_NUCLEO_53L5A1_Motion_Indicator_With_Thresholds_Detection.ino
 * @author  STMicroelectronics
 * @version V1.0.0
 * @date    11 November 2021
 * @brief   Arduino test application for the X-NUCLEO-53L5A1 based on VL53L5CX
 *          proximity sensor.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2021 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include <Arduino.h>
#include <Wire.h>
#include <vl53l5cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#ifdef ARDUINO_SAM_DUE
  #define DEV_I2C Wire1
#else
  #define DEV_I2C Wire
#endif
#define SerialPort Serial

#ifndef LED_BUILTIN
  #define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

#define LPN_PIN 5
#define I2C_RST_PIN 3
#define PWREN_PIN A3
#define INT_PIN A2

// Components.
VL53L5CX sensor_vl53l5cx_top(&DEV_I2C, LPN_PIN, I2C_RST_PIN);

VL53L5CX_Motion_Configuration motion_config; /* Motion configuration*/

volatile int interruptCount = 0;

void measure(void);
void blink_led_loop(void);

void measure(void)
{
  interruptCount = 1;
}

void blink_led_loop(void)
{
  do {
    // Blink the led forever
    digitalWrite(LedPin, HIGH);
    delay(500);
    digitalWrite(LedPin, LOW);
  } while (1);
}

/* Setup ---------------------------------------------------------------------*/

void setup()
{
  VL53L5CX_DetectionThresholds thresholds[VL53L5CX_NB_THRESHOLDS];
  char report[64];
  uint8_t status;

  // Led.
  pinMode(LedPin, OUTPUT);

  // Enable PWREN pin if present
  if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }

  // Set interrupt pin
  pinMode(INT_PIN, INPUT_PULLUP);
  attachInterrupt(INT_PIN, measure, FALLING);

  // Initialize serial for output.
  SerialPort.begin(115200);
  SerialPort.println("Initialize... Please wait, it may take few seconds...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L5CX component.
  sensor_vl53l5cx_top.begin();

  sensor_vl53l5cx_top.init_sensor();

  /*********************************/
  /*   Program motion indicator    */
  /*********************************/

  /* Create motion indicator with resolution 8x8 */
  status = sensor_vl53l5cx_top.vl53l5cx_motion_indicator_init(&motion_config, VL53L5CX_RESOLUTION_8X8);
  if (status) {
    snprintf(report, sizeof(report), "vl53l5cx_motion_indicator_init failed with status : %u\r\n", status);
    SerialPort.print(report);
    blink_led_loop();
  }

  /* (Optional) Change the min and max distance used to detect motions. The
   * difference between min and max must never be >1500mm, and minimum never be <400mm,
   * otherwise the function below returns error 127 */
  status = sensor_vl53l5cx_top.vl53l5cx_motion_indicator_set_distance_motion(&motion_config, 1000, 2000);
  if (status) {
    snprintf(report, sizeof(report), "vl53l5cx_motion_indicator_set_distance_motion failed with status : %u\r\n", status);
    SerialPort.print(report);
    blink_led_loop();
  }

  /* If user want to change the resolution, he also needs to update the motion indicator resolution */
  //status = sensor_vl53l5cx_top.vl53l5cx_set_resolution(VL53L5CX_RESOLUTION_4X4);
  //status = sensor_vl53l5cx_top.vl53l5cx_motion_indicator_set_resolution(&motion_config, VL53L5CX_RESOLUTION_4X4);

  /* Set the device in AUTONOMOUS and set a small integration time to reduce power consumption */
  status = sensor_vl53l5cx_top.vl53l5cx_set_resolution(VL53L5CX_RESOLUTION_8X8);
  if (status) {
    snprintf(report, sizeof(report), "vl53l5cx_set_resolution failed with status : %u\r\n", status);
    SerialPort.print(report);
    blink_led_loop();
  }

  status = sensor_vl53l5cx_top.vl53l5cx_set_ranging_mode(VL53L5CX_RANGING_MODE_AUTONOMOUS);
  if (status) {
    snprintf(report, sizeof(report), "vl53l5cx_set_ranging_mode failed with status : %u\r\n", status);
    SerialPort.print(report);
    blink_led_loop();
  }

  status = sensor_vl53l5cx_top.vl53l5cx_set_ranging_frequency_hz(2);
  if (status) {
    snprintf(report, sizeof(report), "vl53l5cx_set_ranging_frequency_hz failed with status : %u\r\n", status);
    SerialPort.print(report);
    blink_led_loop();
  }

  status = sensor_vl53l5cx_top.vl53l5cx_set_integration_time_ms(10);
  if (status) {
    snprintf(report, sizeof(report), "vl53l5cx_set_integration_time_ms failed with status : %u\r\n", status);
    SerialPort.print(report);
    blink_led_loop();
  }


  /*********************************/
  /*  Program detection thresholds */
  /*********************************/

  /* In this example, we want 1 thresholds per zone for a 8x8 resolution */
  /* Create array of thresholds (size cannot be changed) */

  /* Set all values to 0 */
  memset(&thresholds, 0, sizeof(thresholds));

  /* Add thresholds for all zones (16 zones in resolution 4x4, or 64 in 8x8) */
  for (int i = 0; i < 64; i++) {
    thresholds[i].zone_num = i;
    thresholds[i].measurement = VL53L5CX_MOTION_INDICATOR;
    thresholds[i].type = VL53L5CX_GREATER_THAN_MAX_CHECKER;
    thresholds[i].mathematic_operation = VL53L5CX_OPERATION_NONE;

    /* The value 44 is given as example. All motion above 44 will be considered as a movement */
    thresholds[i].param_low_thresh = 44;
    thresholds[i].param_high_thresh = 44;
  }

  /* The last thresholds must be clearly indicated. As we have 64
   * checkers, the last one is the 63 */
  thresholds[63].zone_num = VL53L5CX_LAST_THRESHOLD | thresholds[63].zone_num;

  /* Send array of thresholds to the sensor */
  sensor_vl53l5cx_top.vl53l5cx_set_detection_thresholds(thresholds);

  /* Enable detection thresholds */
  sensor_vl53l5cx_top.vl53l5cx_set_detection_thresholds_enable(1);

  // Start Measurements
  sensor_vl53l5cx_top.vl53l5cx_start_ranging();

  SerialPort.println("Waiting for a movement into the FOV between 1m and 2m...");
}

void loop()
{
  static uint8_t loop_count = 0;
  VL53L5CX_ResultsData Results;
  char report[128];
  int status;

  if (loop_count < 10) {
    if (interruptCount) {
      interruptCount = 0;

      //Led on
      digitalWrite(LedPin, HIGH);

      status = sensor_vl53l5cx_top.vl53l5cx_get_ranging_data(&Results);

      /* As the sensor is set in 8x8 mode by default, we have a total
       * of 64 zones to print.
       */

      for (int i = 0; i < 64; i++) {
        if (Results.motion_indicator.motion[motion_config.map_id[i]] >= 44) {
          snprintf(report, sizeof(report), " Movement detected in this zone : %3d !\r\n", i);
          SerialPort.print(report);
        }
      }
      SerialPort.println("");
      loop_count++;
      digitalWrite(LedPin, LOW);
    }
  } else if (loop_count == 10) {
    /* Stop measurements */
    status = sensor_vl53l5cx_top.vl53l5cx_stop_ranging();
    if (status) {
      snprintf(report, sizeof(report), "vl53l5cx_stop_ranging failed, status %u\r\n", status);
      SerialPort.print(report);
      blink_led_loop();
    }

    loop_count++;
    /* End of the demo */
    SerialPort.println("End of ULD demo");
  } else {
    delay(1000);
  }
}
