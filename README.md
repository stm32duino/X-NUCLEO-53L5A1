# X-NUCLEO-53L5A1

Arduino library to support the X-NUCLEO-53L5A1 based on VL53L5CX Time-of-Flight 8x8 multizone ranging sensor with wide field view.

This sensor uses I2C to communicate. And I2C instance is required to access to the sensor.
The APIs provide simple distance measure and multizone detection in both polling and interrupt modes.

## Examples

There are 11 examples with the X-NUCLEO-53L5A1 library:

* X_NUCLEO_53L5A1_Calibrate_Xtalk: This example code is to show how to perform the crosstalk calibration.

* X_NUCLEO_53L5A1_Get_Set_Params: This example code is to show how to get/set some parameters of the 
  VL53L5CX sensor.

* X_NUCLEO_53L5A1_HelloWorld: This example code is to show how to get multi-object detection and proximity
  values of the VL53L5CX satellite sensor in polling mode.

* X_NUCLEO_53L5A1_HelloWorld_Interrupt: This example code is to show how to get multi-object detection and proximity
  values of the VL53L5CX satellite sensor in interrupt mode.

* X_NUCLEO_53L5A1_I2C_And_RAM_Optimization: This example code is to show how to optimize the code in terms of 
  number of I2C transactions and RAM occupation.

* X_NUCLEO_53L5A1_Motion_Indicator: This example code is to show how to configure the motion indicator.

* X_NUCLEO_53L5A1_Motion_Indicator_With_Thresholds_Detection: This example code is to show how to configure 
  the motion indicator together with the thresholds detection.
  
* X_NUCLEO_53L5A1_Multiple_Targets_Per_Zone: This example code is to show how to configure multiple targets 
  per zone.

* X_NUCLEO_53L5A1_Power_Modes: This example code is to show how to change the power mode of the VL53L5CX 
  sensor.

* X_NUCLEO_53L5A1_Ranging_Modes: This example code is to show how to change the ranging mode of the VL53L5CX 
  sensor.

* X_NUCLEO_53L5A1_Thresholds_Detection: This example code is to show how to configure the thresholds 
  detection.

## Dependencies

This package requires the following Arduino library:

* STM32duino VL53L5CX: https://github.com/stm32duino/VL53L5CX


## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-53L5A1

The VL53L5CX datasheet is available at
https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html