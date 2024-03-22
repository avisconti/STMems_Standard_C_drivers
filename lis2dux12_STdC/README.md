# LIS2DUX12 driver and examples

This folder contains driver and examples, written in C programming language, for [LIS2DUX12](https://www.st.com/en/mems-and-sensors/lis2dux12.html) sensor.

## Sensor Overview

The LIS2DUX12 is a smart, digital, 3-axis linear accelerometer whose MEMS and ASIC have been expressly designed to combine the lowest current consumption possible with features such as always-on anti-aliasing filtering, a finite state machine (FSM) and machine learning core (MLC) with adaptive self-configuration (ASC).

The FSM and MLC with ASC deliver outstanding always-on, edge processing capabilities to the LIS2DUX12. The LIS2DUX12 MIPI I3C® slave interface and embedded 128-level FIFO buffer complete a set of features that make this accelerometer a reference in terms of system integration from a standpoint of the bill of materials, processing, or power consumption.

The LIS2DUX12 has user-selectable full scales of ±2g/±4g/±8g/±16g and is capable of measuring accelerations with output data rates from 1.6 Hz to 800 Hz.

The LIS2DUX12 has a dedicated internal engine to process motion and acceleration detection including free-fall, wake-up, single/double/triple-tap recognition, activity/inactivity, and 6D/4D orientation.

The LIS2DUX12 is available in a small thin plastic land grid array package (LGA) and it is guaranteed to operate over an extended temperature range from -40°C to +85 °C.

More information available on the [datasheet](https://www.st.com/resource/en/datasheet/lis2dux12.pdf).

## Applications

Examples of applications are available in *examples* directory. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform:

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F411RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f411re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

### Self Test (ST)

Run the device Self Test procedure:

  - [lis2dux12_self_test.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_self_test.c)

### Read data

Read accelerometer and temperature sensor data in both polling and drdy mode:

  - [lis2dux12_read_data_polling.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_read_data_polling.c)
  - [lis2dux12_read_data_drdy.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_read_data_drdy.c)

Read accelerometer and temperature sensor data from FIFO on FIFO threshold event:

  - [lis2dux12_read_fifo.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_read_fifo.c)

### Program and use embedded digital functions

Program LIS2DUX12 to receive free fall events on INT1:

  - [lis2dux12_free_fall.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_free_fall.c)

Program LIS2DUX12 to receive step counter events from FIFO:

  - [lis2dux12_pedo_fifo.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_pedo_fifo.c)

Program LIS2DUX12 to receive step counter events on INT1:

  - [lis2dux12_pedometer.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_pedometer.c)

Program LIS2DUX12 to receive 6D orientation detection events on INT1:

  - [lis2dux12_sixd.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_sixd.c)

Program LIS2DUX12 to receive single/double/triple tap events on INT1:

  - [lis2dux12_tap.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_tap.c)

Program LIS2DUX12 to receive tilt detection events on INT1:

  - [lis2dux12_tilt.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_tilt.c)

Program LIS2DUX12 to receive wakeup from sleep events on INT1:

  - [lis2dux12_wakeup.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_wakeup.c)

### Finite State Machine (FSM)

Program the FSM with ucf file to detect *glance* and *de-glance* gestures typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/STMems_Finite_State_Machine/blob/master/application_examples/lis2dux12/Glance%20detection/README.md)):

  - [lis2dux12_fsm_glance.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_fsm_glance.c)

### Machine Learning Code (MLC)

Program the MLC with ucf file to recognize *stationary*, *walking*, *jogging*, *biking* and *driving* activities (read more [here](https://github.com/STMicroelectronics/STMems_Machine_Learning_Core/blob/master/application_examples/lis2dux12/activity_recognition_for_mobile/README.md)):

  - [lis2dux12_mlc_activity_mobile.c](https://github.com/avisconti/STMems_Standard_C_drivers/blob/change-readmes/lis2dux12_STdC/examples/lis2dux12_mlc_activity_mobile.c)


**Copyright (C) 2024 STMicroelectronics**
