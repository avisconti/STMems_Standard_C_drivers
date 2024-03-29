# Applications

This folder contains application examples. The examples refer to following boards but, since the drivers are platform independent, they can be easily ported to any other platform: 

- STEVAL-MKI109V3 (https://www.st.com/en/evaluation-tools/steval-mki109v3.html)
- NUCLEO-F411RE (https://www.st.com/en/microcontrollers-microprocessors/stm32f411re.html)
- DISCOVERY-SPC584B (https://www.st.com/en/evaluation-tools/spc584b-dis.html)

## Self Test (ST)

Run the device Self Test procedure:

  - lis2dux12_self_test.c

## Read data

Read accelerometer and temperature sensor data in both polling and drdy mode:

  - lis2dux12_read_data_polling.c
  - lis2dux12_read_data_drdy.c

Read accelerometer and temperature sensor data from FIFO on FIFO threshold event:

  - lis2dux12_read_fifo.c

## Program and use embedded digital functions

Program LIS2DUX12 to receive free fall events on INT1:

  - lis2dux12_free_fall.c

Program LIS2DUX12 to receive step counter events from FIFO:

  - lis2dux12_pedo_fifo.c

Program LIS2DUX12 to receive step counter events on INT1:

  - lis2dux12_pedometer.c

Program LIS2DUX12 to receive 6D orientation detection events on INT1:

  - lis2dux12_sixd.c

Program LIS2DUX12 to receive single/double/triple tap events on INT1:

  - lis2dux12_tap.c

Program LIS2DUX12 to receive tilt detection events on INT1:

  - lis2dux12_tilt.c

Program LIS2DUX12 to receive wakeup from sleep events on INT1:

  - lis2dux12_wakeup.c

## Finite State Machine (FSM)

Program the FSM with ucf file to detect *glance* and *de-glance* gestures typically used in smartphone devices (read more [here](https://github.com/STMicroelectronics/STMems_Finite_State_Machine/blob/master/application_examples/lis2dux12/Glance%20detection/README.md)):

  - lis2dux12_fsm_glance.c

## Machine Learning Code (MLC)

Program the MLC with ucf file to recognize *stationary*, *walking*, *jogging*, *biking* and *driving* activities (read more [here](https://github.com/STMicroelectronics/STMems_Machine_Learning_Core/blob/master/application_examples/lis2dux12/activity_recognition_for_mobile/README.md)):

  - lis2dux12_mlc_activity_mobile.c


**Copyright (C) 2024 STMicroelectronics**
