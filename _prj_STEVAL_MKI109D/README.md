# STEVAL-MKI109D platform

This folder contains a STEVAL MKI109D CubeMX .ioc project file and a complete STM32H563ZI project which may be used to set up a basic system based on SPI with interrupt and UART capability and that can be used to test SPI-based examples.

## Basic Project Setup Instructions

You need to execute following steps in order to get a working project based on your favourite toolchain/IDE environment:
                                                 
1. Open the ```$STDC_PATH/_prj_MKI109D/_prj_STEVAL_MKI109D.ioc``` file using the STM32CubeMX tool (e.g. by double clicking the file) and generate the code.

   *Please note that EWARM is the default toolchain declared in the ioc file, hence you may need to configure it accordingly before generating the code.*

2. Add core source files to the project:

```c
$STDC_PATH/_prj_MKI109D/Core/Src/board.c
$STDC_PATH/_prj_MKI109D/Core/Src/queue.c
$STDC_PATH/_prj_MKI109D/Core/Src/spi_3w.c
```

3. Add Middleware source files to the project:

```c
  $STDC_PATH/_prj_MKI109D/Middleware\USB\Device\Class\CDC\Src\usbd_cdc_if.c
  $STDC_PATH/_prj_MKI109D/Middleware\USB\Device\Core\Src\usb_device.c
  $STDC_PATH/_prj_MKI109D/Middleware\USB\Device\Core\Src\usbd_conf.c
  $STDC_PATH/_prj_MKI109D/Middleware\USB\Device\Core\Src\usbd_desc.c
  $STDC_PATH/_prj_MKI109D/Middleware\USB\stm32-mw-usb-device\Class\CDC\Src\usbd_cdc.c
  $STDC_PATH/_prj_MKI109D/Middleware\USB\stm32-mw-usb-device\Core\Src\usbd_core.c
  $STDC_PATH/_prj_MKI109D/Middleware\USB\stm32-mw-usb-device\Core\Src\usbd_ctlreq.c
  $STDC_PATH/_prj_MKI109D/Middleware\USB\stm32-mw-usb-device\Core\Src\usbd_ioreq.c
```

4. Add following directories to the compiler include paths (e.g., in your Makefile or IDE settings):

```c
  CFLAGS += -I $STDC_PATH/_prj_MKI109D/Middleware/USB/stm32-mw-usb-device/Core/Inc \
            -I $STDC_PATH/_prj_MKI109D/Middleware/USB/stm32-mw-usb-device/Class/CDC/Inc \
            -I $STDC_PATH/_prj_MKI109D/Middleware/USB/Device/Core/Inc \
            -I $STDC_PATH/_prj_MKI109D/Middleware/USB/Device/Class/CDC/Inc
```

## How to run a driver example

The project may need to be customized in order to be able to run a driver example. Below there is a short description of the steps required to run *$STDC_PATH/lsm6dsv16x_STdC/examples/lsm6dsv16x_fifo_irq.c*.

### Customize source code

1. Open the *$STDC_PATH/_prj_MKI109D/Core/Src/main.c* file 

2. Define the prototypes (if not there already) for both the example routine and its interrupt handler:

```c
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* add prototypes */
void lsm6dsv16x_fifo_irq(void);
void lsm6dsv16x_fifo_irq_handler(void);

/* USER CODE END PFP */
```

3. Call the example routine inside the main loop:

```c
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* call example main routine */
    lsm6dsv16x_fifo_irq();
  }
  /* USER CODE END 3 */
```

4. Call the example interrupt handler:

```c
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* overwrite default interrupt callback */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  lsm6dsv16x_fifo_irq_handler();
}

/* USER CODE END 0 */
```

### Modify and build the project

1. Add the [$STDC_PATH/lsm6dsv16x_STdC/driver/lsm6dsv16x_reg.c](https://github.com/STMicroelectronics/lsm6dsv16x-pid/blob/main/lsm6dsv16x_reg.c) driver and [$STDC_PATH/lsm6dsv16x_STdC/examples/lsm6dsv_fifo_irq.c](https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/lsm6dsv16x_STdC/examples/lsm6dsv16x_fifo_irq.c) example to the project.

2. Add the *$STDC_PATH/lsm6dsv16x_STdC/driver* directory to the compiler include path, e.g. :

```make
CFLAGS += -I $STDC_PATH/lsm6dsv16x_STdC/driver
```

3. Add STEVAL_MKI109D to the list of preprocessor enabled macros for the compiler, e.g :

```make
CFLAGS += -D STEVAL_MKI109D
```

### Visualize example output messages

Open a serial port emulator and configure it as 115200 8n1:

<p align="center">
  <img src="./serial_port.png" />
</p>

**More information:**
  - [STEVAL-MKI109D](https://www.st.com/en/evaluation-tools/steval-mki109d.html)
  - [STM32H563ZI](https://www.st.com/en/microcontrollers-microprocessors/stm32h563zi.html)

**Copyright (C) 2025 STMicroelectronics**
