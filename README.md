# Learn STM32

Various projects in CubeIDE to learn STM32.

My main goal is to understand deeper on embedded programming so I can work on projects based on other MCUs (such as one based on RISC-V architecture). The Arduino IDE and its wealth of libraries hide too much details.

To accomplish this, I take a series of steps to understand

  1. how to use various components in a typical Arduino kit such as 16x2 LCD, RTC, servos, 240x240 IPS LCD screen, ... in an STM32-based development board (Blue Pill, Black Pill),
  2. the common communication protocols in the embedded world such as I2C, SPI, CAN
  3. the internals such as timer, DMA, etc.

Some projects have source files linked to the files installed with Cube IDE. You can fix them by searching for and replacing my _Firwmare installation repository_ location `C:\Users\Anonymous\STM32Cube\Repository\` (checked _Preferences > STM 32 Cube > Firmware Updater_) with yours in the `.mxproject` file. You may need to change the firmware version as well.

## Blue Pill

Note that I am using the version with the lower spec [STM32F103C6T6](https://www.st.com/en/microcontrollers-microprocessors/stm32f103c6.html) chip instead of the commonly used one (original?) with STM32F103C8T6.

Most project has Serial Wire Debug enabled in SYS and Ceramic/Crystal Resonator for HSE in RCC (under System Core).

  * `F103C6T6_LCD_I2C`: 16x2 LCD + I2C adapter module.

    The library to control the module via I2C is taken from [ControllersTech](https://controllerstech.com/i2c-lcd-in-stm32/).

    It was not mentioned explicitly but one must connect the VCC of the LCD to 5V which is its designated operating voltage. Otherwise, the text will be very faintly visible. So we need to unplug the debugger and connect the Blue Pill via USB or use an external power source.

  * `F103C6T6_RTC_DS1302`: Real-time clock module DS1302.

    __Connection__: DS1302's `VCC, GND, CLK, DATA, RST` pins should be connected to `5V, GND, B6, B7, B5` of the Blue Pill respectively.

    The library to interact with DS1302 is [here](https://github.com/julgonmej/stm32f7-ds1302-lib). Unfortunately, it was written for STM32F7xxxx chips and does not come with any example so I have to make some modifications. This library follows the datasheet so it is not hard to understand the operation.

    One of the most crucial step before using the library is to [enable cycle counter feature]( https://stackoverflow.com/questions/36378280/stm32-how-to-enable-dwt-cycle-counter)
    ```c
    DWT->CTRL = DWT_CTRL_CYCEVTENA_Msk | DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    ```
    Without this, the MCU will be stuck in the infinite loop inside `delayUS_DWT` where the clock cycle count is never updated.

  * `F103C6T6_SERVO`: Servos.

    Servos are controlled with PWM (not the same PWM that is used to control a load such as brightness of LED or motor). They also need a power supply that could output a sufficiently high current.

    I am adapting again from [ControllersTech](https://controllerstech.com/servo-motor-with-stm32/). His configuration is for an STM32F446RE board which runs at a much higher clockspeed 180 MHz (compared to ours at 72 MHz). The key idea is to choose two 16-bit values for the Prescalar (P) and ARR (R) to bring the frequency down to 50Hz:
    $$\frac{72 MHz}{P \times R} = 50 Hz$$
    or
    $$P \times R = \frac{72 \times 1000 \times 1000}{50} = 72 \times 1000 \times 20$$
    (Note here that it is 72 MHz and not 36 MHz or 18 MHz because I did not do the step

    > Keeping this in mind, I am going to set the clock such that my TIM2 gets only 45 MHz.

    in the article. In any case, this number comes from the value marked _ABP2 timer clocks (MHz)_ in the _Clock configuration_ tab.)

    We also want $R$ to be as large as possible (limited to 16-bit though) so we have more precise control (i.e. higher resolution) over the duty cycle. We could choose $P = 72$ and $R = 20000$; or $P = 1440$ and $R = 1000$. I am choosing the later to match the duty cycles of the article.

  * `F103C6T6_STEPPER_28BYJ`: Stepper Motor + ULN2003AN driver module

    Once again, [ControllersTech]( https://controllerstech.com/interface-stepper-motor-with-stm32/) already provides a good starting point. I am simplifying some function to reduce the number of switch cases. Also note that the page (and the video) forgets to mention that we need to connect the GND of the power supply to the driver module and the GND of the Blue Pill.

  * `F103C6T6_ST7789_LCD`: 240x240 LCD module with ST7789 controller

    This module is a break out module for the actual LCD display. Unfortunately, the module does not break out the CS (Chip Select) pin so we cannot do proper SPI. Check out [this good video on SPI](https://www.youtube.com/watch?v=MCi7dCBhVpQ).

    A library with a good instruction is [available](https://github.com/Floyd-Fish/ST7789-STM32). This is what I used. (The version included in the project has DMA disabled and I also remove image rendering in the test as the Blue Pill does not have sufficient memory for that.)

    The connection is similar to [here](https://blog.embeddedexpert.io/?p=1215); but check out the actual pins in CubeIDE (for SCK and **MOSI**, note that the Blue Pill should act as the Master and the module is a Slave and we need to output signal from Blue Pill Master to Slave) and I should say Vcc on the module should be connected to 3.3V instead of 5V. In the project configuration, I am using PA1 for RES and PA2 for DC.

    I had a lot of trouble getting the result. I found out in the end that the display will not render anything *if ST-LINK is connected* (as I usually run the code via Serial Wire Debug in the IDE). I look and watch many tutorials but nobody mentions this crucial fact. So, after uploading the code, unplug the ST-LINK and plug the Blue Pill in via the micro-USB cable.

    The reason might be that the power supply from the ST-Link is not sufficient to drive the power-hungry LCD. Or maybe [this](https://github.com/Bodmer/TFT_eSPI/issues/1758).

  * `F103C6T6_MPU6050_I2C`: Using gyroscope + accelerometer module MPU6050 (see the same example `F411CEU6_MPU6050_I2C` below)

  * `F103C6T6_RC_IO`: Timer's Input Capture

    In the servo example, we _output_ a signal to control a module.
    In this example, we are going to use Timer's Input Capture function to **measure the width of a pulse** in an incoming signal. This allows us to use a [radio control system](https://en.wikipedia.org/wiki/Radio_control) in our projects.

    The theory behind is already given by [ControllersTech]( https://controllerstech.com/input-capture-in-stm32/). Basically, this function captures the time at which a pin transitions from the LOW state to the HIGH state (a _rising_ edge) or vice versa (a _falling_ edge).

    For the Blue Pill, we need to make some adaptation and not just on choosing the right numbers as in the servo example. The problem is that the STM32F1 family does NOT support _"Both Edges"_ for _Polarity Selection_ as in the article. In other words, we only get notified a.k.a. _interrupted_ on either the rising edge or the falling one. To capture both edges of the pulse, we have to _reconfigure_ the timer after capturing the rising edge to capture the falling one in our ISR (Interrupt Service Routine) a.k.a. interrupt handler and once the falling edge is captured, we switch the polarity selection back to rising for the next pulse. This can be accomplished with the convenient macro `__HAL_TIM_SET_CAPTUREPOLARITY`.

    The example is made to be more practical by
	  (i)  generating 4 PWM signals simultaneously off timer 1; and
	  (ii) measure 4 signals simultaneously off timer 2
	so one can connect PA8, PA9, PA10, PA11 (or the PWM outputs of a radio control receiver) to PA0, PA1, PA2, PA3 to see the different pulse widths.

  * `F103C6T6_USB_CDC`: Using USB port to communicate with the computer

    The example is in [this video](https://www.youtube.com/watch?v=92A98iEFmaA). But watch also [the official training video series](https://www.youtube.com/watch?v=rI3yBmnfAZU&list=PLnMKNibPkDnFFRBVD206EfnnHhQZI4Hxa) and [Ben Eater's explanation](https://www.youtube.com/watch?v=wdgULBpRoXk) to know what is going on behind.

  * `F103C6T6_USB_HID_Keyboard`: Turning the Blue Pill into a custom USB keyboard to enter strings (such as password) on a single button press

    Modified from [this video](https://www.youtube.com/watch?v=tj1_hsQ5PR0).

    Follow [this](https://microcontrollerslab.com/push-button-stm32-blue-pill-stm32cube-ide-tutorial/) for the circuit diagram to connect a push button with a pull-up resistor. Unlike the article, I am using pin `B12` for the button input. When the button is pushed, the (hardcoded) password string is entered. (This solution is not very responsive. We could use an interrupt to make it better.)

    Since I am using links for the firmware library files, I have to unlink the file `usbd_hid.c` (by changing the `.project` file) and copy the source directly to `Middlewares` folder in order to make modifications without affecting the original file installed with the IDE.

    I am not going to modify `usbd_hid.h` (so there is no need to make a copy). Instead, I am `#undef` and re`#define HID_MOUSE_REPORT_DESC_SIZE` right in `usbd_hid.c`.

  * `F103C6T6_nRF24L01`: Wireless communication via nRF24L01+ modules.

    Similar to `F411CEU6_nRF24L01` below but configured as a receiver ONLY.

## Black Pill

I am using STM32F411CEU6 with 512KB of Flash.

  * `F411CEU6_MPU6050_I2C`: The idea behind is [here](https://controllerstech.com/how-to-interface-mpu6050-gy-521-with-stm32/). It is fairly straight forward.

  * `F411CEU6_USB_CCID`: Making a security token (work in progress).

    At the moment, Cube IDE does not support code generation for CCID class USB even though the middleware library has it. So I selected the "Custom Human Interface Device Class" and copy the template files in the firmware repository to the project. Various adjustments are necessary to get it compiled. (The worst thing is everytime we perform code regeneration, we have to undo some of its effects. Let's hope that support comes eventually. Another function is composite device i.e. multiple device classes.)

  * `F411CEU6_SSD1306_OLED_I2C`: Another display example using common 128x64 OLED screen

    We use [this library](https://github.com/4ilo/ssd1306-stm32HAL).

  * `F411CEU6_RFID_RC522`: RFID readers.

    A library to drive the module is available [here](https://github.com/Hamid-R-Tanhaei/RFID-MIFARE-RC522-ARM-STM32). Watch [this](https://www.youtube.com/watch?v=hd3ntPzL6Jg) for background information.

    I am following [this article](https://blog.embeddedexpert.io/?p=768) and [this for the pinout](https://lastminuteengineers.com/how-rfid-works-rc522-arduino-tutorial/).

    The datasheet for (the main IC of) the module is [here](https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf). To communicate with the RFID tags, we also need the datasheet for them and this module is frequently sold with several [Mifare One tags](https://www.nxp.com/docs/en/data-sheet/MF1S50YYX_V1.pdf). The library only support this card, I believe.

  * `F411CEU6_nRF24L01`: Wireless communication via nRF24L01+ modules.

    The transmitter to `F103C6T6_nRF24L01`.

    We are using [this library](https://github.com/mokhwasomssi/stm32_hal_nrf24l01p). Note that despite the appearance made on this page, `SPI2_CSN` (just like `CE` and `IRQ`) is not a pin configured by CubeIDE but you can select any pin as `GPIO_Output` and name it like that. I don't know why the author did not name it simply `CSN` like the other two.

    The author also configured the pin for the onboard LED but did nothing with it so I added blinking code when the interrupt is fired.

    For illustration purpose, I also modified the original library (namely, changing all functions' signatures to include a pointer to `struct` containing the module configuration i.e. the SPI to use, CSN, CE pins, ...) so that one can plug in multiple nRF24L01+ modules to the same Black Pill. (In practice, we obviously never do that.) With this change, one can connect `CE`, `CSN`, `IRQ` of the transmitter module to pins `B12`, `B13`, `A8` and those of the receiver module to pins `B3`, `B4`, `B5` and watch the `tx_data` gets copied **over the air** to `rx_data` in *Live Expression*. (Check the pin labels in the project `ioc`.)

  * `F411CEU6_ADC_JSTICK_POT`: Joystick and potentiometer using ADC.

    We follow [this](https://controllerstech.com/stm32-adc-multi-channel-without-dma/) to read multiple ADC channels without DMA. I am simplifying the code a bit by using a single function to select the channel.

    There are other guides such as [this](https://controllerstech.com/stm32-adc-multiple-channels/), [this tutorial](https://www.digikey.com/en/maker/projects/getting-started-with-stm32-working-with-adc-and-dma/f5009db3a3ed4370acaf545a3370c30c) and [this article](https://deepbluembedded.com/stm32-adc-tutorial-complete-guide-with-examples) which goes into explaining the many ADC configuration options.

  * `F411CEU6_LED_SHIFTREG`: 7-Segment displays, 8x8 LED matrix and 8-bit Shift Registers SN74HC595N.

    We are [exploiting SPI](https://github.com/miekush/led-bar-graph) for this. See [here](https://elektronotes.wordpress.com/2015/02/13/daisy-chained-74hc595s-controlled-by-spi-on-stm32f4-part-1-working-principle/) for the idea of exploiting DMA as well.

    Refer to [this](https://www.labs.cs.uregina.ca/207/Online/Lab8/) for the connection. Long story short, pin `OE` should connect to `GND` and pin `SRCLR` should connect to `VCC`. Connect SPI's `MOSI` to `SER` and its `SCK` to `SRCLK`. The final GPIO Output pin (mark `LATCH` in the project configuration) connect to `RCLK`. (Pin names are from this [datasheet](https://www.ti.com/lit/gpn/SN54HC595-SP).)

    For the 8x8 LED matrix, can we daisy-chain two shift registers by connecting one's `QH'` pin to the `SER` of the next one.

  * `F411CEU6_SDCARD_SPI`: SD Card.

    We follow [this](https://controllerstech.com/sd-card-using-spi-in-stm32/) or [the more detailed version](https://embetronicx.com/tutorials/microcontrollers/stm32/stm32-sd-card-interfacing-with-example/). Both articles use the [same library](https://github.com/eziya/STM32_SPI_SDCARD). The main files to download are `fatfs_sd.h` and `fatfs_sd.c`. The rest are autogenerated by CubeIDE.

    If you have an SD card of large capacity (example: 64GB SD card), you need to make **a partition of size less than 32GB** (example: 20GB) and then **format it as FAT32** on your PC. I never succeed in using the full 64GB with an exFAT partition.

    The example code reads a file `Hello.txt` on the (first partition? of the) SD card and its content (100 chars) is put in the buffer.

  * `F411CEU6_MPU9250_SPI`: 9-DOF IMU module MPU-9250 using SPI.

    The module supports I2C like the MPU6050 but we want to use SPI for faster communication.

    I am starting with [this minimal library](https://github.com/desertkun/MPU9250). I will add more functionality based on the datasheet and maybe [this code](https://os.mbed.com/users/kylongmu/code/MPU9250_SPI/file/084e8ba240c1/MPU9250.cpp/). It maybe a good time to learn to do [DMA](https://blog.embeddedexpert.io/?p=708).

	Refer to [this page](https://protosupplies.com/product/mpu-9250-3-axis-accel-gryo-mag-sensor-module/) for the pinout.

  * `F411CEU6_HMC5883L_I2C`: Compass module HMC5883L using I2C.

    A [driver library](https://github.com/jrowberg/i2cdevlib/tree/master/STM32HAL/HMC5883L) by Jeff Rowberg is available. (There are two versions but we use the HAL one.)

    Like BMP/E280, this module is [confusing](https://forum.arduino.cc/t/any-way-to-use-hmc5883l-library-with-qmc5883l/623793). After modifying the code to scan the I2C bus, I found out that the module I got is an QMC5883L. So we are switching to [this library](https://github.com/Farondis/QMC5883L-stm32-hal). (It is unlikely to find an HMC5883L in the future.)

  * `F411CEU6_uBlox_Neo6M`: GPS module Neo-6M from uBlox.

    The main tutorial is from [ControllersTech](https://controllerstech.com/gps-neo-6m-with-stm32/). Before that, we need to [set up UART ring buffer](https://controllerstech.com/ring-buffer-using-head-and-tail-in-stm32/). Also, I commented out the LCD part.

    Unlike many articles claimed, the module only works properly when I connected VCC to 3.3V, not 5V.

    The location and the time error are too large (time is off by 2 hours). The altitude also jumps a lot (25m to 4m above sea level despite staying fix).

  * `F411CEU6_BMP280_I2C`: Pressure and Temperature sensor module BMP280.

    Many cheap [BMP280](https://components101.com/sensors/gy-bmp280-module) [module](https://electrocredible.com/bmp280-pinout-specifications-applications/) are [falsely advertised](https://forum.arduino.cc/t/bme280-problem/451941) as the more advanced BME280 (Pressure, Humidity and Temperature). See [this](https://goughlui.com/2018/08/05/note-bosch-sensortec-bmp280-vs-bme280-sensor-confusion/) to know which chip you get.

    We are using the driver by [ciastkolog](https://github.com/ciastkolog/BMP280_STM32). Thankfully to this library, the same code works with both BMP280 and BME280. The former simply does not supply humidity reading.

  * `F411CEU6_RTC_DS1302`: RTC module again.

  * `F411CEU6_Infrared`: Infrared receiver and remote.

  * `F411CEU6_FreeRTOS`: FreeRTOS.
