# Learn STM32

Various projects in CubeIDE to learn STM32.

My main goal is to understand deeper on embedded programming so I can work on projects based on other MCUs (such as one based on RISC-V architecture). The Arduino IDE and its wealth of libraries hide too much details.

To accomplish this, I take a series of steps to understand

  1. how to use various components in a typical Arduino kit such as 16x2 LCD, RTC, servos, 240x240 IPS LCD screen, ... in an STM32-based development board (Blue Pill, Black Pill),
  2. the common communication protocols in the embedded world such as I2C, SPI, CAN
  3. the internals such as timer, DMA, etc.

## Blue Pill

Note that I am using the version with the lower spec [STM32F103C6T6](https://www.st.com/en/microcontrollers-microprocessors/stm32f103c6.html) chip instead of the commonly used one (original?) with STM32F103C8T6.

Most project has Serial Wire Debug enabled in SYS and Ceramic/Crystal Resonator for HSE in RCC (under System Core).

  * `F103C6T6_LCD_I2C`: 16x2 LCD + I2C adapter module.

    The library to control the module via I2C is taken from [ControllersTech](https://controllerstech.com/i2c-lcd-in-stm32/).

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
    $$\frac{36 MHz}{P \times R} = 50 Hz$$
    or
    $$P \times R = \frac{36 \times 1000 \times 1000}{50} = 36 \times 1000 \times 20$$
    (Note here that I am using Timer 2 which is connected to ABP1 according to the datasheet so the max clockspeed is 36 MHz.)

    We also want R to be as large as possible (limited to 16-bit though) so we have more precise control (i.e. higher resolution) over the duty cycle. We could choose P = 72 and R = 10000; or P = 720 and R = 1000. I am choosing the later. For reason yet to be determined (need oscilloscope), I need to double the values given in the articles (`50` for 0 degree, `250` for 180 degree). Maybe, the value for P should be chosen differently.

  * `F103C6T6_STEPPER_28BYJ`: Stepper Motor + ULN2003AN driver module

    Once again, [ControllersTech]( https://controllerstech.com/interface-stepper-motor-with-stm32/) already provides a good starting point. I am simplifying some function to reduce the number of switch cases. Also note that the page (and the video) forgets to mention that we need to connect the GND of the power supply to the driver module and the GND of the Blue Pill.

## Black Pill

I am using STM32F411CEU6 with 512KB of Flash.