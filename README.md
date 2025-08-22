# Bare Metal Interactive Traffic Light simulator built with custom peripheral drivers on STM32F446RE.

## Table of contents

- [General Info](#general-info)
- [Documentation](#documentation)
- [Project Setup](#project-setup)


## General Info

- Board: **STM32F446RE**
- Each peripheral has a 32 bit address which provides the basis of this project.
- GPIO and RCC peripherals connected to AHB1 bus (0x4002 0000).
- Basic timers connected to APB1 bus (0x 4000 1000).
- GPIOA, GPIOB, GPIOC, TIM6 and RCC were used.

## Documentation

For a more in depth understanding of GPIO,RCC and basic timer drivers, please refer to the following sections in the [reference manual](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf):
- [Memory organization](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=56)
- [RCC registers](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=116)
- [GPIO registers](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=175)
- [Basic Timers](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=620)

## Project Setup

- The GPIO and TIM6 clock is initialized by enabling a bit corresponding to it under the RCC register.
- The GPIO peripheral is initialized with its registers to configure the I/O operation.
- The basic timer (TIM6) perscaler and reload value is set using its designated registers.
- Program the GPIO pin for the desired operation and use TIM6 for delays.
<br>
### Side note: perscaler and reload values are preloaded, so it is important to generate an event after setting their values to save them. 

<div align=center>
    <img width="667" height="475" alt="image" src="https://github.com/user-attachments/assets/c90846e0-6bae-41da-adb1-a24628e071bc" />
    <h2><a href="https://github.com/user-attachments/assets/c833410d-1621-4811-af2f-cb709f47c932"> Live Demo</a></h2>
</div>



