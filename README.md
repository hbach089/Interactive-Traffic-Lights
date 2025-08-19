# Bare Metal Interactive Traffic Light simulator programmed using GPIO drivers.

## Table of contents

- General Info
- Documentation
- Project Setup

## General Info

- Board: STM32F446re
- Each peripheral has a 32 bit address which provides the basis of this project.
- GPIO and RCC peripherals connected to AHB1 bus (0x4002 0000).
- GPIOA, GPIOB, GPIOC and RCC were used.

## Documentation

For a more in depth understanding of GPIO and RCC drivers, please refer to the following sections in the reference manual
- [Memory organization](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [RCC registers](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [GPIO registers](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)

## Project Setup

- The GPIO clock is initialized by enabling a bit corresponding to it under the RCC register.
- The GPIO peripheral is initialized with its registers to configure the I/O operation.
- Program the GPIO pin for the desired operation.


<div float="left">
  <video src="https://github.com/user-attachments/assets/c833410d-1621-4811-af2f-cb709f47c932" width="400" />   
</div>

<div float="right">
  <img width="341" height="259" alt="image" src="https://github.com/user-attachments/assets/e61c98c9-b045-4f82-b705-becbe1ac3031" />  
</div>



