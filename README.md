# Bare Metal Interactive Traffic Light simulator built with custom GPIO drivers on STM32F446RE.

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

<div display="flex">
  <h3> <a href="https://github.com/user-attachments/assets/c833410d-1621-4811-af2f-cb709f47c932"><u>Live Demo!</u>  </a>  </h3>
    <img width="341" height="259" alt="image" src="https://github.com/user-attachments/assets/e61c98c9-b045-4f82-b705-becbe1ac3031" />    

</div>
