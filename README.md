# Bare Metal Interactive Traffic Light simulator built with custom GPIO drivers on STM32F446RE.

## Table of contents

- [General Info](#general-info)
- [Documentation](#documentation)
- [Project Setup](#project-setup)


## General Info

- Board: **STM32F446RE**
- Each peripheral has a 32 bit address which provides the basis of this project.
- GPIO and RCC peripherals connected to AHB1 bus (0x4002 0000).
- GPIOA, GPIOB, GPIOC and RCC were used.

## Documentation

For a more in depth understanding of GPIO and RCC drivers, please refer to the following sections in the [reference manual](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf):
- [Memory organization](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=56)
- [RCC registers](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=116)
- [GPIO registers](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf#page=175)

## Project Setup

- The GPIO clock is initialized by enabling a bit corresponding to it under the RCC register.
- The GPIO peripheral is initialized with its registers to configure the I/O operation.
- Program the GPIO pin for the desired operation.

<div align=center>
  <img width="431" height="360" alt="image" src="https://github.com/user-attachments/assets/e61c98c9-b045-4f82-b705-becbe1ac3031" />  
    <h2><a href="https://github.com/user-attachments/assets/c833410d-1621-4811-af2f-cb709f47c932"> Live Demo</a></h2>
</div>
<video src="https://github.com/hbach089/Interactive-Traffic-Lights/blob/main/Video/463489657-c833410d-1621-4811-af2f-cb709f47c932.mp4" width="320" height="240" controls></video>


