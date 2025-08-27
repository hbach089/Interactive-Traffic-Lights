#ifndef DRIVERS_H_   /* Include guard */
#define DRIVERS_H_

#include <stdint.h>


#define APB1_BASE_ADDRESS 0x40000000
#define APB2_BASE_ADDRESS 0x40010000
#define AHB1_BASE_ADDRESS 0x40020000
#define AHB2_BASE_ADDRESS 0x50000000
#define AHB3_BASE_ADDRESS 0xA0000000

#define GPIOA_BASE_ADDRESS AHB1_BASE_ADDRESS
#define GPIOB_BASE_ADDRESS (AHB1_BASE_ADDRESS+(0x0400))
#define GPIOC_BASE_ADDRESS (AHB1_BASE_ADDRESS+(0x0800))
#define GPIOD_BASE_ADDRESS (AHB1_BASE_ADDRESS+(0x0C00))
#define GPIOE_BASE_ADDRESS (AHB1_BASE_ADDRESS+(0x1000))
#define GPIOF_BASE_ADDRESS (AHB1_BASE_ADDRESS+(0x1400))
#define GPIOG_BASE_ADDRESS (AHB1_BASE_ADDRESS+(0x1800))
#define GPIOH_BASE_ADDRESS (AHB1_BASE_ADDRESS+(0x1C00))

#define TIM6_BASE_ADDRESS (APB1_BASE_ADDRESS+(0X1000))

#define SYSCFG_BASE_ADDRESS (APB2_BASE_ADDRESS+(0X3800))

#define EXTI_BASE_ADDRESS (APB2_BASE_ADDRESS+(0X3C00))

#define NVIC_BASE_ADDRESS 0xE000E100

#define RCC_BASE_ADDRESS (AHB1_BASE_ADDRESS+(0x3800))

#define INPUT_MODE 0b00
#define GPO_MODE   0b01
#define AF_MODE    0b10
#define ALG_MODE   0b11

#define OPT_PUSH_PULL  0b0
#define OPT_OPEN_DRAIN 0b1

#define LOW_SPEED 0b00
#define MEDIUM_SPEED 0b01
#define FAST_SPEED 0b10
#define HIGH_SPEED 0b11

#define NO_PULL 0b00
#define PULL_UP 0b01
#define PULL_DOWN 0b10
#define RESERVED 0b11

#define LOW  0b0
#define HIGH 0b1

#define BIT_MASK_FOUR 15
#define BIT_MASK_TWO 3
#define BIT_MASK_ONE 1

typedef struct{
	 uint32_t MODER;
	 uint32_t OTYPER;
	 uint32_t OSPEEDR;
	 uint32_t PUPDR;
	 uint32_t IDR;
	 uint32_t ODR;
	 uint32_t BSRR;
	 uint32_t LCKR;
	 uint32_t AFRL;
	 uint32_t AFRH;
}GPIO_RegType;

typedef struct{
	 uint32_t CR;
	 uint32_t PLLCFGR;
	 uint32_t CFGR;
	 uint32_t CIR;
	 uint32_t AHB1RSTR;
	 uint32_t AHB2RSTR;
	 uint32_t AHB3RSTR;
	 uint32_t RESERVED1;
	 uint32_t APB1RSTR;
	 uint32_t APB2RSTR;
	 uint32_t RESERVED2[2];
	 uint32_t AHB1ENR;
	 uint32_t AHB2ENR;
	 uint32_t AHB3ENR;
	 uint32_t RESERVED3;
	 uint32_t APB1ENR;
	 uint32_t APB2ENR;
	 uint32_t RESERVED4[2];
	 uint32_t AHB1LPENR;
	 uint32_t AHB2LPENR;
	 uint32_t AHB3LPENR;
	 uint32_t RESERVED5;
	 uint32_t APB1LPENR;
	 uint32_t APB2LPENR;
	 uint32_t RESERVED6[2];
	 uint32_t BDCR;
	 uint32_t CSR;
	 uint32_t RESERVED7[2];
	 uint32_t SSCGR;
	 uint32_t PLLI2SCFGR;
	 uint32_t PLLSAICFGR;
	 uint32_t DCKCFGR;
	 uint32_t CKGATENR;
	 uint32_t DCKCFGR2;
}RCC_RegType_init;

typedef struct{
	uint32_t MEMRMP;
	uint32_t PMC;
	uint32_t EXTICR[4];
	uint32_t CMPCR;
	uint32_t CFGR;
}SYSCFG_RegType;

typedef struct{
	uint32_t IMR;
	uint32_t EMR;
	uint32_t RTSR;
	uint32_t FTSR;
	uint32_t SWIER;
	uint32_t PR;
}EXTI_RegType;

typedef struct
{
  uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[24U];
  uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RESERVED1[24U];
  uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[24U];
  uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[24U];
  uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
        uint32_t RESERVED4[56U];
  uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
        uint32_t RESERVED5[644U];
  uint32_t STIR;                   /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
} NVIC_Type;

typedef struct{
	   uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
	   uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
	   uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
	   uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
	   uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
	   uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
	   uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
	   uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
	   uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
	   uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
	   uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
	   uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
	   uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
	   uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
	   uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
	   uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
	   uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
	   uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
	   uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
	   uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
	   uint32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
}TIM_RefType_init;

typedef enum
{
  /******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
  /******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare global interrupt                             */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  SAI2_IRQn                   = 91,     /*!< SAI2 global Interrupt                                             */
  QUADSPI_IRQn                = 92,     /*!< QuadSPI global Interrupt                                          */
  CEC_IRQn                    = 93,     /*!< CEC global Interrupt                                              */
  SPDIF_RX_IRQn               = 94,     /*!< SPDIF-RX global Interrupt                                          */
  FMPI2C1_EV_IRQn             = 95,     /*!< FMPI2C1 Event Interrupt                                           */
  FMPI2C1_ER_IRQn             = 96      /*!< FMPI2C1 Error Interrupt                                           */
} IRQn_Type;


#define GPIOA ((GPIO_RegType*)GPIOA_BASE_ADDRESS)
#define GPIOB ((GPIO_RegType*)GPIOB_BASE_ADDRESS)
#define GPIOC ((GPIO_RegType*)GPIOC_BASE_ADDRESS)
#define GPIOD ((GPIO_RegType*)GPIOD_BASE_ADDRESS)
#define GPIOE ((GPIO_RegType*)GPIOE_BASE_ADDRESS)
#define GPIOF ((GPIO_RegType*)GPIOF_BASE_ADDRESS)
#define GPIOG ((GPIO_RegType*)GPIOG_BASE_ADDRESS)
#define GPIOH ((GPIO_RegType*)GPIOH_BASE_ADDRESS)

#define TIM6 ((TIM_RefType_init*)TIM6_BASE_ADDRESS)
#define RCC ((RCC_RegType_init*)(RCC_BASE_ADDRESS))
#define SYSCFG ((SYSCFG_RegType*)(SYSCFG_BASE_ADDRESS))
#define EXTI ((EXTI_RegType*)(EXTI_BASE_ADDRESS))
#define NVIC ((NVIC_Type*)(NVIC_BASE_ADDRESS))


//Macro functions:

#define GPIO_CLOCK_CTRL(portNum) do{(RCC->AHB1ENR |= (BIT_MASK_ONE<<portNum));}while(0)
#define SYSCFG_EXTI_CLOCK_CTRL() do{RCC->APB2ENR |= (BIT_MASK_ONE<<14);}while(0)
#define TIM6_CLOCK_INIT() do{RCC->APB1ENR|=(BIT_MASK_ONE<<4);}while(0)

#define BOARD_GREEN_LED_ON do{Write_To_Pin(GPIOA,5,HIGH);}while(0)
#define BOARD_GREEN_LED_OFF do{Write_To_Pin(GPIOA,5,LOW);}while(0)
#define AUTO_GREEN_LED_ON do{Write_To_Pin(GPIOC,0,HIGH);}while(0)
#define AUTO_GREEN_LED_OFF do{Write_To_Pin(GPIOC,0,LOW);}while(0)
#define AUTO_RED_LED_ON do{Write_To_Pin(GPIOB,0,HIGH);}while(0)
#define AUTO_RED_LED_OFF do{Write_To_Pin(GPIOB,0,LOW);}while(0)
#define AUTO_YELLOW_LED_ON do{Write_To_Pin(GPIOC,1,HIGH);}while(0)
#define AUTO_YELLOW_LED_OFF do{Write_To_Pin(GPIOC,1,LOW);}while(0)

#define PDTR_BLUE_LED_ON do{Write_To_Pin(GPIOA,0,HIGH);}while(0)
#define PDTR_BLUE_LED_OFF do{Write_To_Pin(GPIOA,0,LOW);}while(0)
#define PDTR_RED_LED_ON do{Write_To_Pin(GPIOA,10,HIGH);}while(0)
#define PDTR_RED_LED_OFF do{Write_To_Pin(GPIOA,10,LOW);}while(0)

#define BUTTON_PRESSED !(GPIOC->IDR & (1<<13))

#define PAx 0b0000
#define PBx 0b0001
#define PCx 0b0010
#define PDx 0b0011
#define PEx 0b0100
#define PFx 0b0101
#define PGx 0b0110

void delay(uint16_t);
void GPIO_Clock_Init(GPIO_RegType*);
void NVIC_SetPriority(IRQn_Type,uint32_t);
void GPIO_Init(GPIO_RegType*,uint8_t,uint8_t,uint8_t,uint8_t,uint8_t);
void Write_To_Pin(GPIO_RegType*,uint8_t,uint8_t);
void TIM_Init(TIM_RefType_init*);
void EXTI_init(GPIO_RegType*,uint8_t,IRQn_Type);

extern volatile uint8_t set;

#endif
