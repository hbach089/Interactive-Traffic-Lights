#include "drivers.h"

volatile uint8_t set;

void GPIO_Clock_Init(GPIO_RegType *GPIOx){
	if(GPIOx==GPIOA){
		GPIO_CLOCK_CTRL(0);
	}
	else if(GPIOx==GPIOB){
		GPIO_CLOCK_CTRL(1);
	}
	else if(GPIOx==GPIOC){
		GPIO_CLOCK_CTRL(2);
	}
	else if(GPIOx==GPIOD){
		GPIO_CLOCK_CTRL(3);
	}
	else if(GPIOx==GPIOE){
		GPIO_CLOCK_CTRL(4);
	}
	else if(GPIOx==GPIOF){
		GPIO_CLOCK_CTRL(5);
	}
	else if(GPIOx==GPIOG){
		GPIO_CLOCK_CTRL(6);
	}
	else if(GPIOx==GPIOH){
		GPIO_CLOCK_CTRL(7);
	}
}

void NVIC_SetPriority(IRQn_Type IRQn,uint32_t priority){
	NVIC->IP[EXTI15_10_IRQn]=(priority<<(4 & 0xFF));
}

void NVIC_EnableIRQ(IRQn_Type IRQn){
	NVIC->ISER[(((uint32_t)EXTI15_10_IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)EXTI15_10_IRQn) & 0x1FUL));
}

void GPIO_Init(GPIO_RegType*GPIOx,uint8_t pinNum,
		uint8_t mode,
		uint8_t outputType,
		uint8_t speed,
		uint8_t pullmode
		){
	uint32_t temp;


	GPIO_Clock_Init(GPIOx);

	temp=0;
	GPIOx->MODER &=~(BIT_MASK_TWO<<(2*pinNum));
	temp |=(mode<<(2*pinNum));
	GPIOx->MODER |= temp;

	temp=0;
	GPIOx->PUPDR &=~(BIT_MASK_TWO<<(2*pinNum));
	temp |= (pullmode<<(2*pinNum));
	GPIOx->PUPDR |= temp;

	if(mode==GPO_MODE){
		temp=0;
		GPIOx->OTYPER &=~(BIT_MASK_ONE<<(pinNum));
		temp |= (outputType<<pinNum);
		GPIOx->OTYPER |= temp;

		temp=0;
		GPIOx->OSPEEDR &=~(BIT_MASK_TWO<<(2*pinNum));
		temp |= (speed<<(2*pinNum));
		GPIOx->OSPEEDR |= temp;
	}

	//Simple I/O operations; no need for AF registers.
}

void Write_To_Pin(GPIO_RegType*GPIOx,uint8_t pinNum,uint8_t value){
	uint32_t temp=0;
	if(value==HIGH){
		temp |= (HIGH<<pinNum);
		GPIOx->BSRR|=temp;
	}
	else{
		temp |= (HIGH<<(pinNum+16));
		GPIOx->BSRR|=temp;
	}
}

void TIM_Init(TIM_RefType_init*TIMx){
	if(TIMx==TIM6){
		// Initialize clock for TIM6
		TIM6_CLOCK_INIT();

		//Timer will count 2000 times in 1 second, and roll back after 2 (meaning 2 microseconds)
		TIMx->CR1|=(1<<0);
		TIMx->PSC=8000-1;
		TIMx->ARR=2-1;
		TIMx->CNT=0;

		//Timer PSC and ARR are preloaded and can only work if we generate event after loading them
		//with desired values
		TIMx->EGR|=(BIT_MASK_ONE<<0);
		TIMx->SR=~(BIT_MASK_ONE<<0);
	}
}

void delay(uint16_t mseconds){
	for(uint16_t i=0;i<mseconds;i++){
		TIM6->CNT=0;
		while(!(TIM6->SR & (BIT_MASK_ONE<<0))){}
		TIM6->SR=~(BIT_MASK_ONE<<0);
	}
}

void EXTI_init(GPIO_RegType*GPIOx,uint8_t pinNum,IRQn_Type IRQn){
	//Enable SYSCFG clock
	SYSCFG_EXTI_CLOCK_CTRL();

	//EXTI IMR to control which interrupt line will reach processor (enable it):
	EXTI->IMR&=~(BIT_MASK_ONE<<pinNum);
	EXTI->IMR|=(BIT_MASK_ONE<<pinNum);

	// Rising edge detection disabled
	EXTI->RTSR&=~(BIT_MASK_ONE<<pinNum);

	// Falling edge detection enabled
	EXTI->FTSR&=~(BIT_MASK_ONE<<pinNum);
	EXTI->FTSR|=(BIT_MASK_ONE<<pinNum);

	// SYSCFG EXTICR to select which GPIO pin/port will act as interrupt:
	// (the source of interrupt)
	if(pinNum<=15 && pinNum>=12){
		SYSCFG->EXTICR[3]&=~(BIT_MASK_FOUR<<((pinNum%4)*4));
		SYSCFG->EXTICR[3]|=(PCx<<((pinNum%4)*4));
	}

	// NVIC peripheral sets the priority and enables the hardware for interrupt
	uint8_t priority=0;
	NVIC_SetPriority(IRQn,priority);
	NVIC_EnableIRQ(IRQn);
}
