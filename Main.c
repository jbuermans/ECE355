//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)

/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

//Input/Output ports
void myGPIOA_Init(void);
void myGPIOB_Init(void);

//Timer
void myTIM2_Init(void);
void myEXTI_Init(void);

//SPI for LCD
void myLCD_init();
void write_data(uint8_t cmd, uint8_t in);
void send_data(uint8_t sendData);
void mySPI_Init(void);
void print_line(int line, int in);

//Wait function
void notWait(int);


//Converters
void myADC_Init(void);
void myDAC_Init(void);


// Your global variables...
volatile unsigned int count=0;			//counter for frequency calculations
volatile double period1=0;				//Period place holder
volatile int frequency1=0;				//Frequency place holder
volatile int edge= 0;					//Flag for interrupt cycles
volatile int res = 0;					//Resistance place holer
volatile int ADC_Value;					//ADC1 value
volatile int data = ((uint8_t)0x46);	//8bit data to be pushed to SPI/LCD


int
main(int argc, char* argv[])
{
	trace_printf("This is Part 2 of Introductory Lab...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		/* Initialize I/O port PA */
	myGPIOB_Init();		/* Initialize I/O port PB */

	myADC_Init();		/* Initialize ADC */
	myDAC_Init();		/* Initialize DAC */

	mySPI_Init();		/* Initialize SPI */

	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */

	ADC1->CR |= ADC_CR_ADSTART;					//Start ADC

	while (1)
	{
		while((ADC1->ISR & ADC_ISR_EOC) != ADC_ISR_EOC);		//Waits until end of conversion
		ADC_Value = (ADC1 -> DR);								//Grabs ADC1 register value
		DAC->DHR12R1 = ADC_Value;								//Pushes ADC1 value into DAC register to be sent
		res = 5000*ADC_Value/4095;								//Converting ADC value to Resistance
	}
	return 0;
}

	/* Initializes GPIOB */
void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA1 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);						//Sets GPIOA port 1 to input
	GPIOA->MODER |= (GPIO_MODER_MODER6 |GPIO_MODER_MODER4);		//Sets GPIOA ports 4 and 6 to analog mode

	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

}

	/* Initializes GPIOB */
void myGPIOB_Init()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;							//Enables timer for GPIOB
	GPIOB->MODER |= GPIO_MODER_MODER3_1 | GPIO_MODER_MODER5_1;	//Sets GPIOB ports 3 and 5 as outputs
	GPIOB->MODER |= GPIO_MODER_MODER4_0;						//Sets GPIOB port 4 to output.
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR3 | GPIO_PUPDR_PUPDR5);	//Disables pull up/pull down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4);  						//Disables pull up/pull down
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT_4);						//push-pull mode for portB pin4
	GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4);					//high-speed mode for PB4
}

	/* nitializes timer */
void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	//RCC->APB1ENR |= ((uint32_t)0x00000001);
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0001);
	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use
	NVIC_SetPriority(TIM2_IRQn, 0);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);
	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;
}


	/* Initializes EXTI for timer */
void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] &= ((uint32_t)0xFFFFFF0F);   //Page 172 of PDF *** Chapter 10.1.2

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= ((uint32_t)0x02);

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= ((uint32_t)0x0002);

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	//NVIC->IP[0];
	NVIC_SetPriority(5, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	//NVIC->ISER[0];
	NVIC_EnableIRQ(5);
}

	/* Initializes SPI */
void mySPI_Init(){
	//This set up is found on the course web page@ https://www.ece.uvic.ca/~daler/courses/ece355/interfacex.pdf

	SPI_InitTypeDef SPI_InitStructInfo;
	SPI_InitTypeDef* SPI_InitStruct = &SPI_InitStructInfo;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;										//Enables SPI Clock

	SPI_InitStruct->SPI_BaudRatePrescaler 	= SPI_BaudRatePrescaler_256;		//Verified in lab manual BaudRate
	SPI_InitStruct->SPI_CPOL 				= SPI_CPOL_Low;					//Clock Polarity
	SPI_InitStruct->SPI_CPHA 				= SPI_CPHA_1Edge;				//Clock Phase
	SPI_InitStruct->SPI_FirstBit 			= SPI_FirstBit_MSB;				//First bit read is most significant bit
	SPI_InitStruct->SPI_CRCPolynomial 		= 7;							//Selects the polynomial used for CRC calculation
	SPI_InitStruct->SPI_Mode 				= SPI_Mode_Master;				//Mode set to Master
	SPI_InitStruct->SPI_Direction 			= SPI_Direction_1Line_Tx;		//Single directional flow
	SPI_InitStruct->SPI_DataSize 			= SPI_DataSize_8b;				//Made this 8 bits
	SPI_InitStruct->SPI_NSS					= SPI_NSS_Soft;					//Slave select management

	SPI_Init(SPI1, SPI_InitStruct);											//Final Configuration of SPI
	SPI_Cmd(SPI1, ENABLE);													//Enbles SPI
	notWait(4);																//Waits
	myLCD_init();															//Starts myLCD_init()

}
	//printing values to LCD Screen
void myLCD_init(){

	write_data(0x00, 0x2); 		//set 8bit to 4bit
	notWait(4);
	write_data(0x00, 0x28);		//DL=0 N=1 F=0
	notWait(4);
	write_data(0x00, 0x0C);		//D=1, C=0, B=0
	notWait(4);
	write_data(0x00, 0x0F);		//turn off the display
	notWait(4);
	write_data(0x00, 0x06);		//I/D=1, S=0
	notWait(4);


	write_data(0x00, 0x01);		//Clear the display
	notWait(4);

	write_data(0x0, 0x80);		//Chooses top line of display
	notWait(4);
	write_data(0x40, 0x46);		//Pushes 'F' to top line
	notWait(4);
	write_data(0x40, 0x3A);		//Pushes ':' to top line
	notWait(4);
	write_data(0x0, 0x86);		//Changes position to 6
	notWait(4);
	write_data(0x40, 0x48);		//Pushes 'H' to top line
	notWait(4);
	write_data(0x40, 0x7A);		//Pushes 'z' to top line
	notWait(4);

	write_data(0x0, 0xC0);		//Chooses bottom line of display
	notWait(4);
	write_data(0x40, 0x52);		//Pushes 'R' to bottom line
	notWait(4);
	write_data(0x40, 0x3A);		//Pushes ':' to bottom line
	notWait(4);
	write_data(0x0, 0xC6);		//Changes position to 6
	notWait(4);
	write_data(0x40, 0x4F);		//Pushes 'O' to bottom line
	notWait(4);
	write_data(0x40, 0x68);		//Pushes 'h' to bottom line
	notWait(4);
}

	/*
	 * Description: Splits character to be written into two parts
	 * Input:	CMD differentiates between data or instructions
	 * 				CMD = 0x00 IN is instruction
	 * 				CMD = 0x40 IN is data
	 * 			IN Information to be pushed to LCD
	 */
void write_data(uint8_t cmd, uint8_t in){
	int lower = ((uint8_t)0x00);		//Place holder variable for lower 4 bits
	int upper = ((uint8_t)0x00);		//Place holder variable for upper 4 bits

	lower = (in & 0x0f);				//Grabs lower 4 bits
	upper = ((in & 0xf0)>>4);			//Grabs upper 4 bits

		/* Pushes Upper information to SPI data register */
	send_data(0x00| upper| cmd);
	send_data(0x80| upper| cmd);
	send_data(0x00| upper| cmd);

		/* Pushes Lower information to SPI data register */
	send_data(0x00| lower| cmd);
	send_data(0x80| lower| cmd);
	send_data(0x00| lower| cmd);
	notWait(4);							//Waits

}
	/*
	 * Description: Sends data to SPI1
	 * Inputs: 8bit ascii character to be sent
	 */
void send_data(uint8_t sendData){
	GPIOB->BRR = (GPIO_LCKR_LCK4);		//Lock GPIOB 4
	while((SPI1->SR & 0x80)!=0);		//Waits until TXE =1 or BSY = 0

	SPI_SendData8(SPI1, sendData);
	while((SPI1->SR & 0x80)!=0);		//Waits until TXE =1 or BSY = 0

	GPIOB->BSRR = GPIO_LCKR_LCK4;		//Unlocks GPIOB 4
}


	/* Configure the ADC to to read the POT */
void myADC_Init(){
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN; 				//Enables ADC Clock

	ADC1->CR &= ~(ADC_CR_ADEN);						//turn off
	ADC1->CR |= ADC_CR_ADCAL;						//Calibrate
	while((ADC1->CR & ADC_CR_ADCAL) != 0);			//wait

	ADC1->CFGR1 |= (ADC_CFGR1_CONT);				//Enables Continuous mode for ADC
	ADC1->CFGR1 |= ADC_CFGR1_OVRMOD;				//Enables overrun mode
	ADC1->CFGR1 &= ~(ADC_CFGR1_ALIGN);				//Right Aligned
	ADC1->CFGR1 &= ~(ADC_CFGR1_RES);				//Resolution set to 12 bits

	ADC1->CHSELR |= ADC_CHSELR_CHSEL6; 				//Enable Channel 6 for ADC analog input

	ADC1->SMPR  |= ADC_SMPR_SMP;					//Sampling time set to 239.5 ADC clock cycles

	ADC1->CR |= (ADC_CR_ADEN);  					//Enables ADC
	while(((ADC1 ->ISR) & (ADC_ISR_ADRDY)) != 1);	//Waits until ADC is read to start conversion
}

	/* Configures DAC to output read ADC */
void myDAC_Init(){

	RCC->APB1ENR |= RCC_APB1ENR_DACEN;		//Enables DAC Clock
	DAC->CR |= DAC_CR_EN1;					//Powers channel one
}

	/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		//TIM2->SR &= ((uint32_t)0xFFF0);
		TIM2->SR &= ~(TIM_SR_UIF);
		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		//TIM2->CR1 = ((unit32_t));
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}

	/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		if(edge == 1){							//If interrupt happened only once reset timer and start counting
			edge = 0;							//First interrupt flag
			TIM2->CNT = 0x00000000;				//Set count value to 0
			TIM2->CR1 |= 0x1;					//Start counter
			EXTI->PR = 0x02;					//Reset interrupt
		}
		else{

			TIM2->CR1 &= 0x0000;				//On second interrupt
			count = TIM2->CNT;					//Grab Counter value for equations
			EXTI->IMR &= ~((uint32_t)0x0002);	//Mask
			period1 = ((double)count)/48000000;	//Calculate the period
			frequency1 = 48000000/(count);		//Calculate the frequency

			print_line(0, frequency1);			//Prush frequency to top line
			print_line(1, res);					//Push Resistance to bottom line
			edge = 1;							//Reset flag
			EXTI->IMR |= ((uint32_t)0x0002);	//Unmask
			EXTI->PR = 0x02;					//Reset interrupt
		}
	}
}

	/* Description: counts to certain value, meant to stall microprocessor so that SPI can send data to LCD
	 * Input: Integer time
	 */
void notWait(int time){
	static volatile uint32_t i;
	for(i =0; i< time*50000; i++);
}

	/* Description: Creates and array that pushes values to LCD
	 * Input: Two input, values line choses which line of LCD to write to
	 * in value is either resistance or frequency at a instances
	 */
void print_line(int line, int in){
	uint8_t print_v[4];						//Place holder for vvalues to be pushed
	if (line == 0){
		//change to first line
		write_data(0x0, 0x82);
		notWait(4);
		print_v[0]= (in/1000)+0x30;			//Prints Character 3 from 4 character input
		print_v[1]= (in%1000/100)+0x30;		//Prints Character 2 from 4 character input
		print_v[2]= (in%100/10)+0x30;		//Prints Character 1 from 4 character input
		print_v[3]= (in%10/1)+0x30;			//Prints Character 0 from 4 character input
	}
	else if(line == 1){
		//change pointer to second line
		write_data(0x0, 0xC2);
		notWait(4);
		print_v[0]= (in/1000)+0x30;			//Prints Character 3 from 4 character input
		print_v[1]= (in%1000/100)+0x30;		//Prints Character 2 from 4 character input
		print_v[2]= (in%100/10)+0x30;		//Prints Character 1 from 4 character input
		print_v[3]= (in%10/1)+0x30;			//Prints Character 0 from 4 character input
	}
	for(int i=0;i<4;i++){					//Iterates through array pushing values to LCD
		write_data(0x40, print_v[i]);
		notWait(4);
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
