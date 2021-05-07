//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//
// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is the code for the Lab Project
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------
#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"
// ----------------------------------------------------------------------------
//
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"

#pragma GCC diagnostic ignored "-Wreturn-type"
/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM_PERIOD ((uint32_t)0xFFFFFFFF)
// Init Function Prototypes
void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myGPIOC_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myLCD_Init(void);
void myEXTI_Init(void);
void myADC_Init(void);
void myDAC_Init(void);
// Function Prototypes
void writeDAC(int adc_value);
int getResistance(int adc_value);
void delay(int micros);
void writeLCD(uint32_t data);
int getADC(void);
void updateLCD(int resistance, float f);
// Global Variable
volatile unsigned int risingEdge = 0;
volatile float freq = 0;
// ----- main() ---------------------------------------------------------------
int main(int argc, char* argv[])
{
 trace_printf("Nolan Kurylo's ECE355 Lab Project\n");

 myGPIOA_Init();
 myGPIOB_Init();
 myGPIOC_Init();
 myTIM2_Init();
 myTIM3_Init();
 myLCD_Init();
 myEXTI_Init();
 myADC_Init();
 myDAC_Init();
 trace_printf("Initialization Completed Successfully\n");
 while (1)
 {
 int adc_value = getADC();
 writeDAC(adc_value);
 int resistance = getResistance(adc_value);
 updateLCD(resistance, freq);
 }
 return 0;
}
// ----- INIT FUNCTIONS ---------------------------------------------------------------
void myGPIOA_Init() /* Inits PA4->4N35 Optocoupler and PA1->NE555 Timer */
{
 /* Enable clock for GPIOA peripheral */
 RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
 /* Configure PA1 as input (AF) PA4 as output */
 GPIOA->MODER &= ~(GPIO_MODER_MODER1_1 | GPIO_MODER_MODER4_0 );
 /* Ensure no pull-up/pull-down for PA1 and PA4 */
 GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1 | GPIO_PUPDR_PUPDR4);
}
void myGPIOB_Init()/* Inits 8-Bit Parallel Interface for LCD Display*/
{
 /* Enable clock for GPIOAB peripheral */

 RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
 /* Configure PB4-15 as output PB7 as as input (AF) */
 GPIOB->MODER = (GPIO_MODER_MODER4_0 |
 GPIO_MODER_MODER5_0 |
 GPIO_MODER_MODER6_0 |
 GPIO_MODER_MODER7_1 |
 GPIO_MODER_MODER8_0 |
 GPIO_MODER_MODER9_0 |
 GPIO_MODER_MODER10_0 |
 GPIO_MODER_MODER11_0 |
 GPIO_MODER_MODER12_0 |
 GPIO_MODER_MODER13_0 |
 GPIO_MODER_MODER14_0 |
 GPIO_MODER_MODER15_0);
 /* Ensure no pull-up/pull-down for PB4 */
 GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4 |
 GPIO_PUPDR_PUPDR5 |
 GPIO_PUPDR_PUPDR6 |
 GPIO_PUPDR_PUPDR7 |
 GPIO_PUPDR_PUPDR8 |
 GPIO_PUPDR_PUPDR9 |
 GPIO_PUPDR_PUPDR10 |
 GPIO_PUPDR_PUPDR11 |
 GPIO_PUPDR_PUPDR12 |
 GPIO_PUPDR_PUPDR13 |
 GPIO_PUPDR_PUPDR14 |
 GPIO_PUPDR_PUPDR15 );
}
void myGPIOC_Init() /* Inits PC1->Potentiometer */
{
 /* Enable clock for GPIOC peripheral */
 RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
 /* Configure PC1 as analog */
 GPIOC->MODER &= ~(GPIO_MODER_MODER1);
 /* Ensure no pull-up/pull-down for PC1 */
 GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);

}
void myTIM2_Init() /* Inits TIM2 for count between NE555 Timer PWM signals */
{
 /* Enable clock for TIM2 peripheral */
 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
 /* Configure TIM2: buffer auto-reload, count up, stop on overflow, enable update events, interrupt on overflow only
*/
 TIM2->CR1 = ((uint16_t)0x008C);
 /* Set clock prescaler value */
 TIM2->PSC = myTIM_PRESCALER;
 /* Set auto-reloaded delay */
 TIM2->ARR = myTIM_PERIOD;
 /* Update timer registers */
 TIM2->EGR = ((uint16_t)0x0001);
 /* Assign TIM2 interrupt priority = 0 in NVIC */
 NVIC_SetPriority(TIM2_IRQn, 0);
 /* Enable TIM2 interrupts in NVIC */
 NVIC_EnableIRQ(TIM2_IRQn);
 /* Enable update interrupt generation */
 TIM2->DIER |= TIM_DIER_UIE;
}
void myTIM3_Init() /* Inits TIM3 for LCD refresh rate delays */
{
 /* Enable clock for TIM3 peripheral */
 RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
 /* Configure TIM3: buffer auto-reload, count up, stop on overflow, enable update events, interrupt on overflow only
*/
 TIM3->CR1 = ((uint16_t)0x008C);
 /* Set clock prescaler value, */
 TIM3->PSC = myTIM_PRESCALER;
 /* Set auto-reloaded delay */
 TIM3->ARR = myTIM_PERIOD;
 /* Update timer registers */

 TIM3->EGR = ((uint16_t)0x0001);
}
void myLCD_Init() /* Inits LCD peripheral*/
{
 writeLCD(0x3800); // Function Set: set 2 lines with 8 characters each
 writeLCD(0x0C00); // turn display on, no shifting/cursor
 writeLCD(0x0600); // set entry mode -> I/D = 1 -> increments
 writeLCD(0x0100); // clear LCD display
 delay(1000); // Let LCD refresh
}
void myEXTI_Init() /* Inits EXTI->PA1 for rising edges between NE555 Timer PWM signals */
{
 /* Map EXTI2 line to PA1 */
 SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA;
 /* EXTI2 line interrupts: set rising-edge trigger */
 EXTI->RTSR |= EXTI_RTSR_TR1;
 /* Unmask interrupts from EXTI1 line */
 EXTI->IMR |= EXTI_IMR_MR1;
 /* Assign EXTI1 interrupt priority = 0 in NVIC */
 NVIC_SetPriority(EXTI0_1_IRQn, 0);
 /* Enable EXTI1 interrupts in NVIC */
 NVIC_EnableIRQ(EXTI0_1_IRQn);
}
void myADC_Init() /* Inits ADC for input potentiometer voltage */
{
 ADC1->CR = 0x0; // clear control register
 RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // enable clock for ADC
 ADC1->CHSELR |= ADC_CHSELR_CHSEL11; // use channel 11
 ADC1->CFGR1 |= ADC_CFGR1_CONT; // continuous mode
 ADC1->CFGR2 |= ADC_CFGR2_CKMODE; // ADC async clock mode
 ADC1->CR |= ADC_CR_ADCAL; // begin ADC calibration
 while(ADC1->CR == ADC_CR_ADCAL); // wait for ADC to calibrate
 ADC1->CR |= ADC_CR_ADEN; // enable ADC

 while((ADC1->CR & ADC_ISR_ADRDY) == 0); // wait for ADC ready
 ADC1->CR |= ADC_CR_ADSTART; // start ADC
}
void myDAC_Init() /* Inits DAC for out 4N35 Optocoupler voltage */
{
 DAC->CR = 0x0; //clear control register
 RCC->APB1ENR |= RCC_APB1ENR_DACEN;
 DAC->CR = DAC_CR_EN1;
}
// ----- ISRS ---------------------------------------------------------------
/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
 /* Check if update interrupt flag is indeed set */
 if ((TIM2->SR & TIM_SR_UIF) != 0)
 {
 TIM2->SR &= ~(TIM_SR_UIF); /* Clear update interrupt flag */
 TIM2->CR1 |= TIM_CR1_CEN; /* Restart stopped timer */
 }
}
/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
 float counter_value = 0;
 /* Check if EXTI1 interrupt pending flag is indeed set */
 if ((EXTI->PR & EXTI_PR_PR1) != 0)
 {
 if(risingEdge == 0){// If this is the first edge:
 TIM2->CNT = 0x0; // Clear count register
 TIM2->CR1 |= TIM_CR1_CEN; // Start timer
 risingEdge = 1; // set boolean for first rising edge
 }

 else if(risingEdge == 1){ // if this is the second edge
 risingEdge = 0; // reset edge boolean
 TIM2->CR1 &= ~(TIM_CR1_CEN); // Stop timer
 counter_value = TIM2->CNT; // Read out count register
 freq = SystemCoreClock / counter_value; // Calculate frequency and store in global variable
 }
 EXTI->PR = EXTI_PR_PR1; // Clear EXTI1 interrupt pending flag
 }
}
/*---------------------------------------------------FUNCTIONS--------------------------------------------------------*/
void delay(int micros) /* Uses TIM3 to delay main program for x micros*/
{
 TIM3->SR = 0x0; // clear status register
 TIM3->CNT = 0x0; // clear count
 TIM3->ARR = micros; // auto reload desired micros
 TIM3->CR1 |= TIM_CR1_CEN; // enable clock
 while((TIM3->SR & TIM_SR_UIF) == 0); // wait until desired micros reaches 0
 TIM3->SR = 0x0; // clear status register
}
void writeLCD(uint32_t data) /* Uses BSRR register to execute LCD handshaking protocol */
{
 GPIOB->BSRR |= 0xFFFF0000; // reset ODR bits
 GPIOB->BSRR = data; // set the ODR bits to be displayed to LCD
 GPIOB->BSRR |= GPIO_BSRR_BS_4; // Assert Enable PB[4]
 while ((GPIOB->IDR & GPIO_IDR_7) == 0); // Wait for PB[7] to become 1 "Done" -> asserted
 GPIOB->BSRR |= GPIO_BSRR_BR_4; // De-assert Enable PB[4]
 while ((GPIOB->IDR & GPIO_IDR_7) != 0); // Wait for PB[7] to become 0 "Done" -> de-asserted
}
int getADC() /* Waits until ADC value is obtained and returns it */
{
 while((ADC1->ISR & ADC_ISR_EOC) == 0);// wait until ADC is done converting
 int adc_value = ADC1->DR; // store adc value
 return adc_value;

}
void writeDAC(int adc_value) /* Writes ADC value directly to DAC*/
{
 DAC->DHR12R1 = adc_value; // 12 bit right aligned data holding register
}
int getResistance(int adc_value) /* Obtains measured resistance */
{ /* V = IR -> I = V/R -> I = 3.3V/5000Oh - > Imax = 0.00066A */
 return ((( 3.3 * adc_value) / 4095) / 0.00066); // R = Vmeasured / Imax
}
void updateLCD(int resistance, float frequency) /* Prepares data to be sent to LCD */
{
 /* 0x20 -> RS (Data = 1) R/W (Write = 0) */
 /* 0x00 -> RS (Command = 0) R/W (Write = 0) */
 /* Print frequency on first line */
 /* Isolate each digit, convert from ASCII to HEX (add 0x30), shift HEX digit to upper 8 bits, append data/write in
lower 8 bits (0x20) */
 int freq_thou = (((((int) frequency / 1000) % 10) + 0x30) << 8) | 0x20;
 int freq_hund = (((((int) frequency / 100) % 10) + 0x30) << 8) | 0x20;
 int freq_ten = (((((int) frequency / 10) % 10) + 0x30) << 8) | 0x20;
 int freq_one = ((((int) frequency % 10) + 0x30) << 8) | 0x20;
 delay(1000); // wait for LCD to refresh
 writeLCD(0x8000); // go to line one but dont write anything -> append command/write in lower 8 bits (0x20)
 writeLCD(0x4620); // F
 writeLCD(0x3A20); // :
 writeLCD(freq_thou); // #XXX
 writeLCD(freq_hund); // X#XX
 writeLCD(freq_ten); // XX#X
 writeLCD(freq_one);// XXX#
 writeLCD(0x4820); // H
 writeLCD(0x7A20); // z

 /* Print resistance on second line */
 /* Isolate each digit, convert from ASCII to HEX (add 0x30 to digit), shift HEX digit to upper 8 bits, append
data/write in lower 8 bits (0x20) */
 int resistance_thou = (((((int) resistance / 1000) % 10) + 0x30) << 8) | 0x20;
 int resistance_hund = (((((int) resistance / 100) % 10) + 0x30) << 8) | 0x20;
 int resistance_ten = (((((int) resistance / 10) % 10) + 0x30) << 8) | 0x20;
 int resistance_one = ((((int) resistance % 10) + 0x30) << 8) | 0x20;
 delay(1000);// wait for LCD to refresh
 writeLCD(0xC000); // go to line two but dont write anything -> append command/write in lower 8 bits (0x20)
 writeLCD(0x5220); // R
 writeLCD(0x3A20); // :
 writeLCD(resistance_thou); // #XXX
 writeLCD(resistance_hund); // X#XX
 writeLCD(resistance_ten); // XX#X
 writeLCD(resistance_one);// XXX#
 writeLCD(0x4F20); // O
 writeLCD(0x6820); // h
}
#pragma GCC diagnostic pop
