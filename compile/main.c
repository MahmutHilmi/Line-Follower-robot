// EMBEDDED SYSTEM AND FEEDBACK CONTROL CAPSULE
// LINE-FOLLOWER ROBOT PROJECT
/*AUTHORS :
ÖMER BOZKURT
MAHMUT HILMI KAYA
 
 /////////ALL CODES WERE WRITTEN BY THE ABOVE NAMED AUTHORS//////////
        
*/
#include "stm32f10x.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

void GPIO_clock_init(void);
void PWM_generator(void);
void TIM2config(void);
int QTR_position_read(int *sensor);
void delay_us(uint16_t us); 
void delay_ms(uint16_t ms);

int error_vector[8] = { -20, -15, -10, -5, 5, 10, 15, 20 };
int proportionalerror = 0;
double KP = 3.3989;
double KD = 6.0484;
double KI = 0.00015;

#define MaximumPWM 85

void PLLClockConfiguration()
{
		if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL) /* (1) */
		{
		 RCC->CFGR &= (uint32_t) (~RCC_CFGR_SW); /* (2) */
		 while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) /* (3) */
		 {
		 /* For robust implementation, add here time-out management */
		 }
		}
		RCC->CR &= (uint32_t)(~RCC_CR_PLLON);/* (4) */
		while((RCC->CR & RCC_CR_PLLRDY) != 0) /* (5) */
		{
		 /* For robust implementation, add here time-out management */
		}
		
		RCC->CFGR &=  ~(RCC_CFGR_PLLSRC) | RCC_CFGR_PLLSRC_HSI_Div2;		// PLL source is HSI/2
		
		
		RCC->CFGR = RCC->CFGR & (RCC_CFGR_PLLMULL_2) | (RCC_CFGR_PLLMULL_1)| (RCC_CFGR_PLLMULL); /* (6)) */

		RCC->CR |= RCC_CR_PLLON; /* (7) */
		while((RCC->CR & RCC_CR_PLLRDY) == 0) /* (8) */
		{
		 /* For robust implementation, add here time-out management */
		}
		RCC->CFGR |= (uint32_t) (RCC_CFGR_SW_PLL); /* (9) */
		while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) /* (10) */
		{
		 /* For robust implementation, add here time-out management */
		}
}
void TIM2config(void) {
//enable timer clk/
	RCC->APB1ENR |= (1 << 0);
	//ARR value/
	TIM2->ARR = 0xffff - 1;
	//*SET UP PRESCALER */
	TIM2->PSC = 72 - 1; //--us/
	//enable timer/
	TIM2->CR1 |= (1 << 0);
	//Enable update flag/
	while (!(TIM2->SR & (1 << 0)))
		; //wait to set/
}

void delay_us(uint16_t us) {
	TIM2->CNT = 0; //set counter to 0/
	while (TIM2->CNT < us)
		;

}

void delay_ms(uint16_t ms) { // each clock cycle is 1us therefore counter increase 1 for each 1us.
	for (uint16_t i = 0; i < ms; i++) {
		delay_us(1000);
	}
}
void GPIO_clock_init(void) {

	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // enable A port clk 
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; //enable B port clk


	GPIOB->CRL = 0x33333333; //IT IS KEPT ON TO RECEIVE HIGHVOLTAGE FROM ANY B-PIN.

}

int QTR_position_read(int *sensor) {

	GPIOA->CRL |= 0x33333333; //All low pins are set output mode 50MHz, output push-pull
	GPIOA->ODR |= 0xFFFFFFFF; // all pins set 1 
	delay_us(12); //wait 12us for charging the capacitors of sensors

 GPIOA -> CRL &= 0x00000000;
 GPIOA -> CRL |= 0x88888888; //All loow pins are set input pull-up/pull-down
	delay_ms(6); // Capacitor voltages are read for 6 ms 

	int numberofSensors = 0;
	int position = 0;

	for (int i = 0; i < 8; i++) {

		if ((GPIOA->IDR >> i) & 1) {
			sensor[i] = 1;
			position += error_vector[i];
		} else {
			sensor[i] = 0;
		}
		numberofSensors = numberofSensors + sensor[i];
	}
	return position / numberofSensors;
}

void PWM_generator(void) {
	// for generationg PWM signals TIM1 was used 
	// TIM1 has more channels for getting PWM
	// channel 1 and channel 4 was used
	// default duty cycles were entered as 50%
	// channel 1 output is A8 and channel4 output is A11 pins
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_TIM1EN;

	GPIOA->CRH |= GPIO_CRL_MODE0;
	GPIOA->CRH |= (GPIO_CRL_CNF0_1);
	GPIOA->CRH &= ~(GPIO_CRL_CNF0_0);
	GPIOA->CRH |= GPIO_CRL_MODE3;
	GPIOA->CRH |= (GPIO_CRL_CNF3_1);
	GPIOA->CRH &= ~(GPIO_CRL_CNF3_0);
	TIM1->PSC = 710;
	TIM1->ARR = 100;
	TIM1->CR1 |= TIM_CR1_CEN;
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC4E;
	TIM1->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
	TIM1->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1;
	TIM1->BDTR |= (1 << 15);
	AFIO->MAPR &= ~AFIO_MAPR_TIM1_REMAP;
	TIM1->CCR1 = 50;
	TIM1->CCR4 = 50;
}

void initClockHSI (void)
{
    RCC->CR|= RCC_CR_HSION ; // enable internal HSI(RC)

    while((RCC->CR & RCC_CR_HSIRDY)!=RCC_CR_HSIRDY);  // wait HSI to be ready

    RCC->CFGR &= ~(RCC_CFGR_SW); //clear SW bits
    RCC->CFGR |= RCC_CFGR_SW_HSI; //set HSI as system clock

    while( (RCC-> CFGR & RCC_CFGR_SWS_HSI)!= RCC_CFGR_SWS_HSI); // wait HSI to be system clock
}


int main(void) {
		
	PWM_generator();
	GPIO_clock_init();
	TIM2config();
	PLLClockConfiguration();
	
	int32_t currentPositionError = 0;
	int32_t previousPositionError = 0;
	int32_t proportionalPosition = 0;
	int32_t derivativePosition = 0;
	int64_t integralPosition = 0;
	int32_t output = 0;

	while (1) {
		
		int sensor[8];
		int positionError = QTR_position_read(sensor);

		bool empty = true; // In cases where no position information is read from the sensors,
//		the pwm value is increased by taking the previous error value.
		for (int i = 0; i < 8; i++) {
			if (sensor[i] == 1) {
				empty = false;
				break;
			}
		}

		if (empty) {
			currentPositionError = previousPositionError * 12; 
			//The previous error value was multiplied by 12 and equalized to the current error value.
      //The purpose of this process is to ensure that the robot goes at maximum speed in the direction 
			//of the last line it sees when position information is not received.
			
			
			//Since the current position value will be multiplied by 12 in each loop,
			//the current postion error is limited to 1500 at this stage.
			
			if (currentPositionError < -1500)
				currentPositionError = -1500;
			else if (currentPositionError > 1500)
				currentPositionError = 1500;
		} else {
			currentPositionError = positionError;
		}

		proportionalPosition = currentPositionError;
		derivativePosition = currentPositionError - previousPositionError;
		integralPosition += currentPositionError;
		previousPositionError = currentPositionError;
		output = proportionalPosition * KP + derivativePosition * KD	+ integralPosition * KI;

		if (output > MaximumPWM)
			output = MaximumPWM;
		else if (output < -MaximumPWM)
			output = -MaximumPWM;

		if (output < 0) {
			TIM1->CCR1 = MaximumPWM + output;
			TIM1->CCR4 = MaximumPWM;
		} else {
			TIM1->CCR1 = MaximumPWM;
			TIM1->CCR4 = MaximumPWM - output;
		}

	}
}