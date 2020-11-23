//********************************************************************
// INCLUDE FILES
//====================================================================
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
//====================================================================
// SYMBOLIC CONSTANTS
//====================================================================
//INPUT SWITCHES -
#define SW0 GPIO_IDR_0 //USED TO START MOTOR
#define SW1 GPIO_IDR_1	//USED TO SELECT L/R hand rule
#define SW2 GPIO_IDR_2
#define SW3 GPIO_IDR_3
//====================================================================
//INPUT PORTS - Sensors
//====================================================================
//sensors are PB6,7,8,12, 13, 14
//No longer true as I am trying to develop ADC code rn

#define Fleft 	((GPIOA->IDR&GPIO_IDR_8)) //   PA8 	-> LED PB2
#define Left 	((GPIOA->IDR&GPIO_IDR_7)) //   PA7	-> LED PB3
#define Middle 	((GPIOB->IDR&GPIO_IDR_13)) //  PB13 -> LED PB6
#define Right 	((GPIOB->IDR&GPIO_IDR_14)) //  PB14 -> LED PB7
#define Fright 	((GPIOB->IDR&GPIO_IDR_12)) //  PB12 -> LED PB8

//====================================================================
/* OUTPUT PORTS
//====================================================================
 * PWM:PB10 (right Motor),PB11(left motor) or (PB0,1,4,5)
 * LED= (PB2,PB3,PB6,PB7,PB8)
 * TO DO:
 * NOTES PWM:
 * PB0-Right Motor Fwd Drive
 * PB1-Right Motor Reverse Drive
 * PB4-Left Motor Fwd drive
 * PB5-Left Motor Reverse Drive
 */

//====================================================================
// GLOBAL VARIABLES
//====================================================================
int value = 0;	//this is the counter variable
int DELAY1 = 1000;	//both of these are used to create a delay to prevent skipping
int selection = 0;
int junction = 0; //junction type variable
char previous= 'X';	//used in evaluation
int current = 0;	//used in evaluation
int turn = 2000; 	//time used for the turn delay
int rotate = 2000;	//time used for the rotate delay
int counter = 0; //used in sensor testing function
int stop = 2000; //time taken for motor to come to a complete stop
int sw1 = 1;
int sw0 = 0;
int sw2 = 1;
int finished = 0;
// Variable for Optimization

char path[100]; //sample path to test sort algorithm
int length;
int backflag = 0;
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_functions(void);
//SENSING GENERAL DISPLAY
void initPorts(void);
void LED(void);
void Flash(void);

//GENERAL FUNCTIONS
void EXTI0 (void);
void init_EXTI (void);
void EXTI0_1_IRQHandler (void);
void init_TIM14 (void);
void TIM14_IRQHandler (void);
void Delay(int);
void init_EXTI1 (void);
void EXTI2_3_IRQHandler (void);
void init_TIM6 (void);
void TIM6_IRQHandler (void);

//MAZE SOLVING;
void Simplify(void);
void Record(char);
void Race(void);


//DRIVING

void init_pwm (void);
void PWM_left(int);
void PWM_right(int);
void PWM_backright(int duty);
void PWM_backleft(int duty);

void Forward(void);
void LeftTurn (void);
void RightTurn (void);
void LeftTurnSharp(void);
void RightTurnSharp(void);
void Back(void);
void Stop(void);
void ScootRight(void);
void ScootLeft(void);
void Finished(void);
void Peak(void);
void DeadEnd(void);

//Testing
void Test(void);
void SensorTest(void);
void DriveTest(void);
void LEDtest(void);
void SensorDriveTest(void);

//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
{
	init_functions();

	//LEDtest();

	while (selection ==0){
		GPIOB->ODR |= ~0b0;
	}
	Delay(5000);
	for (;;){						//displaying values of sensors (on indicates a line)
		LED();
		SensorDriveTest();
	}
}										    // End of main

//====================================================================
// FUNCTION DEFINITIONS
//====================================================================

//====================================================================
// PORT SETUP and SWITCHES
//====================================================================
void init_functions(void)
{
	initPorts();
	init_LCD();							// Initialise lcd
	lcd_putstring("Max Gobel");		    // Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Mechatronics");		// Display string on line 2
	//init_TIM14();						//initiation for timer 14
	init_pwm();							//pwm program
	//EXTI0();
	//init_EXTI();
	init_EXTI1();

}

void initPorts(void)
{
	/*sets up digital inputs on GPIO A and B
	 * as well as output on GPIOB
	 */
	RCC->AHBENR|=RCC_AHBENR_GPIOAEN;	 //enable input
	GPIOA->PUPDR  |= 0b010101;			//setting pins PA0 - 3 to pull up. goes low when pressed. For switches
	RCC->AHBENR|=RCC_AHBENR_GPIOBEN;

	//Setting up the sensors, PB5,6,7,9 and 12 onwards, with pull up resistors
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR8_0;
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR7_0;
	GPIOB->MODER &= ~GPIO_MODER_MODER12; //set input to PB0
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR12;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR12_0;
	GPIOB->MODER &= ~GPIO_MODER_MODER13; //set input to PB6
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR13;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR13_0; //enable pull-down for PB0
	GPIOB->MODER &= ~GPIO_MODER_MODER14; //set input to PB7
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR14;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR14_0;

	/*setup for the ADC CONVERSION*/

	/* FOR ALTERNATIVE VERSION*/
	GPIOB->MODER |= GPIO_MODER_MODER2_0;
	GPIOB->MODER |= GPIO_MODER_MODER3_0;
	GPIOB->MODER |= GPIO_MODER_MODER6_0;
	GPIOB->MODER |= GPIO_MODER_MODER7_0;
	GPIOB->MODER |= GPIO_MODER_MODER8_0;
	GPIOB->MODER |= GPIO_MODER_MODER15_0;


}

//====================================================================
// PWM, ADC, TIMERS
//====================================================================
void init_pwm (void)

{
	/*initiates PWM on ports PB10 and PB11
	 * PB10 controls
	 * PB11 controls
	 */

	RCC->AHBENR|=RCC_AHBENR_GPIOBEN; //enable clock for port B
	GPIOB->MODER|=GPIO_MODER_MODER0_1; //set PB0 to AF mode
	GPIOB->MODER|=GPIO_MODER_MODER1_1; //set PB1 to AF mode
	GPIOB->MODER|=GPIO_MODER_MODER4_1; //set PB4 to AF mode
	GPIOB->MODER|=GPIO_MODER_MODER5_1; //set PB5 to AF mode

	RCC->APB1ENR|=RCC_APB1ENR_TIM3EN; //enable clock for TIM3

	GPIOB->AFR[0] |= 0b100010000000000010001;//Mapping PB0, PB1, PB4, and PB5

	TIM3->ARR = 9900; //
	TIM3->CCMR1 |= (TIM_CCMR1_OC1M_2) | TIM_CCMR1_OC1M_1; //PWM mode 1
	TIM3->CCMR1 |= (TIM_CCMR1_OC2M_2) | TIM_CCMR1_OC2M_1; //PWM mode 1
	TIM3->CCMR2 |= (TIM_CCMR2_OC3M_2) | TIM_CCMR2_OC3M_1; //PWM mode 1
	TIM3->CCMR2 |= (TIM_CCMR2_OC4M_2) | TIM_CCMR2_OC4M_1; //PWM mode 1

	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCER |= TIM_CCER_CC2E;
	TIM3->CCER |= TIM_CCER_CC3E;
	TIM3->CCER |= TIM_CCER_CC4E;

	TIM3->CR1 |= TIM_CR1_CEN;

}

//====================================================================
// SENSOR CALIBRATION AND ADC INTEGRATION
//====================================================================

//====================================================================
// TESTING FUNCTIONS
//====================================================================

void LEDtest(void){
	/*Tests output LEDS by turning them off
	 * and on, incrementing up and down
	 */
	GPIOB->ODR &= 0b0; //CLEARS LIGHTS
	Delay(500);
	GPIOB->ODR |= ~0b0; //TURNS ON THE FIVE LIGHTS
	Delay(500);
	GPIOB->ODR &= 0b0; //CLEARS LIGHTS
	Delay(500);
	//INCREMENTS
	GPIOB->ODR |=GPIO_ODR_8; //TURNS ON THE FIVE LIGHTS
	Delay(500);
	GPIOB->ODR |=GPIO_ODR_7; //CLEARS LIGHTS
	Delay(500);
	GPIOB->ODR |=GPIO_ODR_6; //TURNS ON THE FIVE LIGHTS
	Delay(500);
	GPIOB->ODR |=GPIO_ODR_3;
	Delay(500);
	GPIOB->ODR |=GPIO_ODR_2;
	Delay(500);
	GPIOB -> ODR &= ~GPIO_ODR_8;
	Delay(500);
	GPIOB -> ODR &= ~GPIO_ODR_7;
	Delay(500);
	GPIOB -> ODR &= ~GPIO_ODR_6;
	Delay(500);
	GPIOB -> ODR &= ~GPIO_ODR_3;
	Delay(500);
	GPIOB -> ODR &= ~GPIO_ODR_2;
	Delay(500);
	GPIOB->ODR &= 0b0; //CLEARS LIGHTS
}




void Test(void){
	counter = -1;
	lcd_command(CLEAR);
	lcd_putstring("Drive Test");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Press SW1 to increment between modes");	// Display string on line 1
	delay(3000);
	while (counter == -1){
		while((GPIOA->IDR & SW1)==0){
			counter =0;
		}
	}
	Forward(); //start driving
	lcd_command(CLEAR);
	lcd_putstring("Test 1");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Drive Both Wheels");	// Display string on line 1
	delay(5000);
	Stop();
	lcd_command(CLEAR);
	lcd_putstring("Test 1");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Complete");	// Display string on line 1
	while (counter == 0){
		while((GPIOA->IDR & SW1)==0)
			{
				counter=1;
				delay(1000);
			}
	}
	lcd_command(CLEAR);
	lcd_putstring("Test 2");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Left Turn");	// Display string on line 1
	delay(1000);
	LeftTurn();
	Stop();
	lcd_command(CLEAR);
	lcd_putstring("Test 2");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Complete");	// Display string on line
	while (counter == 1){
		while((GPIOA->IDR & SW1)==0){
			counter=2;
		}
	}

	lcd_command(CLEAR);
	lcd_putstring("Test 3");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Right Turn");	// Display string on line 1
	RightTurn();
	Stop();
	lcd_command(CLEAR);
	lcd_putstring("Test 3");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Complete");	// Display string on line
	while (counter == 2){
			while((GPIOA->IDR & SW1)==0)	{
				counter=3;
					}
	}

	lcd_command(CLEAR);
	lcd_putstring("Test 4");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Stop Test");	// Display string on line 1
	Forward();
	Delay(4000);
	Stop();
	lcd_command(CLEAR);
	lcd_putstring("Test 4");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Complete");	// Display string on line 1
	while(counter==3){
		while((GPIOA->IDR & SW1)==0)	{
						counter=4;
						Stop();
							}
	}
	lcd_command(CLEAR);
	lcd_putstring("Drive Test");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Complete");	// Display string on line 1
	delay(3000);
	while(counter==4){
		while((GPIOA->IDR & SW1)==0)	{
							counter=5;
							Stop();
									   }
	}

}

void DriveTest(void){
	Stop();//clearing all PWM values

	lcd_command(CLEAR);
	lcd_putstring("Left Motor");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Test");	// Display string on line 1
	PWM_left(100);
	Delay(2000);
	Stop();//clearing
	Delay(1000);

	lcd_command(CLEAR);
	lcd_putstring("Right Motor");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Test");	// Display string on line 1
	PWM_right(100);
	Delay(2000);
	Stop();//clearing
	Delay(1000);

	lcd_command(CLEAR);
	lcd_putstring("Left Motor");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Reverse Test");	// Display string on line 1
	PWM_backleft(100);
	Delay(2000);
	Stop();//clearing
	Delay(1000);

	lcd_command(CLEAR);
	lcd_putstring("Right Motor");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Reverse Test");	// Display string on line 1
	PWM_backright(100);
	Delay(2000);
	Stop();//clearing
	Delay(1000);

	Forward();
	Delay(2000);
	Stop();
	Delay(2000);
	PWM_backleft(100);
	PWM_backright(100);
	Delay(2000);
	Stop();

}

void SensorTest(void){
		counter = 0;
		lcd_command(CLEAR);
		lcd_putstring("Sensor Test");	// Display string on line 1
		lcd_command(LINE_TWO);				// Move cursor to line 2
		lcd_putstring("Left to Right");	// Display string on line 1
		delay(3000);
		while (counter == 0){
		delay(500);
		if (Fleft == 0){
			lcd_command(CLEAR);
			lcd_putstring("Far left");	// Display string on line 1
			lcd_command(LINE_TWO);				// Move cursor to line 2
			lcd_putstring("Line detected");	// Display string on line 1
		}

		if (Fleft!=0){
				lcd_command(CLEAR);
				lcd_putstring("Fleft");	// Display string on line 1
				lcd_command(LINE_TWO);				// Move cursor to line 2
				lcd_putstring("No Line");	// Display string on line 1
			}

		while((GPIOA->IDR & SW1)==0)
			{
				counter=1;
				delay(500);
			}
		}

		while (counter == 1){ 	//before you click
		delay(3000);
		if (Left==0){
			lcd_command(CLEAR);
			lcd_putstring("Left Sensor Test");	// Display string on line 1
			lcd_command(LINE_TWO);				// Move cursor to line 2
			lcd_putstring("Line detected");	// Display string on line 1
		}

		if (Left!=0){
				lcd_command(CLEAR);
				lcd_putstring("left");	// Display string on line 1
				lcd_command(LINE_TWO);				// Move cursor to line 2
				lcd_putstring("No Line");	// Display string on line 1
			}

		while((GPIOA->IDR & SW1)==0)
			{
				counter=2;
				delay(500);
			}
		}

		lcd_command(CLEAR);
		lcd_putstring("Sensor Test");	// Display string on line 1
		lcd_command(LINE_TWO);				// Move cursor to line 2
		lcd_putstring("Middle");	// Display string on line 1
		while (counter == 2){
		delay(3000);
		if (Middle==0){
				lcd_command(CLEAR);
				lcd_putstring("Middle");	// Display string on line 1
				lcd_command(LINE_TWO);				// Move cursor to line 2
				lcd_putstring("Line");	// Display string on line 1
			}
		if (Middle!=0){
					lcd_command(CLEAR);
					lcd_putstring("Middle");	// Display string on line 1
					lcd_command(LINE_TWO);				// Move cursor to line 2
					lcd_putstring("No Line");	// Display string on line 1
			}
		while((GPIOA->IDR & SW1)==0)
				{
					counter=3;
					delay(500);
				}

		}
		lcd_command(CLEAR);
		lcd_putstring("Sensor Test");	// Display string on line 1
		lcd_command(LINE_TWO);				// Move cursor to line 2
		lcd_putstring("Right");	// Display string on line 1
		while (counter == 3){ 	//before you click
		delay(3000);
		if (Right==0){
				lcd_command(CLEAR);
				lcd_putstring("Right");	// Display string on line 1
				lcd_command(LINE_TWO);				// Move cursor to line 2
				lcd_putstring("Line");	// Display string on line 1
			}
		if (Right!=0){
					lcd_command(CLEAR);
					lcd_putstring("Right");	// Display string on line 1
					lcd_command(LINE_TWO);				// Move cursor to line 2
					lcd_putstring("No Line");	// Display string on line 1
			}

		while((GPIOA->IDR & SW1)==0)
			{
				counter=4;
				delay(500);
			}

		}
		lcd_command(CLEAR);
		lcd_putstring("Sensor Test");	// Display string on line 1
		lcd_command(LINE_TWO);				// Move cursor to line 2
		lcd_putstring("Far Right");	// Display string on line 1
		while (counter == 4){ 	//before you click
		delay(3000);
		if (Fright==0){
				lcd_command(CLEAR);
				lcd_putstring("FRight");	// Display string on line 1
				lcd_command(LINE_TWO);				// Move cursor to line 2
				lcd_putstring("Line");	// Display string on line 1
			}
			if (Fright!=0){
					lcd_command(CLEAR);
					lcd_putstring("FRight");	// Display string on line 1
					lcd_command(LINE_TWO);				// Move cursor to line 2
					lcd_putstring("No Line");	// Display string on line 1
				}

			while((GPIOA->IDR & SW1)==0)
			{
				counter=5;
				delay(500);
			}
			}
	delay(500);
	lcd_command(CLEAR);
	lcd_putstring("Test");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("Complete");	// Display string on line 1
	delay(2000);
	lcd_command(CLEAR);
	lcd_putstring("Click SW1");	// Display string on line 1
	lcd_command(LINE_TWO);				// Move cursor to line 2
	lcd_putstring("To Begin");	// Display string on line 1
	delay(2000);
	while (counter == 5){
	if((GPIOA->IDR & SW1)==0)
			{
				counter=6;
				delay(500);
			}
	}
}

void SensorDriveTest(void){
	while (((Middle == 0) || (Right ==0) || (Left ==0)) && (Fright !=0) && (Fleft !=0) ){	//both external sensors detect no line.
		Forward();	//drive forward
	}


	if ((Fleft == 0) && (Fright != 0)){	//Left Turn Only
		Stop();
		Delay(200);
		if (selection ==1){	//Left Hand Rule
			Stop();
			GPIOB->ODR |= ~0b0; //Turns on all the LEDS
			LeftTurnSharp();
			Stop();
			Delay(50);
			GPIOB->ODR &= 0b0; //Turns off all the LEDS
			Forward();
		}

		if (selection == -1){
			Peak();
			Stop();
			Delay(200);
		}
		if (selection == -1 && ( (Middle==0) || (Right ==0) || (Left ==0))){	//There is still a line
			Stop();
			Forward();	//Drive straight anyway (if it is an end the next function will take care of it)
			Record('S');
		};
		if (selection != 1 && ( (Middle!=0) && (Right !=0) && (Left !=0))){	//There is not a line
			Stop();
			LeftTurnSharp();
			Stop();
			Forward();
		}
	}

	if ((Fright == 0) && (Fleft != 0)){	//Right Turn only
		Stop();
		Delay(200);
		if (selection ==1){	//left hand rule
		Peak();
		Stop();
		Delay(200);
		}
		if (selection == -1){	//Right Hand Rule
			Stop();
			RightTurnSharp();
			Stop();
			Forward();
		}

		if (selection == 1 && ( (Middle==0) || (Right ==0) || (Left ==0))){	//There is still a line
				Stop();
				Forward();	//Drive straight anyway (if it is an end the next function will take care of it)
				Record('S');
			};
			if (selection == 1 && ( (Middle!=0) && (Right !=0) && (Left !=0))){	//There is not a line
				Stop();
				RightTurnSharp();
				Stop();
				Forward();
			}
		}

	if (((Fright == 0) && (Fleft == 0))||((Left==0)&&(Right==0))){	//T Junction
				//scoot forward, and check if it is a circle

				Stop();
				Delay(50);
				Peak();
				Delay(200);

				if ((Right == 0) && (Left == 0) && (Middle ==0)){
					Stop();//circle
					Finished();
				}
				if (selection == -1){	//Right Hand Rule
					Stop();
					RightTurnSharp();
					Stop();
					Forward();
				}
				if (selection ==1){	//Left Hand Rule
					Stop();
					LeftTurnSharp();
					Stop();
					Forward();
				}
	}

	DeadEnd();
	//exits loop and polls ports again
}
//====================================================================
// DRIVE FUNCTIONS
//====================================================================


void Forward(void){
	PWM_backleft(0);
	PWM_backright(0);
	PWM_left(100);
	PWM_right(100);
	 if ((Right == 0) && (Middle !=0)){
		 PWM_left(80);
		 PWM_right(30);
	 }
	 if ((Left ==0) && (Middle != 0)){
		 PWM_left(30);
		 PWM_right(80);
	 }
}

void DeadEnd(void){
	int test =0;
	if ((Fright!=0) && (Right !=0) && (Middle!=0) && (Left!=0)&& (Fleft!=0)){
			test = 1;//passed first test, ie. detected no line
			Stop();
			int TimerVariable = 0;
			Peak();
			Stop();
			while ((Fright!=0) && (Right !=0) && (Middle!=0) && (Left!=0)&& (Fleft!=0) && (TimerVariable!=40000)){	//these all remain low
				GPIOB->ODR |= ~0b0;
				ScootLeft();
				TimerVariable++;		//start counting up a dummy variable
			}
			Stop();

			if (TimerVariable==40000){	//reached end of path without find a path
				test = 2;	//passed second test
				TimerVariable =0 ;//resets timer

				while ((Fright!=0) && (Right !=0) && (Middle!=0) && (Left!=0)&& (Fleft!=0) && (TimerVariable!=40000)){	//these all remain low
								GPIOB->ODR |= ~0b0;
								ScootRight();
								TimerVariable++;		//start counting up a dummy variable
							}
				Stop();
				if (TimerVariable == 40000){
					test = 3;	//passed third test
				}

			}

			if (test == 3){

				//passed all tests
				Back();
			}

			//else, it has found a line again
			if (test !=3){
			Forward();
			}
		}
}


void Peak(void){
	/*Scoot Forward and take a peakaru
	 *
	 */
	Stop(); //clearing
	PWM_left(100);
	PWM_right(100);
	Delay(70);
	Stop();
	Delay(150);
}

void LeftTurn(void){
	/*Makes a 90degree left turn, stops, then
	 * drives forward
	 */
	PWM_backleft(0);
	PWM_backright(0);
	PWM_left(0);
	PWM_right(90);
	Delay(50);
	while (Middle!=0){	//turns till we are back on the line
		PWM_backleft(0);
		PWM_right(90);
	}
	Stop();
}

void LeftTurnSharp(void){
	/*Makes a 90degree left turn, stops, then
	 * drives forward
	 */
	Stop();
	PWM_backleft(100);
	PWM_backright(0);
	PWM_left(0);
	PWM_right(100);
	Delay(100);
	//Make 100% certain we aren't writing conflicting signals
	Stop();	//clearing
	Delay(100);
	Stop();
	if (backflag == 0){
	Record('L');
	}
	if (backflag == 1){
		Record('B');
		backflag =0; //reset flag
	}
	while (Middle!=0){	//turns till we are back on the line
		PWM_backleft(100);
		PWM_right(100);
	}
}
void RightTurn(void){
	/*Makes a 90degree Right turn, stops, then
	 * drives forward
	 */
	PWM_backleft(0);
	PWM_backright(0);
	PWM_left(100);
	PWM_right(0);
	Delay(70);
	while (Middle!=0){	//turns till we are back on the line
		PWM_left(90);
		PWM_backright(0);
	}
	Stop();
}

void RightTurnSharp(void){
	/*Makes a 90degree Right turn, stops, then
	 * drives forward
	 */
	Record('R');
	PWM_backleft(0);
	PWM_backright(70);
	PWM_left(70);
	PWM_right(0);
	Delay(150);

	Stop();	//clearing
	Delay(100);
	Stop();

	while (Middle!=0){	//turns till we are back on the line
		PWM_left(100);
		PWM_backright(100);
	}
	Stop();
}

void Back(void){ //180 degree turn
/*Reverses till it reaches either a T-Junction, Left Turn, or Right Turn
 *
 */
	backflag =1; //setting flag to high
	 Stop();//clearing all
	 PWM_left(0);				//setting speed to 0
	 PWM_right(0);				//setting speed to 0

	 PWM_backleft(80);
	 PWM_backright(80);
	 Delay(100);

	 Record('B');
	 Stop();
	 Delay(100);
	 LeftTurnSharp();
	 Stop();
 }

void Stop (void){
	PWM_left(0);				//setting speed to 0
	PWM_right(0);				//setting speed to 0
	PWM_backleft(0);
	PWM_backright(0);
}

void ScootRight(void){
	Stop();
	PWM_left(100);				//setting speed to 0
	PWM_backright(100);				//setting speed to 0
	//if Line detected stop
	if ((Middle ==0)||(Left==0)||(Right==0)){
			Stop();
		}
}

void ScootLeft(void){
	/*Does a small Turn towards the
	 * Left, exits turn if it hits a line
	 */
	Stop();
	PWM_backleft(100);				//setting speed to 0
	PWM_right(100);				//setting speed to 0
	if ((Middle ==0)||(Right==0)||(Left==0)){
				Stop();
			}
	//if Line Detected

}

void Finished(void){
	int finished = 1;
	while(finished != 2){
		Stop();
		Flash();

	}
}
void PWM_left(int duty){
	/*FORWARD, PB4,
	 * ControlsPB4 in alternate one, Forward*
	 */

	TIM3->CCR1 = duty*80;	//setting duty cycle for timer 3 channel 1, links to PB4
	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CR1 |= TIM_CR1_CEN;

}

void PWM_right(int duty){
	/*Controls PB10
	 *controls PB0 in Forward Drive
	 */
	//TIM2->CCR3 = duty*80; //Right motor speed

	TIM3->CCR3 = duty*80;	//setting speed
	TIM3->CCER |= TIM_CCER_CC3E;
	TIM3->CR1 |= TIM_CR1_CEN;

}

void PWM_backleft(int duty){
	/*Controls ReverseLeft
	 * PB5
	 */
	//TIM2->CCR2 = duty*80; //Right motor speed


	TIM3->CCR2 = duty*80;	//setting speed
	TIM3->CCER |= TIM_CCER_CC2E;
	TIM3->CR1 |= TIM_CR1_CEN;

}

void PWM_backright(int duty){
	/*Controls ReverseRight
	 * PB1
	 */
	TIM3->CCR4 = duty*80;	//setting speed
	TIM3->CCER |= TIM_CCER_CC4E;
	TIM3->CR1 |= TIM_CR1_CEN;
}


void init_TIM14 (void)
{
RCC -> APB1ENR |= RCC_APB1ENR_TIM14EN; // ENABLE TIM14 BUS CLK
TIM14 -> PSC = 5999;
TIM14 -> ARR = 500; // OVERFLOW OCCURS EVERY APPROX 0.5 SECOND
TIM14 -> DIER |= TIM_DIER_UIE; // ENABLE UPDATE INTERRUPT
TIM14 -> CR1 |= TIM_CR1_ARPE; // AUTO RELOAD PRELOAD BUFFER ENABLE
TIM14 -> CR1 |= TIM_CR1_CEN; // START THE TIM14 COUNTER
NVIC_EnableIRQ(TIM14_IRQn); // ENABLE TIM14 BUS
}

//====================================================================
//INTERUPT HANDLERS AND INTERRUPT
//====================================================================


void TIM14_IRQHandler (void)
{
value++;
TIM14 -> SR &= ~TIM_SR_UIF;// EXIT TIM14 INTERRUPT EVENT
}

void init_EXTI (void)
{
/*Interrupt Routine for PA1
 *
 */
RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // ENABLE EXTI BUS CLK
SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA; // MAP INTERRUPT TO PA1
EXTI -> IMR |= EXTI_IMR_MR1; // UNBLOCK INTERRUPT LINE 1 BUS
EXTI -> FTSR |= EXTI_FTSR_TR1; // CONDITION CHECK: FALLING-EDGE
//EXTI -> RTSR |= EXTI_FTSR_TR1; // CONDITION CHECK: RISING-EDGE
NVIC_EnableIRQ(EXTI0_1_IRQn); // ENABLE LINE 0 &amp; LINE 1 INTERRUPT
}

void init_EXTI1 (void)
{
	/*Interrupt Routine for PA2
	 *
	 */
RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // ENABLE EXTI BUS CLK
SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PA; // MAP INTERRUPT TO PA2
EXTI -> IMR |= EXTI_IMR_MR2; // Unblock interrupt 2
EXTI -> FTSR |= EXTI_FTSR_TR2; // CONDITION CHECK: FALLING-EDGE
NVIC_EnableIRQ(EXTI2_3_IRQn); // maps to 2_3
}

void EXTI2_3_IRQHandler (void)
{
	GPIOB->ODR &=0b0;

	selection = (selection*(-1));

	if (selection ==0){
		selection =1;
		GPIOB -> ODR |= GPIO_ODR_15;
	}

	if (selection == -1){
		GPIOB -> ODR &= 0b0;
	}
	if (selection == 1){
		GPIOB -> ODR |= GPIO_ODR_15;
	}
	EXTI -> PR |= EXTI_PR_PR1; // EXIT INTERRUPT

	if (finished ==1){	//if race has been  complete, and switch is pressed
		finished =2;
	}
}



void EXTI0 (void)
{
	/*Interrupt Routine for PA0
	 *
	 */
RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN; // enable clock for the sys
SYSCFG -> EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; // map PA0 to EXTI0
EXTI -> IMR |= EXTI_IMR_MR0; // unmask external interrupt 0
EXTI -> FTSR |= EXTI_FTSR_TR0; // trigger on falling edge
NVIC_EnableIRQ(EXTI0_1_IRQn); // enable EXTI0_1 interrupt in the NVIC
}

void EXTI0_1_IRQHandler (void)
{
	lcd_command(CLEAR);
		lcd_putstring("Sw1");	// Display string on line 1
			lcd_command(LINE_TWO);				// Move cursor to line 2
			lcd_putstring("Middle");	// Display string on line 1
	sw0 = 4;
	//if Switch 1 has been pressed, this will be called.
	selection = -1*selection;//sets it to left hand rule
	GPIOB->ODR |= ~0b0;
	Delay(500);
	GPIOB->ODR &= 0b0;

	GPIOB->ODR |= ~0b0;
	Delay(500);
	GPIOB->ODR &= 0b0;


	if((GPIOA->IDR & SW0)==0) //switch 0 has been pressed
		{
			sw0 = sw0 + 1;//increments switch0
			value++;	//upon selection increases
		}
    EXTI -> PR |= EXTI_PR_PR1; // EXIT INTERRUPT
}

void Flash(void){
	/*Flashes the LED's
	 * Generally indicates a call of evaluate Junction
	 */
			GPIOB->ODR &=0b0;	//turning off
			Delay(500);
			GPIOB->ODR |= ~0b0;//turning on lights
			Delay(500);
			GPIOB->ODR &=0b0;	//turning off
			Delay(500);
			GPIOB->ODR |= ~0b0;//turning on lights
			Delay(500);
			GPIOB->ODR &=0b0;	//turning off
			Delay(500);
			GPIOB->ODR |= ~0b0;//turning on lights
			Delay(500);
}

void LED(void){
	/*Displays which sensors are currently active
	 * FLeft->Fright = PB0-PB4
	 */
	if (Middle ==0){
		GPIOB -> ODR |=GPIO_ODR_6;//turn on PB6
	}
	if (Middle != 0){
		GPIOB -> ODR &= ~GPIO_ODR_6;//turn off
	}
	if (Fright == 0){
		GPIOB -> ODR |=GPIO_ODR_8;	//PB8
	}
	if (Fright != 0){
		GPIOB -> ODR &=~GPIO_ODR_8;
	}
	if (Right == 0){	//PB7
		GPIOB -> ODR |= GPIO_ODR_7;
	}
	if (Right != 0){
		GPIOB -> ODR &=~GPIO_ODR_7;
	}
	if (Fleft == 0){	//PB2
		GPIOB -> ODR |= GPIO_ODR_2;
	}
	if (Fleft != 0){
		GPIOB -> ODR &=~GPIO_ODR_2;
	}
	if (Left == 0){	//PB3
		GPIOB -> ODR |= GPIO_ODR_3;
	}
	if (Left != 0){
		GPIOB -> ODR &=~GPIO_ODR_3;
	}
	if (selection == 1){
		GPIOB -> ODR |= GPIO_ODR_15;
	}
	if (selection == -1){
		GPIOB -> ODR &=~GPIO_ODR_15;
		}
}



void Delay(int x){
	for (int i = 0 ;i<=DELAY1;i++)
		      		for(int j = 0;j<=x;j++);
}





//********************************************************************
//Optimization
//********************************************************************
void Record(char a)
{
	if (finished ==0){	//preventing false triggering from the race function
	path[length] = a; //records direction
	length++;
	Simplify(); //summons simplify
	}
}

void Race (void){
	int i = 0;
	//iterate through the path function.
	while (((Middle == 0) || (Right ==0) || (Left ==0)) && (Fright !=0) && (Fleft !=0) ){	//both external sensors detect no line.
			Forward();	//drive forward
		}

		if ((Fleft == 0) && (Fright != 0)){	//Left Turn Only
			Stop();
			if (path[i] == 'S'){
				Forward();
			}
			if ((path[i] == 'L')){
				LeftTurnSharp();
			}

			if (path[i] == 'R'){
				RightTurnSharp();
			}
			if (path[i] =='B'){
				Back();
			}

		if ((Fright == 0) && (Fleft != 0)){	//Right Turn only

			}

		if (((Fright == 0) && (Fleft == 0))||((Left==0)&&(Right==0))){	//T Junction
					//scoot forward, and check if it is a circle

					Stop();
					Delay(50);

		}

		DeadEnd();
		//exits loop and polls ports again
	}
	i++; //increment to next junction call.
}


void Simplify()
{
  // only simplify the path if the second-to-last turn was a 'B'
  if(length < 3 || path[length-2] != 'B')
    return;

  int totalAngle = 0;
  int i;
  for(i=1;i<=3;i++)
  {
    switch(path[length-i])
    {
      case 'R':
        totalAngle += 90;
	break;
      case 'L':
	totalAngle += 270;
	break;
      case 'B':
	totalAngle += 180;
	break;
    }
  }

  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;

  // Replace all of those turns with a single one.
  switch(totalAngle)
  {
    case 0:
	path[length - 3] = 'S';
	break;
    case 90:
	path[length - 3] = 'R';
	break;
    case 180:
	path[length - 3] = 'B';
	break;
    case 270:
	path[length - 3] = 'L';
	break;
  }

  // The path is now two steps shorter.
  length -= 2;

}


