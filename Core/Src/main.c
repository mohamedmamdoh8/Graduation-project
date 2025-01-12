/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define Black 1
#define White 0
#define Stop_Distance 15
#define Low_Distance 30
#define Medium_Distance 40
#define High_Distance 50
#define Stop 0
#define ADLowSpeed 200
#define ADMediumSpeed 250
#define ADHighSpeed 300
#define LowSpeed 100
#define MediumSpeed 200
#define HighSpeed 350
#define VeryHighSpeed 400
#define TSpeed 300
#define RLSpeed 350
#define LimitSpeed 300
#define TURN_ON_AUTOPILOT O
#define TURN_OFF_AUTOPILOT o
#define TURN_ON_ACC D
#define TURN_OFF_ACC d
#define TURN_ON_LKAS K
#define TURN_OFF_LKAS k
#define TURN_ON_AEB A
#define TURN_OFF_AEB a
#define TURN_ON_LCW C
#define TURN_OFF_LCW c
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */
uint16_t ULTRASONIC_u16GetDistance(void);
void Car_vSpeed(uint16_t Distance);
void Auto_vPilot(uint16_t);
void Move_vForward(void);
void Move_vReverse(void);
void Move_vRight(void);
void Move_vLeft(void);
void Move_vRF(void);
void Move_vLF(void);
void Move_vRR(void);
void Move_vLR(void);
void Car_vFroward(void);
void Car_vStop(void);
void Car_vReverse(void);
void Car_vTRight(void);
void Car_vTLeft(void);
void ACC(uint16_t);
void LKAS(void);
void LCW(void);
void AEB(uint16_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t ULTRASONIC_u16GetDistance(void){

	uint32_t pMillis;
	uint32_t Value1 = 0;
	uint32_t Value2 = 0;
	uint16_t Distance  = 0;  // CM

HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

    pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
    // wait for the echo pin to go high
    while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
    Value1 = __HAL_TIM_GET_COUNTER (&htim1);

    pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
    // wait for the echo pin to go low
    while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
    Value2 = __HAL_TIM_GET_COUNTER (&htim1);


    Distance = (Value2-Value1)* 0.034/2;
    HAL_Delay(50);

    return Distance;
}

void Car_vSpeed(uint16_t Distance){

	// Fun take Distance to select the speed (i have 5 cases : 0% - 25% - 50% - 75% - 100% )

    if (Distance <= Stop_Distance )	// Distance less than 15cm car will stop.
   {
 	  TIM2->CCR1 = Stop;
 	  TIM3->CCR1 = Stop;
 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
 	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
 	  Car_vStop();
   }
    else if((Distance > Stop_Distance) && (Distance < Low_Distance)) // Distance more than 15cm and less than 30cm car will take 25% of power .
   {
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
      HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
 	  TIM2->CCR1 = ADLowSpeed;
 	  TIM3->CCR1 = ADLowSpeed;
 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
 	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
 	  Car_vFroward();
   }
   else if ( (Distance >= Low_Distance) && (Distance < Medium_Distance) ) // Distance more than 30cm and less than 40cm car will take 50% of power .
   {
 	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
 	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
 	  TIM2->CCR1 = ADMediumSpeed;
 	  TIM3->CCR1 = ADMediumSpeed;
 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
 	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
 	  Car_vFroward();
   }
   else if ( (Distance >= Medium_Distance) && (Distance < High_Distance) ) // Distance more than 40cm and less than 50cm car will take 75% of power .
   {
 	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
 	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
 	  TIM2->CCR1 = ADHighSpeed;
 	  TIM3->CCR1 = ADHighSpeed;
 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
 	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
 	  Car_vFroward();
   }
   else  // Distance more than 35cm car will take full power ..
   {
 	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
 	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
 	  TIM2->CCR1 = ADHighSpeed;
 	  TIM3->CCR1 = ADHighSpeed;
 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
 	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
 	  Car_vFroward();
   }
}

void Car_vFroward(void){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_SPEED_HIGH);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_SPEED_LOW);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_SPEED_HIGH);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_SPEED_LOW);
}
void Car_vStop(void){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_SPEED_LOW);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_SPEED_LOW);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_SPEED_LOW);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_SPEED_LOW);
}
void Car_vReverse(void){
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_SPEED_LOW);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_SPEED_HIGH);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_SPEED_LOW);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_SPEED_HIGH);
}
void Car_vTRight(void){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_SPEED_LOW);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_SPEED_LOW);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_SPEED_HIGH);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_SPEED_LOW);
}
void Car_vTLeft(void){
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_SPEED_HIGH);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_SPEED_LOW);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_SPEED_LOW);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_SPEED_LOW);
}
// Move Forward
void Move_vForward(void){
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  TIM2->CCR1 = VeryHighSpeed;
	  TIM3->CCR1 = VeryHighSpeed;
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  Car_vFroward();
}
// Move Reverse
void Move_vReverse(void){
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  TIM2->CCR1 = VeryHighSpeed;
	  TIM3->CCR1 = VeryHighSpeed;
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  Car_vReverse();
}
//Move Right
void Move_vRight(void){
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  TIM2->CCR1 = Stop;
	  TIM3->CCR1 = HighSpeed;
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  Car_vTRight();
}
//Move Left
void Move_vLeft(void){
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  TIM2->CCR1 = HighSpeed;
	  TIM3->CCR1 = Stop;
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  Car_vTLeft();
}
// move forward with Right angle
void Move_vRF(void){
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  TIM2->CCR1 = TSpeed;
	  TIM3->CCR1 = VeryHighSpeed;
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  Car_vFroward();
}
 // move forward with left angle
void Move_vLF(void){
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  TIM2->CCR1 = VeryHighSpeed;
	  TIM3->CCR1 = TSpeed;
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  Car_vFroward();
}
// move Reverse with right angle
void Move_vRR(void){
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  TIM2->CCR1 =TSpeed;
	  TIM3->CCR1 = VeryHighSpeed;
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  Car_vReverse();
}
//  // move Reverse with left angle
void Move_vLR(void){
	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	  TIM2->CCR1 = VeryHighSpeed;
	  TIM3->CCR1 = TSpeed;
	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  Car_vReverse();
}
// car move with all features
void Auto_vPilot(uint16_t distance_car){

	  ACC(distance_car);
	  LCW();

}

// Adaptive cruise control
void ACC(uint16_t distance_car){

	  distance_car = ULTRASONIC_u16GetDistance();
	  Car_vSpeed(distance_car);

}
// Lane Keep Assist System
void LKAS(void){

	 uint8_t S_RIR;
	 uint8_t S_LIR;
	  S_RIR =HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	  S_LIR =HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

	  if(S_RIR==Black && S_LIR==Black){
	 	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	 	  TIM2->CCR1 = LimitSpeed;
	 	  TIM3->CCR1 = LimitSpeed;
	 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		Car_vFroward();
	  }
	  else if (S_RIR==Black && S_LIR==White){
	 	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	 	  TIM2->CCR1 = RLSpeed;
	 	  TIM3->CCR1 = Stop;
	 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		Car_vTLeft();
	  }
	  else if (S_RIR==White && S_LIR==Black){
	 	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	 	  TIM2->CCR1 = Stop;
	 	  TIM3->CCR1 = RLSpeed;
	 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		Car_vTRight();
	  }
	  else
		  Car_vStop();
}
// Lane collision Warning
void LCW(void){
	 uint8_t S_RIR;
	 uint8_t S_LIR;
	  S_RIR =HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
	  S_LIR =HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

	  if(S_RIR==Black && S_LIR==Black){
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_SPEED_LOW);
	 	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	 	  TIM2->CCR1 = LimitSpeed;
	 	  TIM3->CCR1 = LimitSpeed;
	 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		Car_vFroward();
	  }
	  else if (S_RIR==Black && S_LIR==White){
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_SPEED_HIGH);
	 	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	 	  TIM2->CCR1 = RLSpeed;
	 	  TIM3->CCR1 = RLSpeed;
	 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		Car_vTLeft();
	  }
	  else if (S_RIR==White && S_LIR==Black){
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_SPEED_HIGH);
	 	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	 	  TIM2->CCR1 = RLSpeed;
	 	  TIM3->CCR1 = RLSpeed;
	 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		Car_vTRight();
	  }
	  else{
	      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_SPEED_LOW);
	 	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	 	  TIM2->CCR1 = Stop;
	 	  TIM3->CCR1 = Stop;
	 	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	 	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		Car_vStop();
	  }
}
// Automatic Emergency Brake
void AEB (uint16_t distance_car){

	  distance_car = ULTRASONIC_u16GetDistance();
	  if (distance_car < Stop_Distance )	// Distance less than 5cm car will stop.
	     {
	   	  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	   	  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	   	  TIM2->CCR1 = Stop;
	   	  TIM3->CCR1 = Stop;
	   	  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	   	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	   	  Car_vStop();
	     }
	   else // Distance more than 35cm car will take full power .
	   {
		  HAL_Delay(20);
		  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
		  TIM2->CCR1 = ADHighSpeed;
		  TIM3->CCR1 = ADHighSpeed;
		  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		  Car_vFroward();
	   }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start(&htim1);
    HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);
    uint16_t distance_car = Stop;
    uint8_t Str = 'Z';
    char car [] = "Hello";
  /* USER COD E END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	      HAL_UART_Receive(&huart1, &Str, sizeof(Str), 500 );
	 	  HAL_UART_Transmit(&huart6, &Str, sizeof(car), 500 );
	 	  switch (Str) {
	 	  	case ('O') :         //TURN_ON_AUTOPILOT
	 	  		Auto_vPilot(distance_car);
	 	  		break;
	 	  	case ('o') :         //TURN_OFF_AUTOPILOT
	 	  		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	 	  		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	 	  		break;
	 	  	case ('K') :        //TURN_ON_LKAS
	 	  		LKAS();
	 	  		break;
	 	  	case ('k') :       //TURN_OFF_LKAS
	 	  		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	 	  		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	 	  		break;
	 	  	case ('D') :        //TURN_ON_ACC
	 	  		ACC(distance_car);
	 	  		break;
	 	  	case ('d') :       //TURN_OFF_ACC
	 	  		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	 	  		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	 	  		break;
	 	  	case ('E') :      //TURN_ON_AEC
	 	  		AEB(distance_car);
	 	  		break;
	 	  	case ('e') :      //TURN_OFF_AEC
	 	  		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	 	  		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	 	  		break;
	 	  	case ('C') :     //TURN_ON_LCW
	 	  		LCW();
	 	  		break;
	 	  	case ('c') :    //TURN_OFF_LCW
	 	  		HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	 	  		HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	 	  		break;
	 	  	case ('F') :    //MOVE FORWARD
	 	  		Move_vForward();
	 	  		break;
	 	  	case ('B') :    //MOVE REVERSE
	 	          Move_vReverse();
	 	  		break;
	 	  	case ('L') :    //MOVE LEFT
	 	          Move_vLeft();
	 	  		break;
	 	  	case ('R') :    //MOVE RIGHT
	 	          Move_vRight();
	 	  		break;
	 	  	case ('W') :    //MOVE RIGHT Forward
	 	          Move_vRF();
	 	  		break;
	 	  	case ('Y') :    //MOVE Left Forward
	 	          Move_vLF();
	 	  		break;
	 	  	case ('r') :    //MOVE Right Reverse
	 	          Move_vRR();
	 	  		break;
	 	  	case ('X') :    //MOVE Left Reverse
	 	          Move_vLR();
	 	  		break;
	 	  	default: Car_vStop();
			 	  	    break;
			 	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 499;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 4800;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 4800;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
