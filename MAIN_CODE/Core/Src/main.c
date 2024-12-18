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
#include "string.h"
#include "stdio.h" // For sprintf
#include "fonts.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DT_PIN GPIO_PIN_8
#define DT_PORT GPIOB
#define SCK_PIN GPIO_PIN_9
#define SCK_PORT GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
int i = 11; //points axit y in lcd oled

//state of the system
uint8_t counter = 0; //variable for system
uint8_t status = 0; //variable for start/stop
uint8_t limitswitch = 0; //variable for run mode

//set up variables
uint8_t set_mode = 1;
uint8_t set_times = 1;
uint8_t set_step = 1;
uint8_t set_lengh = 1;

//variable when system is run
uint8_t times = 0;
uint8_t lengh = 0;


//variables for loadcell
uint32_t tare = 8418394U; //error loadcell
float knownOriginal = 5830000U;  // (in milli gram) real weight 5830000
float knownHX711 = 859763U;
float knownOriginalForce = 4614;
int weight;
int offset = 0U;
int force = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void microDelay(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (__HAL_TIM_GET_COUNTER(&htim2) < time);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
  if (HAL_GPIO_ReadPin(START_STOP_GPIO_Port, START_STOP_Pin) == 0) {
  	  if (HAL_GPIO_ReadPin(START_STOP_GPIO_Port, START_STOP_Pin) == 0){
  		  if (counter < 9){
  			  counter++;
  		  }
  		  else{
  			  status = 1;
  			  if (status == 1){
  				  SSD1306_GotoXY (56, 11);
  				  SSD1306_Puts ("START    ", &Font_7x10, 1);
  				  SSD1306_UpdateScreen();
  			  }
  		  }
  	  }
  	  while (HAL_GPIO_ReadPin(START_STOP_GPIO_Port, START_STOP_Pin) == 0){};
    }
    else if (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port, EMERGENCY_Pin) == 0) {
    	if (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port, EMERGENCY_Pin) == 0){
    		if (counter > 7) {
  	 			status = 2;
  	 			SSD1306_GotoXY (56, 11);
  	 			SSD1306_Puts ("E_STOP     ", &Font_7x10, 1);
  	 			SSD1306_UpdateScreen();
  	 		}
  	 	}
  	 	while (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port, EMERGENCY_Pin) == 0){};
    }
}

int32_t getHX711(void)
{
  uint32_t data = 0;
  uint32_t startTime = HAL_GetTick();
  while(HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
  {
    if(HAL_GetTick() - startTime > 200)
      return 0;
  }
  for(int8_t len=0; len<24 ; len++)
  {
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
    microDelay(1);
    data = data << 1;
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
    microDelay(1);
    if(HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
      data ++;
  }
  data = data ^ 0x800000;
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
  microDelay(1);
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
  microDelay(1);
  return data;
}

int weigh()
{
  int32_t  total = 0;
  int32_t  samples = 30;
  int milligram;
  float coefficient;
  for(uint16_t i = 0 ; i < samples ; i++)
  {
      total += getHX711();
  }
  int32_t average = (int32_t)(total / samples);
  coefficient = knownOriginal / knownHX711;
  milligram = (int)(average-tare)*coefficient;
  return milligram;
}

int getForce(){
	int32_t ratio;
	float Force;
	float ConfigForce;
	for (int y = 0; y < 3; y++){
		weight = weigh() - offset; // in milligram
	}
	Force = weight / 1000;
	ratio = (Force*100)/knownOriginalForce;
	ConfigForce = Force + ratio*offset/100000;
	return ConfigForce;
}

void send_values(uint32_t value1, uint32_t value2, uint32_t value3) {
    char buffer[25]; // Buffer to hold the formatted string

    // Format the string: "value1,value2,value3\n"
    sprintf(buffer, "%lu,%lu,%lu\n", value1, value2, value3);

    // Transmit the string over USART3
    HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 15);
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim4);

  SSD1306_Init();

  char sum[5];

  HAL_Delay (10);


  SSD1306_GotoXY (0,0);
  SSD1306_Puts ("PROGRAMMED", &Font_7x10, 1);
  SSD1306_GotoXY (11, 20);
  SSD1306_Puts ("BY TRIET NGUYEN", &Font_7x10, 1);
  SSD1306_GotoXY (0, 40);
  SSD1306_Puts ("STARTING.......", &Font_7x10, 1);
  SSD1306_UpdateScreen();
  SSD1306_ScrollRight(0,7);
  HAL_Delay(3000);
  SSD1306_ScrollLeft(0,7);
  HAL_Delay(3000);
  SSD1306_Stopscroll();
  SSD1306_Clear();
  HAL_Delay(10);
  counter = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch (counter)
	  {
	  	  case 1:
	  		  SSD1306_Clear();
	  		  HAL_Delay(10);
	  		  counter++;
	  		  break;
	  	  case 2:
	  		  SSD1306_GotoXY (11,0);
	  		  SSD1306_Puts ("TESTING MACHINE", &Font_7x10, 1);

	  		  SSD1306_GotoXY (0, 15);
	  		  SSD1306_Puts ("SETINGS GRIP BLOCK", &Font_7x10, 1);
	  		  SSD1306_GotoXY (0, 27);
	  		  SSD1306_Puts ("CLICK MOVE UP", &Font_7x10, 1);
	  		  SSD1306_GotoXY (0, 39);
	  		  SSD1306_Puts (" OR CLICK MOVE DW ", &Font_7x10, 1);
	  		  SSD1306_GotoXY (0, 51);
	  		  SSD1306_Puts ("START TO CONTINUE ", &Font_7x10, 1);

	  		  SSD1306_UpdateScreen();
	  		  counter++;
	  		  break;
	  	  case 3:
	  		  if (HAL_GPIO_ReadPin(MOVE_UP_GPIO_Port, MOVE_UP_Pin) == 0) {
	  			  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 0);
	  			  while (HAL_GPIO_ReadPin(MOVE_UP_GPIO_Port, MOVE_UP_Pin) == 0){
	  				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
	  			  }
	  			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	  		  }
	  		  if (HAL_GPIO_ReadPin(MOVE_DOWN_GPIO_Port, MOVE_DOWN_Pin) == 0) {
	  			  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 1);
	  			  while (HAL_GPIO_ReadPin(MOVE_DOWN_GPIO_Port, MOVE_DOWN_Pin) == 0){
	  				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
	  			  }
	  			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	  		  }

	  		  break;
	  	  case 4:
	  		  SSD1306_Clear();
	  		  HAL_Delay(10);
	  		  counter++;
	  		  break;
	  	  case 5:
	  		  SSD1306_GotoXY (0, 15);
	  		  SSD1306_Puts ("SETINGS OFFSET", &Font_7x10, 1);
	  		  SSD1306_GotoXY (0, 27);
	  		  SSD1306_Puts ("WAITING ......", &Font_7x10, 1);
	  		  SSD1306_UpdateScreen();
	  		  for (int z; z < 3; z++) {
	  			offset = weigh();

	  		  }
	  		  SSD1306_Clear();
	  		  HAL_Delay(10);
	  		  SSD1306_Clear();
	  		  counter++;
	  		  break;

	  	  case 6:
	  		  SSD1306_Clear();
	  		  HAL_Delay(10);

	  		  SSD1306_GotoXY (11,0);
	  		  SSD1306_Puts ("TESTING MACHINE", &Font_7x10, 1);

	  		  SSD1306_GotoXY (0, 11);
	  		  SSD1306_Puts (">MODE :           ", &Font_7x10, 1);
	  		  SSD1306_GotoXY (0, 22);
	  		  SSD1306_Puts (" TIMES:           ", &Font_7x10, 1);
	  		  SSD1306_GotoXY (0, 33);
	  		  SSD1306_Puts (" STEP :           ", &Font_7x10, 1);
	  		  SSD1306_GotoXY (0, 44);
	  		  SSD1306_Puts (" LEN  :           ", &Font_7x10, 1);

	  		  SSD1306_UpdateScreen();

	  		  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 0);
	  		  counter++;
	  		  break;
	  	  case 7:
	  		  if (set_mode == 1){
	  			  SSD1306_GotoXY (56, 11);
	  			  SSD1306_Puts ("PULL     ", &Font_7x10, 1);
	  		  }
	  		  else if (set_mode == 2){
	  			  SSD1306_GotoXY (56, 11);
	  			  SSD1306_Puts ("PULL & RE", &Font_7x10, 1);
	  		  }
	  		  itoa(set_step, sum, 10);
	  		  SSD1306_GotoXY (56, 33);
	  		  SSD1306_Puts (sum, &Font_7x10, 1);
	  		  itoa(set_times, sum, 10);
	  		  SSD1306_GotoXY (56, 22);
	  		  SSD1306_Puts (sum, &Font_7x10, 1);
	  		  itoa(set_lengh, sum, 10);
	  		  SSD1306_GotoXY (56, 44);
	  		  SSD1306_Puts (sum, &Font_7x10, 1);
	  		  SSD1306_UpdateScreen();
	  		  if (HAL_GPIO_ReadPin(MOVE_DOWN_GPIO_Port, MOVE_DOWN_Pin) == 0) {
	  			  HAL_Delay(20);
	  			  if (HAL_GPIO_ReadPin(MOVE_DOWN_GPIO_Port, MOVE_DOWN_Pin) == 0){
	  				  SSD1306_GotoXY (0, i);
	  				  SSD1306_Puts (" ", &Font_7x10, 1);
	  				  i = i + 11;
	  				  if (i > 44)	i = 11;
	  				  SSD1306_GotoXY (0, i);
	  				  SSD1306_Puts (">", &Font_7x10, 1);
	  				  SSD1306_UpdateScreen();
	  			  }
	  			  while (HAL_GPIO_ReadPin(MOVE_DOWN_GPIO_Port, MOVE_DOWN_Pin) == 0){};
	  		  }
	  		  else if (HAL_GPIO_ReadPin(MOVE_UP_GPIO_Port, MOVE_UP_Pin) == 0) {
	  			  HAL_Delay(20);
	  			  if (HAL_GPIO_ReadPin(MOVE_UP_GPIO_Port, MOVE_UP_Pin) == 0){

	  				  SSD1306_GotoXY (0, i);
	  				  SSD1306_Puts (" ", &Font_7x10, 1);
	  				  i = i - 11;
	  				  if (i < 11)	i = 44;
	  				  SSD1306_GotoXY (0, i);
	  				  SSD1306_Puts (">", &Font_7x10, 1);
	  				  SSD1306_UpdateScreen();
	  			  }
	  			  while (HAL_GPIO_ReadPin(MOVE_UP_GPIO_Port, MOVE_UP_Pin) == 0){};
	  		  }
	  		  else if (HAL_GPIO_ReadPin(ADDITION_GPIO_Port, ADDITION_Pin) == 0) {
	  			  HAL_Delay(20);
	  			  if (HAL_GPIO_ReadPin(ADDITION_GPIO_Port, ADDITION_Pin) == 0){
	  				  if (i == 11){
	  					  set_mode++;
	  					  if (set_mode > 2) set_mode = 2;
	  				  }
	  				  else if (i == 33){
	  					  set_step++;
	  					  if (set_step > 2) set_step = 2;
	  				  }
	  			  }
	  			  while (HAL_GPIO_ReadPin(ADDITION_GPIO_Port, ADDITION_Pin) == 0)
	  			  {
	  				  if ((i == 22) && (set_mode == 2)){
	  					  set_times++;
	  					  SSD1306_GotoXY (56, 22);
	  					  SSD1306_Puts ("      ", &Font_7x10, 1);
	  					  itoa(set_times, sum, 10);
	  					  SSD1306_GotoXY (56, 22);
	  					  SSD1306_Puts (sum, &Font_7x10, 1);
	  					  SSD1306_UpdateScreen();
	  					  HAL_Delay(200);
	  				  }
	  				  else if (i == 44){
	  					  set_lengh++;
	  					  if (set_lengh > 100) set_lengh = 100;
	  					  SSD1306_GotoXY (56, 44);
	  					  SSD1306_Puts ("      ", &Font_7x10, 1);
	  					  itoa(set_lengh, sum, 10);
	  					  SSD1306_GotoXY (56, 44);
	  					  SSD1306_Puts (sum, &Font_7x10, 1);
	  					  SSD1306_UpdateScreen();
	  					  HAL_Delay(200);
	  				  }
	  			  };
	  		  }
	  		  else if (HAL_GPIO_ReadPin(SUBTRACTION_GPIO_Port, SUBTRACTION_Pin) == 0) {
	  			  HAL_Delay(20);
	  			  if (HAL_GPIO_ReadPin(SUBTRACTION_GPIO_Port, SUBTRACTION_Pin) == 0){
	  				  if (i == 11){
	  					  set_mode--;
	  					  if (set_mode < 1) set_mode = 1;
	  				  }
	  				  else if (i == 33){
	  					  set_step--;
	  					  if (set_step < 1) set_step = 1;
	  				  }
	  			  }
	  			  while (HAL_GPIO_ReadPin(SUBTRACTION_GPIO_Port, SUBTRACTION_Pin) == 0)
	  			  {
	  				  if ((i == 22) && (set_mode == 2)){
	  					  set_times--;
	  					  if (set_times < 1) set_times = 1;
	  					  SSD1306_GotoXY (56, 22);
	  					  SSD1306_Puts ("      ", &Font_7x10, 1);
	  					  itoa(set_times, sum, 10);
	  					  SSD1306_GotoXY (56, 22);
	  					  SSD1306_Puts (sum, &Font_7x10, 1);
	  					  SSD1306_UpdateScreen();
	  					  HAL_Delay(200);
	  				  }
	  				  else if (i == 44){
	  					  set_lengh--;
	  					  if (set_lengh < 1) set_lengh = 1;
	  					  SSD1306_GotoXY (56,44);
	  					  SSD1306_Puts ("      ", &Font_7x10, 1);
	  					  itoa(set_times, sum, 10);
	  					  SSD1306_GotoXY (56, 44);
	  					  SSD1306_Puts (sum, &Font_7x10, 1);
	  					  SSD1306_UpdateScreen();
	  					  HAL_Delay(200);
	  				  }
	  			  };
	  		  }
	  		  break;
	  	  case 8:

	  		  SSD1306_Clear();
	  		  HAL_Delay(10);
	  		  counter++;
	  		  break;
	  	  case 9:

	  		  SSD1306_Clear();
	  		  HAL_Delay(10);
	  		  SSD1306_GotoXY (11,0);
	  		  SSD1306_Puts ("TESTING MACHINE", &Font_7x10, 1);
	  		  SSD1306_GotoXY (0, 11);
	  		  SSD1306_Puts ("STATUS:           ", &Font_7x10, 1);
	  		  SSD1306_GotoXY (0, 22);
	  		  SSD1306_Puts ("FORCE :           ", &Font_7x10, 1);
	  		  SSD1306_GotoXY (0, 33);
	  		  SSD1306_Puts ("LEN   :           ", &Font_7x10, 1);
	  		  SSD1306_GotoXY (0, 44);
	  		  SSD1306_Puts ("TIMES :           ", &Font_7x10, 1);
	  		  SSD1306_UpdateScreen();
	  		  counter++;
	  		  break;

	  	  case 10:
	  		  if ((status == 1) && (set_mode == 1))
	  		  {
	  			  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 1);
	  			  if (set_step == 1)
	  			  {
	  				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
	  				  HAL_Delay(1370);
	  				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	  				  lengh++;
	  			  }
	  			  else
	  			  {
	  				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
	  				  HAL_Delay(2740);
	  				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	  				  lengh = lengh + 2;
	  			  }
	  			  force = getForce();
		  		  if (status != 0) {
		  			  send_values(force, lengh, times);
		  		  }
	  			  if (lengh >= set_lengh)	{
	  				  status = 0;
	  				  times = 1;
	  			  }
	  		  }
	  		  else if ((status == 1) && (set_mode == 2))
	  		  {
	  			  if (limitswitch == 0)
	  			  {
					  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 1);
					  if (set_step == 1) {
						  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
						  HAL_Delay(1370);
						  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
						  lengh++;
					  }
					  else {
						  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
						  HAL_Delay(2740);
						  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
						  lengh = lengh + 2;
					  }
					  force = getForce();
			  		  if (status != 0) {
			  			  send_values(force, lengh, times);
			  		  }
					  if (lengh == set_lengh){
						  limitswitch = 1;
					  }
	  			  }
	  			  else if (limitswitch == 1)
	  			  {
	  				  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 0);
	  				  if (set_step == 1) {
	  					  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
	  					  HAL_Delay(1370);
	  					  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	  					  lengh--;
	  				  }
	  				  else {
	  					  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
	  					  HAL_Delay(2740);
	  					  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	  					  lengh = lengh - 2;
	  				  }
	  				  force = getForce();
	  		  		  if (status != 0) {
	  		  			  send_values(force, lengh, times);
	  		  		  }
	  				  if (lengh == 0){
	  					  times++;
	  					  limitswitch = 0;
	  					  if (times == set_times) status = 0;
	  				  }
	  			  }
	  		  }

	  		  counter++;
	  		  break;

	  	  case 11:

	  		  if (status == 0){
	  			  SSD1306_GotoXY (56, 11);
	  			  SSD1306_Puts ("STOP     ", &Font_7x10, 1);
	  		  }
	  		  SSD1306_GotoXY (56, 22);
	  		  SSD1306_Puts ("      ", &Font_7x10, 1);
	  		  itoa(force, sum, 10);
	  		  SSD1306_GotoXY (56, 22);
	  		  SSD1306_Puts (sum, &Font_7x10, 1);
	  		  SSD1306_GotoXY (56, 33);
	  		  SSD1306_Puts ("      ", &Font_7x10, 1);
	  		  itoa(lengh, sum, 10);
	  		  SSD1306_GotoXY (56, 33);
	  		  SSD1306_Puts (sum, &Font_7x10, 1);
	  		  SSD1306_GotoXY (56, 44);
	  		  SSD1306_Puts ("      ", &Font_7x10, 1);
	  		  itoa(times, sum, 10);
	  		  SSD1306_GotoXY (56, 44);
	  		  SSD1306_Puts (sum, &Font_7x10, 1);
	  		  SSD1306_UpdateScreen();
	  		  counter--;
	  		  break;
	  	  }

    /* USER CODE END WHILE */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 17999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 399;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : START_STOP_Pin EMERGENCY_Pin MOVE_DOWN_Pin MOVE_UP_Pin
                           SUBTRACTION_Pin ADDITION_Pin */
  GPIO_InitStruct.Pin = START_STOP_Pin|EMERGENCY_Pin|MOVE_DOWN_Pin|MOVE_UP_Pin
                          |SUBTRACTION_Pin|ADDITION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_Pin */
  GPIO_InitStruct.Pin = DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
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
