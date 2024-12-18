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
#include "fonts.h"
#include "ssd1306.h"
#include "stdio.h"
#include "software_timer.h"
#include "button.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DIR_PIN GPIO_PIN_11
#define DIR_PORT GPIOA
#define STEP_PIN GPIO_PIN_10
#define STEP_PORT GPIOA

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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
int i = 11; //points axit y in lcd oled

uint8_t counter = 0;
uint8_t status = 0;

uint8_t set_mode = 1;
uint8_t set_times = 1;
uint8_t set_step = 1;
uint8_t set_lengh = 1;

uint8_t times = 1;
uint8_t lengh = 0;


//variables for step motor
uint8_t dir = 0;

//variables for loadcell
uint32_t force = 0;
uint32_t tare = 8349807;
float knownOriginal = 46000000;  // in milli gram
float knownHX711 = 244104; // 244104
int weight;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void microDelay(uint32_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < delay);
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
  int32_t  samples = 200;
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

  HAL_TIM_Base_Start(&htim2);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);

  SSD1306_Init();

  char sum[5];

  SSD1306_GotoXY (0,0);
  SSD1306_Puts ("PROGRAMMED", &Font_7x10, 1);
  SSD1306_GotoXY (11, 20);
  SSD1306_Puts ("BY TRIET NGUYEN", &Font_7x10, 1);
  SSD1306_GotoXY (0, 40);
  SSD1306_Puts ("STARTING.......", &Font_7x10, 1);

  SSD1306_UpdateScreen();
  HAL_Delay (1000);

  SSD1306_ScrollRight(0,7);
  HAL_Delay(3000);
  SSD1306_ScrollLeft(0,7);
  HAL_Delay(3000);
  SSD1306_Stopscroll();
  SSD1306_Clear();

  counter = 1;
  set_timer4(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (HAL_GPIO_ReadPin(START_STOP_GPIO_Port, START_STOP_Pin) == 0) {
		  HAL_Delay(20);
	  	  if (HAL_GPIO_ReadPin(START_STOP_GPIO_Port, START_STOP_Pin) == 0){
	  		  if (counter < 5){
	  			  counter++;
	  		  }
	  		  else{
	  			  status++;
	  			  if (status == 2) status = 0;
	  			  if (status == 3) status = 1;
	  			  if (status == 0){
					  SSD1306_GotoXY (56, 11);
					  SSD1306_Puts ("STOP     ", &Font_7x10, 1);
					  SSD1306_UpdateScreen();
	  			  }
	  			  else if (status == 1){
	  				  SSD1306_GotoXY (56, 11);
	  				  SSD1306_Puts ("START    ", &Font_7x10, 1);
	  				  SSD1306_UpdateScreen();
	  			  }
	  		  }
	  	  }
	      while (HAL_GPIO_ReadPin(START_STOP_GPIO_Port, START_STOP_Pin) == 0){};
	  }

	  else if (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port, EMERGENCY_Pin) == 0) {
		  HAL_Delay(20);
	  	  if (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port, EMERGENCY_Pin) == 0){
	  		  if (counter > 4) {
				  status = 2;
				  SSD1306_GotoXY (56, 11);
				  SSD1306_Puts ("E_STOP     ", &Font_7x10, 1);
				  SSD1306_UpdateScreen();
	  		  }
	  	  }
	  	  while (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port, EMERGENCY_Pin) == 0){};
	  }

	  switch (counter)
	  {
	  case 1:

		  SSD1306_Clear();
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
			  while (HAL_GPIO_ReadPin(MOVE_UP_GPIO_Port, MOVE_UP_Pin) == 0){
				  dir = 0;
				  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_PIN, dir);
				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
			  }
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		  }

		  if (HAL_GPIO_ReadPin(MOVE_DOWN_GPIO_Port, MOVE_DOWN_Pin) == 0) {
			  while (HAL_GPIO_ReadPin(MOVE_DOWN_GPIO_Port, MOVE_DOWN_Pin) == 0){
				  dir = 1;
				  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_PIN, dir);
				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
			  }
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		  }
		  break;
	  case 4:
		  SSD1306_Clear();
		  SSD1306_GotoXY (11,0);
		  SSD1306_Puts ("TESTING MACHINE", &Font_7x10, 1);

		  SSD1306_GotoXY (0, 11);
		  SSD1306_Puts (">MODE :           ", &Font_7x10, 1);
		  SSD1306_GotoXY (0, 22);
		  SSD1306_Puts (" TIMES:           ", &Font_7x10, 1);
		  SSD1306_GotoXY (0, 33);
		  SSD1306_Puts (" STEP :           ", &Font_7x10, 1);
		  SSD1306_GotoXY (0, 44);
		  SSD1306_Puts (" LENGH:           ", &Font_7x10, 1);

		  SSD1306_UpdateScreen();
		  counter++;
		  break;
	  case 5:
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
				  if (i > 44) {
					  i = 11;
				  }
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
				  if (i < 11) {
					  i = 44;
				  }
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
				  else if (i == 22){
					  if(set_mode == 2) set_times++;
					  SSD1306_GotoXY (56, 22);
					  SSD1306_Puts ("      ", &Font_7x10, 1);
				  }

				  else if (i == 33){
					  set_step++;
					  if (set_step > 2) set_step = 2;
				  }
				  else if (i == 44){
					  set_lengh++;
					  if (set_lengh > 100) set_lengh = 100;
					  SSD1306_GotoXY (56, 44);
					  SSD1306_Puts ("      ", &Font_7x10, 1);
				  }
			  }
			  while (HAL_GPIO_ReadPin(ADDITION_GPIO_Port, ADDITION_Pin) == 0){};
		  }

		  else if (HAL_GPIO_ReadPin(SUBTRACTION_GPIO_Port, SUBTRACTION_Pin) == 0) {
			  HAL_Delay(20);
			  if (HAL_GPIO_ReadPin(SUBTRACTION_GPIO_Port, SUBTRACTION_Pin) == 0){
				  if (i == 11){
					  set_mode--;
					  if (set_mode < 1) set_mode = 1;
				  }
				  else if (i == 22){
					  if (set_mode == 2) {
						  set_times--;
						  if (set_times < 1) set_times = 1;
						  SSD1306_GotoXY (56, 22);
						  SSD1306_Puts ("      ", &Font_7x10, 1);
					  }

				  }
				  else if (i == 33){
					  set_step--;
					  if (set_step < 1) set_step = 1;
				  }
				  else if (i == 44){
					  set_lengh--;
					  if (set_lengh < 1) set_lengh = 1;
					  SSD1306_GotoXY (56,44);
					  SSD1306_Puts ("      ", &Font_7x10, 1);
				  }
			  }
			  while (HAL_GPIO_ReadPin(SUBTRACTION_GPIO_Port, SUBTRACTION_Pin) == 0){};
		  }
		  break;
	  case 6:
		  SSD1306_Clear();
		  SSD1306_GotoXY (11,0);
		  SSD1306_Puts ("TESTING MACHINE", &Font_7x10, 1);

		  SSD1306_GotoXY (0, 11);
		  SSD1306_Puts ("STATUS:           ", &Font_7x10, 1);
		  SSD1306_GotoXY (0, 22);
		  SSD1306_Puts ("FORCE :           ", &Font_7x10, 1);
		  SSD1306_GotoXY (0, 33);
		  SSD1306_Puts ("LENGH :           ", &Font_7x10, 1);
		  SSD1306_GotoXY (0, 44);
		  SSD1306_Puts ("TIMES :           ", &Font_7x10, 1);

		  SSD1306_UpdateScreen();
		  counter++;
		  break;
	  case 7:
		  if ((status == 1) && (set_mode == 1))
		  {
			  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_PIN, 1);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
			  if (set_step == 1)
			  {
				  HAL_Delay(1400);
				  lengh++;
			  }
			  else
			  {
				  HAL_Delay(2800);
				  lengh = lengh + 2;
			  }
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			  weight = weigh();
			  force = weight/1000000;

			  if (lengh == set_lengh)	status = 0;

		  }
		  else if ((status == 1) && (set_mode == 2))
		  {
			  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_PIN, 1);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
			  if (set_step == 1)
			  {
				  HAL_Delay(1400);
				  lengh++;
			  }
			  else
			  {
				  HAL_Delay(2800);
				  lengh = lengh + 2;
			  }
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			  weight = weigh();
			  force = weight/1000000;

			  if (lengh == set_lengh)	times++;
		  }
	  }

	  /*if (counter == 1){
		  counter++;
		  SSD1306_Clear();
	  }

	  else if (counter == 2){

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

		  if (HAL_GPIO_ReadPin(MOVE_UP_GPIO_Port, MOVE_UP_Pin) == 0) {
			  while (HAL_GPIO_ReadPin(MOVE_UP_GPIO_Port, MOVE_UP_Pin) == 0){
				  dir = 0;
				  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_PIN, dir);
				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
			  }
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		  }

		  if (HAL_GPIO_ReadPin(MOVE_DOWN_GPIO_Port, MOVE_DOWN_Pin) == 0) {
			  while (HAL_GPIO_ReadPin(MOVE_DOWN_GPIO_Port, MOVE_DOWN_Pin) == 0){
				  dir = 1;
				  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_PIN, dir);
				  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
		  	  }
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		  }
	  }

	  else if (counter == 3){

		  counter++;

		  SSD1306_Clear();
		  SSD1306_GotoXY (11,0);
		  SSD1306_Puts ("TESTING MACHINE", &Font_7x10, 1);

		  SSD1306_GotoXY (0, 11);
		  SSD1306_Puts (">MODE :           ", &Font_7x10, 1);
		  SSD1306_GotoXY (0, 22);
		  SSD1306_Puts (" TIMES:           ", &Font_7x10, 1);
		  SSD1306_GotoXY (0, 33);
		  SSD1306_Puts (" STEP :           ", &Font_7x10, 1);
		  SSD1306_GotoXY (0, 44);
		  SSD1306_Puts (" LENGH:           ", &Font_7x10, 1);

		  SSD1306_UpdateScreen();

	  }*/
	  /*else if (counter == 4){

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
  	  	  		  if (i > 44) {
 	  	  			  i = 11;
  	  	  		  }
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
		  		  if (i < 11) {
		  			  i = 44;
		  		  }
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
				  else if (i == 22){
					  if(set_mode == 2) set_times++;
						 SSD1306_GotoXY (56, 22);
						 SSD1306_Puts ("      ", &Font_7x10, 1);
				  }

				  else if (i == 33){
					  set_step++;
					  if (set_step > 2) set_step = 2;
				  }
				  else if (i == 44){
					  set_lengh++;
					  if (set_lengh > 100) set_lengh = 100;
					  SSD1306_GotoXY (56, 44);
					  SSD1306_Puts ("      ", &Font_7x10, 1);
				  }
			  }
			  while (HAL_GPIO_ReadPin(ADDITION_GPIO_Port, ADDITION_Pin) == 0){};
		  }

		  else if (HAL_GPIO_ReadPin(SUBTRACTION_GPIO_Port, SUBTRACTION_Pin) == 0) {
			  HAL_Delay(20);
			  if (HAL_GPIO_ReadPin(SUBTRACTION_GPIO_Port, SUBTRACTION_Pin) == 0){
				  if (i == 11){
					  set_mode--;
					  if (set_mode < 1) set_mode = 1;
  				  }
  				  else if (i == 22){
  					  if (set_mode == 2) {
  						  set_times--;
  						  if (set_times < 1) set_times = 1;
  						  SSD1306_GotoXY (56, 22);
  						  SSD1306_Puts ("      ", &Font_7x10, 1);
  					  }

  				  }
  				  else if (i == 33){
  					  set_step--;
  					  if (set_step < 1) set_step = 1;
  				  }
  				  else if (i == 44){
				  	  set_lengh--;
				  	  if (set_lengh < 1) set_lengh = 1;
				  	  SSD1306_GotoXY (56,44);
				  	  SSD1306_Puts ("      ", &Font_7x10, 1);
				  }
			  }
			  while (HAL_GPIO_ReadPin(SUBTRACTION_GPIO_Port, SUBTRACTION_Pin) == 0){};
		  }
	  }*/
	  /*else if (counter == 5){
		  SSD1306_Clear();
		  SSD1306_GotoXY (11,0);
		  SSD1306_Puts ("TESTING MACHINE", &Font_7x10, 1);

		  SSD1306_GotoXY (0, 11);
		  SSD1306_Puts ("STATUS:           ", &Font_7x10, 1);
		  SSD1306_GotoXY (0, 22);
		  SSD1306_Puts ("FORCE :           ", &Font_7x10, 1);
		  SSD1306_GotoXY (0, 33);
		  SSD1306_Puts ("LENGH :           ", &Font_7x10, 1);
		  SSD1306_GotoXY (0, 44);
		  SSD1306_Puts ("TIMES :           ", &Font_7x10, 1);

		  SSD1306_UpdateScreen();

		  counter++;
	  }*/

	  /*else if (counter == 7)
	  {
		  if ((status == 1) && (set_mode == 1))
		  {
			  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_PIN, 1);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
			  if (set_step == 1)
			  {
				  HAL_Delay(1400);
				  lengh++;
			  }
			  else
			  {
				  HAL_Delay(2800);
				  lengh = lengh + 2;
			  }
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			  weight = weigh();
			  force = weight/1000000;

			  if (lengh == set_lengh)	status = 0;

		  }
		  else if ((status == 1) && (set_mode == 2))
		  {
			  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_PIN, 1);
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 499);
			  if (set_step == 1)
			  {
			  	  HAL_Delay(1400);
			  	  lengh++;
			  }
			  else
			  {
			  	  HAL_Delay(2800);
			  	  lengh = lengh + 2;
			  }
			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			  weight = weigh();
			  force = weight/1000000;

			  if (lengh == set_lengh)	times++;
		  }

	  }
	  else if (counter == 8)
	  {


	  }*/

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 71;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  htim4.Init.Prescaler = 9999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(htim);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedHalfCpltCallback could be implemented in the user file
   */
}

/**
  * @brief  Output Compare callback in non-blocking mode
  * @param  htim TIM OC handle
  * @retval None
  */

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
