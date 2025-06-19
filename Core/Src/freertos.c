/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/*
 * #PROJECT GOAL
 * Create a system to acquire a signal through ADC (Analog to Digital Converter)
 * The sampling frequency must be 1kHz.
 * When the number of acquired samples reach 500, the data must be copied from one buffer to another.
 * The entire system must work in FreeRTOS.
 * Every 500 samples we want to estimate the signal frequency using the zero crossing method.
 * (suppose the signal is sinusoidal and it's generated from the outside)
 * The result must be send to the asynchronous serial port.
 *
 * #INPUT
 * In STM32CubeMx the ADC1: IN13 Single-ended was activated
 * The ADC input pin is PA6
 * The voltage range is 0V to 3.3V
 *
 * #OUTPUT
 * The estimated signal frequency (expressed in Hz) is provided using the UART3 serial port
 *
 * #PROGRAM STRUCTURE
 * This program is composed of:
 *  - 3 RTOS Tasks (Idle Task, Frequency Estimation Task and Serial Print Task)
 * 	- 2 buffers (one used for sampling, one used for frequency estimation)
 * 	- 1 RTOS periodic Timer (used to trigger a signal sampling)
 * 	- 1 RTOS binary Semaphore (used to signal that we reach the number of samples needed for the frequency estimation)
 * 	- 1 RTOS Queue (used to communicate the estimated frequency to the Serial Print task)
 * 	- 1 Timer Callback (handles the ADC conversion and writes the result to the buffer)
 * 	- 1 Utility Function (used to copy the content of the ADC buffer to the processing buffer)
 *
 * */

typedef struct {
	unsigned long int iter;
	float freq;
}xMessage;

// Number of samples required for the frequency estimation
#define NUM_SAMPLES 500

// The frequency used to sample the input signal
#define SAMPLING_FREQUENCY_HZ 100

#define SERIAL_QUEUE_MAX_MSG 10

uint32_t adc_buffer[NUM_SAMPLES];

volatile int current_sample_n = 0;

uint32_t processing_buffer[NUM_SAMPLES];

// Sampling Timer
osTimerId_t xTimer;
osStatus_t xTimerStatus;


osSemaphoreId_t xProcessingSemaphore;

osMessageQueueId_t xSerialQueue;

// The Frequency Estimation Task has priority 25
const osThreadAttr_t frequencyEstimationTask_attribute = {
  .name = "frequencyEstimationTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};

// The Serial Print Task has priority 26
const osThreadAttr_t serialPrintTask_attribute = {
  .name = "serialPrintTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void vCopyBuffer(){
	for(int i=0; i < NUM_SAMPLES; i++){
		processing_buffer[i] = adc_buffer[i];
	}
}

// The Timer Callback function
void vADC_Acquire(){
	uint32_t value = 0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 0);
	value = HAL_ADC_GetValue(&hadc1);
	adc_buffer[current_sample_n] = value;

	current_sample_n++;

	if(current_sample_n == NUM_SAMPLES){
		current_sample_n = 0;
		osSemaphoreRelease(xProcessingSemaphore);
	}
}


void vFrequencyEstimationTask(void *pvParameters){

	unsigned long int iteractions = 0;
	unsigned long int zero_crossing;

	//The ADC use 16bits (0V->0  3.3V->65536)
	unsigned long int logical_zero = 32768; // =(2^16)/2  indicates the logical zero (1.65V)

	// Represent the duration in seconds of the sampling process
	float sampling_period = (float)NUM_SAMPLES / SAMPLING_FREQUENCY_HZ;

	float frequency;

	while(1){
		osSemaphoreAcquire(xProcessingSemaphore, osWaitForever);
		vCopyBuffer();
		zero_crossing = 0;

		// calculate the number of times the signal pass through the logical zero
		for(int i=1; i<NUM_SAMPLES; i++){
			unsigned long int prev = processing_buffer[i-1];
			unsigned long int curr = processing_buffer[i];
			if ((prev < logical_zero && curr > logical_zero) || (prev > logical_zero && curr < logical_zero)){
				zero_crossing++;
			}
		}

		// A sinusoidal wave cycle has 2 zero crossing
		// The frequency is define as the number of cycles (number of zero crossing / 2) divided by the period
		frequency = ((float)zero_crossing/2.0) / sampling_period;

		xMessage msg = {.iter = iteractions, .freq = frequency};

		iteractions++;

		osMessageQueuePut(xSerialQueue, &msg, 0, 0);
	}
}

void vSerialPrintTask(void *pvParameters){

	xMessage msg;

	unsigned long int lowBound = 0;

	unsigned long int upperBound = NUM_SAMPLES;

	while(1){
		osMessageQueueGet(xSerialQueue, &msg, 0, osWaitForever);

		lowBound = msg.iter * NUM_SAMPLES;

		upperBound = lowBound + NUM_SAMPLES - 1;

		printf("Samples [%lu - %lu] Estimated Frequency = %.2f Hz\r\n", lowBound, upperBound, msg.freq);
	}
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	xProcessingSemaphore = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	xTimer = osTimerNew(vADC_Acquire, osTimerPeriodic, NULL, NULL);

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	xSerialQueue = osMessageQueueNew(SERIAL_QUEUE_MAX_MSG, sizeof(xMessage), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTimerStatus = osTimerStart(xTimer, pdMS_TO_TICKS((1.0/(float)SAMPLING_FREQUENCY_HZ)*1000));
  if(xTimerStatus == osOK && xProcessingSemaphore != NULL && xSerialQueue != NULL){
	  osThreadNew(vFrequencyEstimationTask, NULL, &frequencyEstimationTask_attribute);
	  osThreadNew(vSerialPrintTask, NULL, &serialPrintTask_attribute);
  }

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

