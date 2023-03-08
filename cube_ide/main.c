/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dma.h"
#include "gpio.h"
#include "usart.h"
#include "tim.h"
#include "53l5a1_ranging_sensor.h"
#include "53l5a1_conf.h"
#include "vl53l5cx_api.h"
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PKG_SIZE 48
#define PKG_HEAD 4

typedef enum PackageType {
	RangePkg
} PackageType;

typedef enum StartingBytes {
	PkgType,
	PkgNum,
	PkgNumEst,
	PkgId
} StartingBytes;

typedef enum ControlFlags {
	mode_continuous,
	power_on,
	sensor_to_sleep,
	resolution_4x4,
	order_by_strongest
} ControlFlags;

/**
 **************************************************************************************************
 * \brief				Node (package) that is sent on the USB-interface
 * \see					HAL_TIM_PeriodElapsedCallback()
 * \see					enqueue()
 * \see					dequeue()
 **************************************************************************************************
 */
typedef struct {
	uint8_t data[PKG_SIZE];
	struct node* next;
} node;

/**
 **************************************************************************************************
 * \brief				Queue for the Packages for USB (FIFO)
 **************************************************************************************************
 */
typedef struct {
	node* head;
	node* tail;
} queue;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static 		uint8_t				flags = 0;
/// @brief Catch errors from sensor
static 		int32_t 			status = 0;
/// @brief Notification measurement complete
volatile 	uint8_t 			new_data = 0;
/// @brief Block incoming messages during initialization
volatile 	uint8_t				init = 1;
/// @brief Sensor capiabilities FlightSense Driver
RANGING_SENSOR_Capabilities_t 	Cap;
/// @brief Sensor settings FlightSense Driver
RANGING_SENSOR_ProfileConfig_t 	Profile;
/// @brief Measurement results
RANGING_SENSOR_Result_t			Results;
/// \brief Queue for data packages to transmit
queue 							tx_queue;

VL53L5CX_ResultsData			ll_results;

/* Some ridiculous casting to bypass the flight sense driver */
VL53L5CX_Object_t 				*ll_sensor_obj;
VL53L5CX_Configuration 			*ll_sensor_dev;
extern void *VL53L5A1_RANGING_SENSOR_CompObj[RANGING_SENSOR_INSTANCES_NBR] ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USB */
static void 				prepare_packages(); 				// prepare data package USB
static void					init_queue(queue* q);				// initialize empty queue
static uint8_t				enqueue(queue* q, uint8_t* data);	// enqueue package
static uint8_t				dequeue(queue* q);					// dequeue (send) package
static void 				set_flag(ControlFlags flag, uint8_t value);
static uint8_t 				get_flag(ControlFlags flag, uint8_t flag_variable);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  HAL_Delay(20); // <- this is crucial
  SystemClock_Config();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_GPIO_Init();
  init_queue(&tx_queue);
  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize Sensor, may take 2 secs */
  HAL_Delay(100);
  status = VL53L5A1_RANGING_SENSOR_Init(VL53L5A1_DEV_CENTER);
  if (status != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  uint32_t Id;
  VL53L5A1_RANGING_SENSOR_ReadID(VL53L5A1_DEV_CENTER, &Id);
  VL53L5A1_RANGING_SENSOR_GetCapabilities(VL53L5A1_DEV_CENTER, &Cap);
  /* load some default settings */
  Profile.RangingProfile = RS_PROFILE_8x8_CONTINUOUS;
  Profile.TimingBudget = 15; /* 5 ms < TimingBudget < 100 ms */
  Profile.Frequency = 10; /* Hz */
  Profile.EnableAmbient = 1; /* Enable: 1, Disable: 0 */
  Profile.EnableSignal = 1; /* Enable: 1, Disable: 0 */

  set_flag(mode_continuous, 1);
  set_flag(resolution_4x4, 0);
  set_flag(power_on, 1);
  set_flag(order_by_strongest, 0);
  set_flag(sensor_to_sleep, 0);

  /* set the profile if different from default one */
  VL53L5A1_RANGING_SENSOR_ConfigProfile(VL53L5A1_DEV_CENTER, &Profile);
  /* get low-level driver pointers */
  ll_sensor_obj = (VL53L5CX_Object_t*)VL53L5A1_RANGING_SENSOR_CompObj[VL53L5A1_DEV_CENTER];
  ll_sensor_dev = &ll_sensor_obj->Dev;
  /* start measuring */
  status = VL53L5A1_RANGING_SENSOR_Start(VL53L5A1_DEV_CENTER, RS_MODE_BLOCKING_CONTINUOUS);
  if (status != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (new_data) {
		status = VL53L5A1_RANGING_SENSOR_GetDistance(VL53L5A1_DEV_CENTER, &Results);
//		status = vl53l5cx_get_ranging_data(ll_sensor_dev, &ll_results);
		prepare_packages();
	}
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
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

/* USER CODE BEGIN 4 */
/**
 **************************************************************************************************
 * @brief 	Interrupt handler GPIOs
 *
 * @details Catch interrupt from sensor and set new_data variable
 **************************************************************************************************
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin) {
		case INT_C_Pin: {
			new_data = 1;
			break;
		}
		case B1_Pin: {
//			reset_sensor_i2c();
		}
		default: {
			break;
		}
	}
}

/**
 **************************************************************************************************
 * @brief 	Interrupt handler TIMs
 *
 * @details Catch interrupt from TIM and dequeue a pkg from the queue
 **************************************************************************************************
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3) {
		dequeue(&tx_queue);
	}
}

/**
 **************************************************************************************************
 * @brief 	Interrupt handler UART
 *
 * @details not implemented yet
 **************************************************************************************************
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	uint16_t size = huart->RxXferSize;
//	uint8_t *data = huart->pRxBuffPtr;
//
//	if (size != 5)
//		return;
//
//	/* Needed due to driver, always stays enabled */
//	Profile.EnableAmbient = 1; /* Enable: 1, Disable: 0 */
//	Profile.EnableSignal = 1; /* Enable: 1, Disable: 0 */
//
//	/* Stop sensor before applying settings */
//	VL53L5A1_RANGING_SENSOR_Stop(VL53L5A1_DEV_CENTER);
//
//	if (data[0] & (uint8_t)(0x01 << mode_continuous))
	return;
}


/**
 **************************************************************************************************
 * @brief 	Convert sensor data to Pkgs
 *
 * @details Add header to pkg and byte stream of data and enqueue th pkg to the tx queue
 **************************************************************************************************
 */
void prepare_packages(void)
{
	uint8_t zones = 64;
	uint16_t tmp_d = 0;
	uint8_t tmp[PKG_SIZE] 	= {0};
	tmp[(uint8_t)PkgType] 	= (uint8_t)RangePkg;
	tmp[(uint8_t)PkgNum] 	= 1;
	if (get_flag(resolution_4x4, flags)) {
		tmp[(uint8_t)PkgNumEst]	= 1;
		zones = 16;
	} else {
		tmp[(uint8_t)PkgNumEst]	= 3;
		zones = 64;
	}
	tmp[(uint8_t)PkgId] 	= 0; // UNUSED

	uint8_t pos = PKG_HEAD;

	for (uint8_t n = 0; n < zones; ++n) {
		if (pos >= PKG_SIZE) {
			enqueue(&tx_queue, tmp);
			pos = PKG_HEAD;
			tmp[(uint8_t)PkgNum]++;
			memset((uint8_t*)&tmp[PKG_HEAD], 0x00, (PKG_SIZE - PKG_HEAD));
		}
		tmp_d = (uint16_t)Results.ZoneResult[n].Distance[0];
		tmp[pos] = (uint8_t)(tmp_d & 0x00FF);
		tmp[pos+1] = (uint8_t)((tmp_d & 0xFF00) >> 8);
		pos += 2;
	}
	enqueue(&tx_queue, tmp);
}

/**
 **************************************************************************************************
 * \brief				Initialize queue for data packages (set head and tail to NULL)
 * \param				q: Queue to be initialized
 * \retval				None
 **************************************************************************************************
 */
static void init_queue(queue* q)
{
	q->head = NULL;
	q->tail = NULL;
}

/**
 **************************************************************************************************
 * \brief				Create a new package with given data and enqueue in FIFO
 * \param q				Pointer to queue
 * \param data			Pointer to data
 * \see 				prepare_package()
 * \retval				0: fail (no memory available)
 * \retval				1: success
 **************************************************************************************************
 * \details
 * The function is called by prepare_package(), after the data was converted to desired output.
 * The first byte contains information about the following 48 data bytes, so that they can be
 * related to their type.
 **************************************************************************************************
 */
static uint8_t enqueue(queue* q, uint8_t* data)
{
	// create new node
	node* new_node = malloc(sizeof(node));
	// no memory available
	if(new_node == NULL){
		return 0;
	}
	// copy data, no next node
	memset(new_node->data, 0x00, PKG_SIZE);
	memcpy(new_node->data, data, PKG_SIZE);
	new_node->next = NULL;
	// if queue not empty make following of previous tail
	if(q->tail != NULL){
		q->tail->next = new_node;
	}
	// make new tail
	q->tail = new_node;
	// if queue empty make head
	if(q->head == NULL){
		q->head = new_node;
	}
	return 1;
}

/**
 **************************************************************************************************
 * \brief				Transmit first package in queue
 * \param				q: Queue to dequeue a node (FIFO)
 * \retval				0: Queue empty
 * \retval				1: Success
 **************************************************************************************************
 * \details
 * Transmits first package (head) of the queue and and readjusts the head and tail if the queue is
 * empty, frees the memory of the node previously allocated.
 **************************************************************************************************
 */
static uint8_t dequeue(queue* q)
{
	// if queue empty to nothing
	if(q->head == NULL)
		return 0;
	else{
		// save head
		node* tmp = q->head;
		// transmit data via USB
		HAL_UART_Transmit_DMA(&huart2, tmp->data, PKG_SIZE);
		// remove node, save new head
		q->head = q->head->next;
		// if queue empty make sure there is no tail
		if(q->head == NULL){
			q->tail = NULL;
		}
		// free memory
		free(tmp->data);
		free(tmp);
		return 1;
	}
}

/**
 **************************************************************************************************
 * @brief 	Set a Flag in a Variable containing the bit-flags
 *
 * @details not used yet
 **************************************************************************************************
 */
static void set_flag(ControlFlags flag, uint8_t value)
{
	uint8_t tmp = ((value != 0) ? 1 : 0);
	if (tmp)
		flags |= (tmp << flag);
	else
		flags &= ~(tmp << flag);
}


/**
 **************************************************************************************************
 * @brief 	Check a Variable containing the bit-flags for a specific flag
 *
 * @details not used yet
 **************************************************************************************************
 */
static uint8_t get_flag(ControlFlags flag, uint8_t flag_variable)
{
	if (flag_variable & (uint8_t)(0x01 << flag))
		return 0x01;
	return 0x00;
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
