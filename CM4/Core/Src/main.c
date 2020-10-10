/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "eth.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "UM7_UART.h"
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define TRUE		1
#define FALSE		0

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

char uart_msg[100];

uint8_t tx_data[100];
uint8_t rx_data[100];

uint8_t rx_length;
uint8_t tx_length;

volatile uint8_t uart7_rcv_flag = FALSE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

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

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_UART7_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  uint8_t command_status = UM7_send_command(&huart7, CALIBRATE_ACCELEROMETER);

  if(command_status == COMMAND_SEND_ERROR){
	  while(1){
		  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		  HAL_Delay(500);
	  }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  HAL_Delay(1);
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);


	  UM7_read_operation(&huart7, DREG_ACCEL_PROC_X, 4, rx_data, &rx_length);

	  UM7_packet_struct_typedef packet;
	  float float_value_table[10];

	  parse_serial_data(rx_data, rx_length, &packet);

	  convert_to_ieee(packet.data, 4, float_value_table);

//	  sprintf(uart_msg, "Accel proc: X: %f  Y: %f  Z: %f Time: %f\n\r", float_value_table[0], float_value_table[1], float_value_table[2], float_value_table[3]);

	  sprintf(uart_msg, "%f\t%f\t%f\t%f\n\r", float_value_table[0], float_value_table[1], float_value_table[2], float_value_table[3]);


	  HAL_UART_Transmit(&huart6, (uint8_t*) uart_msg, strlen(uart_msg), HAL_MAX_DELAY);



//	  if(uart7_rcv_flag == TRUE){
//
//		  uart7_rcv_flag = FALSE;
//
//	      UM7_packet_struct_typedef um7_packet_data;
//
//    	  parse_serial_data(rx_data, PROC_ACCLE_DATA_LENGTH, &um7_packet_data);
//
////    	  uint8_t i=0;
////    	  for(i = 0; i < 4; i++){
////    		  uart_msg[i] = um7_packet_data.data[i];
////    	  }
////    	  uart_msg[i] = '\r';
////    	  uart_msg[i+1] = '\n';
////    	  uart_msg[i+2] = '\0';
//
//
////    	  sprintf(uart_msg, "%d %d %d %d\t%d %d %d %d\t%d %d %d %d\r\n", um7_packet_data.data[0], um7_packet_data.data[1],
////    			  um7_packet_data.data[2], um7_packet_data.data[3],
////			      um7_packet_data.data[4], um7_packet_data.data[5],
////				  um7_packet_data.data[6], um7_packet_data.data[7],
////				  um7_packet_data.data[8], um7_packet_data.data[9],
////				  um7_packet_data.data[10], um7_packet_data.data[11]);
//
//
//    	  memset(rx_data, '\0', sizeof(rx_data));
//
//   		  HAL_UART_Receive_IT(&huart7, rx_data, PROC_ACCLE_DATA_LENGTH);
//
//		  HAL_UART_Transmit(&huart6, (uint8_t*) uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
//
//		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
//
//
//	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
