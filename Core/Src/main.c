/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
//Чем ниже - тем выше приоритет
#define ID_SET_LED 0x0001
#define ID_DATA 0x0002
#define ID_CONNECT 0x0003
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
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

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_TxHeaderTypeDef TxHeader_Connectivity;
CAN_TxHeaderTypeDef TxHeader_SetLED;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[3] = {0,};
uint8_t TxDataSet[3] = {0,};
uint8_t RxData[3] = {0,};
uint32_t TxMailBox = 0;
uint8_t ConnectionErr = 0;
uint8_t ConErrMsg[] = "L4R5ZI: CAN соединение разорвано\r";
uint8_t ConResMsg[] = "L4R5ZI: CAN соединение восстановлено\r";
uint8_t* USB_Rx;
uint8_t BlinkSkip = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Прерывание на сообщение по CAN шине
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1) {
    if(HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
    	HAL_TIM_Base_Stop_IT(&htim8);
    	//Если связь была потеряна, отключение светодиодов
    	if (ConnectionErr) {
    		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
    	    CDC_Transmit_FS(ConResMsg, strlen((char *)ConResMsg));
    	}
    	ConnectionErr = 0;
    	if (RxHeader.StdId == ID_DATA) {
        	if (RxData[0]) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
        	if (RxData[1]) HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        	if (RxData[2]) HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
    	}
    	if (RxHeader.StdId == ID_SET_LED) {
			TxData[0] = RxData[0];
			TxData[1] = RxData[1];
			TxData[2] = RxData[2];
    	}
    	//Сброс счётчика таймера отсутствия соединения и его запуск
    	__HAL_TIM_SetCounter(&htim8,0);
    	HAL_TIM_Base_Start_IT(&htim8);
    }
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin) {
	if(GPIO_Pin == GPIO_PIN_13) {
		//Отключение прерывания для устранения дребезга
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
		//Есть ли данные от USB. Да = перезапись пинов для зажигания
		//Посылать так "echo -ne "\x1\x1\x1\x1\x1" > /dev/ttyACM1"
		//\x LD3 \x LD2 \x LD1 \x Отправить данные из USB на другую плату по CAN \x Записать данные из USB локально
		USB_Rx = USB_Rx_Msg();
		if (USB_Rx) {
			if (USB_Rx[3]) {
				TxDataSet[0] = USB_Rx[0];
				TxDataSet[1] = USB_Rx[1];
				TxDataSet[2] = USB_Rx[2];
				HAL_CAN_AddTxMessage(&hcan1, &TxHeader_SetLED, TxDataSet, &TxMailBox);
			}
			if (USB_Rx[4]) {
				TxData[0] = USB_Rx[0];
				TxData[1] = USB_Rx[1];
				TxData[2] = USB_Rx[2];
			}
		}
		//Отправка сообщения
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailBox);
		//Включение прерывания через 500 мс
		HAL_TIM_Base_Start_IT(&htim1);
	}
}
//Прерывания на таймеры
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
        if(htim->Instance == TIM1) {
        	HAL_TIM_Base_Stop_IT(&htim1);
        	__HAL_TIM_SetCounter(&htim1,0);
        	if (ConnectionErr) {
        		//Поскольку таймер настроен на 250мс, для мигания каждые 500мс надо пропускать одно переполнение
        		if (BlinkSkip) BlinkSkip = 0;
        		else {
        			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
        			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
        			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
        			BlinkSkip = 1;
        		}
        		HAL_TIM_Base_Start_IT(&htim1);
        	}
        	else {
        		__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);  // очищаем бит EXTI_PR (бит прерывания)
        		NVIC_ClearPendingIRQ(EXTI15_10_IRQn); // очищаем бит NVIC_ICPRx (бит очереди)
        		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);   // включаем внешнее прерывание
        	}
        }
        if(htim->Instance == TIM8) {
        	HAL_TIM_Base_Stop_IT(&htim8);
        	__HAL_TIM_SetCounter(&htim8,0);
        	ConnectionErr = 1;
            BlinkSkip = 1;
        	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        	CDC_Transmit_FS(ConErrMsg, strlen((char *)ConErrMsg));
        	//Поскольку TIM1 настроен на 250мс, можно использовать его для мигания светодиодами
        	HAL_TIM_Base_Start_IT(&htim1);
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
  MX_CAN1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  //Пакет с данными
  TxHeader.StdId = ID_DATA;
  TxHeader.DLC = 3;
  //Красный светодиод
  TxData[0] = 0;
  //Синий светодиод
  TxData[1] = 1;
  //Зелёный светодиод
  TxData[2] = 0;
  //Пакет проверки соединения
  TxHeader_Connectivity.StdId = ID_CONNECT;
  TxHeader_Connectivity.DLC = 0;
  //Пакет установки светодиода
  TxHeader_SetLED.StdId = ID_SET_LED;
  TxHeader_SetLED.DLC = 3;
  //Одинаковые параметры пакетов
  TxHeader.ExtId = TxHeader_Connectivity.ExtId = TxHeader_SetLED.ExtId = 0;
  TxHeader.RTR = TxHeader_Connectivity.RTR = TxHeader_SetLED.RTR = 0;
  TxHeader.IDE = TxHeader_Connectivity.IDE = TxHeader_SetLED.IDE = 0;
  TxHeader.TransmitGlobalTime = TxHeader_Connectivity.TransmitGlobalTime = TxHeader_SetLED.TransmitGlobalTime = 0;
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  //Если не сбросить флаг, первое прерывание произойдёт непредсказуемо
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF);
  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_SR_UIF);
  //Таймер на 3 секунды для проверки соединения
  HAL_TIM_Base_Start_IT(&htim8);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0);
	if (HAL_OK != HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Connectivity, 0, &TxMailBox))
		Error_Handler();
	while (HAL_CAN_IsTxMessagePending(&hcan1, TxMailBox));
  	HAL_Delay(1000);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
