/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"
#include "i2c.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "onewire.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
//extern char SDPath[4];  /* SD logical drive path */

void delay (uint16_t time) {
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim10, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim10))<time);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    IMU_EN_SENSOR_TYPE enMotionSensorType, enPressureType;
    IMU_ST_ANGLES_DATA stAngles;
    IMU_ST_SENSOR_DATA stGyroRawData;
    IMU_ST_SENSOR_DATA stAccelRawData;
    IMU_ST_SENSOR_DATA stMagnRawData;
    int32_t s32PressureVal = 0, s32TemperatureVal = 0, s32AltitudeVal = 0;

    imuInit(&enMotionSensorType, &enPressureType);

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
  MX_USART1_UART_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  MX_TIM10_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim10);

  //some lora init...
    HAL_GPIO_WritePin(LoRa_M0_GPIO_Port, LoRa_M0_Pin, 0);
    HAL_GPIO_WritePin(LoRa_M1_GPIO_Port, LoRa_M1_Pin, 1);

    uint8_t command[8];
    command[0] = 0xC0;
    command[1] = 0x00;
    command[2] = 0x05;
    command[3] = 0x00;
    command[4] = 0x00;
    command[5] = 0x00;
    command[6] = 0x61;
    command[7] = 0x17;
    HAL_UART_Transmit(&huart1, command, 8, 500);

    //normal mode
    HAL_GPIO_WritePin(LoRa_M0_GPIO_Port, LoRa_M0_Pin, 0);
    HAL_GPIO_WritePin(LoRa_M1_GPIO_Port, LoRa_M1_Pin, 0);
    uint8_t data[6];
    data[0] = 0x00;
    data[1] = 0x00;
    data[2] = 0x17;
    data[3] = 0xAA;
    data[4] = 0xBB;
    data[5] = 0xCC;

    // SD test
    FRESULT res;                                          /* FatFs function common result code */
    uint32_t byteswritten, bytesread;                     /* File write/read counts */
    uint8_t wtext[] = "Hello write from Stm32 to Micro SD!!!"; /* File write buffer */

      if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
      {
          Error_Handler();
      }
      else
      {
          HAL_GPIO_WritePin(RGBblue_GPIO_Port,RGBblue_Pin , GPIO_PIN_SET);
          HAL_Delay(1000);
          HAL_GPIO_WritePin(RGBblue_GPIO_Port,RGBblue_Pin , GPIO_PIN_RESET);
          HAL_Delay(100);
      }

      if(f_open(&MyFile, "my001.txt",FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
      {
          Error_Handler();
      }
      else
      {
          HAL_GPIO_WritePin(RGBblue_GPIO_Port,RGBblue_Pin , GPIO_PIN_SET);
          HAL_Delay(1000);
          HAL_GPIO_WritePin(RGBblue_GPIO_Port,RGBblue_Pin , GPIO_PIN_RESET);
          HAL_Delay(100);
      }

      res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);

      if((byteswritten == 0) || (res != FR_OK))
      {
          Error_Handler();
      }
      else
      {
          HAL_GPIO_WritePin(RGBblue_GPIO_Port,RGBblue_Pin , GPIO_PIN_SET);
          HAL_Delay(1000);
          HAL_GPIO_WritePin(RGBblue_GPIO_Port,RGBblue_Pin , GPIO_PIN_RESET);
          HAL_Delay(100);
      }
      f_close(&MyFile);

    // I2C test
      uint8_t buf[8] = {0};
      uint8_t separator[] = " . ";
      uint8_t new_line[] = "\r\n";
      uint8_t start_text[] = "Start scanning I2C: \r\n";
      uint8_t end_text[] = "\r\nStop scanning";

      uint8_t row = 0, state;

      HAL_Delay(1000);

      // процедура сканирования
      HAL_UART_Transmit(&huart1, start_text, sizeof(start_text), 128);
      for( uint8_t i=1; i<128; i++ ){
          state = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 3, 5);

          if ( state != HAL_OK ){ // нет ответа от адреса
              HAL_UART_Transmit(&huart1, separator, sizeof(separator), 128);
          }

          else if(state == HAL_OK){ // есть ответ
              sprintf(buf, "0x%X", i);
              HAL_UART_Transmit(&huart1, buf, sizeof(buf), 128);
          }

          if( row == 15 ){
              row = 0;
              HAL_UART_Transmit(&huart1, new_line, sizeof(new_line), 128);
          } else
              row ++;
      }
      HAL_UART_Transmit(&huart1, end_text, sizeof(end_text), 128);

    // 10DOF test
      imuDataGet( &stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
      pressSensorDataGet(&s32TemperatureVal, &s32PressureVal, &s32AltitudeVal);
      printf("\r\n /-------------------------------------------------------------/ \r\n");
      //printf("\r\n Roll: %.2f     Pitch: %.2f     Yaw: %.2f \r\n",stAngles.fRoll, stAngles.fPitch, stAngles.fYaw);
      printf("\r\n Acceleration: X: %d     Y: %d     Z: %d \r\n",stAccelRawData.s16X, stAccelRawData.s16Y, stAccelRawData.s16Z);
      printf("\r\n Gyroscope: X: %d     Y: %d     Z: %d \r\n",stGyroRawData.s16X, stGyroRawData.s16Y, stGyroRawData.s16Z);
      printf("\r\n Magnetic: X: %d     Y: %d     Z: %d \r\n",stMagnRawData.s16X, stMagnRawData.s16Y, stMagnRawData.s16Z);
      //printf("\r\n Pressure: %.2f     Altitude: %.2f \r\n",(float)s32PressureVal/100, (float)s32AltitudeVal/100);
      //printf("\r\n Temperature: %.1f \r\n", (float)s32TemperatureVal/100);
      HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_GPIO_WritePin(RGBgreen_GPIO_Port, RGBgreen_Pin, GPIO_PIN_SET);

      //HAL_GPIO_TogglePin(RedLED_GPIO_Port, RedLED_Pin);
      //HAL_Delay(700);
      uint8_t deb[] = "12345\n";
      deb[2] = 'x';

      float Temperature = (float)DS18B20_Getemp()/16;

      HAL_UART_Transmit(&huart1, deb, 6, 100);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
      HAL_GPIO_WritePin(RGBred_GPIO_Port, RGBred_Pin, GPIO_PIN_SET);

      HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
      HAL_Delay(500);
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
