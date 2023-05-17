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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib_lcd.h"
#include "stdio.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

static rgb_lcd lcdData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


//#define BMP280_ADDRESS      0x77   // Adresse I2C du capteur BMP280
#define BMP280_TEMPERATURE_MSB_REG 0xFA   // Registre de température MSB
#define BMP280_TEMPERATURE_LSB_REG 0xFB   // Registre de température LSB
#define BMP280_TEMPERATURE_XLSB_REG 0xFC  // Registre de température XLSB
#define BMP280_CTRL_MEAS_REG 0xF4         // Registre de configuration de mesure
#define BMP280_TEMP_MEASUREMENT 0x2E      // Valeur de commande pour mesurer la température
#define BMP280_PRESS_MEASUREMENT 0x34     // Valeur de commande pour mesurer la pression

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static const uint8_t BMP280_ADDR = 0x77 << 1; // ADDR BMP280
static const uint8_t ADDR_REG_TEMP = 0xf4; //Adresse du registre de contrôle de la mesure de température
unsigned short dig_T1, dig_P1;
signed short dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
float temperature, pressure, altitude, init_height;
signed long temperature_raw, pressure_raw;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void BMP280_get_calib_values(void)
{
	uint8_t rx_buff[24], starting_address=0x88;

	HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDR, &starting_address, 1, 10000);
	HAL_I2C_Master_Receive(&hi2c1, BMP280_ADDR, &rx_buff[0], 24, 10000);

	dig_T1=(rx_buff[0])+(rx_buff[1]<<8);
	dig_T2=(rx_buff[2])+(rx_buff[3]<<8);
	dig_T3=(rx_buff[4])+(rx_buff[5]<<8);
	dig_P1=(rx_buff[6])+(rx_buff[7]<<8);
	dig_P2=(rx_buff[8])+(rx_buff[9]<<8);
	dig_P3=(rx_buff[10])+(rx_buff[11]<<8);
	dig_P4=(rx_buff[12])+(rx_buff[13]<<8);
	dig_P5=(rx_buff[14])+(rx_buff[15]<<8);
	dig_P6=(rx_buff[16])+(rx_buff[17]<<8);
	dig_P7=(rx_buff[18])+(rx_buff[19]<<8);
	dig_P8=(rx_buff[20])+(rx_buff[21]<<8);
	dig_P9=(rx_buff[22])+(rx_buff[23]<<8);
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
    MX_USART2_UART_Init();
    MX_I2C1_Init();
    /* USER CODE BEGIN 2 */
    lcd_init(&hi2c1, &lcdData);
    reglagecouleur(255,0,0);
    clearlcd();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /*
	  uint8_t buf[2];
	  uint8_t mesure[2];

	  BMP280_get_calib_values();


	  buf[0] = 0xfb; // Adresse du registre LSB de la température
	  buf[1] = 0;
	  HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDR, buf, 2, 100);
	  HAL_I2C_Master_Receive(&hi2c1, BMP280_ADDR , mesure, 3, 100);
*/


	  BMP280_get_calib_values();

	  uint8_t buf[2];
	  buf[0] = ADDR_REG_TEMP;
	  buf[1] = 0x2e; // Valeur à écrire dans le registre pour lancer la mesure
	  float temperature;

	    // Écrire la valeur 0x2e dans le registre 0xf4 pour lancer la mesure de température
	  HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDR, buf, 2, 200);

	    // Attendre la fin de la mesure de température (conversion de l'ADC interne)

	    // Lire la température depuis les registres 0xfa et 0xfb
	  uint8_t mesure[6];
	  buf[0] = 0xf7; // Adresse du registre LSB de la température
	  HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDR , buf, 1, 100);
	  HAL_I2C_Master_Receive(&hi2c1, BMP280_ADDR , mesure, 6, 100);
	  int x;


	  volatile uint32_t temp[3];
	  temp[2]=mesure[3];
	  temp[1]=mesure[4];
	  temp[0]=mesure[5];
	  temperature_raw=(temp[2]<<12)+(temp[1]<<4)+(temp[0]>>4);

	  temp[2]=mesure[0];
	  temp[1]=mesure[1];
	  temp[0]=mesure[2];
	  pressure_raw=(temp[2]<<12)+(temp[1]<<4)+(temp[0]>>4);

	  double var1, var2;
	  var1=(((double)temperature_raw)/16384.0-((double)dig_T1)/1024.0)*((double)dig_T2);
	  var2=((((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0)*(((double)temperature_raw)/131072.0-((double)dig_T1)/8192.0))*((double)dig_T3);
	  double t_fine = (int32_t)(var1+var2);
	  volatile	float T = (var1+var2)/5120.0;

	  x = T*100;
	  T=(float)x/100;
	  temperature = T;
	  char Data[10];
	  sprintf(Data,"T = %.2f C", temperature);

	  double var3, var4;
	  var3=((double)t_fine/2.0)-64000.0;
	  var4=var3*var3*((double)dig_P6)/32768.0;
	  var4=var4+var3*((double)dig_P5)*2.0;
	  var4=(var4/4.0)+(((double)dig_P4)*65536.0);
	  var3=(((double)dig_P3)*var3*var3/524288.0+((double)dig_P2)*var3)/524288.0;
	  var3=(1.0+var3/32768.0)*((double)dig_P1);
	  volatile	double p=1048576.0-(double)pressure_raw;
	  p=(p-(var4/4096.0))*6250.0/var3;
	  var3=((double)dig_P9)*p*p/2147483648.0;
	  var4=p*((double)dig_P8)/32768.0;
	  p=p+(var3+var4+((double)dig_P7))/16.0;
	  pressure=p/100;




	  char Data2[10];

	  sprintf(Data2,"P = %.2f Pa", pressure);

	  lcd_position(&hi2c1, 0, 0);
	  lcd_print(&hi2c1, Data);
	  lcd_position(&hi2c1, 0, 1);
	  lcd_print(&hi2c1, Data2);


	  HAL_Delay(200);



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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
