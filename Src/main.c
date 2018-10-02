/* Includes ------------------------------------------------------------------*/
#include "header.h"
/* Private variables ---------------------------------------------------------*/
BMP280_HandleTypedef bmp280;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
HAL_StatusTypeDef	flash_ok = HAL_ERROR;
/* Private variables ---------------------------------------------------------*/
float pressure, temperature, humidity;
uint8_t Data_ERROR[] = {0xDC, 0x00, 0x00, 0x00, 0x00};
uint8_t Data_Tx[32];
unsigned int result = 0;
uint8_t buffer_TX[RF_DATA_SIZE]; ////
uint8_t size_UART = 0;

uint8_t Tcounter = 0;
uint8_t Tcounter1 = 0;

uint8_t Data[10];
uint8_t Data_Config[10] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09};
uint8_t str=0;
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_NVIC_Init(void);
extern void    FLASH_PageErase(uint32_t PageAddress);
uint32_t flash_read(uint32_t address) {
	return (*(__IO uint32_t*) address);
}
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/


/* Раскомментировать, если здесь нужны микросекунды---------------------------*/
__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
{
  micros *= (SystemCoreClock / 1000000) / 8;
  /* Wait till done */
  while (micros--) ;
}
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
	Tcounter ++;
	Tcounter1 ++;
}

int main(void)
{
	uint8_t Flag_Start_colect = 0;
//	uint8_t Flag_Error_funct = 0;
	uint8_t Flag_Receive_valid = 0;
	uint8_t Flag_Receive_Buff_is_Full = 0;
	uint8_t count = 0;
	
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
	MX_NVIC_Init();
	HAL_UART_MspInit(&huart2);

	HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim2);

	
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;

	HAL_Delay(500);


	
	while (!bmp280_init(&bmp280, &bmp280.params)) {
		HAL_Delay(2000);
	}
	while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
		HAL_Delay(2000);
	}	
	
			uint16_t rezAtmPressureGPa_uint =	(uint16_t)pressure;
			uint16_t rezHumidity_uint =      	(uint16_t)(humidity * 10);
			int16_t rezTemperature_int = 			(int16_t)(temperature * 10);			
			uint16_t rezAtmPressure_uint = 		(uint16_t)pressure * 0.75;

	
	
			buffer_TX[0]=0xDC;
			buffer_TX[1]=0x03;
			buffer_TX[2]=0x10;
			buffer_TX[3]=0;
			buffer_TX[4]=0;
			buffer_TX[5]=rezTemperature_int >> 8;		// старший
			buffer_TX[6]=rezTemperature_int & 0xFF;	// младший
			buffer_TX[7]=rezHumidity_uint >> 8;
			buffer_TX[8]=rezHumidity_uint & 0xFF;
			buffer_TX[9]=rezAtmPressure_uint >> 8;
			buffer_TX[10]=rezAtmPressure_uint & 0xFF;
			buffer_TX[11]=rezAtmPressureGPa_uint >> 8;
			buffer_TX[12]=rezAtmPressureGPa_uint & 0xFF;
			result = calculate_Crc16(buffer_TX, 13);
			buffer_TX[13] = result & 0xFF;
			buffer_TX[14] = result >> 8;			

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//##################################################################
		if (flash_read(0x08003E80) == 0){
			flash_ok = HAL_ERROR;			
			while(flash_ok != HAL_OK){			//Делаем память открытой
				flash_ok = HAL_FLASH_Unlock();
			}
			flash_ok = HAL_ERROR;
			while(flash_ok != HAL_OK){			
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, 0x08003E80, 0x57);
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, 0x08003E90, 9600);
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, 0x08003EA0, 0);
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, 0x08003EB0, 1);	
			}	
			while(flash_ok != HAL_OK){			//Закрываем память
				flash_ok = HAL_FLASH_Lock();
			}			
		}
//##################################################################
FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t PAGEError = 0;

EraseInitStruct.TypeErase   =  TYPEERASE_PAGES;
EraseInitStruct.PageAddress = 0x08003E80;
EraseInitStruct.NbPages     = (0x08003F10 - 0x08003E80) / FLASH_PAGE_SIZE;
//EraseInitStruct..NbPages = 1;

	
if (flash_read(0x08003E80) == 0x57){
HAL_UART_Transmit(&huart2, &Data_Config[0], 1, 0xFFFF);
			flash_ok = HAL_ERROR;
			
				flash_ok = HAL_FLASH_Unlock();//Делаем память открытой

			while(flash_ok != HAL_OK){
				flash_ok = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
HAL_UART_Transmit(&huart2, &Data_Config[1], 1, 0xFFFF);
			}
		
				flash_ok = HAL_FLASH_Lock();//Закрываем память


		
				flash_ok = HAL_FLASH_Unlock();//Делаем память открытой

HAL_UART_Transmit(&huart2, &Data_Config[2], 1, 0xFFFF);			
			
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, 0x08003E80, 0x58);
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, 0x08003E90, 9600);
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, 0x08003EA0, 0);
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, 0x08003EB0, 1);

HAL_UART_Transmit(&huart2, &Data_Config[3], 1, 0xFFFF);			
		
				flash_ok = HAL_FLASH_Lock();//Закрываем память
	
	}




			
//		HAL_Delay(500);
//		if (flash_read(0x08003E80) == 0x57){
//			HAL_UART_Transmit(&huart2, &Data_Config[0], 1, 0xFFFF);
//			flash_ok = HAL_ERROR;			
//			while(flash_ok != HAL_OK){			//Делаем память открытой
//				flash_ok = HAL_FLASH_Unlock();
//			}		
//			HAL_UART_Transmit(&huart2, &Data_Config[1], 1, 0xFFFF);
//			HAL_Delay(500);
//			flash_ok = HAL_ERROR;
//			FLASH_PageErase(0x08003E80);
//			FLASH_PageErase(0x08003F00);
//			HAL_UART_Transmit(&huart2, &Data_Config[2], 1, 0xFFFF);
//			HAL_Delay(500);
//			while(flash_ok != HAL_OK){			//Закрываем память
//				flash_ok = HAL_FLASH_Lock();
//			}	
//			HAL_UART_Transmit(&huart2, &Data_Config[3], 1, 0xFFFF);
//			flash_ok = HAL_ERROR;			
//			while(flash_ok != HAL_OK){			//Делаем память открытой
//				flash_ok = HAL_FLASH_Unlock();
//			}
//			HAL_Delay(500);
//			flash_ok = HAL_ERROR;
//			while(flash_ok != HAL_OK){			
//				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, 0x08003E80, 0x58);
//				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, 0x08003E90, 9600);
//				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, 0x08003EA0, 0);
//				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, 0x08003EB0, 1);
//				HAL_UART_Transmit(&huart2, &Data_Config[4], 1, 0xFFFF);
//			}
//			HAL_Delay(500);
//			while(flash_ok != HAL_OK){			//Закрываем память
//				flash_ok = HAL_FLASH_Lock();
//			}			
//			
//		}
//HAL_Delay(500);

						
		
		
		

		







	HAL_UART_Receive_IT(&huart2,&str, 1);
  while (1)
  {	
		if (Tcounter >= TIME_FLASH_LED) {
			GPIOB->ODR^=(GPIO_PIN_1);
			Tcounter = 0;	
		}

//  ******************************************************************************		
		if(huart2.RxXferCount==0){																										//
			if (str == 0xDC){																																				// Проверяем кому адресовано сообщение. Если нам - (0xDC) то
				Flag_Start_colect = 1;																																// ставим флаг разрешения сбора данных
			}
			if (Flag_Start_colect == 1){																														// Наполняем буфер
				Data[count] = str;			
				count ++;
			}	
			if (count == 8){																																				// Если буфер полный - останавливаем UART
//				HAL_UART_MspDeInit(&huart2);																													//
				Flag_Receive_Buff_is_Full = 1;																												//  Ставим флаг буфер полный
				Flag_Start_colect = 0;																																// 
				count = 0;																																						// 
			}		
			HAL_UART_Receive_IT(&huart2, &str, 1);																										// Start UART Recive again					
		}		
//  ******************************************************************************
			if (Flag_Receive_Buff_is_Full == 1){																										// Проверяем CRC принятого сообщения и устанавливаем флаг валидности
				result = calculate_Crc16(Data, 6);																										// 
				uint8_t q = result & 0xFF;																														//
				uint8_t w = result >> 8;																															//
				if (Data[6] == q & Data[7] == w){																											// Если CRC ок - ставим флаг что CRC валидный
					Flag_Receive_valid = 1;																															//					
				} else {																																							//
					Flag_Receive_Buff_is_Full = 0;																											// Если иначе - очищаем буфер и флаги
					for (uint8_t i = 0; i<=7; i++){																											//
						Data[i] = 0;																																			//
					}																																										//
				}																																											//
			}																																												//
	
			if (Flag_Receive_valid == 1 && Flag_Receive_Buff_is_Full == 1 && Data[1] != 0x03){				// Проверяем что сообщение содержит функцию 03
				Data_ERROR[1] = Data[1]|0x80;																													// если нет - то формируем массив с ответом-исключением
				Data_ERROR[2] = 0x01;
				result = calculate_Crc16(Data_ERROR, 3);																							// и отправляем ответ исключение
				Data_ERROR[3] = result & 0xFF;																												//
				Data_ERROR[4] = result >> 8;																													//
				DelayMicro(4);
				HAL_UART_Transmit(&huart2, Data_ERROR, 5, 0xFFFF);																		//
				Flag_Receive_Buff_is_Full = 0;																											  // очищаем буфер и флаги
				Flag_Receive_valid = 0;
				for (uint8_t i = 0; i<=7; i++){
					Data[i] = 0;
				}				
			}
			
			if (Flag_Receive_valid == 1 && Flag_Receive_Buff_is_Full == 1 && Data[2] != 0x00 | Data[3] != 0x00){// Проверяем что сообщение содержит address 0000
				Data_ERROR[1] = Data[1]|0x80;																													// если нет - то формируем массив с ответом-исключением
				Data_ERROR[2] = 0x02;
				result = calculate_Crc16(Data_ERROR, 3);																							// и отправляем ответ исключение
				Data_ERROR[3] = result & 0xFF;																												//
				Data_ERROR[4] = result >> 8;																													//
				DelayMicro(4);
				HAL_UART_Transmit(&huart2, Data_ERROR, 5, 0xFFFF);																		//
				Flag_Receive_Buff_is_Full = 0;																											  // очищаем буфер и флаги
				Flag_Receive_valid = 0;
				for (uint8_t i = 0; i<=7; i++){
					Data[i] = 0;
				}				
			}			

			if (Flag_Receive_valid == 1 && Flag_Receive_Buff_is_Full == 1 && Data[4] != 0x00 | Data[5] != 0x05){				// Проверяем что сообщение содержит data 0005
				Data_ERROR[1] = Data[1]|0x80;																													// если нет - то формируем массив с ответом-исключением
				Data_ERROR[2] = 0x03;
				result = calculate_Crc16(Data_ERROR, 3);																							// и отправляем ответ исключение
				Data_ERROR[3] = result & 0xFF;																												//
				Data_ERROR[4] = result >> 8;																													//
				DelayMicro(4);
				HAL_UART_Transmit(&huart2, Data_ERROR, 5, 0xFFFF);																		//
				Flag_Receive_Buff_is_Full = 0;																											  // очищаем буфер и флаги
				Flag_Receive_valid = 0;
				for (uint8_t i = 0; i<=7; i++){
					Data[i] = 0;
				}				
			}	
			
			if (Flag_Receive_valid == 1 && Flag_Receive_Buff_is_Full == 1 && Data[5] == 0x05){				//Если принятая функция отвечает требованиям и данные валидны -
				DelayMicro(4);
				HAL_UART_Transmit(&huart2, buffer_TX, 15, 0xFFFF);																		// отправляем ответ с данными
				Flag_Receive_Buff_is_Full = 0;																												// очищаем буфер и флаги
				Flag_Receive_valid = 0;																																//
				for (uint8_t i = 0; i<=7; i++){																												//
					Data[i] = 0;																																				//
				}																																											//
			}			
			
		
		if (Tcounter1 >= TIME_SENDING) {
			Tcounter1 = 0;
			while (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
				HAL_Delay(2000);
				}			
			uint16_t rezAtmPressureGPa_uint =  (uint16_t)pressure;
			uint16_t rezHumidity_uint =        (uint16_t)(humidity * 10);
			int16_t rezTemperature_int = (int16_t)(temperature * 10);			
			uint16_t rezAtmPressure_uint = (uint16_t)pressure * 0.75;
				
			buffer_TX[5]=rezTemperature_int >> 8;		// старший
			buffer_TX[6]=rezTemperature_int & 0xFF;	// младший
			buffer_TX[7]=rezHumidity_uint >> 8;
			buffer_TX[8]=rezHumidity_uint & 0xFF;
			buffer_TX[9]=rezAtmPressure_uint >> 8;
			buffer_TX[10]=rezAtmPressure_uint & 0xFF;
			buffer_TX[11]=rezAtmPressureGPa_uint >> 8;
			buffer_TX[12]=rezAtmPressureGPa_uint & 0xFF;				
				result = calculate_Crc16(buffer_TX, 13);
				buffer_TX[13] = result & 0xFF;
				buffer_TX[14] = result >> 8;				
				
				
				
//		size_UART = sprintf((char *)Data_Tx,"Pressure: %.2d GPa, Temperature: %.1d C, Humidity: %.2d\n\r",
//				rezAtmPressureGPa_uint, rezTemperature_int, rezHumidity_uint);
//		HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);				
//				
//				size_UART = sprintf((char *)Data_Tx,"I am alive\n\r");
//				HAL_UART_Transmit(&huart2, Data_Tx, size_UART, 0xFFFF);				
				
//_______________________________________
//№ байта  |    описание   buffer_TX[]   |
//_________|_____________________________| 
//   0     |    статус - LSB 
//   1     |    статус - MSB 
//   2     |    температура int16_t - LSB 
//   3     |    температура int16_t - MSB 
//   4     |    влажность uint16_t   - LSB 
//   5     |    влажность uint16_t   - MSB 
//   6     |    давление, мм. рт. ст uint16_t - LSB 
//   7     |    давление, мм. рт. ст uint16_t - MSB 
//   8     |    давление, гПа uint16_t - LSB 
//   9     |    давление, гПа uint16_t - MSB      		
				
		}	
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{	
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);	
	
//  RCC_OscInitTypeDef RCC_OscInitStruct;
//  RCC_ClkInitTypeDef RCC_ClkInitStruct;
//  RCC_PeriphCLKInitTypeDef PeriphClkInit;
//    /**Configure the main internal regulator output voltage 
//    */
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//    /**Initializes the CPU, AHB and APB busses clocks 
//    */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.HSICalibrationValue = 16;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
//  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
//    /**Initializes the CPU, AHB and APB busses clocks 
//    */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
//  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
//  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
//  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//    /**Configure the Systick interrupt time 
//    */
//  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

//    /**Configure the Systick 
//    */
//  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

//  /* SysTick_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);	
}

void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
	
		HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(USART2_IRQn);	
	
}



/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000708;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
//static void MX_SPI1_Init(void)
//{

//  /* SPI1 parameter configuration*/
//  hspi1.Instance = SPI1;
//  hspi1.Init.Mode = SPI_MODE_MASTER;
//  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi1.Init.NSS = SPI_NSS_SOFT;
//  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
//  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi1.Init.CRCPolynomial = 7;
////	CSN1;
////	CE0;	
//	
//  if (HAL_SPI_Init(&hspi1) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }

//}




/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 32000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



}

void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

		
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
