/* Includes ------------------------------------------------------------------*/
#include "header.h"
/* Private variables ---------------------------------------------------------*/
BMP280_HandleTypedef bmp280;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
//PWR_PVDTypeDef sConfigPVD;
HAL_StatusTypeDef	flash_ok = HAL_ERROR;
/* Private variables ---------------------------------------------------------*/
float pressure, temperature, humidity;

uint16_t result = 0;


uint8_t Tcounter = 0;															//	�������  ��� ������ � ���������
uint8_t Tcounter1 = 0;														//	�������  ��� ������ � ���������
uint8_t Tcounter2 = 0;														//	�������  ��� ������ � ���������
uint8_t buffer_TX[15]; 														// 	����� ��� �������� ������
uint8_t Data[20];																	//	����� ��� ������ ������ �� �������
uint16_t Data_Config[4];													//	����� ��� ������ ������������ �� ����
uint8_t str=0;																		//	����� ��� ������ UART
uint8_t Data_ERROR[5];														//	����� ��� ������ � �������

uint8_t ID_Device;																//	ID ����������
uint8_t Flag_Start_colect = 0;										//	����, ����������� ���� ������ � �����
uint8_t Flag_Receive_conf = 0;										// 	����, ��� �� �������� ������������
uint8_t Flag_Receive_data = 0;										// 	����, ��� �� �������� ������
uint8_t Flag_Receive_valid = 0;										//	���� CRC ��
uint8_t Flag_Receive_Buff_is_Full = 0;						//	���� ����� ������ ������
uint8_t count = 0;																//	�������  ��� ������ UART	
uint8_t size_count = 0;	
	

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void WriteDefaultConf(void);
void ReadWeatherData(void);
void ClearDataBufer(void);
void AnswerWithError(uint8_t);
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


/* �����������������, ���� ����� ����� ������������---------------------------*/
//__STATIC_INLINE void DelayMicro(__IO uint32_t micros)
//{
//  micros *= (SystemCoreClock / 1000000) / 8;
//  /* Wait till done */
//  while (micros--) ;
//}

void USART2_IRQHandler(void)
{

	if (__HAL_UART_GET_FLAG(&huart2, USART_ISR_PE) != RESET){	
				ClearDataBufer();
			}
__HAL_UART_CLEAR_PEFLAG(&huart2);
  HAL_UART_IRQHandler(&huart2);
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
	Tcounter ++;
	Tcounter1 ++;
	Tcounter2 ++;
}

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  MX_I2C1_Init();
  MX_TIM2_Init();
	MX_NVIC_Init();

	HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim2);

//##########################################################################################
	if (flash_read(Address_Config_Bank1) == 0){																						//## ������ ������������ �� ���������
		WriteDefaultConf();																																	//## ���� � ���� ������� ������ ���
	} 																																										//## ���������� ������������
//##########################################################################################	
//##########################################################################################
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0){																				//## 
		ID_Device = flash_read(Address_Config_Bank2);																				//## ���� ����� ��������� 
	}else{																																								//## ������ ID_Device �� ������� �����
		ID_Device = flash_read(Address_Config_Bank1);																				//##
	}	 																																										//##
//##########################################################################################	
		
	Data_ERROR[0] = ID_Device;
	HAL_UART_MspInit(&huart2);
	MX_USART2_UART_Init();	
	
	bmp280_init_default_params(&bmp280.params);
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &hi2c1;
	if (!bmp280_init(&bmp280, &bmp280.params)) {buffer_TX[3] |=(1<<0);}														// �������� �������. ���� �� �������� - ������ ��� ������ ����������
	if (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {buffer_TX[3] |=(1<<0);}	// �������� �������. ���� �� �������� - ������ ��� ������ ����������	
			uint16_t rezAtmPressureGPa_uint =	(uint16_t)pressure;
			uint16_t rezHumidity_uint =      	(uint16_t)(humidity * 10);
			int16_t rezTemperature_int = 			(int16_t)(temperature * 10);			
			uint16_t rezAtmPressure_uint = 		(uint16_t)pressure * 0.75;
			if (pressure == 0){buffer_TX[3] |=(1<<0);} 																								// �������� �������. ���� �������� == 0, ������ ��� ������ ����������
		
			buffer_TX[0]=ID_Device;
			buffer_TX[1]=0x03;
			buffer_TX[2]=0x0A;
			buffer_TX[3]=0;
//			buffer_TX[4]=0;
			buffer_TX[5]=rezTemperature_int >> 8;		// �������
			buffer_TX[6]=rezTemperature_int & 0xFF;	// �������
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
			
	HAL_UART_Receive_IT(&huart2,&str, 1);
//  *********************************************************************************************		
  while (1)
  {	
		if (Tcounter2 >= 2) {ClearDataBufer();}																										// ���� ������� ����� ������ 1/20 ������� ������ ����� � �����
//  ******************************************************************************		
		if(huart2.RxXferCount==0){																										//
//  ******************************************************************************	
			Tcounter2 = 0;																																					// ���������� ������� �������� �����
			if (str == ID_Device){																																	// ��������� ���� ���������� ���������. ���� ��� - (0xDC) ��
				Flag_Start_colect = 1;																																// ������ ���� ���������� ����� ������
			}																																												//
			if (Flag_Start_colect == 1){																														// ��������� �����
				Data[count] = str;																																		//	
				count ++;																																							//
			}																																												//
			if (count == 2 && str == 0x10){																													// ���� ������ ������ ����������� ������ - ������� 16 - ��
				size_count = 13;																																			// ������ ��������� ������������ � ������
				Flag_Receive_conf = 1;																																// ���� ��� ��� ������������	
			}																																												//
			if (count == 2 && str != 0x10){																													// �����
				size_count = 8;																																				// ��������� ������ � ������
				Flag_Receive_data = 1;																																// ���� ��� ��� ������
			}																																												//
			if (count == size_count){																																// ���� ����� ������ 
				Flag_Receive_Buff_is_Full = 1;																												// ������ ���� ��� ����� ������ 
				Flag_Start_colect = 0;																																// ���������� ����, ����������� ��������� ������
				count = 0;																																						// 
			}																																												//
			
//			if (USART_ISR_PE == 1){AnswerWithError(0xFF);}
//			HAL_UART_Transmit(&huart2, &str, 1, 0xFFFF); //........................................................������ ��� ������� !!!!....................................................................
			HAL_UART_Receive_IT(&huart2, &str, 1);																									// Start UART Recive again					
		}		
//  ********************************************************************************************
			if (Flag_Receive_Buff_is_Full == 1){																										// ��������� CRC ��������� ��������� � ������������� ���� ����������
				if (Flag_Receive_conf == 1){size_count = 11;} 																				//
				else if (Flag_Receive_data == 1){size_count = 6;}																			//
					result = calculate_Crc16(Data, size_count);																					// 
					uint8_t q = result & 0xFF;																													//
					uint8_t w = result >> 8;																														//
				if ((Flag_Receive_data == 1 && Data[6] == q && Data[7] == w)||(Flag_Receive_conf == 1 && Data[11] == q && Data[12] == w)){// ���� CRC �� - ������ ���� ��� CRC ��������
					Flag_Receive_valid = 1;																															//					
				} else {																																							//
					ClearDataBufer();																																		// ���� ����� - ������� ����� � �����
				}																																											//
			}																																												//
//  *********************************************************************************************
			if (Flag_Receive_valid == 1 && Flag_Receive_data == 1){																		// ���� ��������� ������ � �������� ������
				if (Data[1] != 0x03){																																		// ��������� ��� ��������� �������� ������� 03
					AnswerWithError(0x01);																																//
				}																																												//
			
				if ((Flag_Receive_data == 1) && (Data[2] != 0x00 || (Data[3] != 0x00 && Data[3] != 0x05))){// ��������� ��� ��������� �������� address 0000 ��� 0005
					AnswerWithError(0x02);																																//
				}																																												//

				if ((Flag_Receive_data == 1) && (Data[4] != 0x00 || (Data[5] != 0x05 && Data[5] != 0x02))){// ��������� ��� ��������� �������� data 0005 ��� 0002
					AnswerWithError(0x03);																																//
				}																																												//	
//  *********************************************************************************************			
				if (Flag_Receive_data == 1 && Data[3] == 0 && Data[5] == 0x05){													// ���� �������� ������� ������ ������ -
					HAL_UART_Transmit(&huart2, buffer_TX, 15, 0xFFFF);																		// ���������� ����� � �������
							ClearDataBufer();																																	//	 ������� ����� � �����
				}																																												//
//  *********************************************************************************************
				if (Flag_Receive_data == 1 && Data[3] == 0x05 && Data[5] == 0x02){											// ���� �������� ������� ������ ������������ -
					Data[0] = ID_Device;																																	//
					Data[1] = 0x03;																																				//
					Data[2] = 0x04;																																				//
					Data[3] = flash_read(Address_Config_Bank1);																						//
					switch(flash_read(Address_Config_Bank1+0x10)){																				// 
						case 600: Data[4] = 1; break;																												// 
						case 1200: Data[4] = 2; break; 																											// 
						case 2400: Data[4] = 3; break;    																									// 
						case 4800: Data[4] = 4; break;   																										//  
						case 9600: Data[4] = 5; break;  																										// 
						case 14400: Data[4] = 6; break;																											// 
						case 19200: Data[4] = 7; break; 																										//    
						case 38400: Data[4] = 8; break;   																									//   
						case 57600: Data[4] = 9; break;																											// 
					}																																											//
					Data[5] = flash_read(Address_Config_Bank1+0x20);																			//
					Data[6] = flash_read(Address_Config_Bank1+0x30);																			//
					result = calculate_Crc16(Data, 7);																										//
					Data[7] = result & 0xFF;																															//
					Data[8] = result >> 8;																																//
					HAL_UART_Transmit(&huart2, Data, 9, 0xFFFF);																					// ���������� ����� � �������
							ClearDataBufer();																																	//	 ������� ����� � �����
				}																																												//			
			}																																													//	
//  *********************************************************************************************
//  ********************************************************************************************* ���� ��������� � ������ ���������� ������������ 			
			if (Flag_Receive_valid == 1 && Flag_Receive_conf == 1){																		// 			
				if (Data[2] != 0x00 || Data[3] != 0x05){																								// ��������� ��� ����� �������� address 0005
					AnswerWithError(0x02);
				}																																												//
//  ********************************************************************************************//				
				if ((Flag_Receive_conf == 1) && (Data[4] != 0x00 || Data[5] != 0x02)){									// ��������� ��� ����� �������� data 0002
					AnswerWithError(0x04);
				}																																												//
//  ********************************************************************************************//				
				if ((Flag_Receive_conf == 1) && Data[6] != 0x04){																				// ��������� ��� ����� �������� Byte 04
					AnswerWithError(0x04);
				}																																												//				
//  ********************************************************************************************//				
				if ((Flag_Receive_conf == 1) && (Data[7] == 0x00 || Data[7] >= 247)){										// ��������� ��� ID �������� ���������� ��������
					AnswerWithError(0x03);
				}																																												//
//  ********************************************************************************************//				
				if ((Flag_Receive_conf == 1) && (Data[8] == 0x00 || Data[8] > 9)){											// ��������� ��� BaudRate �������� ���������� ��������
					AnswerWithError(0x03);
				}																																												//				
//  ********************************************************************************************//				
				if ((Flag_Receive_conf == 1) && (Data[9] > 2)){																					// ��������� ��� Parity �������� ���������� ��������
					AnswerWithError(0x03);
				}																																												//	
//  ********************************************************************************************//				
				if ((Flag_Receive_conf == 1) && (Data[10] != 1) && (Data[10] != 2)){											// ��������� ��� StpBit �������� ���������� ��������
					AnswerWithError(0x03);
				}																																												//
//  ********************************************************************************************//				
				if (Flag_Receive_conf == 1){																														// ������� ������ �� ������
					Data_Config[0] = Data[7];																															// Data_Config[0] 
					switch(Data[8]){																																			// 
						case 1: Data_Config[1] = 600; break;																								// Data_Config[1]
						case 2: Data_Config[1] = 1200; break; 																							// 
						case 3: Data_Config[1] = 2400; break;    																						// 
						case 4: Data_Config[1] = 4800; break;   																						//  
						case 5: Data_Config[1] = 9600; break;  																							// 
						case 6: Data_Config[1] = 14400; break;																							// 
						case 7: Data_Config[1] = 19200; break; 																							//    
						case 8: Data_Config[1] = 38400; break;   																						//  
						case 9: Data_Config[1] = 57600; break;																							// 
					}																																											//
					switch(Data[9]){																																			// 
						case 0: Data_Config[2] = 0; break;																									// Data_Config[2]
						case 1: Data_Config[2] = 1; break; 																									// 
						case 2: Data_Config[2] = 2; break;    																							// 
					}																																											//
					switch(Data[10]){																																			// 
						case 1: Data_Config[3] = 1; break;																									// Data_Config[3]
						case 2: Data_Config[3] = 2; break; 																									// 
					}																																											//
					FLASH_EraseInitTypeDef EraseInitStruct;																								//
					uint32_t PAGEError = 0;																																//
					EraseInitStruct.TypeErase   =  TYPEERASE_PAGES;																				//
					EraseInitStruct.PageAddress = Address_Config_Bank1;																		//
					EraseInitStruct.NbPages     = 1;																											//												
					flash_ok = HAL_ERROR;																																	//
					flash_ok = HAL_FLASH_Unlock();																												// ������ ������ ��������
					flash_ok = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);														// ������� ������
					flash_ok = HAL_FLASH_Lock();																													// ��������� ������					
					if (flash_read(Address_Config_Bank1) != 0){																						// ��������� ��� �������
						AnswerWithError(0x04);																															// ���� ��� - �� ��������� ������ � �������-�����������
					} 	 																																									//				
					flash_ok = HAL_FLASH_Unlock();																												// ������ ������ ��������				
					flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank1, Data_Config[0]);				//## �����
					flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank1+0x10, Data_Config[1]);	//##
					flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank1+0x20, Data_Config[2]);	//## ���� 1
					flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank1+0x30, Data_Config[3]);	//##
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank2, OWN_ADDRESS_MODBUS);			//##	
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank2+0x10, BaudRate_MODBUS);		//## ���� 2
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank2+0x20, Parity_MODBUS);			//##
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank2+0x30, StpBit_MODBUS);			//##					
					
					flash_ok = HAL_FLASH_Lock();																													// ��������� ������				
					buffer_TX[1] = 0x10;																																	// ��������� ������ � �������
					buffer_TX[2] = 0x00;																																	//
					buffer_TX[3] = 0x05;																																	//
					buffer_TX[4] = 0x00;																																	//
					buffer_TX[5] = 0x02;																																	//
					result = calculate_Crc16(buffer_TX, 6);																								// 
					uint8_t q = result & 0xFF;																														//
					uint8_t w = result >> 8;																															//
					buffer_TX[6] = q;																																			//
					buffer_TX[7] = w;																																			//				
					HAL_UART_Transmit(&huart2, buffer_TX, 8, 0xFFFF);																			// ���������� ����� � �������
					NVIC_SystemReset();																																		// ���������� ����������		
				}																																												//	
			}																																													//
//  ********************************************************************************************//
//  ********************************************************************************************//		
		if (Tcounter1 >= TIME_ASK_WEATHER) {																												// ���� ���������� �������� ������
			Tcounter1 = 0;																																						// 
			ReadWeatherData();																																				//
		}		    																																										//
//  ********************************************************************************************//	
  }
}

void SystemClock_Config(void)
{	
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	__HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_PVD_EXTI_ENABLE_IT();	
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
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == 0){
		huart2.Instance = USART2;
		huart2.Init.BaudRate = flash_read(Address_Config_Bank2+0x10);	
			if (flash_read(Address_Config_Bank2+0x20) == 0){huart2.Init.Parity = UART_PARITY_NONE; huart2.Init.WordLength = UART_WORDLENGTH_8B;}
			else if (flash_read(Address_Config_Bank2+0x20) == 1){huart2.Init.Parity = UART_PARITY_ODD; huart2.Init.WordLength = UART_WORDLENGTH_9B;}
			else if (flash_read(Address_Config_Bank2+0x20) == 2){huart2.Init.Parity = UART_PARITY_EVEN; huart2.Init.WordLength = UART_WORDLENGTH_9B;}
			
			if (flash_read(Address_Config_Bank2+0x30) == 1){huart2.Init.StopBits = UART_STOPBITS_1;}
			else if (flash_read(Address_Config_Bank2+0x30) == 2){huart2.Init.StopBits = UART_STOPBITS_2;}									
		huart2.Init.Mode = UART_MODE_TX_RX;
		huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
		huart2.Init.OverSampling = UART_OVERSAMPLING_16;
		huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
		huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
		if (HAL_UART_Init(&huart2) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}
	}else{
		huart2.Instance = USART2;
		huart2.Init.BaudRate = flash_read(Address_Config_Bank1+0x10);
			if (flash_read(Address_Config_Bank1+0x20) == 0){huart2.Init.Parity = UART_PARITY_NONE; huart2.Init.WordLength = UART_WORDLENGTH_8B;}
			else if (flash_read(Address_Config_Bank1+0x20) == 1){huart2.Init.Parity = UART_PARITY_ODD; huart2.Init.WordLength = UART_WORDLENGTH_9B;}
			else if (flash_read(Address_Config_Bank1+0x20) == 2){huart2.Init.Parity = UART_PARITY_EVEN; huart2.Init.WordLength = UART_WORDLENGTH_9B;}
			
			if (flash_read(Address_Config_Bank1+0x30) == 1){huart2.Init.StopBits = UART_STOPBITS_1;}
			else if (flash_read(Address_Config_Bank1+0x30) == 2){huart2.Init.StopBits = UART_STOPBITS_2;}									
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
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;																							//	���������������� ����� ����� ������ �����
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;																								//	������������������ ����� ����� ������ �����		
  GPIO_InitStruct.Pull = GPIO_NOPULL;	
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



}
//########################################################################################################
void WriteDefaultConf(void)																																						//## ������� ������ ������������ �� ���������
{																																																			//##
			FLASH_EraseInitTypeDef EraseInitStruct;																													//##
			uint32_t PAGEError = 0;																																					//##
			EraseInitStruct.TypeErase   =  TYPEERASE_PAGES;																									//##
			EraseInitStruct.PageAddress = 0x08003FA0;																												//##
			EraseInitStruct.NbPages     = 1;																																//##												
			flash_ok = HAL_ERROR;																																						//##
			flash_ok = HAL_FLASH_Unlock();																																	//## 
			flash_ok = HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);																			//## ������� ������
			flash_ok = HAL_FLASH_Lock();																																		//## 
			flash_ok = HAL_ERROR;																																						//##
			while(flash_ok != HAL_OK){																																			//##
				flash_ok = HAL_FLASH_Unlock();																																//##
			}																																																//##
			flash_ok = HAL_ERROR;																																						//##
			while(flash_ok != HAL_OK){																																			//##
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank1, OWN_ADDRESS_MODBUS);			//##			
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank1+0x10, BaudRate_MODBUS);		//##
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank1+0x20, Parity_MODBUS);			//## ���� 1
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank1+0x30, StpBit_MODBUS);			//##
//  **************************************************************************************************//##			
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank2, OWN_ADDRESS_MODBUS);			//##	
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank2+0x10, BaudRate_MODBUS);		//## ���� 2
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank2+0x20, Parity_MODBUS);			//##
				flash_ok = HAL_FLASH_Program(TYPEPROGRAM_WORD, Address_Config_Bank2+0x30, StpBit_MODBUS);			//##								
			}																																																//##
			while(flash_ok != HAL_OK){																																			//##
				flash_ok = HAL_FLASH_Lock();																																	//##
			}																																																//##	
}																																																			//##
//########################################################################################################
//  **************************************************************************************//
void ReadWeatherData(void)																																// ������� ������ ������ � �������
{																																													// � ���������� ������ buffer_TX[]
			if (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {							//
				buffer_TX[3] |=(1<<0);
			}else{
				buffer_TX[3] &=~(1<<0);
			}	
//  ************
			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == 0){																			// ������ ����� ������
				buffer_TX[4] |=(1<<1);																														//
			}	else {																																						//
				buffer_TX[4] &=~(1<<1);																														//
			}																																										//
//  ************																																					// 

//  ************
			uint16_t rezAtmPressureGPa_uint =  (uint16_t)pressure;															//
			uint16_t rezHumidity_uint =        (uint16_t)(humidity * 10);												//
			int16_t rezTemperature_int = (int16_t)(temperature * 10);														//
			uint16_t rezAtmPressure_uint = (uint16_t)pressure * 0.75;														//	
			if (pressure == 0){buffer_TX[3] |=(1<<0);} 																					// �������� �������. ���� �������� == 0, ������ ��� ������ ����������				
			buffer_TX[5]=rezTemperature_int >> 8;		//MSB (Most Significant Bit) �������				// ��������� �����
			buffer_TX[6]=rezTemperature_int & 0xFF;	//LSB (Least Significant Bit) �������				//
			buffer_TX[7]=rezHumidity_uint >> 8;																									//
			buffer_TX[8]=rezHumidity_uint & 0xFF;																								//
			buffer_TX[9]=rezAtmPressure_uint >> 8;																							//
			buffer_TX[10]=rezAtmPressure_uint & 0xFF;																						//
			buffer_TX[11]=rezAtmPressureGPa_uint >> 8;																					//
			buffer_TX[12]=rezAtmPressureGPa_uint & 0xFF;																				//
				result = calculate_Crc16(buffer_TX, 13);																					//
				buffer_TX[13] = result & 0xFF;																										//
				buffer_TX[14] = result >> 8;																											//	
}																																													//
//****************************************************************************************
void ClearDataBufer(void)
{
	Flag_Receive_Buff_is_Full = 0;																											// ������� ����� � �����
	Flag_Receive_valid = 0;																															//
	Flag_Receive_data = 0;																															//
	Flag_Receive_conf = 0;	
	for (uint8_t i = 0; i<=19; i++){Data[i] = 0;}
}
//****************************************************************************************
void AnswerWithError(uint8_t err)
{
	Data_ERROR[1] = Data[1]|0x80;																												// 
	Data_ERROR[2] = err;																																//
	result = calculate_Crc16(Data_ERROR, 3);																						//
	Data_ERROR[3] = result & 0xFF;																											//
	Data_ERROR[4] = result >> 8;																												//
	HAL_UART_Transmit(&huart2, Data_ERROR, 5, 0xFFFF);																	//
	ClearDataBufer();																																		//	
}
//****************************************************************************************
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
