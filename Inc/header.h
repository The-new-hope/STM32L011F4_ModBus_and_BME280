#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include <inttypes.h>
#include "main.h"
#include "stm32l011xx.h"
#include "stm32l0xx_hal.h"
#include "crc16.h"
#include "BMP280/bmp280.h"	// Description of registers and commands BMP280
//#include "NRF/NRF24L01.h"		// Description of registers and commands NRF24L01+

// Setting of NRF24L01+
//										M			E			T			E			O			//ASCII code
//#define OWN_ADDRESS	{0x4D, 0x45, 0x54, 0x45, 0x4F}; // Own address
//#define REMOTE_ADDRESS	{0x4D, 0x45, 0x54, 0x45, 0x4F}; // Remote address
//#define OWN_ADDRESS	"METEO"; // Own address
//#define REMOTE_ADDRESS	"METEO"; // Remote address
//#define CHAN		10 // channel number
#define RF_DATA_SIZE		15 // size of sendind data, bytes
//#define RF_SPEED	RF_SETUP_250KBPS // size of sendind data: RF_SETUP_2MBPS, RF_SETUP_1MBPS or RF_SETUP_250KBPS

#define TIME_SENDING		50 // Frequnce of sending data = 1/10 sek // 50 = 5 sek
#define TIME_FLASH_LED		10 // Frequnce of flash led = 1/10 sek // 10 = 1 sek
#define OWN_ADDRESS_MODBUS	0x; // Own address


//See also settings have in file NRF24L01.c

//// SPI setting
//#define SPIx_RCC		RCC_APB1Periph_SPI1
//#define SPIx			SPI1
//#define SPI_GPIO_RCC	RCC_APB2Periph_GPIOA
//#define SPI_GPIO		GPIOA
//#define SPI_PIN_MOSI	GPIO_PIN_7
//#define SPI_PIN_MISO	GPIO_PIN_6
//#define SPI_PIN_SCK		GPIO_PIN_5
//#define SPI_PIN_CSN		GPIO_PIN_4
//#define SPI_PIN_IRQ		GPIO_PIN_1
//#define SPI_PIN_CE		GPIO_PIN_3
//// ���������� ������ CSN
//#define CSN1 HAL_GPIO_WritePin(SPI_GPIO, SPI_PIN_CSN, GPIO_PIN_SET)
//#define CSN0 HAL_GPIO_WritePin(SPI_GPIO, SPI_PIN_CSN, GPIO_PIN_RESET)
//// ���������� ������ CE
//#define CE1   HAL_GPIO_WritePin(SPI_GPIO, SPI_PIN_CE, GPIO_PIN_SET)
//#define CE0   HAL_GPIO_WritePin(SPI_GPIO, SPI_PIN_CE, GPIO_PIN_RESET)


// I2C setting
#define I2C_GPIO_RCC	RCC_APB2Periph_GPIOA
#define I2C_GPIO		GPIOA
#define I2C_SDA		GPIO_PIN_10
#define I2C_SCL		GPIO_PIN_9

//See also settings have in file stm32l0xx_hal.c






