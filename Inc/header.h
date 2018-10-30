#include "stdio.h"
#include "string.h"
#include "stdint.h"
#include <inttypes.h>
#include "main.h"
#include "stm32l011xx.h"
#include "stm32l0xx_hal.h"
#include "crc16.h"
#include "BMP280/bmp280.h"	// Description of registers and commands BMP280


#define TIME_SENDING		50 // Частота чтения данных из датчика = 1/10 sek // 50 = 5 sek

#define TIME_ASK_RAIN		10 // Частота опроса датчика дождя = 1/10 sek // 10 = 1 sek

#define Address_Config_Bank1	0x08003FA0 //
#define Address_Config_Bank2	Address_Config_Bank1+0x04 //

#define OWN_ADDRESS_MODBUS	0xDC // Own address in HEX
#define BaudRate_MODBUS	9600 // Default baud rate: 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 56000, 57600
#define Parity_MODBUS	0 // Default parity: 0-none, 1-odd, 2-even
#define StpBit_MODBUS	1 // Default stop bit: 1-1 stop bit, 15-1.5 stop bit, 2-2 stop bit

// I2C setting
#define I2C_GPIO_RCC	RCC_APB2Periph_GPIOA
#define I2C_GPIO		GPIOA
#define I2C_SDA		GPIO_PIN_10
#define I2C_SCL		GPIO_PIN_9

//See also settings have in file stm32l0xx_hal.c




