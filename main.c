#include "stm32f0xx.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_conf.h"
#include "registers.h"
#include "math.h"

#define RX_BUFFER_SIZE 10
#define I2C_ADDRESS 0b10000
#define CRC8_TABLE_SIZE 256

void Startup_Sequence(void);
void Error_Handler(void);
void SystemClock_Config(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void USART1_IRQHandler(void);
void UART_Transmit(uint8_t * data, uint8_t size);
void i2c1_send_data(uint8_t addr, uint8_t* data, uint8_t length);
void i2c1_receive_data(uint8_t addr, uint8_t* data, uint8_t length);
void crc8_populate_msb(uint8_t table[CRC8_TABLE_SIZE], uint8_t polynomial);
uint8_t crc8(const uint8_t table[CRC8_TABLE_SIZE], uint8_t *pdata, size_t nbytes, uint8_t crc);
float calculateTemperature(float r);
uint8_t * calculateCRCByte(uint8_t deviceRegister, uint8_t dataByte, uint8_t table[CRC8_TABLE_SIZE]);

UART_HandleTypeDef UartHandle;
I2C_HandleTypeDef I2cHandle;
__IO ITStatus UartTXReady = RESET;
__IO ITStatus UartRXReady = RESET;
uint8_t buff[RX_BUFFER_SIZE];
int8_t rx_buff_pos = -1;
uint8_t USART1_DataAvailableBuff(void);
uint8_t USART1_ReadBuff(void);
uint8_t receivedData[2];
uint8_t temp = SYS_CTRL1;
uint8_t temp1[2] = {BAT_HI};
uint8_t tempStore[4];
uint16_t tempADC;
float v;
float r;
uint8_t temperatureReading;
uint8_t crcTable[CRC8_TABLE_SIZE];
uint8_t poly = 0x07;
uint8_t * toSend;


/**
 * This function is called when transmitting
 * @param UartHandle Pointer to UartHandle
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {
    UartTXReady = SET;
}

/**
 * This function is called when receiving
 * @param UartHandle Pointer to UartHandle
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {
    UartRXReady = SET;
}

/**
 * Associates the interrupt handler with the UartHandle
 */
void USART1_IRQHandler(void) {
    if ((USART1->ISR & USART_ISR_RXNE) != RESET) {
        if ((1+rx_buff_pos) < RX_BUFFER_SIZE) {
            buff[1+rx_buff_pos++] = USART1->RDR; // reading from DR resets RXNE flag, no need to manually clear it!
        }
    }
    HAL_UART_IRQHandler(&UartHandle);
}

/**
 * This function handles I2C event interrupt requests.
 */
void I2C1_IRQHandler(void) {
    if (I2cHandle.Instance->ISR & (I2C_FLAG_BERR | I2C_FLAG_ARLO | I2C_FLAG_OVR)) {
      HAL_I2C_ER_IRQHandler(&I2cHandle);
    } else {
      HAL_I2C_EV_IRQHandler(&I2cHandle);
    }
}


int main(void) {
        // Calling required HAL functions
	HAL_Init();
	SystemInit();
	SystemClock_Config();

        // Enabling peripheral clocks
	GPIO_InitTypeDef  GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_I2C1_CLK_ENABLE();

  	/* System interrupt init*/
  	/* SVC_IRQn interrupt configuration */
  	HAL_NVIC_SetPriority(SVC_IRQn, 0, 0);
  	/* PendSV_IRQn interrupt configuration */
  	HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  	/* SysTick_IRQn interrupt configuration */
  	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);


	// Setup LED Pin
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Setup I2C pins
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	// Setup UART Tx Pin
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Setup UART Rx pin
    	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
    	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Setup UART Instance
    	UartHandle.Instance = USART1;
    	UartHandle.Init.BaudRate = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    	UartHandle.Init.StopBits = UART_STOPBITS_1;
    	UartHandle.Init.Parity = UART_PARITY_NONE;
    	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  	UartHandle.Init.Mode = UART_MODE_TX_RX;

	// Error handling
	if(HAL_UART_Init(&UartHandle) != HAL_OK) {
        	Error_Handler();
   	}

	Startup_Sequence();

        // Setup UART interrupts
	HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
    	HAL_NVIC_EnableIRQ(USART1_IRQn);
	__HAL_UART_ENABLE_IT(&UartHandle, UART_IT_RXNE);

	HAL_I2C_MspInit(&I2cHandle);

	// Setup I2C
	I2cHandle.Instance = I2C1;
    	I2cHandle.Init.Timing = 0x00B0DBFF;
	I2cHandle.Init.OwnAddress1 = 0;
    	I2cHandle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	I2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2cHandle.Init.OwnAddress2 = 0;
	I2cHandle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	I2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2cHandle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

        // Setup I2C interrupts
    	HAL_NVIC_SetPriority(I2C1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(I2C1_IRQn);

    	if (HAL_I2C_Init(&I2cHandle) != HAL_OK) {
        	Error_Handler();
    	}

    	//Configure Analogue filter
	if (HAL_I2CEx_ConfigAnalogFilter(&I2cHandle, I2C_ANALOGFILTER_DISABLE) != HAL_OK) {
		Error_Handler();
	}

    	//Configure Digital filter
	if (HAL_I2CEx_ConfigDigitalFilter(&I2cHandle, 0) != HAL_OK) {
		Error_Handler();
	}

        // Generate CRC8 table for calculating CRC byte
	crc8_populate_msb(crcTable, poly);

	 /**
     	* Start of BQ76920 Initialisation. Enables ADC and CC. Configures registers and
     	* sends welcome log to UART.
     	*/
    	i2c1_send_data(I2C_ADDRESS, &temp, 1); // 1
    	i2c1_receive_data(I2C_ADDRESS, receivedData, 2); // 2
    	receivedData[0] |= 0b11 << 3;
    	toSend = calculateCRCByte(SYS_CTRL1, receivedData[0], crcTable);
    	i2c1_send_data(I2C_ADDRESS, toSend, 3);
    	temp1[0] = CC_CFG;
    	i2c1_send_data(I2C_ADDRESS, &temp1[0], 1);
    	tempStore[0] = 0x19;
    	toSend = calculateCRCByte(CC_CFG, tempStore[0], crcTable);
    	i2c1_send_data(I2C_ADDRESS, toSend, 3);
    	while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
   	temp1[0] = SYS_CTRL2;
    	i2c1_send_data(I2C_ADDRESS, &temp1[0], 1); // 1
    	i2c1_receive_data(I2C_ADDRESS, tempStore, 2); // 2
    	while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
    	tempStore[0] |= 1 << 6;
    	toSend = calculateCRCByte(SYS_CTRL2, tempStore[0], crcTable);
    	i2c1_send_data(I2C_ADDRESS, toSend, 3); // 3
    	while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}

        // Send welcome log
    	char shinyocto[16] = {'S', 'h', 'i','n', 'y', '-', 'O', 'c', 't', 'o', ' ', 'v', '1', '.', '0','\n'};
    	if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)shinyocto, 16) != HAL_OK) {
        	Error_Handler();
    	}
    	HAL_Delay(100);
    	char ready[6] = {'R', 'e', 'a', 'd', 'y', '\n'};
    	if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)ready, 6) != HAL_OK) {
        	Error_Handler();
    	}

        // Define static messages to be sent
	char ok[3] = {'O', 'K', '\n'};
	char version[5] = {'V', '1', '.', '0', '\n'};
	char cc[3] = {'C', 'C', '\n'};
	char bad[4] = {'B', 'A', 'D', '\n'};

	HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)ok, 3);
	uint8_t address = 0b10000;
	uint8_t data = 0x12;
	HAL_I2C_Master_Transmit_IT(&I2cHandle, address, &data, 2);

	while(1) {
                // If 2 or more characters in UART buffer
		if (USART1_DataAvailableBuff() >= 2) {
			switch(buff[0]) {
				/**
				* Command 0 - Returns OK
				*/
				case 0x30 :
					UART_Transmit((uint8_t*)ok, 3);
					break;
				/**
				* Command 1 - Returns voltage + CRC bytes of Cell 1
				*/
				case 0x31 :
					temp1[0] = VC1_HI;
                			i2c1_send_data(I2C_ADDRESS, &temp1[0], 1); // 4
					while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
                			i2c1_receive_data(I2C_ADDRESS, tempStore, 4); // 5
                			while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
					if(HAL_UART_Transmit_IT(&UartHandle, tempStore, 4) != HAL_OK) {
                    				Error_Handler();
                			}
                			while (UartTXReady != SET){}
                			UartTXReady = RESET;
					break;
				/**
				* Command 2 - Returns voltage + CRC bytes of Cell 2
				*/
				case 0x32 :
					temp1[0] = VC2_HI;
                			i2c1_send_data(I2C_ADDRESS, &temp1[0], 1); // 4
					while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
                			i2c1_receive_data(I2C_ADDRESS, tempStore, 4); // 5
                			while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
					if(HAL_UART_Transmit_IT(&UartHandle, tempStore, 4) != HAL_OK) {
                    				Error_Handler();
                			}
                			while (UartTXReady != SET){}
                			UartTXReady = RESET;
					break;
				/**
				* Command 3 - Returns voltage + CRC bytes of Cell 3
				*/
				case 0x33 :
					temp1[0] = VC3_HI;
                			i2c1_send_data(I2C_ADDRESS, &temp1[0], 1); // 4
					while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
                			i2c1_receive_data(I2C_ADDRESS, tempStore, 4); // 5
                			while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
					if(HAL_UART_Transmit_IT(&UartHandle, tempStore, 4) != HAL_OK) {
                    				Error_Handler();
                			}
                			while (UartTXReady != SET){}
                			UartTXReady = RESET;
					break;
				/**
				* Command 4 - Returns voltage + CRC bytes of Cell 5
				*/
				case 0x34 :
					temp1[0] = VC5_HI;
                			i2c1_send_data(I2C_ADDRESS, &temp1[0], 1); // 4
					while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
                			i2c1_receive_data(I2C_ADDRESS, tempStore, 4); // 5
                			while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
					if(HAL_UART_Transmit_IT(&UartHandle, tempStore, 4) != HAL_OK) {
                    				Error_Handler();
                			}
                			while (UartTXReady != SET){}
                			UartTXReady = RESET;
					break;
				/**
				* Command 5 - Returns total voltage of cell + CRC bytes
				*/
				case 0x35 :
					temp1[0] = BAT_HI;
                			i2c1_send_data(I2C_ADDRESS, &temp1[0], 1); // 4
					while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
                			i2c1_receive_data(I2C_ADDRESS, tempStore, 4); // 5
                			while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
					if(HAL_UART_Transmit_IT(&UartHandle, tempStore, 4) != HAL_OK) {
                    				Error_Handler();
                			}
                			while (UartTXReady != SET){}
                			UartTXReady = RESET;
					break;
				/**
				* Command 6 - Returns external thermistor temperature
				*/
				case 0x36 :
					temp1[0] = TS1_HI;
                			i2c1_send_data(I2C_ADDRESS, &temp1[0], 1); // 4
					while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
                			i2c1_receive_data(I2C_ADDRESS, tempStore, 4); // 5
                			while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
                			tempADC = (tempStore[0]<<8)|tempStore[2];
                			v = tempADC * 382.0e-6;
                			r = (10000.0*v)/(3.3-v);
                			temperatureReading = calculateTemperature(r) - 273.0;
                			if(HAL_UART_Transmit_IT(&UartHandle, &temperatureReading, 1) != HAL_OK) {
                    				Error_Handler();
                			}
                			while (UartTXReady != SET){}
                			UartTXReady = RESET;
					break;
				/**
				* Command 7 - Returns CC register + CRC bytes
				*/
				case 0x37 :
					UART_Transmit((uint8_t *)cc, 3);
					break;
				/**
				* Command v - Returns version of firmware
				*/
				case 0x76 :
					UART_Transmit((uint8_t *)version, 5);
					break;
				/**
				* Default case - returns BAD when incorrect character given#
				*/
				default:
                			if(HAL_UART_Transmit_IT(&UartHandle, (uint8_t *)bad,  4 ) != HAL_OK) {
                    				Error_Handler();
                			}
			}
			// WARNING: Don't forget to reset rx_buff_pos :D
            rx_buff_pos = -1;
        }
	HAL_Delay(1); // Required, IDK why don't change it :/
	}
}

/**
 * Blinks onboard LED at 10 times with delay decreasing
 */
void Startup_Sequence(void) {
 	int i;
	for (i=100; i>0;i-=10) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
		HAL_Delay(i);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

/**
 * Blinks onboard LED at constant rate when program errors
 */
void Error_Handler(void) {
	while(1) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
		HAL_Delay(1000);
	}
}

/**
 * Returns number of bytes in UART buffer
 */
uint8_t USART1_DataAvailableBuff(void) {
    return rx_buff_pos+1;
}

/**
 * Sends data on UART1
 * @param data  Pointer to data to be sent
 * @param size  Size of data in bytes
 */
void UART_Transmit(uint8_t * data, uint8_t size) {
	if(HAL_UART_Transmit_IT(&UartHandle, data, size) != HAL_OK) {
		Error_Handler();
	}
	while (UartTXReady != SET){}
    	UartTXReady = RESET;
}

/**
 * Waits until slave is ready to receive data and then sends data to slave.
 * @param addr   8 bit address of slave device
 * @param data   Data to be sent to slave device
 * @param length Length of data being sent in bytes
 */
void i2c1_send_data(uint8_t addr, uint8_t* data, uint8_t length) {
    while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
    while (HAL_I2C_Master_Transmit_IT(&I2cHandle, addr, data, length) != HAL_OK) {
        if (HAL_I2C_GetError(&I2cHandle) == HAL_I2C_ERROR_BERR) {
            Error_Handler();
        }
    while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY){}
    }
}

/**
 * Calculates CRC byte over I2C address + R/W bit, and then sends data + CRC byte to device
 * @param crcRegister Address of the register to be written to
 * @param crcDataByte The data byte to be written to the register
 * @param table       Pregenerated CRC table. Generated with crc8_populate_msb function
 */
uint8_t * calculateCRCByte(uint8_t deviceRegister, uint8_t dataByte, uint8_t table[CRC8_TABLE_SIZE]) {
    static uint8_t crcData[3];
    uint8_t crcBuf[3];
    crcBuf[0] = I2C_ADDRESS;
    crcBuf[1] = deviceRegister;
    crcBuf[2] = dataByte;
    crcData[0] = deviceRegister;
    crcData[1] = dataByte;
    crcData[2] = crc8(table, crcBuf, 3, 0);
    return crcData;
}


/**
 * Waits until slave is ready to transmit data and then receives data from slave
 * @param addr   8 bit address of slave device
 * @param data   Pointer to where data will be stored
 * @param length Length of data being received in bytes
 */
void i2c1_receive_data(uint8_t addr, uint8_t* data, uint8_t length) {
    while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY) {}
    while (HAL_I2C_Master_Receive_IT(&I2cHandle, addr, data, length) != HAL_OK) {
    	if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF) {
            Error_Handler();
        }
    }
}

/**
 * Fills CRC table for given polynomial in reverse bit order.
 *
 * @param table   table to be filled.
 * @param polynomial  polynomial for which table is to be filled. 0x07 for CRC8.
 */
void crc8_populate_msb(uint8_t table[CRC8_TABLE_SIZE], uint8_t polynomial) {
    int i, j;
    const uint8_t msbit = 0x80;
    uint8_t t = msbit;

    table[0] = 0;

    for (i = 1; i < CRC8_TABLE_SIZE; i *= 2) {
        t = (t << 1) ^ (t & msbit ? polynomial : 0);
        for (j = 0; j < i; j++)
            table[i+j] = table[j] ^ t;
    }
}


/**
 * Calculates the CRC checksum for given data.
 *
 * @param table Filled lookup table. Generated with crc8_populate_msb()
 * @param pdata Pointer to the data which the crc checksum will be calculated
 * @param nbytes Size of the data in bytes
 * @param crc Previous CRC checksum. Set to zero if not carrying on from previous checksum.
 * @return crc Returns the CRC checksum for the data pdata.
 */
uint8_t crc8(const uint8_t table[CRC8_TABLE_SIZE], uint8_t *pdata, size_t nbytes, uint8_t crc) {
    /* loop over the buffer data */
    while (nbytes-- > 0)
        crc = table[(crc ^ *pdata++) & 0xff];

    return crc;
}


/**
 * Calculates the temperature of a 10K thermistor (Semitec 103AT-2) given its resistance.
 * Uses the Steinhart-Hart equation. Explanation of the equation and how the coefficients
 * are calculated is shown in docs/thermistor.pdf
 *
 * @param r Resistance of thermistor in ohms.
 * @return Temperature of thermistor in Kelvins.
 */
float calculateTemperature(float r) {
    //Steinhart-Hart equation coefficients.
    float a = 0.000888046597091356;
    float b = 0.0002515533487662026;
    float c = 1.929642106865123e-7;

    return (1.0)/(a + (b*log(r)) + (c * pow(log(r),3)));
}

/**
 * Configures system clocks and connects to peripherals.
 */
void SystemClock_Config(void) {
 	RCC_OscInitTypeDef RCC_OscInitStruct;
   	RCC_ClkInitTypeDef RCC_ClkInitStruct;
   	RCC_PeriphCLKInitTypeDef PeriphClkInit;

    	// Initializes the CPU, AHB and APB busses clocks
   	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
   	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
   	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
   	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
   	}

    	// Initializes the CPU, AHB and APB busses clocks
   	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                               |RCC_CLOCKTYPE_PCLK1;
   	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
   	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

   	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
   		Error_Handler();
	}

   	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_I2C1;
   	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_SYSCLK;
   	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
   	}

     	// Configure the Systick interrupt time
   	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

     	// Configure the Systick
   	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

   	// SysTick_IRQn interrupt configuration
   	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
