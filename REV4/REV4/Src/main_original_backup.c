/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include <stdlib.h>
//#include "../Inc/parser.h"  
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
PCD_HandleTypeDef hpcd_USB_FS;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
   
   
/* Structs for data from sensors */
struct baro_data{
	uint8_t p; //pressure bits
	uint8_t pt; //pressure and temperature bits
	uint8_t t; //temperature bits
};

//struct for holding the accelerometer and gyroscope data from the MPU
struct mpu_data{
	uint16_t accelx; //acceleromter x-axis
	uint16_t accely; //acceleromter y-axis
	uint16_t accelz; //acceleromter z-axis
	uint16_t gyrox; //gyroscope x-axis
	uint16_t gyroy; //gyroscope y-axis
	uint16_t gyroz; //gyroscope z-axis
};

//struct for holding the gps data latitude, longitude, altitude
struct gps_data{
	float lat; //latitude
	float lon; //longitude
	float alt; //GPS altitude
};

//struct holding a float after raw data is processed
struct data_float{
	float data;
};

//A struct for holding a set of data from all the sensors.
struct packet{
	struct data_float alt;
	struct data_float vel;
	struct data_float lat;
	struct data_float lon;
	uint16_t time;

};
//struct for holding the lighlty processed returns from sensors
struct raw{
	uint16_t id;
	struct baro_data baro;
	struct mpu_data mpu;
	struct gps_data gps;
	uint16_t time;
	char delim;
};

/*************************************************
* Title: spi_send8() and spi_read8
* Description: Sends/reads data on SPI bus
*************************************************/

void spi_send8(SPI_TypeDef* spi, uint8_t data)
{
	HAL_SPI_Transmit(spi, &data, 1, 0xFFFFFFFF);
}

uint8_t spi_read8(SPI_TypeDef* spi)
{
	uint8_t data;
	HAL_SPI_Receive(spi, &data, 1, 0xFFFFFFFF);
	return data;
}

/*************************************************
* Title: parser
* Description: Parses incoming GPS data for 
* latitude and longitude.
*************************************************/
struct gps_data parser(char data[])
{
    const char s1[2] = "\r";
    const char s2[2] = ",";
    char *token1, *token2;
    char *saveptr1, *saveptr2;
    token1 = strtok_r(data, s1, &saveptr1);
    char des[80];
//    struct GPSObj r;
    struct gps_data r;
    while (token1 != NULL)
    {
        strcpy(des, token1);
        token2 = strtok_r(des, s2, &saveptr2);
        const char *t[15];
        int i = 0;
        if (strncmp(token2, "$GNRMC", 7) == 0)
        {
            while (token2)
            {
                t[i] = token2;
                token2 = strtok_r(NULL, s2, &saveptr2);
                i++;
            }
            i = 0;
            if (strncmp(t[2], "A", 2) == 0)
            {
                r.lat = atof(t[3]);
                r.lon = atof(t[5]);
            }
            else {
                r.lat = 0.0;
                r.lon = 0.0;
            }
        }
        token1 = strtok_r(NULL, s1, &saveptr1);
    }
    r.alt = 0;
    return r;
}

/**************************************************
* Title USART_IRQHandler
* Description: Interrupt handler for UART4. Recives
* 1 Byte from GPS upon interupt.
***************************************************/
void USART4_IRQHandler(void){//called when interrupt recieved
        /* RXNE handler */
        if(USART_GetITStatus(USART4, USART_IT_RXNE) != RESET) //USART4 = GPS
        {
                uart_msg[uart_it] = USART_RecieveData(USART4); //get character
                uart_it++; //increment iterator 
                if((int)uart_msg[uart_it-1] == 13){ //if carriage return was the recieved charater aka end of GPS data 
                        uart_it = 0; //reset iterator
                        process_gps(); //process the string
                }

        }
}

/******************************************
*Title: Read_baro
*Description: Reads data from the barometer
*and returns it as a struct baro_data.
******************************************/
struct baro_data read_baro(uint8_t* mask){
	struct baro_data temp;
	//read ADC
	spi_send8(SPI3, 0x00);
	//read and save return
	temp.p = spi_read8(SPI3);
	temp.pt = spi_read8(SPI3);
	temp.t = spi_read8(SPI3);
	//if any data is bad clear the whole struct 
	if(temp.p == 0x00){ 
		temp.pt = 0x00;
		temp.t = 0x00;
		*mask = *mask & 0xFE; //set this control bit low
	}
	else if(temp.pt == 0x00){
		temp.p = 0x00;
		temp.t = 0x00;
		*mask = *mask & 0xFE; //set this control bit low
	}
	else if(temp.t == 0x00){
		temp.p = 0x00;
		temp.pt = 0x00;
		*mask = *mask & 0xFE; //set this control bit low
	}
	else{ //data is good
		*mask = *mask | 0x01; //set this control bit high
	}
	return temp;
}

/****************************************
*Title: get_mpu_byte
*Description: gets next data byte from MPU
****************************************/

uint8_t get_mpu_byte(uint8_t reg){
    uint8_t data = 0;
//    uint8_t addr = 0x69;

    I2C2->CR2 &= ~(I2C_CR2_RD_WRN);
    I2C2->CR2 &= ~(0x69 << 16); //address of the MPU
    I2C2->CR2 |= I2C_CR2_START | (1 << 16); 
    while(I2C2->CR2 & I2C_CR2_START);

    I2C2->TXDR = reg; // write address of register
    while (!(I2C2->ISR & I2C_ISR_TXE));

    I2C2->CR2 |= I2C_CR2_RD_WRN;
    I2C2->CR2 |= I2C_CR2_START | (1 << 16);
    while(I2C2->CR2 & I2C_CR2_START);
    while (!(I2C2->ISR & I2C_ISR_RXNE));
    data = I2C2->RXDR;
    I2C2->CR2 |= I2C_CR2_STOP;
    while(I2C2->CR2 & I2C_CR2_STOP);
    return data;
}

/****************************************
*Title: get_mpu_data
*Description: gets data from MPU byte
****************************************/

void get_mpu_data(struct mpudata* out){
	uint8_t temph, templ;
	uint8_t* pos;
	//get accel x high
	temph = get_mpu_byte(0x3B);
	//get accel x low
	templ = get_mpu_byte(0x3C);
	//put in to struct
	pos = &(out->accelx)
	*pos = tmph;
	pos++;
	*pos = tmpl;
	pos++;
	//get accel y high	
	temph = get_mpu_byte(0x3D);
	//get accel y low
	templ = get_mpu_byte(0x3E);
	//put in to struct
	*pos = temph;
	pos++;
	*pos = tmpl;
	pos++;
	//get accel z high
	temph = get_mpu_byte(0x3F);
	//get accel z low
	templ = get_mpu_byte(0x40);
	//put in to struct
	*pos = temph;
	pos++;
	*pos = templ;
	pos++;
	//get gyro x high
	temph = get_mpu_byte(0x3F);
	//get gyro x low
	templ = get_mpu_byte(0x3F);
	//put in to struct
	*pos = temph;
	pos++;
	*pos = templ;
	pos++;
	//get gyro y high
	temph = get_mpu_byte(0x3F);
	//get gyro y low
	templ = get_mpu_byte(0x3F);
	//put in to struct
	*pos = temph;
	pos++;
	*pos = templ;
	pos++;
	//get gyro z high
	temph = get_mpu_byte(0x3F);
	//get gyro z low
	templ = get_mpu_byte(0x3F);
	//put in to struct
	*pos = temph;
	pos++;
	*pos = templ;
}

/****************************************
*Title: write_to_eeprom
*Description: writes struct contents to
*non-volatile EEPROM memory
****************************************/
void write_to_eeprom(struct raw* data, int* addr){
	uint8_t sreg; //status register return
	uint8_t* tmp; //used for typcasting;
	int i=0;
	//set up for write
	spi_send8(SPI2, 0x06);//write enable
	//set cs low 
	GPIO_PIN_15 = 0;
	spi_send8(SPI2, 0x02);//page program
	//write address
	tmp = (uint8_t*)(addr); //typcast
	spi_send8(SPI2, tmp[2]);//write address high byte
	spi_send8(SPI2, tmp[1]);//write address mid byte
	spi_send8(SPI2, tmp[0]);//write address low byte
	//write data
	for(i; i<8; i++){//write 8 sets of raw data to eeprom
		tmp = (uint8_t*)(&((data[i]).id)); //typcast
		spi_send8(SPI2, (uint8_t)(tmp[0]));
		spi_send8(SPI2, (uint8_t)(tmp[1]));//write id
		//spi_send8(SPI3, (uint8_t)(data[i]).baro.p);
		//spi_send8(SPI3, (uint8_t)(data[i]).baro.pt);
		//spi_send8(SPI3, (uint8_t)(data[i]).baro.t);//write baro return
		spi_send8(SPI2, (uint8_t)(tmp[2]));
		spi_send8(SPI2, (uint8_t)(tmp[3]));
		spi_send8(SPI2, (uint8_t)(tmp[4])); //write barometer data
		//tmp = (uint8_t*)(&((data[i]).mpu.accelx)); //typcast
		spi_send8(SPI2, (uint8_t)(tmp[5]));
		spi_send8(SPI2, (uint8_t)(tmp[6]));
		//tmp = (uint8_t*)(&((data[i]).mpu.accely)); //typcast
		spi_send8(SPI2, (uint8_t)(tmp[7]));
		spi_send8(SPI2, (uint8_t)(tmp[8]));
		//tmp = (uint8_t*)(&((data[i]).mpu.accelz)); //typcast
		spi_send8(SPI2, (uint8_t)(tmp[9]));
		spi_send8(SPI2, (uint8_t)(tmp[11])); //write MPU acceleromter data
		//tmp = (uint8_t*)(&((data[i]).mpu.gyrox)); //typcast
		spi_send8(SPI2, (uint8_t)(tmp[11]));
		spi_send8(SPI2, (uint8_t)(tmp[12]));
		//tmp = (uint8_t*)(&((data[i]).mpu.gyroy)); //typcast
		spi_send8(SPI2, (uint8_t)(tmp[13]));
		spi_send8(SPI2, (uint8_t)(tmp[14]));
		//tmp = (uint8_t*)(&((data[i]).mpu.gyroz)); //typcast
		spi_send8(SPI2, (uint8_t)(tmp[15]));
		spi_send8(SPI2, (uint8_t)(tmp[16])); //write MPU gyroscope data
		//tmp = (uint8_t*)(&((data[i]).gps.lat)); //typcast
		spi_send8(SPI2, (uint8_t)(tmp[17]));
		spi_send8(SPI2, (uint8_t)(tmp[18]));
		spi_send8(SPI2, (uint8_t)(tmp[19]));
		spi_send8(SPI2, (uint8_t)(tmp[20])); //wrte GPS latitude data
		//tmp = (uint8_t*)(&((data[i]).gps.lon)); //typcast
		spi_send8(SPI2, (uint8_t)(tmp[21]));
		spi_send8(SPI2, (uint8_t)(tmp[22]));
		spi_send8(SPI2, (uint8_t)(tmp[23]));
		spi_send8(SPI2, (uint8_t)(tmp[24])); //wrte GPS longitude data
		//tmp = (uint8_t*)(&((data[i]).gps.alt)); //typcast
		spi_send8(SPI2, (uint8_t)(tmp[25]));
		spi_send8(SPI2, (uint8_t)(tmp[26]));
		spi_send8(SPI2, (uint8_t)(tmp[27]));
		spi_send8(SPI2, (uint8_t)(tmp[28])); //wrte GPS altitude data
		//tmp = (uint8_t*)(&((data[i]).time)); //typcast
		spi_send8(SPI2, (uint8_t)(tmp[29])); 
		spi_send8(SPI2, (uint8_t)(tmp[30])); //write time stamp
		//spi_send8(SPI3, (uint8_t)(data[i]).delim); //write delim char (|)
		spi_send8(SPI2, (uint8_t)(tmp[31])); //write delim char (|)
	}
	(*addr)+=256; //size of a raw set (32) * 8 sets (also exactly 1 page of EEPROM)
	//set cs bit high 
	GPIO_PIN_15 = 1;
	do{
		spi_send8(SPI2, 0x05); //read EEPROM SR1
		sreg = spi_read8(SPI2); //get SR1 
		sreg = sreg & 0x02; // get only BUSY read only bit
	}while(sreg > 0); //loop unitl page write is complete
}

void get_data(){
	struct raw raw_sets[8]; //raw sets we are building
	struct packet cur_packet; //packet we are building
	int i=0;//iterator
	int addr; //holds EEPROM address
	uint8_t mask = 0x00; //clear the mask used for checking completed sections of data
	uint8_t temp = 0x00; //mask check
	uint16_t id = 0; //ide for logging to eeprom
	addr = 0; //set starting address to 0
	i=0;
	while(i<8){
		while(mask < 0x0F){ //this value may change, it is assuming 4 pieces of data
			temp = mask;
			temp = temp | 0x01; //clear first 7 bits to test for barometer data retrieval completion
			if(temp > 0x00){//need to get baro data
				(raw_sets[i]).baro = read_baro(&mask); //get baro data
			}
		}
		(raw_sets[i]).id = id; //set id of raw data before writing to eeprom
		id++; //increase id value by one
		(raw_sets[i]).delim = '|';
		i++;
	}
	i=0;
}

/*****************************************
*Title: power_led and tx_led control functions
*Description: Turns on/off power LEDs and blinks
TX LED when TX is successful -> goes into TX loop
*****************************************/

void power_led_on(){
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
}

void power_led_off(){
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
}

void tx_led_blink(){ //put this in TX loop and send to EEPROM loop
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
     HAL_Delay(50);
     HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
}

/*****************************************
*Title: get_time
*Description: Returns time since system
start of runtime in milliseconds
*****************************************/

unsigned int get_time(){
	return HAL_GetTick(); //should return time in ticks since HAL_InitTick(), needs to be converted to milliseconds
}

/****************************************
*Title: Main
*Description: Main loop
****************************************/                       
int main(void)
{

  /* Inits here */
  HAL_Init(); //initializes HAL, also automatically calls HAL_InitTick() for the get_time() function
  SystemClock_Config();
  	MX_GPIO_Init();
	MX_USB_PCD_Init();
	MX_TIM1_Init();
	MX_I2C2_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();
	power_led_on(); //must happen after MX_GPIO_Init()
	
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	//start_time = get_time();
  	//while(get_time() - start_time < timeout && NOT all sensors have data in packet){
		/*If baro data is available && packet does not have data yet{
		retrieve data from baro
		put data into raw packet
		}*/

		/*If MPU data is available && packet does not have data yet{
		retrieve data from MPU
		put data into raw packet
		}*/

		/*If GPS data is available && packet does not have data yet{
		retrieve data from GPS
		put data into raw packet
		}*/
		}
	}*/
		
		/*If baro data is null{
		put dummy data in baro struct
		}*/
		
		/*If MPU data is null{
			put dummy data in MPU struct
		}*/
		
		/*If GPS data is null{
			put dummy data in GPS struct
		}*/
		
		//convert get_time() to milliseconds and append value onto packet
		//write raw packet to EEPROM
		//compile data into organized packet
		//send packet to TXRX
	}
  power_led_off();
}

//*************************************************************************
//*************************************************************************
//DO NOT TOUCH ANYTHING BELOW THIS LINE. THESE ARE INITS FOR THE SYSTEM
//CREATED BY TRUESTUDIO
//*************************************************************************
//*************************************************************************

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART5 init function */
static void MX_UART5_Init(void)
{

  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USB init function */
static void MX_USB_PCD_Init(void)
{

  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC1 PC2 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = 0x00000001U;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB11 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
  printf("Something fucked up in infinite loop: file %s on line %d\r\n", file, line);
}

#ifdef USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
    printf("One of the asserts fucked up: file %s on line %d\r\n", file, line);
}

#endif
