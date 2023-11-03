/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <main.h>
#include <string.h>
#include <stdbool.h>
//-----переменные для UART
#define size_tx_buffer 10 //размер буфера tx
bool flag_send; // флаг  разрешения на отправку
uint8_t queue_message; //
uint8_t tx_buffer[size_tx_buffer]; // tx_buffer
uint8_t rx_buffer[1]; //
uint8_t error_message[] = "tx buffer is crowded\n\r"; //
uint8_t error_counter;
volatile uint8_t rx_counter;
volatile uint8_t size_message;
//--------переменные для UART
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Adress 0x27 << 1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* прототип функций для lcd1602 */
void lcd1602_Backlight(bool state);//gjlcdtnrf
void lcd1602_Init(void);//инициализация
void lcd1602_Clean(void);
void lcd1602_SetCursor(uint8_t x, uint8_t y);
void lcd1602_Print_symbol(uint8_t symbol);
void lcd1602_Print_text(char *message);
void lcd1602_Move_to_the_left(void);
void lcd1602_Clean_Text(void);
/* прототип функций для lcd1602 */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {//Символ принялся, сработало прерывание
	if (rx_buffer[0] != '\r') { //если входящий символ не равен \r
		tx_buffer[rx_counter] = rx_buffer[0]; //добаквляем его в buffer
		rx_counter++;//
		size_message = rx_counter;
		if (size_message >= size_tx_buffer && error_counter == 0) {//
			HAL_UART_Transmit_DMA(&huart1, error_message,
					sizeof error_message / sizeof error_message[0]);
			error_counter++;//  увеличеваем счетчик входящих сообщений                              .                            .
			flag_send = 0;
		}
	} else if (rx_buffer[0] == '\r') {//если входящий символ равен \r
		tx_buffer[rx_counter] = '\n';//сообщение полностью получено.Добавляем к нему перевод на следующую строку
		tx_buffer[rx_counter + 1] = '\r';//добавляем возрат коретки
		tx_buffer[rx_counter + 2] = '\0';//
		size_message = rx_counter + 3;//соответственно увеличиваем размер отправляемого сообщения
		rx_counter = 0;//запрещаем выводить сообщение в порт , ведь оно превышено по размеру
		if (size_message >= size_tx_buffer) {//
			flag_send = 0;////                                   ,                              .
			HAL_UART_Transmit_DMA(&huart1, error_message,
					sizeof error_message / sizeof error_message[0]);
		} else {
			flag_send = 1;//;//если же сообщение не превышено по размеру tx_buffer, то разрешаем выводить в порт то, что вывели
			error_counter = 0;// //сбросим счетчик ошибок
		}
	}
	HAL_UART_Receive_IT(&huart1, rx_buffer, 1);//Запускаем прием данных после каждого прерывания.
}
extern I2C_HandleTypeDef hi2c1;         //Шина I2C.
bool backlight = true;             //Начальная установка для подсветки вкл
char lcd1602_tx_buffer[40] = { 0, }; //глобальный буфер данных. В него записывается текст.
uint8_t global_buffer = 0; //глобальная переменная байта данных, отправляемая дисплею.


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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100);
  lcd1602_Init();//Инициализируем дислей
  HAL_Delay(30);
  flag_send = 0;
  	queue_message = 1;
  	rx_counter = 0;
  	size_message = 0;
  	error_counter = 0;
  	HAL_UART_Receive_IT(&huart1, rx_buffer, 1);

//____Выводим стартовое сообщение запуска программы
  lcd1602_SetCursor(0,0);
  lcd1602_Print_text(" sasha.luzay@gmail.com");
  for(int mv=0;mv<6;mv++){//сдвигаем текст
	  lcd1602_Move_to_the_left();
	  HAL_Delay(700);
  }
  HAL_Delay(1500);
  lcd1602_Clean();
  lcd1602_SetCursor(0,0);
  lcd1602_Print_text(" sasha.luzay@gmail.com");
  lcd1602_SetCursor(0,1);
  lcd1602_Print_text("+375297314972");
  HAL_Delay(1500);
//----------------------------------------------------

/*настраиваем PWM*/
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
 /*настраиваем PWM*/
    TIM2->CCR1=0;//выключаем
    TIM2->CCR2=50;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

   /* TIM2->CCR2=100;
    TIM2->CCR1=100;*/
    /* USER CODE BEGIN 3 */

    if (flag_send == 1 && queue_message != 255) {//если разрешено отсылать и очередь не равна максимальной
    	if (queue_message == 1 && huart1.gState == HAL_UART_STATE_READY) {//если порт свободен и очередь сообщений отсутствует
    		HAL_UART_Transmit_DMA(&huart1, tx_buffer, size_message);//отправляем сообщение
    		queue_message = 255;//ставим следующую очередь , если захотим ещё отослать в порт
    					}
    				} else if (flag_send == 1 && queue_message == 255) {//
    					queue_message = 1;//сбросим очередь отправки , чтоб сообщения снова могли по очереди отсылаться
    					flag_send = 0;//отправка сообщения запрещена.Разрешение можно получить в прерывании
    				}
    				if (tx_buffer[0] == 'O' && tx_buffer[1] == 'n' && tx_buffer[2] == '\n' && tx_buffer[3] == '\r') { //если пришло 'On'
    					TIM2->CCR1=100; //Настраиваю ШИМ на 100%
    					lcd1602_Clean();
    					 lcd1602_SetCursor(0,0);
    					  lcd1602_Print_text(" RED LED ON");//вывод текста на дисплей
    				} else if (tx_buffer[0] == 'O' && tx_buffer[1] == 'f' && tx_buffer[2] == 'f' && tx_buffer[3] == '\n' && tx_buffer[4] == '\r') { //если пришло 'Off'
    					TIM2->CCR1=0; //Настраиваю ШИМ на 0%
    					lcd1602_Clean();
    					lcd1602_SetCursor(0,0);
    					lcd1602_Print_text(" RED LED OFF");//вывод текста на дисплей
    				}
    				    else if (tx_buffer[0] == 'H' && tx_buffer[1] == 'f' &&  tx_buffer[2] == '\n' && tx_buffer[3] == '\r') { //если пришло 'Hf'
    				    					TIM2->CCR2=50; //Настраиваю ШИМ на 50%
    				    					lcd1602_Clean();
    				    					lcd1602_SetCursor(0,0);
    				    					lcd1602_Print_text(" RED LED HF"); //вывод текста на дисплей
    				    }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
static void lcd1602_Send_init_Data(uint8_t *init_Data) {
	if (backlight) {
		*init_Data |= 0x08; //Включить подсветку
	} else {
		*init_Data &= ~0x08; //Выключить подсветку
	}
	*init_Data |= 0x04; // Устанавливаем стробирующий сигнал E в 1
	HAL_I2C_Master_Transmit(&hi2c1, Adress, init_Data, 1, 10);
	HAL_Delay(5);
	*init_Data &= ~0x04; // Устанавливаем стробирующий сигнал E в 0
	HAL_I2C_Master_Transmit(&hi2c1, Adress, init_Data, 1, 10);
	HAL_Delay(5);
}
/*-------------Функция для отправки данных при инициализации дисплея-------------*/

/*--------------------Функция отправки байта информации на дисплей---------------*/
/// Функция отправки байта информации на дисплей
/// \param Data - Байт данныйх
static void lcd1602_Write_byte(uint8_t Data) {
	HAL_I2C_Master_Transmit(&hi2c1, Adress, &Data, 1, 10);
}
/*--------------------Функция отправки байта информации на дисплей---------------*/

/*----------------------Функция отправки пол байта информации--------------------*/
/// Функция отправки пол байта информации
/// \*param Data - байт данных
static void lcd1602_Send_cmd(uint8_t Data) {
	Data <<= 4;
	lcd1602_Write_byte(global_buffer |= 0x04); // Устанавливаем стробирующий сигнал E в 1
	lcd1602_Write_byte(global_buffer | Data); // Отправляем в дисплей полученный и сдвинутый байт
	lcd1602_Write_byte(global_buffer &= ~0x04);	// Устанавливаем стробирующий сигнал E в 0.
}
/*----------------------Функция отправки пол байта информации--------------------*/

/*----------------------Функция отправки байта данных----------------------------*/
/// Функция отправки байта данных на дисплей
/// \param Data - байт данных
/// \param mode - отправка команды. 1 - RW = 1(отправка данных). 0 - RW = 0(отправка команды).
static void lcd1602_Send_data_symbol(uint8_t Data, uint8_t mode) {
	if (mode == 0) {
		lcd1602_Write_byte(global_buffer &= ~0x01); // RS = 0
	} else {
		lcd1602_Write_byte(global_buffer |= 0x01); // RS = 1
	}
	uint8_t MSB_Data = 0;
	MSB_Data = Data >> 4; // Сдвигаем полученный байт на 4 позичии и записываем в переменную
	lcd1602_Send_cmd(MSB_Data);	// Отправляем первые 4 бита полученного байта
	lcd1602_Send_cmd(Data);	   // Отправляем последние 4 бита полученного байта
}
/*----------------------Функция отправки байта данных----------------------------*/

/*----------------------Основная функция для отправки данных---------------------*/
/// Функция предназначена для отправки байта данных по шине i2c
/// \param *init_Data - байт, например 0x25, где 2 (0010) это DB7-DB4 или DB3-DB0, а 5(0101) это сигналы LED, E, RW, RS соответственно
static void lcd1602_Send_data(uint8_t *Data) {

	if (backlight) {
		*Data |= 0x08;
	} else {
		*Data &= ~0x08;
	}
	*Data |= 0x04; // устанавливаем стробирующий сигнал E в 1
	HAL_I2C_Master_Transmit(&hi2c1, Adress, Data, 1, 10);
	*Data &= ~0x04; // устанавливаем стробирующий сигнал E в 0
	HAL_I2C_Master_Transmit(&hi2c1, Adress, Data, 1, 10);
}

/*----------------------Основная функция для отправки данных---------------------*/


/*-------------------------Функция инициализации дисплея-------------------------*/
/// Функция инициализации дисплея
void lcd1602_Init(void) {
	/*========Power on========*/
	uint8_t tx_buffer = 0x30;
	/*========Wait for more than 15 ms after Vcc rises to 4.5V========*/
	HAL_Delay(15);
	/*========BF can not be checked before this instruction.========*/
	/*========Function set ( Interface is 8 bits long.========*/
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Wait for more 4.1 ms========*/
	HAL_Delay(5);
	/*========BF can not be checked before this instruction.========*/
	/*========Function set ( Interface is 8 bits long.========*/
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Wait for more 100 microsec========*/
	HAL_Delay(1);
	/*========BF can not be checked before this instruction.========*/
	/*========Function set ( Interface is 8 bits long.========*/
	lcd1602_Send_init_Data(&tx_buffer);

	/*========Включаем 4х-битный интерфейс========*/
	tx_buffer = 0x20;
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Включаем 4х-битный интерфейс========*/

	/*======2 строки, шрифт 5х8======*/
	tx_buffer = 0x20;
	lcd1602_Send_init_Data(&tx_buffer);
	tx_buffer = 0x80;
	lcd1602_Send_init_Data(&tx_buffer);
	/*======2 строки, шрифт 5х8======*/

	/*========Выключить дисплей========*/
	tx_buffer = 0x00;
	lcd1602_Send_init_Data(&tx_buffer);
	tx_buffer = 0x80;
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Выключить дисплей========*/

	/*========Очистить дисплей========*/
	tx_buffer = 0x00;
	lcd1602_Send_init_Data(&tx_buffer);
	tx_buffer = 0x10;
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Очистить дисплей========*/

	/*========Режим сдвига курсора========*/
	tx_buffer = 0x00;
	lcd1602_Send_init_Data(&tx_buffer);
	tx_buffer = 0x30;
	lcd1602_Send_init_Data(&tx_buffer);
	/*========Режим сдвига курсора========*/

	/*========�?нициализация завершена. Включить дисплей========*/
	tx_buffer = 0x00;
	lcd1602_Send_init_Data(&tx_buffer);
	tx_buffer = 0xC0;
	lcd1602_Send_init_Data(&tx_buffer);
	/*========�?нициализация завершена. Включить дисплей========*/
}

/*-------------------------Функция инициализации дисплея-------------------------*/

/*-------------------------Функция вывода символа на дисплей---------------------*/
/// Функция вывода символа на дисплей
/// \param* symbol - символ в кодировке utf-8
void lcd1602_Print_symbol(uint8_t symbol) {
	uint8_t command;
	command = ((symbol & 0xf0) | 0x09); //Формирование верхнего полубайта в команду для дисплея
	lcd1602_Send_data(&command);
	command = ((symbol & 0x0f) << 4) | 0x09; //Формирование нижнего полубайта в команду для дисплея
	lcd1602_Send_data(&command);
}
/*-------------------------Функция вывода символа на дисплей---------------------*/

/*-------------------------Функция вывода текста на дисплей----------------------*/
/// Функция вывода символа на дисплей
/// \param *message - массив, который отправляем на дисплей.
/// Максимальная длина сообщения - 40 символов.
void lcd1602_Print_text(char *message) {
	for (int i = 0; i < strlen(message); i++) {
		lcd1602_Print_symbol(message[i]);
	}
}
/*-------------------------Функция вывода текста на дисплей----------------------*/

/*-------------------Функция установки курсора для вывода текста----------------*/
/// Функция установки курсора для вывода текста на дисплей
/// \param x - координата по оси x. от 0 до 15.
/// \param y - координата по оси y. от 0 до 1.
/// Видимая область:
/// Для дисплеев 1602 max x = 15, max y = 1.

void lcd1602_SetCursor(uint8_t x, uint8_t y) {
	uint8_t command, adr;
	if (y > 1)
		y = 1;
	if (x > 39)
			x = 39;
	if (y == 0) {
		adr = x;
	}
	if (y == 1) {
		adr = x + 0x40;
	}
	if (y == 2) {
		adr = x + 0x14;
	}
	if (y == 3) {
		adr = x + 0x54;
	}
	command = ((adr & 0xf0) | 0x80);
	lcd1602_Send_data(&command);

	command = (adr << 4);
	lcd1602_Send_data(&command);

}

/*-------------------Функция установки курсора для вывода текста----------------*/

/*------------------------Функция перемещения текста влево-----------------------*/
/// Функция перемещения текста влево
/// Если ее повторять с периодичностью, получится бегущая строка
void lcd1602_Move_to_the_left(void) {
	uint8_t command;
	command = 0x18;
	lcd1602_Send_data(&command);

	command = 0x88;
	lcd1602_Send_data(&command);
}
/*------------------------Функция перемещения текста влево-----------------------*/

/*-------------------------Функция очистки дисплея-------------------------------*/

void lcd1602_Clean(void) {
/// Аппаратная функция очистки дисплея.
/// Удаляет весь текст, возвращает курсор в начальное положение.
	uint8_t tx_buffer = 0x00;
	lcd1602_Send_init_Data(&tx_buffer);
	tx_buffer = 0x10;
	lcd1602_Send_init_Data(&tx_buffer);

}

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
