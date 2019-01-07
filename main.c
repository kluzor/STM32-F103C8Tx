/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
unsigned char Rx_data[2];
volatile typedef enum {true,false} bool;
volatile int distance_S = 0;
volatile int distance_L = 0;
volatile int distance_P = 0;

volatile int Duty;
volatile int DutyServ1;
volatile int DutyServ2;
volatile int DutyServ3;
volatile int DutyServ4;
volatile int DutyServ5;
volatile bool Flaga_Auto;

volatile uint16_t sizeRST;
volatile uint16_t sizeCWMODE;
volatile uint16_t sizeCIPMODE;
volatile uint16_t sizeCIPMUX;
volatile uint16_t sizeCIPSERVER;
volatile uint16_t sizeATE;

volatile uint32_t echo_start_S;
volatile uint32_t echo_finish_S;
volatile uint32_t measured_time_S;
volatile uint32_t echo_start_L;
volatile uint32_t echo_finish_L;
volatile uint32_t measured_time_L;
volatile uint32_t echo_start_P;
volatile uint32_t echo_finish_P;
volatile uint32_t measured_time_P;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

int S1_PWM(int Duty)
{
	return TIM2->CCR1 = Duty;
}
int S2_PWM(int Duty)
{
	return TIM2->CCR2 = Duty;
}
void S1_przod()
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
}
void S1_tyl()
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
}

void S2_przod()
{
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
}

void S2_tyl()
{
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}

void S12_stop()
{
	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
}

void ESP8266_Init()
{
	sizeRST = 0;
	sizeCWMODE = 0;
	sizeCIPMODE = 0;
	sizeCIPMUX = 0;
	sizeCIPSERVER = 0;
	sizeATE = 0;

	uint8_t RST[8];
	uint8_t CWMODE[13];
	uint8_t CIPMODE[14];
	uint8_t CIPMUX[13];
	uint8_t CIPSERVER[19];
	uint8_t ATE[6];

	sizeRST = sprintf(RST,"AT+RST\r\n"); //reset modu³u
	sizeCWMODE = sprintf(CWMODE,"AT+CWMODE=2\r\n"); // 1 klient, 2 access point, 3 klient + AP
	sizeCIPMODE = sprintf(CIPMODE,"AT+CIPMODE=0\r\n"); //0 - format ramki(TCP), 1 - UDP
	sizeCIPMUX = sprintf(CIPMUX,"AT+CIPMUX=1\r\n"); //0 - jedno po³¹czenie, 1 - wiele po³¹czeñ
	sizeCIPSERVER = sprintf(CIPSERVER,"AT+CIPSERVER=1,80\r\n"); //uruchomienie serwera na porcie 80
	sizeATE = sprintf(ATE,"ATE0\r\n"); //wy³¹czenie echa ESP8266

	HAL_UART_Transmit_IT(&huart1, RST, sizeRST);
	HAL_Delay(1000);
	HAL_UART_Transmit_IT(&huart1, CWMODE, sizeCWMODE);
	HAL_Delay(1000);
	HAL_UART_Transmit_IT(&huart1, CIPMODE, sizeCIPMODE);
	HAL_Delay(1000);
	HAL_UART_Transmit_IT(&huart1, CIPMUX, sizeCIPMUX);
	HAL_Delay(1000);
	HAL_UART_Transmit_IT(&huart1, CIPSERVER, sizeCIPSERVER);
	HAL_Delay(1000);
	HAL_UART_Transmit_IT(&huart1, ATE, sizeATE);
	HAL_Delay(1000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1)	//wybór UART
	{
		if (Rx_data[0]==65) //A Zmiana trybu jazdy
		{
			Flaga_Auto=true;
		}

		if (Rx_data[0]==81) //Q TURBO
		{
			S1_przod();
			S2_przod();
			S1_PWM(1000);
			S2_PWM(1000);
		}

		if (Rx_data[0]==33) //! PRZOD
		{
			S1_przod();
			S2_przod();
			S1_PWM(300);
			S2_PWM(300);
		}

		if (Rx_data[0]==35) //# TYL
		{
			S1_tyl();
			S2_tyl();
			S1_PWM(300);
			S2_PWM(300);
		}

		if (Rx_data[0]==36) //$ LEWO
		{
			S1_tyl();
			S2_przod();
			S1_PWM(900);
			S2_PWM(900);
		}

		if (Rx_data[0]==37) //% PRAWO
		{
			S1_przod();
			S2_tyl();
			S1_PWM(900);
			S2_PWM(900);
		}

		if (Rx_data[0]==38) //& STOP
		{
			Flaga_Auto=false;
			S12_stop();
			S1_PWM(0);
			S2_PWM(0);
		}

		if (Rx_data[0]==40) //( L1
		{
			if(DutyServ1<=2400)
			{
				DutyServ1 = DutyServ1+5;
				TIM3->CCR1 = DutyServ1;
			}
		}

		if (Rx_data[0]==64) // @ S1
		{
			if(DutyServ1!=1500)
			{
				TIM3->CCR1 = DutyServ1;
			}
		}

		if (Rx_data[0]==91) // [ R1
		{
			if(DutyServ1>=600)
			{
				DutyServ1 = DutyServ1-5;
				TIM3->CCR1 = DutyServ1;
			}
		}

		if (Rx_data[0]==41) // ) L2
		{
			if(DutyServ2<=2400)
			{
				DutyServ2 = DutyServ2+5;
				TIM3->CCR2 = DutyServ2;
			}
		}

		if (Rx_data[0]==63) // ? S2
		{
			if(DutyServ2!=1500)
			{
				TIM3->CCR2 = DutyServ2;
			}
		}

		if (Rx_data[0]==93) // ] R2
		{
			if(DutyServ2>=600)
			{
				DutyServ2 = DutyServ2-5;
				TIM3->CCR2 = DutyServ2;
			}
		}

		if (Rx_data[0]==42) // * L3
		{
			if(DutyServ3<=2400)
			{
				DutyServ3 = DutyServ3+5;
				TIM3->CCR3 = DutyServ3;
			}
		}

		if (Rx_data[0]==62) // > S3
		{
			if(DutyServ3!=1500)
			{
				TIM3->CCR3 = DutyServ3;
			}
		}

		if (Rx_data[0]==94) // ^ R3
		{
			if(DutyServ3>=600)
			{
				DutyServ3 = DutyServ3-5;
				TIM3->CCR3 = DutyServ3;
			}
		}
		if (Rx_data[0]==45) // - L4
		{
			if(DutyServ4<=2400)
			{
				DutyServ4 = DutyServ4+5;
				TIM3->CCR4 = DutyServ4;
			}
		}

		if (Rx_data[0]==61) // = S4
		{
			if(DutyServ4!=1500)
			{
				TIM3->CCR4 = DutyServ4;
			}
		}

		if (Rx_data[0]==123) // { R4
		{
			if(DutyServ4>=600)
			{
				DutyServ4 = DutyServ4-5;
				TIM3->CCR4 = DutyServ4;
			}
		}

		if (Rx_data[0]==47) // / L5
		{
			if(DutyServ5<=2400)
			{
				DutyServ5 = DutyServ5+5;
				TIM4->CCR1 = DutyServ5;
			}
		}

		if (Rx_data[0]==60) // < S5
		{
			if(DutyServ5!=1500)
			{
				TIM4->CCR1 = DutyServ5;
			}
		}

		if (Rx_data[0]==125) // } R5
		{
			if(DutyServ5>=600)
			{
				DutyServ5 = DutyServ5-5;
				TIM4->CCR1 = DutyServ5;
			}
		}
		HAL_UART_Receive_IT(&huart1, Rx_data, 1);	//activate UART receive interrupt every time
	}
}

#define LOOP_FREQ (SystemCoreClock/4000000)

inline void udelay_asm (volatile uint32_t useconds)
{
	useconds *= LOOP_FREQ;
	asm volatile("   mov r0, %[useconds]    \n\t"
				 "1: subs r0, #1            \n\t"
				 "   bhi 1b                 \n\t"
				 :
				 : [useconds] "r" (useconds)
				 : "r0");
}

volatile typedef enum {
 IDLE_S,
 TRIGGERING_S,
 WAITING_FOR_ECHO_START_S,
 WAITING_FOR_ECHO_STOP_S,
 TRIG_NOT_WENT_LOW_S,
 ECHO_TIMEOUT_S,
 ECHO_NOT_WENT_LOW_S,
 READING_DATA_S,
 ERROR_S
} state_S;

volatile typedef enum {
 IDLE_L,
 TRIGGERING_L,
 WAITING_FOR_ECHO_START_L,
 WAITING_FOR_ECHO_STOP_L,
 TRIG_NOT_WENT_LOW_L,
 ECHO_TIMEOUT_L,
 ECHO_NOT_WENT_LOW_L,
 READING_DATA_L,
 ERROR_L
} state_L;

volatile typedef enum {
 IDLE_P,
 TRIGGERING_P,
 WAITING_FOR_ECHO_START_P,
 WAITING_FOR_ECHO_STOP_P,
 TRIG_NOT_WENT_LOW_P,
 ECHO_TIMEOUT_P,
 ECHO_NOT_WENT_LOW_P,
 READING_DATA_P,
 ERROR_P
} state_P;

volatile state_S S_state = IDLE_S;
volatile state_L L_state = IDLE_L;
volatile state_P P_state = IDLE_P;

void HAL_GPIO_EXTI_Callback(volatile uint16_t GPIO_Pin)
{
	if (GPIO_Pin == ECHO_S_Pin )
	{
		switch (S_state)
		{
		  case WAITING_FOR_ECHO_START_S:
		  {
			  echo_start_S = 1000 * HAL_GetTick();
			  S_state = WAITING_FOR_ECHO_STOP_S;
			  break;
		  }
		  case WAITING_FOR_ECHO_STOP_S:
		  {
			  echo_finish_S = 1000 * HAL_GetTick();
			  measured_time_S = echo_finish_S - echo_start_S;
			  S_state = READING_DATA_S;
			  break;
		  }
		  default:
			  S_state = ERROR_S;
		}
		distance_S = measured_time_S/58;
	}

	if (GPIO_Pin == ECHO_L_Pin )
	{
		switch (L_state)
		{
		  case WAITING_FOR_ECHO_START_L:
		  {
			  echo_start_L = 1000 * HAL_GetTick();
			  L_state = WAITING_FOR_ECHO_STOP_L;
			  break;
		  }
		  case WAITING_FOR_ECHO_STOP_L:
		  {
			  echo_finish_L = 1000 * HAL_GetTick();
			  measured_time_L = echo_finish_L - echo_start_L;
			  L_state = READING_DATA_L;
			  break;
		  }
		  default:
			  L_state = ERROR_L;
		}
		distance_L = measured_time_L/58;
	}

	if (GPIO_Pin == ECHO_P_Pin )
	{
		switch (P_state)
		{
		  case WAITING_FOR_ECHO_START_P:
		  {
			  echo_start_P = 1000 * HAL_GetTick();
			  P_state = WAITING_FOR_ECHO_STOP_P;
			  break;
		  }
		  case WAITING_FOR_ECHO_STOP_P:
		  {
			  echo_finish_P = 1000 * HAL_GetTick();
			  measured_time_P = echo_finish_P - echo_start_P;
			  P_state = READING_DATA_P;
			  break;
		  }
		  default:
			  P_state = ERROR_P;
		}
		distance_P = measured_time_P/58;
	}
}

void pomiar_S()
{
	HAL_GPIO_WritePin(TRIG_S_GPIO_Port, TRIG_S_Pin, GPIO_PIN_SET);
	udelay_asm(18);
	HAL_GPIO_WritePin(TRIG_S_GPIO_Port, TRIG_S_Pin, GPIO_PIN_RESET);
	S_state = WAITING_FOR_ECHO_START_S;
	while( S_state == WAITING_FOR_ECHO_START_S && S_state != ERROR_S )
	{}
	while( S_state == WAITING_FOR_ECHO_STOP_S && S_state != ERROR_S )
	{}
}

void pomiar_L()
{
	HAL_GPIO_WritePin(TRIG_L_GPIO_Port, TRIG_L_Pin, GPIO_PIN_SET);
	udelay_asm(18);
	HAL_GPIO_WritePin(TRIG_L_GPIO_Port, TRIG_L_Pin, GPIO_PIN_RESET);
	L_state = WAITING_FOR_ECHO_START_L;
	while( L_state == WAITING_FOR_ECHO_START_L && L_state != ERROR_L )
	{}
	while( L_state == WAITING_FOR_ECHO_STOP_L && L_state != ERROR_L )
	{}
}

void pomiar_P()
{
	HAL_GPIO_WritePin(TRIG_P_GPIO_Port, TRIG_P_Pin, GPIO_PIN_SET);
	udelay_asm(18);
	HAL_GPIO_WritePin(TRIG_P_GPIO_Port, TRIG_P_Pin, GPIO_PIN_RESET);
	P_state = WAITING_FOR_ECHO_START_P;
	while( P_state == WAITING_FOR_ECHO_START_P && P_state != ERROR_P )
	{}
	while( P_state == WAITING_FOR_ECHO_STOP_P && P_state != ERROR_P )
	{}
}

void prawo()
{
	S1_przod();
	S2_tyl();
	S1_PWM(900);
	S2_PWM(900);
}

void lewo()
{
	S1_tyl();
	S2_przod();
	S1_PWM(900);
	S2_PWM(900);
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

  Duty = 0;
  TIM3->CCR1 = 1600;
  TIM3->CCR2 = 1400;
  TIM3->CCR3 = 1400;
  TIM3->CCR4 = 1400;
  TIM4->CCR1 = 1580;
  Flaga_Auto = false;

  DutyServ1=1600;
  DutyServ2=1400;
  DutyServ3=1400;
  DutyServ4=1400;
  DutyServ5=1580;

  ESP8266_Init();
  HAL_UART_Receive_IT(&huart1, Rx_data, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  pomiar_S();
	  pomiar_L();
	  pomiar_P();

	  while(Flaga_Auto==false)
	  {
		  if(distance_S > 35  && distance_L > 35 && distance_P > 35)
		  {
			S1_przod();
			S2_przod();
			S1_PWM(300);
			S2_PWM(300);
		  }

		  if(distance_P <= 34 && distance_S <= 34 && distance_L <= 34)
		  {
			  prawo();
		  }
		  if(distance_P <= 34 && distance_S > 35 && distance_L > 35)
		  {
			  lewo();
		  }
		  if(distance_P <= 34 && distance_S <= 34 && distance_L > 35)
		  {
			  lewo();
		  }
		  if(distance_S <= 34 && distance_P > 35 && distance_L > 35)
		  {
			  lewo();
		  }
		  if(distance_L <= 34 && distance_S > 35 && distance_P > 35)
		  {
			  prawo();
		  }
		  if(distance_L <= 34 && distance_S <= 34 && distance_P > 35)
		  {
			  prawo();
		  }
		  if(distance_P <= 34 && distance_L <= 34 && distance_S > 35)
		  {
			  prawo();
		  }
	  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
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

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_L_GPIO_Port, TRIG_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRIG_P_Pin|TRIG_S_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ECHO_L_Pin */
  GPIO_InitStruct.Pin = ECHO_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_L_Pin */
  GPIO_InitStruct.Pin = TRIG_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO_P_Pin ECHO_S_Pin */
  GPIO_InitStruct.Pin = ECHO_P_Pin|ECHO_S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_P_Pin TRIG_S_Pin */
  GPIO_InitStruct.Pin = TRIG_P_Pin|TRIG_S_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
