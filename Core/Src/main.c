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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "IQS7211A.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Max_RxTxMessage_Len 19	//'#',:,<Mode>(2),:,<State>(3),:,<Param1>(3),:,<Param2>(3),:,'$\n'(2)
#define Rx_Request_Mode_Len 7	//'#',:,<Mode>(2),:,'$\n'(2)
#define ADC_Array_len 50		// get 49 ADC average values to work with
#define Track_X_Max 1700
#define Track_Y_Max 768
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// -------------------------------------PUSH-BUTTONS-------------------------------------------------- //
//Button Presses
volatile int Mid_ButtonPress_Count = 0;
volatile int Left_ButtonPress_Count = 0;
volatile int Right_ButtonPress_Count = 0;

//BUTTONS
volatile uint8_t ButtonPressed[5]  = {0,0,0,0,0};
volatile uint16_t ButtonPressCounter[5] = {0,0,0,0,0};
volatile uint16_t Threshold_Num[5] = {0,0,0,0,0};  //Threshold_Num
volatile uint8_t i,n;
// -------------------------------------PUSH-BUTTONS-------------------------------------------------- //


// -------------------------------------UART CODE-------------------------------------------------- //
uint8_t charCount = 0;
uint8_t rx_busy = 0;
uint8_t RX_Character;

//Variables
uint8_t tx_Stdnum[13] = "#:23582022:$\n";
uint8_t Tx_StartStatus[Max_RxTxMessage_Len] = "#:MF:000:000:000:$\n";
uint8_t Rx_Message[Max_RxTxMessage_Len] = "";

uint8_t Rx_Command[14];
uint8_t rx_Char[1];

//RX STATUS VARIABLES (Default Values included)
uint8_t FM_MODE[2] = "MF"; //Flash Mode
uint8_t EM_MODE[2] = "ME"; //Emergency Mode
uint8_t MM_MODE[2] = "MM"; //Mood Mode
uint8_t FM_LED_Intensity[3] = "512"; // <state> for Flashlight Mode
uint8_t EM_LED_Intensity[3] = "000"; // <state> for Emergency Mode
uint8_t Flash_Param1[3] = "000"; // <Param1> for Flashlight Mode (ALWAYS '000')
uint8_t Flash_Param2[3] = "000"; // <Param2> for Flashlight Mode (ALWAYS '000')
uint8_t EM_Strobe_OnTime[3] = "300"; // <Param1> for Emergency Mode
uint8_t EM_MorseCode[3] = "000"; // <Param2> for Emergency Mode
uint8_t EM_CustomMorse[3] = "000"; // <Param2> for Emergency Mode
//Set Command Mode VAR (*need to be uint8_t as COUNT is passed as integer)
//TX PARAMETERS
uint8_t MODE[2] = "MF";
uint8_t M_STATE[3] = "000";
uint8_t PARAM_1[3] = "000";
uint8_t PARAM_2[3] = "000";

//RGB MODE VARIABLES
uint8_t R_Channel_RGB[3] = "000"; // <state> for Mood Mode
uint8_t G_Channel_RGB[3] = "000"; // <Param1> for Mood Mode
uint8_t B_Channel_RGB[3] = "000"; // <Param2> for Mood Mode

// -------------------------------------UART CODE-------------------------------------------------- //


// -------------------------------------FLAGS------------------------------------------------------ //
//-------------MAIN SYSTEM FLAGS--------------//
uint8_t b_OFF_STATE = 0;

//MENU FLAGS - System Modes
uint8_t b_FLASH_MODE = 1;
uint8_t b_EMERGENCY_MODE = 0;
uint8_t b_MOOD_MODE = 0;

//White LED FLAG - INITIALLY OFF
uint8_t b_WhiteLED_State = 0;

//RGB LED - INITIALLY OFF
uint8_t b_RGBLED_State = 0;

//Uart/Trackpad flags RGB MOOD LIGHT
uint8_t b_UartRGB_SET = 0;
uint8_t b_TrackRGB_SET = 0;
//-------------MAIN SYSTEM FLAGS--------------//

//UART RX FLAGS
uint8_t b_CharRecieve = 0;		//1 when 1 byte Received by TS/PC
uint8_t b_MsgReceived = 0;		//1 if full Rx Message is received
uint8_t b_ModeREQReceived = 0;	//1 when 7 byte request mode status received via TS/PC
uint8_t b_ModeCOMReceived = 0; 	//1 when 19 byte Mode Command Received from TS/PC


//UART MODE SET FLAGS
uint8_t b_UART_SET_FM = 0;
uint8_t b_UART_SET_EM = 0;
uint8_t b_UART_SET_MM  = 0;

//Emergency Mode Flags
uint8_t b_EM_Strobe = 1;
uint8_t b_EM_SOS = 0;
uint8_t b_EM_CustomMsg = 0;

//Right Button EM Flags
//MORSE Flag
uint8_t b_Right_Morse = 0;
//Strobe Flag
uint8_t b_Right_Strobe = 0;
//SOS Flag
uint8_t b_Right_SOS = 0;

//Request Flags
uint8_t b_Req_Flash = 0;
uint8_t b_Req_Emergency = 0;
uint8_t b_Req_Mood = 0;


//UART EM Flags
//MORSE Flag
uint8_t b_UART_Morse = 0;
//Strobe Flag
uint8_t b_UART_Strobe = 0;
//SOS Flag
uint8_t b_UART_SOS = 0;

//Intensity set flags - FM/EM MODES
uint8_t b_Slider_ADC_Intensity = 0;
uint8_t b_UART_Intensity = 0;
uint8_t b_TRACK_Intensity = 0;

//Slider ADC FALGS
uint8_t b_Slider_Up = 0;
uint8_t b_Slider_Down = 0;

//MM Flags
//UART
uint8_t b_RSET = 0;
uint8_t b_GSET = 0;
uint8_t b_BSET = 0;


uint8_t b_MOOD_Track_TAP = 0;
uint8_t b_Track_TAP = 0;
uint8_t b_Track_HOLD = 0;
uint8_t b_Track_FIRST_HOLD = 0;
uint8_t b_Track_SLIDE = 0;
uint8_t b_Track_RELEASE = 0;

//RGB-Channels
uint8_t b_RTrack = 0;
uint8_t b_GTrack = 0;
uint8_t b_BTrack = 0;
//RDY LINE
uint8_t RDY_LOW = 0;
// -------------------------------------FLAGS------------------------------------------------------ //


// ---------------------------------------ADC------------------------------------------------------ //
uint16_t ADC_Val;
uint16_t ADC_raw_MEAS_VAL[ADC_Array_len];
uint16_t ADC_avg_MEAS_VAL;
uint16_t ADC_Average_VAL[10]; //Average value of 10000 samples 0 -> 4031
uint16_t ADC_CALIBRATION_VAL;
uint16_t ADC_Slider_Intensity[10]; // Slider Values between 0 -> 502
uint32_t ADC_Total_VAL = 0;
uint16_t ADC_Vin = 0;
uint16_t ADC_Voltage[10] ; // Output Voltage of slider
uint16_t ADC_Slider_Change_Down;
uint16_t ADC_Slider_Change_UP;
uint16_t Vin;

//NEW continuous
uint16_t callibrated_ADC;
uint16_t OLD_ADC;

char Output[100];
char Output_CAl[100];
uint32_t adc_outlen;
uint32_t adc_CAL_outlen;
//---------------------------------------ADC------------------------------------------------------ //

//---------------------------------------DAC------------------------------------------------------ //
//MOOD MODE INENSITIES
uint16_t Get_R[3] ;
uint16_t Get_G[3] ;
uint16_t Get_B[3] ;
float Red_Val;
float Green_Val;
float Blue_Val;

//FM & EM INTENSITIES
uint16_t Get_Adc_For_Dac[3] ;
float Dac_Val;
//---------------------------------------DAC------------------------------------------------------ //


//---------------------------------------TIM-STROBE------------------------------------------------------ //
uint8_t STROBE_ARR[3];
uint8_t strobe_on = 0;
uint16_t strobe_delay;
//---------------------------------------TIM-MORSE------------------------------------------------------ //


//---------------------------------------TIM-MORSE------------------------------------------------------ //
//MORSE RELATED VARIABLES
uint8_t next = 1;  //Managing sequential execution
uint32_t gendelay_check = 0;
uint32_t gendelay_start = 0;
uint8_t first_call = 1; //to check whether the check delay function is still in process
uint8_t morse_high = 1;
uint8_t cur_element = 0;
uint8_t char_count=0;
uint8_t char_for_morse;
//---------------------------------------TIM-MORSE------------------------------------------------------ //

//---------------------------------------TRACKPAD------------------------------------------------------ //
uint8_t GestureReceive[10];

//MOOD MODE
uint16_t Track_X  ;
uint16_t Track_Y  ;

//SLIDER
uint16_t Track_INIT_X = 0 ;
uint16_t Track_FINAL_X = 0;
uint16_t Track_DELTA_X = 0;

//Coordinates_Trackpad
uint8_t transferCoordxy[10];
uint8_t X_LSB;
uint8_t X_MSB;
uint8_t Y_LSB;
uint8_t Y_MSB;

uint16_t Callibrated_Y_POS;
//---------------------------------------TRACKPAD------------------------------------------------------ //
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

//UART FUNCTIONS
void RX_Msg_Evaluate(void);
void TX_SendStartMessage(void);
void TX_SendCurrentStatus(uint8_t mode[2],uint8_t state[3],uint8_t param_1[3],uint8_t param_2[3]);
void RX_Msg_Request();

//DEFAULT FUNCTIONS TO CALL WHEN NEEDED
void EM_STROBE_Msg(uint8_t strobe_set);
void EM_MORSE_Msg(uint8_t Custom_Morse_Send, uint8_t Right_Morse);
void EM_SOS_Msg(uint8_t SOS_Receive);

void SET_Default_RGB_Values();

void SET_FM_MODE_DEFAULT();
void SET_MM_MODE_DEFAULT();
void SET_MM_RGB_DEFAULT();

//RIGHT BUTTON PRESS/TRACK TAP FUNCTION
void Cycle_EM_SubMODES();

//EM SUB MODES
void SET_EM_SubMODES();

//ON/OFF MODES
void SET_FM_MODE_ON();
void SET_EM_MODE_ON();
void SET_MM_RGB_ON();

void SET_FM_MODE_OFF();
void SET_FM_PARAMS();
void SET_EM_MODE_OFF();
void SET_MM_RGB_OFF();

//TRACKPAD PRESS & HOLD/ MIDDEL BUTTON
void TOGGLE_LEDS();

//ADC
uint16_t Get_AVG_ADC_Value();
uint16_t Get_ADC_Value();
uint16_t Get_Callibrated_ADC(uint16_t ADC_AVG);
uint16_t Get_ADC_Voltage(uint16_t ADC_AVG);
void Transmit_ADC(uint16_t ADC_Val);
void Slider_LED_Intensity(uint16_t Slider_Intensity);
void Uart_LED_Intensity(uint8_t Uart_FM_Intensity[3],uint8_t Uart_EM_Intensity[3]);
void SET_LED_Intensity();
void Clear_Adc_Values();
void Look_Slider_Change();

//STROBE
void SET_STROBE_ARR();

//DAC/PWM
void SET_DAC_PWM();
void SET_DAC_PWM_ON();
void SET_DAC_PWM_OFF();
void RESET_DAC_PWM();

//TRACKPAD
void TRACK_INITIALIZE();
void CHECK_TRACK();
void SET_RGB_Intensity(uint16_t X_POS,uint16_t Y_POS);
uint16_t SET_Callibrated_TrackIntensity(uint16_t Y_POSITION);
uint16_t getAbsXCoordinate();
uint16_t getAbsYCoordinate();
void SET_RGB_ON();

//CUSTOM MORSE EM MODE
void MorseChar(uint8_t inp_char);
void MorseTransmit(uint8_t inpArr[3]);
void SetaDelay(uint32_t input_delay);
void EndCharTimeout(void);
void EndWordTimeout(void);
void Dash(void);
void Dot(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */



	HAL_UART_Receive_IT(&huart2, rx_Char, 1);// SET UART FOR SINGLE BYTE MODE
	if(b_FLASH_MODE){
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
	}

	//ADC
	HAL_ADC_Start(&hadc1);

	//DAC
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);

	//START PWN
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	//STROBE TIMER
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start_IT(&htim16);

	//BUTTONS
	SysTick_Config(72000000/1000);

	htim3.Instance->CCR1 = 0;
	htim3.Instance->CCR2 = 0;
	htim3.Instance->CCR3 = 0;
	htim3.Instance->CCR4 = 0;

	//TRACKPAD INITIALIZATION
	TRACK_INITIALIZE();


	TX_SendStartMessage();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		//CHECK IF UART COMMAND IS RECEIVED
		if(b_MsgReceived){
			if(b_ModeCOMReceived){ 	//SET MODE COMMAND
				RX_Msg_Evaluate();
				b_ModeCOMReceived = 0;
			}else
				if (b_ModeREQReceived){ //REQUEST MODE STATUS
					RX_Msg_Request();
					b_ModeREQReceived = 0;
				}
			b_MsgReceived = 0;
		}

		//LOOK TRACKPAD GESTURES
		CHECK_TRACK();

		//REACT ON GESTURES
		if((b_Track_TAP)&&(b_EMERGENCY_MODE)){
			b_Track_TAP = 0;
			Cycle_EM_SubMODES();

		}else
			if((b_Track_TAP)&&(b_MOOD_MODE)&&(b_RGBLED_State)){
				b_Track_TAP = 0;
				SET_RGB_Intensity(Track_X,Track_Y);
			}
		if(b_Track_HOLD){
			b_Track_HOLD = 0;
			TOGGLE_LEDS();
		}


		//ADC
		Clear_Adc_Values();
		callibrated_ADC = Get_Callibrated_ADC(Get_AVG_ADC_Value());

		//See if LED Intensity is determined by slider movement
		Look_Slider_Change();

		//EM MODE STROBE AND MORSE
		if((b_EMERGENCY_MODE) && (b_EM_Strobe) && (b_WhiteLED_State)){
			SET_STROBE_ARR();
		}
		//Custom MORSE
		if((b_EMERGENCY_MODE) &&(b_EM_CustomMsg) && (b_WhiteLED_State)){
			MorseTransmit(EM_MorseCode);
		}
		//SOS MORSE
		if((b_EMERGENCY_MODE) && (b_EM_SOS) && (b_WhiteLED_State)){
			MorseTransmit(EM_MorseCode);
		}


		//PROGRAM CORRECTION
		if((b_FLASH_MODE)||(b_EMERGENCY_MODE)){
			b_RGBLED_State = 0;
			htim3.Instance->CCR2 = 0;
			htim3.Instance->CCR3 = 0;
			htim3.Instance->CCR4 = 0;
		}

		if((b_FLASH_MODE)||(b_MOOD_MODE)){
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
		}
		//PROGRAM CORRECTION

		//Intensity change in MOOD MODE
		if(b_MOOD_MODE){
			htim3.Instance->CCR1 = 0;
			SET_RGB_ON();
		}
		//Intensity change in MOOD MODE


		//FM & ME LED INTENSITY CHANGES
		if(b_WhiteLED_State == 1){
			if((b_UART_Intensity) && (!b_Slider_ADC_Intensity)){
				SET_LED_Intensity();
			}else
				if((b_Slider_ADC_Intensity) && (!b_UART_Intensity)){
					SET_LED_Intensity();
				}else
					if((!b_UART_Intensity) && (!b_Slider_ADC_Intensity) && (!b_TRACK_Intensity)){
						b_Slider_ADC_Intensity = 1;
						SET_LED_Intensity();
					}
		}else
			if(b_WhiteLED_State == 0){
				SET_LED_Intensity();
			}
		//FM & ME LED INTENSITY CHANGES


		//REQUEST MODE RESPONSE VIA UART
		if(b_Req_Flash == 1){
			TX_SendCurrentStatus(MODE, M_STATE, PARAM_1, PARAM_2);
			b_Req_Flash = 0;
		}else if(b_Req_Emergency == 1){
			TX_SendCurrentStatus(MODE, M_STATE, PARAM_1, PARAM_2);
			b_Req_Emergency = 0;
		}else if(b_Req_Mood == 1){
			TX_SendCurrentStatus(MODE, M_STATE, PARAM_1, PARAM_2);
			b_Req_Mood = 0;
		}
		//REQUEST MODE RESPONSE VIA UART

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM16
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 719;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 511;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 256;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 128;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 128;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 128;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7199;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 5119;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MCLR_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LED5_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Middle_Button_Pin */
  GPIO_InitStruct.Pin = Middle_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Middle_Button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MCLR_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = MCLR_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RDY_Pin */
  GPIO_InitStruct.Pin = RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Right_Button_Pin Left_Button_Pin */
  GPIO_InitStruct.Pin = Right_Button_Pin|Left_Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//-------------------------------------------------------------------START UART FUNCTIONS--------------------------------------------------------------------------------------------------------//
//UART RX_EVALUATION
void RX_Msg_Evaluate(void){

	// Special characters @ 0,17,18
	if((Rx_Message[0] == 35 /*'#'*/) && (Rx_Message[(Max_RxTxMessage_Len - 2)] == 36/*'$'*/) && (Rx_Message[(Max_RxTxMessage_Len - 1)] == 10/*'\n'*/)){
		// Space Between fields @ 1,4,8,12,16
		if ((Rx_Message[1] == 58/*':'*/) && (Rx_Message[4] == 58/*':'*/) && (Rx_Message[8] == 58/*':'*/) && (Rx_Message[12] == 58/*':'*/) && (Rx_Message[(Max_RxTxMessage_Len - 3)] == 58/*':'*/)){
			if(Rx_Message[2] == 77/*'M'*/){

				MODE[0] = Rx_Message[2];

				switch(Rx_Message[3]){
				case(70/*'F'*/):	//SET TO FLASHLIIGHT-MODE 70
				MODE[1] = Rx_Message[3];

				b_EMERGENCY_MODE = 0;
				b_MOOD_MODE = 0;

				//HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

				FM_MODE[0] = MODE[0];
				FM_MODE[1] = MODE[1];

				// <STATE> intensity of white LED(001 -> 512) *000 if White LED = OFF
				//Look if white LED is SET ON/OFF
				if ((Rx_Message[5] == 48/*'0'*/) && (Rx_Message[6] == 48/*'0'*/) && (Rx_Message[7] == 48/*'0'*/) ){
					SET_FM_MODE_OFF();

					b_WhiteLED_State = 0;
					//Set UART intensity flag
					b_UART_Intensity = 1;
					b_Slider_ADC_Intensity = 0;
					b_TRACK_Intensity = 0;
				}else if((Rx_Message[5] != 48/*'0'*/) || (Rx_Message[6] != 48/*'0'*/) || (Rx_Message[7] != 48/*'0'*/)){
					FM_LED_Intensity[0] = Rx_Message[5];
					FM_LED_Intensity[1] = Rx_Message[6];
					FM_LED_Intensity[2] = Rx_Message[7];
					b_WhiteLED_State = 1;
					//Set UART intensity flag
					b_UART_Intensity = 1;
					b_Slider_ADC_Intensity = 0;
					b_TRACK_Intensity = 0;
				}

				//FLASHLIGHT MODE: PARAM_1 & PARAM_2  ALWAYS '000'
				if ((Rx_Message[9] == 48/*'0'*/) && (Rx_Message[10] == 48/*'0'*/) && (Rx_Message[11] == 48/*'0'*/)){
					if ((Rx_Message[13] == 48/*'0'*/) && (Rx_Message[14] == 48/*'0'*/) && (Rx_Message[15] == 48/*'0'*/)){
						// <PARAM_1> '000' &  <PARAM_2> '000'
						SET_FM_PARAMS();
					}
				}
				b_FLASH_MODE = 1;

				break;
				case(69/*'E'*/):	//SET TO EMERGANCY-MODE
				MODE[1] = Rx_Message[3];

				b_FLASH_MODE = 0;
				b_MOOD_MODE = 0;

				//HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);

				EM_MODE[0] = MODE[0];
				EM_MODE[1] = MODE[1];

				// <STATE> intensity of white LED(001 -> 512) *000 if White LED = OFF
				//Look if white LED is SET ON/OFF
				if ((Rx_Message[5] == 48/*'0'*/) && (Rx_Message[6] == 48/*'0'*/) && (Rx_Message[7] == 48/*'0'*/) ){
					SET_EM_MODE_OFF();
					htim3.Instance->CCR1 = 0;
					b_WhiteLED_State = 0;
					//Set UART intensity flag
					b_UART_Intensity = 1;
					b_Slider_ADC_Intensity = 0;
					b_TRACK_Intensity = 0;
				}else if((Rx_Message[5] != 48/*'0'*/) || (Rx_Message[6] != 48/*'0'*/) || (Rx_Message[7] != 48/*'0'*/)){
					EM_LED_Intensity[0] = Rx_Message[5];
					EM_LED_Intensity[1] = Rx_Message[6];
					EM_LED_Intensity[2] = Rx_Message[7];
					b_WhiteLED_State = 1;
					//Set UART intensity flag
					b_UART_Intensity = 1;
					b_Slider_ADC_Intensity = 0;
					b_TRACK_Intensity = 0;
				}

				// <PARAM_1>
				if ((Rx_Message[9] != 48/*'0'*/) || (Rx_Message[10] != 48/*'0'*/) || (Rx_Message[11] != 48/*'0'*/)){
					b_UART_Strobe = 1;
					b_UART_SOS = 0;
					b_UART_Morse = 0;
					//ON Time of Srobe Light(ms) (001 -> 512)
					EM_Strobe_OnTime[0] = Rx_Message[9];
					EM_Strobe_OnTime[1] = Rx_Message[10];
					EM_Strobe_OnTime[2] = Rx_Message[11];
					//<param_2>
					if((Rx_Message[13] == 48/*'0'*/) && (Rx_Message[14] == 48/*'0'*/) && (Rx_Message[15] == 48/*'0'*/)){
						EM_MorseCode[0] = 48;
						EM_MorseCode[1] = 48;
						EM_MorseCode[2] = 48;
					}else
						if((Rx_Message[13] == 'S') && (Rx_Message[14] == 'O') && (Rx_Message[15] == 'S')){
							EM_MorseCode[0] = 'S';
							EM_MorseCode[1] = 'O';
							EM_MorseCode[2] = 'S';
						}
					b_EM_Strobe = 1;
					b_EM_CustomMsg = 0;
					b_EM_SOS = 0;

					HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
				}else
					if((Rx_Message[9] == 48/*'0'*/) && (Rx_Message[10] == 48/*'0'*/) && (Rx_Message[11] == 48/*'0'*/)){
						b_EM_Strobe = 0;
						EM_Strobe_OnTime[0] = 48;
						EM_Strobe_OnTime[1] = 48;
						EM_Strobe_OnTime[2] = 48;
						//<param_2>
						if ((Rx_Message[13] == 48/*'0'*/) && (Rx_Message[14] == 48/*'0'*/) && (Rx_Message[15] == 48/*'0'*/)){
							b_UART_SOS = 1;
							b_UART_Strobe = 0;
							b_UART_Morse = 0;
							EM_MorseCode[0] = 'S';
							EM_MorseCode[1] = 'O';
							EM_MorseCode[2] = 'S';
							b_EM_SOS = 1;
							b_EM_CustomMsg = 0;

							HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
						}else
							if ((Rx_Message[13] != 48/*'0'*/) || (Rx_Message[14] != 48/*'0'*/) || (Rx_Message[15] != 48/*'0'*/)){
								b_UART_Morse = 1;
								b_UART_SOS = 0;
								b_UART_Strobe = 0;

								 next = 1;  //Managing sequential execution
								 gendelay_check = 0;
								 gendelay_start = 0;
								 first_call = 1; //to check whether the check delay function is still in process
								 morse_high = 1;
								 cur_element = 0;
								 char_count=0;
								 strobe_on = 0;
								//<PARAM_2> 3 letter Morse Code Message Display
								EM_MorseCode[0] = Rx_Message[13];
								EM_MorseCode[1] = Rx_Message[14];
								EM_MorseCode[2] = Rx_Message[15];

//								EM_CustomMorse[0] = Rx_Message[13];
//								EM_CustomMorse[1] = Rx_Message[14];
//								EM_CustomMorse[2] = Rx_Message[15];
								b_EM_SOS = 0;
								b_EM_CustomMsg = 1;

								HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
							}
					}

				b_EMERGENCY_MODE = 1;
				break;
				case(77/*'M'*/):	//SET TO MOOD-MODE
				MODE[1] = Rx_Message[3];

				b_FLASH_MODE = 0;
				b_EMERGENCY_MODE = 0;

				//HAL_GPIO_WritePin(GPIOx, GPIO_Pin, PinState);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);

				MM_MODE[0] = MODE[0];
				MM_MODE[1] = MODE[1];

				// <STATE>  Intensity of R channel of RGB LED(000 -> 512)
				if((Rx_Message[5] != 48) || (Rx_Message[6] != 48) || (Rx_Message[7] != 48)){
					R_Channel_RGB[0] = Rx_Message[5];
					R_Channel_RGB[1] = Rx_Message[6];
					R_Channel_RGB[2] = Rx_Message[7];
					b_RSET = 1;
					b_RTrack = 0;
				}else
					if((Rx_Message[5] == 48) && (Rx_Message[6] == 48) && (Rx_Message[7] == 48)){
						R_Channel_RGB[0] = 48;
						R_Channel_RGB[1] = 48;
						R_Channel_RGB[2] = 48;
						b_RSET = 0;
					}

				// <PARAM_1> Intensity of G channel of RGB LED(000 -> 512)
				if((Rx_Message[9] != 48) || (Rx_Message[10] != 48) || (Rx_Message[11] != 48)){
					G_Channel_RGB[0] = Rx_Message[9];
					G_Channel_RGB[1] = Rx_Message[10];
					G_Channel_RGB[2] = Rx_Message[11];
					b_GSET = 1;
					b_GTrack = 0;
				}else
					if((Rx_Message[9] == 48) && (Rx_Message[10] == 48) && (Rx_Message[11] == 48)){
						G_Channel_RGB[0] = 48;
						G_Channel_RGB[1] = 48;
						G_Channel_RGB[2] = 48;
						b_GSET = 0;
					}

				// <PARAM_2> Intensity2of B channel of RGB LED(000 -> 512)
				if((Rx_Message[13] != 48) || (Rx_Message[14] != 48) || (Rx_Message[15] != 48)){
					B_Channel_RGB[0] = Rx_Message[13];
					B_Channel_RGB[1] = Rx_Message[14];
					B_Channel_RGB[2] = Rx_Message[15];
					b_BSET = 1;
					b_BTrack = 0;
				}else
					if((Rx_Message[13] == 48) && (Rx_Message[14] == 48) && (Rx_Message[15] == 48)){
						B_Channel_RGB[0] = 48;
						B_Channel_RGB[1] = 48;
						B_Channel_RGB[2] = 48;
						b_BSET = 0;
					}

				//Set RGB LED STATE - UART
				if((b_RSET == 1) || (b_GSET == 1) || (b_BSET == 1)){
					SET_MM_RGB_ON();
					b_RGBLED_State = 1;
					htim3.Instance->CCR2 = 1;
					htim3.Instance->CCR3 = 1;
					htim3.Instance->CCR4 = 1;
					b_WhiteLED_State = 0;
					htim3.Instance->CCR1 = 0;

					b_UartRGB_SET = 1;
				}else if((b_RSET == 0) && (b_GSET == 0) && (b_BSET == 0)){
					SET_MM_RGB_OFF();
					b_RGBLED_State = 0;
					b_UartRGB_SET = 0;
				}

				b_MOOD_MODE = 1;
				break;
				default:
					break;
				}
			}

		}
	}
}

void RX_Msg_Request(){

	// Special characters @ 0,5,6
	if((Rx_Message[0] == 35 /*'#'*/) && (Rx_Message[(Rx_Request_Mode_Len - 2)] == 36 /*'$'*/) && (Rx_Message[(Rx_Request_Mode_Len - 1)] == 10/*'\n'*/)){
		// Space Between fields @ 1,4
		if ((Rx_Message[1] == 58/*':'*/) && (Rx_Message[4] == 58/*':'*/)){
			if(Rx_Message[2] == 77/*'M'*/){
				MODE[0] = Rx_Message[2];
				switch(Rx_Message[3]){

				case(70/*'F'*/):	//FLASHLIIGHT-MODE Request STATUS

	  			b_Req_Emergency = 0;
				b_Req_Mood = 0;
				MODE[1] = FM_MODE[1];

				M_STATE[0] = FM_LED_Intensity[0];
				M_STATE[1] = FM_LED_Intensity[1];
				M_STATE[2] = FM_LED_Intensity[2];


				//FLASHLIGHT MODE: PARAM_1 & PARAM_2  ALWAYS 000
				PARAM_1[0] = Flash_Param1[0];
				PARAM_1[1] = Flash_Param1[1];
				PARAM_1[2] = Flash_Param1[2];

				PARAM_2[0] = Flash_Param2[0];
				PARAM_2[1] = Flash_Param2[1];
				PARAM_2[2] = Flash_Param2[2];

				b_Req_Flash = 1;

				break;

				case(69/*'E'*/):	//EMERGANCY-MODE STATUS

	  			b_Req_Flash = 0;
				b_Req_Mood = 0;
				MODE[1] = EM_MODE[1];

				SET_EM_SubMODES();
				M_STATE[0] = EM_LED_Intensity[0];
				M_STATE[1] = EM_LED_Intensity[1];
				M_STATE[2] = EM_LED_Intensity[2];

				PARAM_1[0] = EM_Strobe_OnTime[0];
				PARAM_1[1] = EM_Strobe_OnTime[1];
				PARAM_1[2] = EM_Strobe_OnTime[2];

				PARAM_2[0] = EM_MorseCode[0];
				PARAM_2[1] = EM_MorseCode[1];
				PARAM_2[2] = EM_MorseCode[2];

				b_Req_Emergency = 1;
				break;
				case(77/*'M'*/):	//MOOD-MODE STATUS

	  			b_Req_Flash = 0;
				b_Req_Emergency = 0;

				MODE[1] = MM_MODE[1];

				if(b_RGBLED_State == 1){

					M_STATE[0] = R_Channel_RGB[0];
					M_STATE[1] = R_Channel_RGB[1];
					M_STATE[2] = R_Channel_RGB[2];

					PARAM_1[0] = G_Channel_RGB[0];
					PARAM_1[1] = G_Channel_RGB[1];
					PARAM_1[2] = G_Channel_RGB[2];

					PARAM_2[0] = B_Channel_RGB[0];
					PARAM_2[1] = B_Channel_RGB[1];
					PARAM_2[2] = B_Channel_RGB[2];
				}else
					if(b_RGBLED_State == 0){
						M_STATE[0] = 48;
						M_STATE[1] = 48;
						M_STATE[2] = 48;

						PARAM_1[0] = 48;
						PARAM_1[1] = 48;
						PARAM_1[2] = 48;

						PARAM_2[0] = 48;
						PARAM_2[1] = 48;
						PARAM_2[2] = 48;
					}
				b_Req_Mood = 1;
				break;
				default:
					break;
				}
			}
		}
	}
}

//UART RECEIVE
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	Rx_Message[charCount] = rx_Char[0];
	charCount++;

	if(rx_Char[0] == '\n'){
		b_MsgReceived = 1;
		if(charCount == Max_RxTxMessage_Len){//19 characters received
			b_ModeCOMReceived = 1;
			b_ModeREQReceived = 0;
		}else
			if(charCount == Rx_Request_Mode_Len){//7 characters received
				b_ModeREQReceived = 1;
				b_ModeCOMReceived = 0;
			}
		charCount = 0;
	}

	HAL_UART_Receive_IT(&huart2, rx_Char, 1);//RE-PRIME RECEIVER TO EXPECT 1 BYTE
	b_CharRecieve = 1;
}

void ECHO_RX(){
	HAL_UART_Transmit(&huart2, rx_Char, sizeof(rx_Char), 10);
}

void TX_SendStartMessage(void){

	HAL_UART_Transmit(&huart2, tx_Stdnum, sizeof(tx_Stdnum), 100);

}

void TX_SendCurrentStatus(uint8_t mode[2],uint8_t state[3],uint8_t param_1[3],uint8_t param_2[3]){

	//HAL_UART_Transmit(huart2, pData, Size, Timeout)
	HAL_UART_Transmit(&huart2, (uint8_t *)"#:", 2, 10);
	HAL_UART_Transmit(&huart2, mode, 2, 10);
	HAL_UART_Transmit(&huart2, (uint8_t *)":", 1, 10);
	HAL_UART_Transmit(&huart2, state, 3, 10);
	HAL_UART_Transmit(&huart2, (uint8_t *)":", 1, 10);
	HAL_UART_Transmit(&huart2, param_1, 3, 10);
	HAL_UART_Transmit(&huart2, (uint8_t *)":", 1, 10);
	HAL_UART_Transmit(&huart2, param_2, 3, 10);
	HAL_UART_Transmit(&huart2, (uint8_t *)":$\n", 3, 10);

}
//-------------------------------------------------------------------END UART FUNCTIONS--------------------------------------------------------------------------------------------------------//


// FUNCTION FOR SELECTING A LARGE DELAY - ACCURACY IN ACCEPTABLE RANGE WITHOUT COUNTER
void SetaDelay(uint32_t input_delay){

if(next==1){
next = 0;}
if(first_call==1){
first_call=0;
gendelay_start = HAL_GetTick();
gendelay_check = gendelay_start;}

if((gendelay_check - gendelay_start)>=input_delay){

	next=1;
	first_call=1;
}
else{
	gendelay_check = HAL_GetTick();
}
}
//------------------------------------------------START MORSE FUNCTIONS---------------------------------------//
//FUNCTION FOR DASH
void Dash(void){

if(morse_high==1){
	 SET_DAC_PWM();
			SetaDelay(3000);
			if(next==1){
	morse_high=0;
 }
}
if (morse_high==0){
	 RESET_DAC_PWM();
	SetaDelay(1000);
	if(next==1){
morse_high=1;
cur_element++;
 }
}
}

//FUNCTION FOR DOT

void Dot(void){

	if(morse_high==1){
		SET_DAC_PWM();
		SetaDelay(1000);
		if(next==1){
			morse_high=0;
		}
	}
	if (morse_high==0){
		RESET_DAC_PWM();
		SetaDelay(1000);
		if(next==1){
			morse_high=1;
			cur_element++;
		}
	}
}


//FUNCTION FOR GAP BETWEEN CHARACTERS

void EndCharTimeout(void){

	SetaDelay(2000);
	if(next==1){
		cur_element=0;
		char_count++;
	}
}

//FUNCTION FOR GAP BETWEEN WORDS

void EndWordTimeout(void){

	SetaDelay(4000);
	if(next==1){
		char_count=0;
		cur_element=0;
	}
}

//FUNCTION FOR HANDLING EACH MORSE CHARACTER

void MorseChar(uint8_t inp_char){


	switch(inp_char){

	case 48: //0
		if(cur_element<5){
			Dash();
		}
		if(cur_element==5){
			EndCharTimeout();
		}
		break;

	case 49: //1
		if(cur_element==0){
			Dot();
		}
		if((cur_element>0)&&(cur_element<5)){
			Dash();
		}
		if(cur_element==5){
			EndCharTimeout();
		}
		break;

	case 50: //2
		if(cur_element<2){
			Dot();
		}
		if((cur_element>1)&&(cur_element<5)){
			Dash();
		}
		if(cur_element==5){
			EndCharTimeout();
		}
		break;

	case 51: //3
		if(cur_element<3){
			Dot();
		}
		if((cur_element>2)&&(cur_element<5)){
			Dash();
		}
		if(cur_element==5){
			EndCharTimeout();
		}
		break;

	case 52: //4
		if(cur_element<4){
			Dot();
		}
		if((cur_element>3)&&(cur_element<5)){
			Dash();
		}
		if(cur_element==5){
			EndCharTimeout();
		}
		break;

	case 53: //5
		if(cur_element<5){
			Dot();
		}
		if(cur_element==5){
			EndCharTimeout();
		}
		break;

	case 54: //6
		if(cur_element==0){
			Dash();
		}
		if((cur_element>0)&&(cur_element<5)){
			Dot();
		}
		if(cur_element==5){
			EndCharTimeout();
		}
		break;

	case 55: //7
		if(cur_element<2){
			Dash();
		}
		if((cur_element>1)&&(cur_element<5)){
			Dot();
		}
		if(cur_element==5){
			EndCharTimeout();
		}
		break;

	case 56: //8
		if(cur_element<3){
			Dash();
		}
		if((cur_element>2)&&(cur_element<5)){
			Dot();
		}
		if(cur_element==5){
			EndCharTimeout();
		}
		break;

	case 57: //9
		if(cur_element<4){
			Dash();
		}
		if((cur_element>3)&&(cur_element<5)){
			Dot();
		}
		if(cur_element==5){
			EndCharTimeout();
		}
		break;

	case 65: //A
		if(cur_element==0){
			Dot();
		}
		if(cur_element==1){
			Dash();
		}
		if(cur_element==2){
			EndCharTimeout();
		}
		break;

	case 66: //B
		if(cur_element==0){
			Dash();
		}
		if((cur_element>0)&&(cur_element<4)){
			Dot();
		}
		if(cur_element==4){
			EndCharTimeout();
		}
		break;

	case 67: //C
		if(cur_element==0){
			Dash();
		}
		if(cur_element==1){
			Dot();
		}
		if(cur_element==2){
			Dash();
		}
		if(cur_element==3){
			Dot();
		}
		if(cur_element==4){
			EndCharTimeout();
		}
		break;

	case 68: //D
		if(cur_element==0){
			Dash();
		}
		if((cur_element<3)&&(cur_element>0)){
			Dot();
		}
		if(cur_element==3){
			EndCharTimeout();
		}
		break;

	case 69: //E
		if(cur_element==0){
			Dot();
		}
		if(cur_element==1){
			EndCharTimeout();
		}
		break;

	case 70: //F
		if(cur_element<2){
			Dot();
		}
		if(cur_element==2){
			Dash();
		}
		if(cur_element==3){
			Dot();
		}
		if(cur_element==4){
			EndCharTimeout();
		}
		break;

	case 71: //G
		if(cur_element<2){
			Dash();
		}
		if(cur_element==2){
			Dot();
		}
		if(cur_element==3){
			EndCharTimeout();
		}

		break;

	case 72: //H
		if(cur_element<4){
			Dot();
		}
		if(cur_element==4){
			EndCharTimeout();
		}
		break;

	case 73: //I
		if(cur_element<2){
			Dot();
		}
		if(cur_element==2){
			EndCharTimeout();
		}
		break;

	case 74: //J
		if(cur_element==0){
			Dot();
		}
		if((cur_element>0)&&(cur_element<4)){
			Dash();
		}
		if(cur_element==4){
			EndCharTimeout();
		}
		break;

	case 75: //K
		if(cur_element==0){
			Dash();
		}
		if(cur_element==1){
			Dot();
		}
		if(cur_element==2){
			Dash();
		}
		if(cur_element==3){
			EndCharTimeout();
		}
		break;

	case 76: //L
		if(cur_element==0){
			Dot();
		}
		if(cur_element==1){
			Dash();
		}
		if((cur_element>1)&&(cur_element<4)){
			Dot();
		}
		if(cur_element==4){
			EndCharTimeout();
		}
		break;

	case 77: //M
		if(cur_element<2){
			Dash();
		}
		if(cur_element==2){
			EndCharTimeout();
		}
		break;

	case 78: //N
		if(cur_element==0){
			Dash();
		}
		if(cur_element==1){
			Dot();
		}
		if(cur_element==2){
			EndCharTimeout();
		}
		break;

	case 79: //O
		if(cur_element<3){
			Dash();
		}
		if(cur_element==3){
			EndCharTimeout();
		}
		break;

	case 80: //P
		if(cur_element==0){
			Dot();
		}
		if((cur_element>0)&&(cur_element<3)){
			Dash();
		}
		if(cur_element==3){
			Dot();
		}
		if(cur_element==4){
			EndCharTimeout();
		}
		break;

	case 81: //Q
		if(cur_element<2){
			Dash();
		}
		if(cur_element==2){
			Dot();
		}
		if(cur_element==3){
			Dash();
		}
		if(cur_element==4){
			EndCharTimeout();
		}
		break;

	case 82: //R
		if(cur_element==0){
			Dot();
		}
		if(cur_element==1){
			Dash();
		}
		if(cur_element==2){
			Dot();
		}
		if(cur_element==3){
			EndCharTimeout();
		}
		break;

	case 83: //S
		if(cur_element<3){
			Dot();
		}
		if(cur_element==3){
			EndCharTimeout();
		}
		break;

	case 84: //T
		if(cur_element==0){
			Dash();
		}
		if(cur_element==1){
			EndCharTimeout();
		}
		break;

	case 85: //U
		if(cur_element<2){
			Dot();
		}
		if(cur_element==2){
			Dash();
		}
		if(cur_element==3){
			EndCharTimeout();
		}
		break;

	case 86: //V
		if(cur_element<3){
			Dot();
		}
		if(cur_element==3){
			Dash();
		}
		if(cur_element==4){
			EndCharTimeout();
		}
		break;

	case 87: //W
		if(cur_element==0){
			Dot();
		}
		if((cur_element>0)&&(cur_element<3)){
			Dash();
		}
		if(cur_element==3){
			EndCharTimeout();
		}
		break;

	case 88: //X
		if(cur_element==0){
			Dash();
		}
		if((cur_element>0)&&(cur_element<3)){
			Dot();
		}
		if(cur_element==3){
			Dash();
		}
		if(cur_element==4){
			EndCharTimeout();
		}
		break;

	case 89: //Y
		if(cur_element==0){
			Dash();
		}
		if(cur_element==1){
			Dot();
		}
		if((cur_element>1)&&(cur_element<4)){
			Dash();
		}
		if(cur_element==4){
			EndCharTimeout();
		}
		break;

	case 90: //Z
		if(cur_element<2){
			Dash();
		}
		if((cur_element>1)&&(cur_element<4)){
			Dot();
		}
		if(cur_element==4){
			EndCharTimeout();
		}
		break;


	}

}
//FUNCTION TO HANDLE MORSE TRANSMISSION

void MorseTransmit(uint8_t inpArr[3]){

	switch(char_count){

	case 0:

		MorseChar(inpArr[0]);
		break;

	case 1:

		MorseChar(inpArr[1]);
		break;

	case 2:
		MorseChar(inpArr[2]);
		break;

	case 3:
		EndWordTimeout();
		break;
	}

}
//-----------------------------------------------END MORSE FUNCTIONS---------------------------------------//


//-----------------------------------------------START ADC FUNCTIONS---------------------------------------//
uint16_t Get_AVG_ADC_Value(){

	for (int a_sample = 0; a_sample < ADC_Array_len; ++a_sample) {
		ADC_raw_MEAS_VAL[a_sample] = HAL_ADC_GetValue(&hadc1);

	}
	for (int b_sample = 10; b_sample < ADC_Array_len; ++b_sample) {
		ADC_Total_VAL = ADC_Total_VAL + ADC_raw_MEAS_VAL[b_sample];
	}

	ADC_avg_MEAS_VAL = ADC_Total_VAL / (ADC_Array_len - 10);

	return ADC_avg_MEAS_VAL;

}

uint16_t Get_ADC_Voltage(uint16_t ADC_AVG){

	ADC_Vin = (ADC_AVG*3330)/(4095);
	return ADC_Vin;
}

uint16_t Get_Callibrated_ADC(uint16_t ADC_AVG){

	ADC_CALIBRATION_VAL = (ADC_AVG*512)/(4095) + 1;
	return ADC_CALIBRATION_VAL;
}

void Transmit_ADC(uint16_t ADC_Val){

	adc_outlen = sprintf(Output,"%d\n", ADC_Val);
	HAL_Delay(10);
	HAL_UART_Transmit(&huart2, (uint8_t *)Output, adc_outlen, 10);

}

void Clear_Adc_Values(){
	for (int y = 0; y < ADC_Array_len; ++y) {
		ADC_raw_MEAS_VAL[y] = 0;
	}
	ADC_avg_MEAS_VAL = 0;
	ADC_Total_VAL = 0;
	ADC_Vin = 0;
	ADC_CALIBRATION_VAL = 0;

}

void Look_Slider_Change(){
	if((callibrated_ADC - OLD_ADC) > 20){//CHECK ID SLIDER MOVES LEFT TO RIGHT "UP"
		OLD_ADC = callibrated_ADC;
		b_Slider_Up = 1;
		b_Slider_Down = 0;
		b_Slider_ADC_Intensity = 1;
		b_UART_Intensity = 0;
		b_TRACK_Intensity = 0;

	}else
		if((callibrated_ADC - OLD_ADC) < (-20)){//CHECK IF SLIDER MOVES RIGHT TO LEFT "DOWN"
			OLD_ADC = callibrated_ADC;
			b_Slider_Up = 0;
			b_Slider_Down = 1;
			b_Slider_ADC_Intensity = 1;
			b_UART_Intensity = 0;
			b_TRACK_Intensity = 0;
		}

}
//-----------------------------------------------END ADC FUNCTIONS---------------------------------------//


//-----------------------------------------------START DAC/PWM FUNCTIONS---------------------------------------//
void SET_DAC_PWM(){
	if(b_FLASH_MODE){
		Get_Adc_For_Dac[0] = FM_LED_Intensity[0] - 48;
		Get_Adc_For_Dac[1] = FM_LED_Intensity[1] - 48;
		Get_Adc_For_Dac[2] = FM_LED_Intensity[2] - 48;

		Dac_Val = ((Get_Adc_For_Dac[0]*100)+(Get_Adc_For_Dac[1]*10)+(Get_Adc_For_Dac[2]))/2;
		if(b_WhiteLED_State){
			if(Dac_Val == 256){
				Dac_Val = 255;
				SET_DAC_PWM_ON();
			}else
				if(Dac_Val < 2){
					Dac_Val = 1;
					RESET_DAC_PWM();
				}
			SET_DAC_PWM_ON();
		}else
			if(!b_WhiteLED_State){
				SET_DAC_PWM_OFF();
			}
	}else
		if(b_EMERGENCY_MODE){
			Get_Adc_For_Dac[0] = EM_LED_Intensity[0] - 48;
			Get_Adc_For_Dac[1] = EM_LED_Intensity[1] - 48;
			Get_Adc_For_Dac[2] = EM_LED_Intensity[2] - 48;

			Dac_Val = ((Get_Adc_For_Dac[0]*100)+(Get_Adc_For_Dac[1]*10)+(Get_Adc_For_Dac[2]))/2;
			if(b_WhiteLED_State){
				if(Dac_Val == 256){
					Dac_Val = 255;
					SET_DAC_PWM_ON();
				}else
					if(Dac_Val < 2){
						Dac_Val = 1;
						RESET_DAC_PWM();
					}
				SET_DAC_PWM_ON();
			}else
				if(!b_WhiteLED_State){
					SET_DAC_PWM_OFF();
				}
		}
}

void SET_DAC_PWM_ON(){
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R,Dac_Val);
	htim3.Instance->CCR1 = Dac_Val*2;
}


void SET_DAC_PWM_OFF(){
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R,0);
	htim3.Instance->CCR1 = 0;
}

void RESET_DAC_PWM(){
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_8B_R,1);
	htim3.Instance->CCR1 = 1;
}

//-----------------------------------------------END DAC/PWM FUNCTIONS---------------------------------------//


//-----------------------------------------------START MOOD MODE FUNCTIONS---------------------------------------//
void SET_RGB_ON(){
	if(b_MOOD_MODE){
		if(b_RGBLED_State){
			//RED CHANNEL
			Get_R[0] = R_Channel_RGB[0] - 48;
			Get_R[1] = R_Channel_RGB[1] - 48;
			Get_R[2] = R_Channel_RGB[2] - 48;

			Red_Val = ((Get_R[0]*100)+(Get_R[1]*10)+(Get_R[2]))/2;

			if((Red_Val == 256)){
				Red_Val = 255;
				htim3.Instance->CCR2 = Red_Val*2;
			}else
				if(Red_Val < 2){
					Red_Val = 1;
					htim3.Instance->CCR2 = Red_Val*2;
				}
			htim3.Instance->CCR2 = Red_Val*2;

			//GREEN CHANNEL
			Get_G[0] = G_Channel_RGB[0] - 48;
			Get_G[1] = G_Channel_RGB[1] - 48;
			Get_G[2] = G_Channel_RGB[2] - 48;

			Green_Val = ((Get_G[0]*100)+(Get_G[1]*10)+(Get_G[2]))/2;

			if((Green_Val == 256)){
				Green_Val = 255;
				htim3.Instance->CCR3 = Green_Val*2;
			}else
				if(Green_Val < 2){
					Green_Val = 1;
					htim3.Instance->CCR3 = Green_Val*2;
				}
			htim3.Instance->CCR3 = Green_Val*2;

			//BLUE CHANNEL
			Get_B[0] = B_Channel_RGB[0] - 48;
			Get_B[1] = B_Channel_RGB[1] - 48;
			Get_B[2] = B_Channel_RGB[2] - 48;

			Blue_Val = ((Get_B[0]*100)+(Get_B[1]*10)+(Get_B[2]))/2;
			if((Blue_Val == 256)){
				Blue_Val = 255;
				htim3.Instance->CCR4 = Blue_Val*2;
			}else
				if(Blue_Val < 2){
					Blue_Val = 1;
					htim3.Instance->CCR4 = Blue_Val*2;
				}
			htim3.Instance->CCR4 = Blue_Val*2;


		}else
			if(!b_RGBLED_State){
				htim3.Instance->CCR2 = 0;
				htim3.Instance->CCR3 = 0;
				htim3.Instance->CCR4 = 0;
			}
	}
}

void SET_RGB_Intensity(uint16_t X_POS,uint16_t Y_POS){

	//uint16_t New_Y;
	Y_POS = Track_Y_Max - Y_POS;
	Y_POS = (Y_POS*512)/Track_Y_Max + 1;
	if(Y_POS <= 40){
		Y_POS = 0;
	}
	if(Y_POS >= 480){
		Y_POS = 512;
	}
	if(Y_POS == 0){
		Y_POS = 1;
	}
	//Y_POS = SET_Callibrated_TrackIntensity(Y_POS);
	if((X_POS >= 20) && (X_POS < 566 )){
		b_RTrack = 1;
		R_Channel_RGB[2] = ((uint8_t)(Y_POS)%10) + 48 ;
		R_Channel_RGB[1] = ((uint8_t)(Y_POS/10)%10) + 48 ;
		R_Channel_RGB[0] = (uint8_t)(Y_POS/100)%10 + 48 ;

//		HAL_UART_Transmit(&huart2, R_Channel_RGB, 3, 10);
	}else
		if((X_POS > 566) && (X_POS < 1132 )){
			b_GTrack = 1;
			G_Channel_RGB[2] = ((uint8_t)(Y_POS)%10) + 48 ;
			G_Channel_RGB[1] = ((uint8_t)(Y_POS/10)%10) + 48 ;
			G_Channel_RGB[0] = ((uint8_t)(Y_POS/100)%10) + 48 ;
//			HAL_UART_Transmit(&huart2, G_Channel_RGB, 3, 10);
		}else
			if((X_POS > 1132) && (X_POS <= 1700 )){
				b_BTrack = 1;
				B_Channel_RGB[2] = ((uint8_t)(Y_POS)%10) + 48 ;
				B_Channel_RGB[1] = ((uint8_t)(Y_POS/10)%10) + 48 ;
				B_Channel_RGB[0] = ((uint8_t)(Y_POS/100)%10) + 48 ;
//				HAL_UART_Transmit(&huart2, B_Channel_RGB, 3, 10);
			}

}
uint16_t SET_Callibrated_TrackIntensity(uint16_t Y_POSITION){

	//Y_POSITION = Track_Y_Max - Track_Y;
	Callibrated_Y_POS = (Y_POSITION/Track_Y_Max)*(512);
	if(Callibrated_Y_POS <= 0){
		Callibrated_Y_POS = 0;
	}
	if(Callibrated_Y_POS >= 490){
		Callibrated_Y_POS = 512;
	}
	return Callibrated_Y_POS;
}

void SET_Default_RGB_Values(){

	if((b_RGBLED_State == 1) && (b_MOOD_MODE == 1)){
		if((b_RSET == 0)){
			R_Channel_RGB[0] = 49;//'1'
			R_Channel_RGB[1] = 50;//'2'
			R_Channel_RGB[2] = 56;//'8'
		}else
			if((b_RSET == 1)){
				R_Channel_RGB[0] = Rx_Message[5];
				R_Channel_RGB[1] = Rx_Message[6];
				R_Channel_RGB[2] = Rx_Message[7];
		}
		if((b_GSET == 0)){
			G_Channel_RGB[0] = 49;
			G_Channel_RGB[1] = 50;
			G_Channel_RGB[2] = 56;
		}else
			if((b_GSET == 1))
		{
				G_Channel_RGB[0] = Rx_Message[9];
				G_Channel_RGB[1] = Rx_Message[10];
				G_Channel_RGB[2] = Rx_Message[11];
		}
		if((b_BSET == 0)){
			B_Channel_RGB[0] = 49;
			B_Channel_RGB[1] = 50;
			B_Channel_RGB[2] = 56;
		}else
			if((b_BSET == 1))
		{
				B_Channel_RGB[0] = Rx_Message[13];
				B_Channel_RGB[1] = Rx_Message[14];
				B_Channel_RGB[2] = Rx_Message[15];
		}
	}else
		if((b_RGBLED_State == 0) && (b_MOOD_MODE == 1)){

			SET_MM_RGB_OFF();
		}
	b_RSET = 0;
	b_GSET = 0;
	b_BSET = 0;

}

void SET_MM_RGB_ON(){

	R_Channel_RGB[0] = R_Channel_RGB[0];
	R_Channel_RGB[1] = R_Channel_RGB[1];
	R_Channel_RGB[2] = R_Channel_RGB[2];

	G_Channel_RGB[0] = G_Channel_RGB[0];
	G_Channel_RGB[1] = G_Channel_RGB[1];
	G_Channel_RGB[2] = G_Channel_RGB[2];

	B_Channel_RGB[0] = B_Channel_RGB[0];
	B_Channel_RGB[1] = B_Channel_RGB[1];
	B_Channel_RGB[2] = B_Channel_RGB[2];
}

void SET_MM_RGB_OFF(){

	htim3.Instance->CCR2 = 0;
	R_Channel_RGB[0] = 48;
	R_Channel_RGB[1] = 48;
	R_Channel_RGB[2] = 48;

	htim3.Instance->CCR3 = 0;
	G_Channel_RGB[0] = 48;
	G_Channel_RGB[1] = 48;
	G_Channel_RGB[2] = 48;

	htim3.Instance->CCR4 = 0;
	B_Channel_RGB[0] = 48;
	B_Channel_RGB[1] = 48;
	B_Channel_RGB[2] = 48;
}
//-----------------------------------------------END MOOD MODE FUNCTIONS---------------------------------------//

//-------------------------------------------START FM/EM MODE FUNCTIONS----------------------------------------//
//Send Calibrated ADC 001 - 512 as argument to function
void Slider_LED_Intensity(uint16_t Slider_Intensity){

	if(b_WhiteLED_State){
		if(b_FLASH_MODE){
			FM_LED_Intensity[2] = (uint8_t)(Slider_Intensity)%10 + 48;
			FM_LED_Intensity[1] = (uint8_t)(Slider_Intensity/10)%10 + 48;
			FM_LED_Intensity[0] = (uint8_t)(Slider_Intensity/100)%10 + 48;

		}else
			if(b_EMERGENCY_MODE){
				EM_LED_Intensity[2] = (uint8_t)(Slider_Intensity)%10 + 48;
				EM_LED_Intensity[1] = (uint8_t)(Slider_Intensity/10)%10 + 48;
				EM_LED_Intensity[0] = (uint8_t)(Slider_Intensity/100)%10 + 48;

			}
	}else
		if(!b_WhiteLED_State){

			if(b_FLASH_MODE){
				SET_FM_MODE_OFF();
			}else
				if(b_EMERGENCY_MODE){
					SET_EM_MODE_OFF();
				}
		}
}

void Uart_LED_Intensity(uint8_t Uart_FM_Intensity[3],uint8_t Uart_EM_Intensity[3]){

	if(b_FLASH_MODE){
		FM_LED_Intensity[0] = Uart_FM_Intensity[0];
		FM_LED_Intensity[1] = Uart_FM_Intensity[1];
		FM_LED_Intensity[2] = Uart_FM_Intensity[2];
		//SET_DAC_PWM();

	}else
		if(b_EMERGENCY_MODE){
			EM_LED_Intensity[0] = Uart_EM_Intensity[0];
			EM_LED_Intensity[1] = Uart_EM_Intensity[1];
			EM_LED_Intensity[2] = Uart_EM_Intensity[2];
			//SET_DAC_PWM();
		}
}

void SET_LED_Intensity(){


	if(b_WhiteLED_State == 0){
		if(b_FLASH_MODE){
			SET_FM_MODE_OFF();
			SET_DAC_PWM();
		}
		if(b_EMERGENCY_MODE){
			SET_EM_MODE_OFF();
		}
	}else
		if(b_WhiteLED_State == 1){
			if(b_FLASH_MODE){
				if(b_UART_Intensity){
					Uart_LED_Intensity(FM_LED_Intensity,EM_LED_Intensity);
				}else
					if(b_Slider_ADC_Intensity){
						Slider_LED_Intensity(callibrated_ADC);
					}
				SET_DAC_PWM();
			}else
				if(b_EMERGENCY_MODE){
					if(b_UART_Intensity){
						Uart_LED_Intensity(FM_LED_Intensity,EM_LED_Intensity);
						//SET_EM_SubMODES();
					}else
						if(b_Slider_ADC_Intensity){
							Slider_LED_Intensity(callibrated_ADC);
							//SET_EM_SubMODES();
						}
				}

		}
}

void TOGGLE_LEDS(){
	if((b_WhiteLED_State == 1)){
		if(b_FLASH_MODE){
			b_WhiteLED_State = 0;
			SET_FM_MODE_OFF();
		}
		if(b_EMERGENCY_MODE){
			b_WhiteLED_State = 0;
			SET_EM_MODE_OFF();
		}
		SET_DAC_PWM();

	}else if((b_WhiteLED_State == 0)){
		if(b_FLASH_MODE){
			b_WhiteLED_State = 1;
			b_UART_Intensity = 0;
			b_TRACK_Intensity = 0;
			b_Slider_ADC_Intensity = 1;
			SET_LED_Intensity();
		}
		if(b_EMERGENCY_MODE){
			b_WhiteLED_State = 1;
			b_UART_Intensity = 0;
			b_TRACK_Intensity = 0;
			b_Slider_ADC_Intensity = 1;
			SET_LED_Intensity();
		}
		SET_DAC_PWM();
	}

	if(b_RGBLED_State == 1){
		if(b_MOOD_MODE){
			b_RGBLED_State = 0;
			SET_Default_RGB_Values();
			SET_RGB_ON();
		}
	}else
		if(b_RGBLED_State == 0){
			if(b_MOOD_MODE){
				b_RGBLED_State = 1;
				SET_Default_RGB_Values();
				SET_RGB_ON();
			}
		}
}

//MIDDLE BUTTON ON/OFF IN EACH MODE
void SET_FM_MODE_ON(){
	FM_LED_Intensity[0] = FM_LED_Intensity[0];
	FM_LED_Intensity[1] = FM_LED_Intensity[1];
	FM_LED_Intensity[2] = FM_LED_Intensity[2];
}

void SET_FM_PARAMS(){

	Flash_Param1[0] = 48;
	Flash_Param1[1] = 48;
	Flash_Param1[2] = 48;

	Flash_Param2[0] = 48;
	Flash_Param2[1] = 48;
	Flash_Param2[2] = 48;
}

void SET_EM_MODE_ON(){
	EM_LED_Intensity[0] = EM_LED_Intensity[0];
	EM_LED_Intensity[1] = EM_LED_Intensity[1];
	EM_LED_Intensity[2] = EM_LED_Intensity[2];
}

void Cycle_EM_SubMODES(){
	if( (b_EMERGENCY_MODE == 1) ){//&& (HAL_GPIO_ReadPin(LED3_GPIO_Port, LED3_Pin) == GPIO_PIN_SET)
		if(b_EM_Strobe == 1){
			b_EM_SOS = 1;
			b_EM_Strobe = 0;
			b_EM_CustomMsg = 0;
			b_Right_SOS = 1;
			b_Right_Strobe = 0;
			b_Right_Morse = 0;

			//EM_SOS_Msg(b_Right_SOS);
			HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
		}else
			if(b_EM_SOS == 1){
				b_EM_CustomMsg = 1;
				b_EM_SOS = 0;
				b_EM_Strobe = 0;
				b_Right_Morse = 1;
				b_Right_SOS = 0;
				b_Right_Strobe = 0;

				//EM_MORSE_Msg(b_UART_Morse,b_Right_Morse);
				HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_SET);
			}else
				if(b_EM_CustomMsg == 1){
					b_EM_Strobe = 1;
					b_EM_CustomMsg = 0;
					b_EM_SOS = 0;
					b_Right_Strobe = 1;
					b_Right_Morse = 0;
					b_Right_SOS = 0;

					//EM_STROBE_Msg(b_Right_Strobe);
					HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
				}
	}
}

void SET_EM_SubMODES(){

	if(b_EM_Strobe){
		if((b_UART_Strobe)){
			EM_Strobe_OnTime[0] = EM_Strobe_OnTime[0];
			EM_Strobe_OnTime[1] = EM_Strobe_OnTime[1];
			EM_Strobe_OnTime[2] = EM_Strobe_OnTime[2];
			EM_MorseCode[0] = 'S';
			EM_MorseCode[1] = 'O';
			EM_MorseCode[2] = 'S';
		}else
			if(!b_UART_Strobe){
				//EM_Strobe_OnTime[3] = "512"; Default STrobe OnTime
				EM_Strobe_OnTime[0] = '5';
				EM_Strobe_OnTime[1] = '1';
				EM_Strobe_OnTime[2] = '2';
				EM_MorseCode[0] = 'S';
				EM_MorseCode[1] = 'O';
				EM_MorseCode[2] = 'S';
			}
	}else
		if(b_EM_SOS){
				if(b_UART_SOS){
					EM_Strobe_OnTime[0] = 48;
					EM_Strobe_OnTime[1] = 48;
					EM_Strobe_OnTime[2] = 48;
					EM_MorseCode[0] = 'S';
					EM_MorseCode[1] = 'O';
					EM_MorseCode[2] = 'S';
				}else
					if(!b_UART_SOS){
						//EM_Strobe_OnTime[3] = "512"; Default STrobe OnTime
						EM_Strobe_OnTime[0] = 48;
						EM_Strobe_OnTime[1] = 48;
						EM_Strobe_OnTime[2] = 48;
						EM_MorseCode[0] = 'S';
						EM_MorseCode[1] = 'O';
						EM_MorseCode[2] = 'S';
					}
		}else
			if(b_EM_CustomMsg){
				if(b_UART_Morse){
					EM_Strobe_OnTime[0] = 48;
					EM_Strobe_OnTime[1] = 48;
					EM_Strobe_OnTime[2] = 48;
					EM_MorseCode[0] = EM_MorseCode[0];
					EM_MorseCode[1] = EM_MorseCode[1];
					EM_MorseCode[2] = EM_MorseCode[2];
				}else
					if(!b_UART_Morse){
						EM_Strobe_OnTime[0] = 48;
						EM_Strobe_OnTime[1] = 48;
						EM_Strobe_OnTime[2] = 48;
						EM_MorseCode[0] = 'S';
						EM_MorseCode[1] = 'O';
						EM_MorseCode[2] = 'S';
					}
			}
}

//SET ARR - STROBE SUBMODE
void SET_STROBE_ARR(){

	STROBE_ARR[0] = EM_Strobe_OnTime[0] - 48;
	STROBE_ARR[1] = EM_Strobe_OnTime[1] - 48;
	STROBE_ARR[2] = EM_Strobe_OnTime[2] - 48;

	strobe_delay = (((STROBE_ARR[0]*100)+(STROBE_ARR[1]*10)+(STROBE_ARR[2]))*10) - 1;

	htim16.Instance->ARR = strobe_delay;

}

//CALLBACK FUNCTION TIM16 - STROBE MODE
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if((htim==&htim16)&&(b_EM_Strobe)&&(b_EMERGENCY_MODE)&&(b_WhiteLED_State)){
		htim16.Instance->ARR = strobe_delay;
		if(strobe_on==0){
			SET_DAC_PWM();
			strobe_on=1;}
		else if(strobe_on==1){
			RESET_DAC_PWM();
			strobe_on=0;}
	}
}

void EM_STROBE_Msg(uint8_t strobe_set){

	if(strobe_set){
		if((!b_UART_Strobe)){
			EM_Strobe_OnTime[0] = '5';
			EM_Strobe_OnTime[1] = '1';
			EM_Strobe_OnTime[2] = '2';
			EM_MorseCode[0] = 'S';
			EM_MorseCode[1] = 'O';
			EM_MorseCode[2] = 'S';
		}else
			if((b_UART_Strobe)){
				EM_Strobe_OnTime[0] = EM_Strobe_OnTime[0];
				EM_Strobe_OnTime[1] = EM_Strobe_OnTime[1];
				EM_Strobe_OnTime[2] = EM_Strobe_OnTime[2];
				EM_MorseCode[0] = 'S';
				EM_MorseCode[1] = 'O';
				EM_MorseCode[2] = 'S';
			}
	}
//	}else
//		if(!strobe_set){
//			if((b_WhiteLED_State)){
//				EM_Strobe_OnTime[0] = 48;
//				EM_Strobe_OnTime[1] = 48;
//				EM_Strobe_OnTime[2] = 48;
//				EM_MorseCode[0] = '0';
//				EM_MorseCode[1] = '0';
//				EM_MorseCode[2] = '0';
//			}else
//				if((!b_WhiteLED_State)){
//					EM_Strobe_OnTime[0] = 48;
//					EM_Strobe_OnTime[1] = 48;
//					EM_Strobe_OnTime[2] = 48;
//					EM_MorseCode[0] = '0';
//					EM_MorseCode[1] = '0';
//					EM_MorseCode[2] = '0';
//				}
//		}
}

void EM_SOS_Msg(uint8_t SOS_Receive){

	if((SOS_Receive)){
		EM_Strobe_OnTime[0] = '0';
		EM_Strobe_OnTime[1] = '0';
		EM_Strobe_OnTime[2] = '0';
		EM_MorseCode[0] = 'S';
		EM_MorseCode[1] = 'O';
		EM_MorseCode[2] = 'S';
	}


}
void EM_MORSE_Msg(uint8_t Custom_Morse_Send,uint8_t Right_Morse){

	EM_LED_Intensity[0] = EM_LED_Intensity[0];
	EM_LED_Intensity[1] = EM_LED_Intensity[1];
	EM_LED_Intensity[2] = EM_LED_Intensity[2];

	if((Custom_Morse_Send == 0) && (Right_Morse == 1)){
		EM_Strobe_OnTime[0] = '0';
		EM_Strobe_OnTime[1] = '0';
		EM_Strobe_OnTime[2] = '0';
		EM_MorseCode[0] = 'S';
		EM_MorseCode[1] = 'O';
		EM_MorseCode[2] = 'S';
//		EM_MorseCode[0] = EM_MorseCode[0];
//		EM_MorseCode[1] = EM_MorseCode[1];
//		EM_MorseCode[2] = EM_MorseCode[2];
	}else
		if((Custom_Morse_Send == 1) &&  (Right_Morse == 1)){
			EM_Strobe_OnTime[0] = '0';
			EM_Strobe_OnTime[1] = '0';
			EM_Strobe_OnTime[2] = '0';
			EM_MorseCode[0] = EM_MorseCode[0];
			EM_MorseCode[1] = EM_MorseCode[1];
			EM_MorseCode[2] = EM_MorseCode[2];
		}

}

void SET_FM_MODE_OFF(){
	FM_LED_Intensity[0] = 48;
	FM_LED_Intensity[1] = 48;
	FM_LED_Intensity[2] = 48;

}

void SET_EM_MODE_OFF(){
	EM_LED_Intensity[0] = 48;
	EM_LED_Intensity[1] = 48;
	EM_LED_Intensity[2] = 48;
}

//----------------------------------------END FM/EM MODE SET FUNCTIONS----------------------------------//

//----------------------------------------START TRACKPAD FUNCTIONS----------------------------------//
void TRACK_INITIALIZE(){


		//HW RESET
		HAL_GPIO_WritePin(GPIOB, MCLR_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(GPIOB, MCLR_Pin, GPIO_PIN_SET);

		//ACK
		acknowledgeReset();

		//Write to TRACKPAD registers
		Trackpad_writeMM();

		//START ATI ROUTINE
		TP_ReATI();

}

void CHECK_TRACK(){
	if(RDY_LOW == 1){
		while(RDY_LOW==0){
			continue;
		}
		RDY_LOW = 0;
		//HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)
		HAL_I2C_Mem_Read(&hi2c1, (0x56<<1), IQS7211A_MM_GESTURES, I2C_MEMADD_SIZE_8BIT, GestureReceive, 10, 10);

		//CONDITION IF TAP IS DETECTED
		if((((GestureReceive[0] & 0x01) == 0x01) && (b_EMERGENCY_MODE)) || (((GestureReceive[0] & 0x01) == 0x01) && (b_MOOD_MODE))){
			b_Track_TAP = 1;
			X_LSB = GestureReceive[6];
			X_MSB = GestureReceive[7];
			Y_LSB = GestureReceive[8];
			Y_MSB = GestureReceive[9];
			Track_X = getAbsXCoordinate();
			Track_Y = getAbsYCoordinate();
		}

		//CONDITION IF HOLD IS DETECTED
		if(((GestureReceive[0] & 0x02) == 0x02) && (!b_Track_FIRST_HOLD)){
			b_Track_FIRST_HOLD = 1;
			X_LSB = GestureReceive[6];
			X_MSB = GestureReceive[7];
			Track_X = getAbsXCoordinate();
			Track_INIT_X = Track_X;
		}

		//CONDITION IF RELEASE HAS BEEN DETECTED
		if((b_Track_FIRST_HOLD) && ((GestureReceive[0] & 0x02) != 0x02)){
			X_LSB = GestureReceive[6];
			X_MSB = GestureReceive[7];
			Track_X = getAbsXCoordinate();
			Track_FINAL_X = Track_X;
			b_Track_HOLD = 1;

			b_Track_FIRST_HOLD = 0;
		}

	}
}
uint16_t getAbsXCoordinate()
{
	uint16_t absXCoordReturn = 0;     // The 16bit return value.
	//char xCoordString[10];

	absXCoordReturn = X_LSB;
	absXCoordReturn |= (X_MSB << 8);

	//sprintf(xCoordString,"X coordinate: %u ", absXCoordReturn);

	//HAL_UART_Transmit(&huart2, (uint8_t *)xCoordString, strlen(xCoordString), 10);

	return absXCoordReturn;
}

uint16_t getAbsYCoordinate()
{
	uint16_t absYCoordReturn = 0;     // The 16bit return value.

	absYCoordReturn = Y_LSB;
	absYCoordReturn |= (Y_MSB << 8);


	return absYCoordReturn;
}
//----------------------------------------START TRACKPAD FUNCTIONS----------------------------------//

//--------------------------------------START EXTI (...it.c) FUNCTIONS------------------------------//
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(B1_Pin);
  HAL_GPIO_EXTI_IRQHandler(RDY_Pin);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  RDY_LOW = 1;

  /* USER CODE END EXTI15_10_IRQn 1 */
}

void EXTI9_5_IRQHandler(void)
{
	/* USER CODE BEGIN EXTI9_5_IRQn 0 */

	if (EXTI->PR & EXTI_PR_PR7){
		if (ButtonPressed[0] == 0){ //Middle Button Press
			ButtonPressed[0] = 1;
		}
		EXTI->PR |= EXTI_PR_PR7;
	}else
		if (EXTI->PR & EXTI_PR_PR9){ //Left Button Press
			if (ButtonPressed[1] == 0){
				ButtonPressed[1] = 1;
			}
			EXTI->PR |= EXTI_PR_PR9;
		}else
			if (EXTI->PR & EXTI_PR_PR6){ //Right Button Press
				if (ButtonPressed[2] == 0){
					ButtonPressed[2] = 1;
				}
				EXTI->PR |= EXTI_PR_PR6;
			}

	/* USER CODE END EXTI9_5_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(Right_Button_Pin);
	HAL_GPIO_EXTI_IRQHandler(Middle_Button_Pin);
	HAL_GPIO_EXTI_IRQHandler(Left_Button_Pin);
	/* USER CODE BEGIN EXTI9_5_IRQn 1 */
	//////////////
	/* USER CODE END EXTI9_5_IRQn 1 */
}
void SysTick_Handler(void)
{
	/* USER CODE BEGIN SysTick_IRQn 0 */

	//DEBOUNCE CODE
	for(i = 0; i < 5; i++){
		if(ButtonPressed[i] != 0){
			if(Threshold_Num[i] < 20){
				Threshold_Num[i]++;
				switch(i){
				case(0)://Middle Button
									 if(HAL_GPIO_ReadPin(Middle_Button_GPIO_Port, Middle_Button_Pin) != GPIO_PIN_SET){
										 ButtonPressCounter[i]++;
									 }
				break;
				case(1)://Left Button
									 if(HAL_GPIO_ReadPin(Left_Button_GPIO_Port, Left_Button_Pin) != GPIO_PIN_SET){
										 ButtonPressCounter[i]++;
									 }
				break;
				case(2)://Right Button
									 if(HAL_GPIO_ReadPin(Right_Button_GPIO_Port, Right_Button_Pin) != GPIO_PIN_SET){
										 ButtonPressCounter[i]++;
									 }
				break;
				default:
					break;
				}
			}else{
				switch(i){
				case(0)://Middele button
									 if(ButtonPressCounter[i] > 15){
										 Mid_ButtonPress_Count++;
										 TOGGLE_LEDS();
									 }
				ButtonPressCounter[i] = 0;
				break;
				case(1)://Left Button
									 if(ButtonPressCounter[i] > 15){
										 Left_ButtonPress_Count++;
										 if (b_FLASH_MODE == 1){
											 b_WhiteLED_State = 0;
											 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
											 HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
											 HAL_GPIO_WritePin(LED5_GPIO_Port, LED5_Pin, GPIO_PIN_RESET);
											 SET_LED_Intensity();
											 b_FLASH_MODE = 0;
											 b_MOOD_MODE = 0;
											 b_EMERGENCY_MODE = 1;
											 b_EM_Strobe = 1;
											 EM_Strobe_OnTime[0] = '5';
											 EM_Strobe_OnTime[1] = '1';
											 EM_Strobe_OnTime[2] = '2';
											 b_Slider_ADC_Intensity = 1;
											 b_UART_Intensity = 0;
											 b_TRACK_Intensity = 0;
											 HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);

										 }else
											 if(b_EMERGENCY_MODE == 1){
												 b_UART_Strobe = 0;
												 b_WhiteLED_State = 0;
												 htim3.Instance->CCR1 = 0;
												 b_RGBLED_State = 0;
												 SET_MM_RGB_OFF();
												 HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
												 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
												 b_EMERGENCY_MODE = 0;
												 b_FLASH_MODE = 0;
												 b_MOOD_MODE = 1;
												 HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);


											 }else
												 if(b_MOOD_MODE == 1){
													 b_WhiteLED_State = 0;
													 HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
													 HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
													 b_MOOD_MODE = 0;
													 b_EMERGENCY_MODE = 0;
													 b_FLASH_MODE = 1;
													 b_Slider_ADC_Intensity = 1;
													 b_UART_Intensity = 0;
													 b_TRACK_Intensity = 0;
													 HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
												 }
									 }
				ButtonPressCounter[i] = 0;
				break;
				case(2)://Right button
									 if(ButtonPressCounter[i] > 15){
										 Right_ButtonPress_Count++;
										 Cycle_EM_SubMODES();
									 }
				ButtonPressCounter[i] = 0;
				break;

				default:
					break;

				}
				ButtonPressed[i] = 0;
				Threshold_Num[i] = 0;
			}
		}
	}

	/* USER CODE END SysTick_IRQn 0 */
	HAL_IncTick();
	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */
}
//--------------------------------------END EXTI (...it.c) FUNCTIONS------------------------------//
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
