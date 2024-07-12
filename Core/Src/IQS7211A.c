/**
  **********************************************************************************
  * @file     IQS7211A.cpp
  * @author   Azoteq - Ported to C and STM by Karl Voigt
  * @version  V1.0.0
  * @date     2021-07-06
  * @brief    This file contains the constructors and methods which allow ease of
  *           use of an IQS7211A capactive touch controller. The IQS7211A is a capacitive
  *           touch Integrated Circuit (IC) which provides multiple channel
  *           functionality. This class provides an easy means of initializing
  *           and interacting with the IQS7211A device from an Arduino.
  **********************************************************************************
  * @attention  Makes use of the following standard Arduino libraries:
  * - Arduino.h -> Included in IQS7211A.h, comes standard with Arduino
  * - Wire.h    -> Included in IQS7211A.h, comes standard with Arduino
  *
  **********************************************************************************
  */

// Include Files
#include "IQS7211A.h"
#include "IQS7211A_init_AZP1189A3_v0.1.h"
#include <string.h>
#include <stdio.h>
#include "stm32f3xx_hal.h"
// Private Definitions
#define IQS_ADR   0x56
#define MCLR_Pin GPIO_PIN_2
#define MCLR_GPIO_Port GPIOB
#define RDY_Pin GPIO_PIN_15
#define RDY_GPIO_Port GPIOB

extern UART_HandleTypeDef huart2;
extern I2C_HandleTypeDef hi2c1;
extern uint8_t RDY_LOW;
extern uint16_t Track_X ;
extern uint16_t Track_Y ;
extern uint8_t b_Track_TAP ;
extern uint8_t b_Track_HOLD ;
extern uint8_t b_TRACK_Intensity;
extern uint8_t X_LSB;
extern uint8_t X_MSB;
extern uint8_t Y_LSB;
extern uint8_t Y_MSB;
extern uint8_t b_MOOD_Track_TAP;

extern uint8_t R_Channel_RGB[3]; // <state> for Mood Mode
extern uint8_t G_Channel_RGB[3]; // <Param1> for Mood Mode
extern uint8_t B_Channel_RGB[3]; // <Param2> for Mood Mode
//extern void SET_RGB_Intensity(uint16_t X_POS,uint16_t Y_POS);

void ENABLE_TP_EVENT(){
	uint8_t transferByte[2];

	readRandomBytes(IQS7211A_MM_CONFIG_SETTINGS, 2, transferByte);

	transferByte[1] |= 0x04;

	readRandomBytes(IQS7211A_MM_CONFIG_SETTINGS, 2, transferByte);

}

uint8_t CheckReset(void){

	uint8_t bytesArray[2];
	readRandomBytes(IQS7211A_MM_INFOFLAGS, 2, bytesArray);

	if((bytesArray[0] & 0x80) == 0x80){
		return 1;
	}else
	{
		return 0;
	}
}
void SW_Reset()
{
  uint8_t transferByte[2]; // Array to store the bytes transferred.
  	  	  	  	  	  	   // Use an array to be consistent with other methods in this class.
  while(!waitForReady(RDY_Pin, RDY_GPIO_Port));
  readRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 2, transferByte);
  // Mask the settings with the SW_RESET_BIT.
  transferByte[1] |= 0x02;  // This is the bit required to perform SW Reset.
  // Write the new byte to the required device.
  while(!waitForReady(RDY_Pin, RDY_GPIO_Port));
  writeRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 2, transferByte);
}
void writeRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[]){

	uint16_t DATA[2];
	DATA[0] = memoryAddress;
	while(RDY_LOW==0){
		continue;
	}
	HAL_I2C_Mem_Write(&hi2c1, (IQS_ADR<<1),  DATA[0], I2C_MEMADD_SIZE_8BIT, bytesArray, (uint16_t)numBytes,50);
	RDY_LOW=0;

}

void readRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[]){

	uint16_t DATA[2];
	DATA[0] = memoryAddress;
	while(RDY_LOW==0){
		continue;
	}

	HAL_I2C_Mem_Read(&hi2c1, (IQS_ADR<<1), DATA[0], I2C_MEMADD_SIZE_8BIT, bytesArray, (uint16_t)numBytes, 50);
	RDY_LOW=0;
}


uint8_t waitForReady(uint16_t readyPin, GPIO_TypeDef *readyPort)
{
  int readyLow = 0;      // The return value. Set to true if the ready pin is pulled low by the IQS7211A within 100ms.
  uint16_t notReadyCount = 0 ;  // Increments every time the loop executes to keep track of how long the request is going on.

  // Wait for communication from IQS7211A device. Timeout after 100ms.
  while(HAL_GPIO_ReadPin(readyPort, readyPin)==GPIO_PIN_SET)
  {
    notReadyCount++;
    HAL_Delay(1);

    if((notReadyCount%100) == 0)
      return readyLow;
  }
  // If the processing gets here then a response has been received.
  readyLow = 1;
  return readyLow;
}

void TP_ReATI(){

	uint8_t transferByte[1];

	while(RDY_LOW==0){
		continue;
	}
	RDY_LOW=0;

	HAL_I2C_Mem_Read(&hi2c1, (0x56<<1), IQS7211A_MM_SYSTEM_CONTROL, I2C_MEMADD_SIZE_8BIT, transferByte, 1, HAL_MAX_DELAY);

	transferByte[0] |= TP_REATI_BIT;  // This is the bit required to start an ATI routine.

	while(RDY_LOW==0){
		continue;
	}
	RDY_LOW=0;

	HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_SYSTEM_CONTROL, I2C_MEMADD_SIZE_8BIT, transferByte, 1, HAL_MAX_DELAY);


}

void acknowledgeReset()
{
	uint8_t transferBytes[2];

	while(RDY_LOW==0){
		 continue;
	 }
	 RDY_LOW=0;

	HAL_I2C_Mem_Read(&hi2c1, (0x56<<1), IQS7211A_MM_SYSTEM_CONTROL, I2C_MEMADD_SIZE_8BIT, transferBytes, 2, HAL_MAX_DELAY);

	 while(RDY_LOW==0){
		  continue;
	 }
	 RDY_LOW=0;

	// SWrite the AAck Reset bit to 1 to clear the Show Reset Flag.
	transferBytes[0] |= (1<<7);
	// Write the new byte to the System Flags address.

	HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_SYSTEM_CONTROL, I2C_MEMADD_SIZE_8BIT, transferBytes, 2, HAL_MAX_DELAY);
}

void UpdateGestures(){

	uint8_t transferBytes[1];

	 while(!waitForReady(RDY_Pin, RDY_GPIO_Port));
	readRandomBytes(IQS7211A_MM_GESTURES, 1, transferBytes);
}

void ENABLE_Gestures(){
	uint8_t transferBytes[2];

	 while(!waitForReady(RDY_Pin, RDY_GPIO_Port));
	readRandomBytes(IQS7211A_MM_GESTURE_ENABLE, 2, transferBytes);

	transferBytes[1] |= 0x1F;

	while(!waitForReady(RDY_Pin, RDY_GPIO_Port));
	readRandomBytes(IQS7211A_MM_GESTURE_ENABLE, 2, transferBytes);

}

void ENABLE_GESTURE_EVENT(){
	uint8_t transferBytes[2];

	 while(!waitForReady(RDY_Pin, RDY_GPIO_Port));
	readRandomBytes(IQS7211A_MM_CONFIG_SETTINGS, 2, transferBytes);

	transferBytes[1] |= 0x02;

	while(!waitForReady(RDY_Pin, RDY_GPIO_Port));
	readRandomBytes(IQS7211A_MM_CONFIG_SETTINGS, 2, transferBytes);
}

void Trackpad_writeMM()
{
	uint8_t transferBytes[30];	// Temporary array which holds the bytes to be transferred.

  /* Change the ATI Settings */
  /* Memory Map Position 0x30 - 0x3D */
  transferBytes[0] = TP_ATI_MULTIPLIERS_DIVIDERS_0;
  transferBytes[1] = TP_ATI_MULTIPLIERS_DIVIDERS_1;
  transferBytes[2] = TP_COMPENSATION_DIV_0;
  transferBytes[3] = TP_COMPENSATION_DIV_1;
  transferBytes[4] = TP_ATI_TARGET_0;
  transferBytes[5] = TP_ATI_TARGET_1;
  transferBytes[6] = TP_REF_DRIFT_LIMIT_0;
  transferBytes[7] = TP_REF_DRIFT_LIMIT_1;
  transferBytes[8] = TP_MIN_COUNT_REATI_0;
  transferBytes[9] = TP_MIN_COUNT_REATI_1;
  transferBytes[10] = REATI_RETRY_TIME_0;
  transferBytes[11] = REATI_RETRY_TIME_1;
  transferBytes[12] = ALP_ATI_MULTIPLIERS_DIVIDERS_0;
  transferBytes[13] = ALP_ATI_MULTIPLIERS_DIVIDERS_1;
  transferBytes[14] = ALP_COMPENSATION_DIV_0;
  transferBytes[15] = ALP_COMPENSATION_DIV_1;
  transferBytes[16] = ALP_ATI_TARGET_0;
  transferBytes[17] = ALP_ATI_TARGET_1;
  transferBytes[18] = ALP_LTA_DRIFT_LIMIT_0;
  transferBytes[19] = ALP_LTA_DRIFT_LIMIT_1;

  /* Change the ALP ATI Compensation */
  /* Memory Map Position 0x3A - 0x3D */
  transferBytes[20] = ALP_COMPENSATION_A_0;
  transferBytes[21] = ALP_COMPENSATION_A_1;
  transferBytes[22] = ALP_COMPENSATION_B_0;
  transferBytes[23] = ALP_COMPENSATION_B_1;

  /*
  writeRandomBytes(IQS7211A_MM_TP_ATI_MIR, 24, transferBytes, RESTART);
  */

	while(RDY_LOW==0){
		 continue;
	 }
	 RDY_LOW=0;

  HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_TP_ATI_MIR, I2C_MEMADD_SIZE_8BIT, transferBytes, 24, HAL_MAX_DELAY);

//  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)){
// 	  continue;
//   }

  /* Change the Report Rates and Timing */
  /* Memory Map Position 0x40 - 0x4A */
  transferBytes[0] = ACTIVE_MODE_REPORT_RATE_0;
  transferBytes[1] = ACTIVE_MODE_REPORT_RATE_1;
  transferBytes[2] = IDLE_TOUCH_MODE_REPORT_RATE_0;
  transferBytes[3] = IDLE_TOUCH_MODE_REPORT_RATE_1;
  transferBytes[4] = IDLE_MODE_REPORT_RATE_0;
  transferBytes[5] = IDLE_MODE_REPORT_RATE_1;
  transferBytes[6] = LP1_MODE_REPORT_RATE_0;
  transferBytes[7] = LP1_MODE_REPORT_RATE_1;
  transferBytes[8] = LP2_MODE_REPORT_RATE_0;
  transferBytes[9] = LP2_MODE_REPORT_RATE_1;
  transferBytes[10] = ACTIVE_MODE_TIMEOUT_0;
  transferBytes[11] = ACTIVE_MODE_TIMEOUT_1;
  transferBytes[12] = IDLE_TOUCH_MODE_TIMEOUT_0;
  transferBytes[13] = IDLE_TOUCH_MODE_TIMEOUT_1;
  transferBytes[14] = IDLE_MODE_TIMEOUT_0;
  transferBytes[15] = IDLE_MODE_TIMEOUT_1;
  transferBytes[16] = LP1_MODE_TIMEOUT_0;
  transferBytes[17] = LP1_MODE_TIMEOUT_1;
  transferBytes[18] = REF_UPDATE_TIME_0;
  transferBytes[19] = REF_UPDATE_TIME_1;
  transferBytes[20] = I2C_TIMEOUT_0;
  transferBytes[21] = I2C_TIMEOUT_1;
  /*
  writeRandomBytes(IQS7211A_MM_ACTIVE_MODE_RR, 22, transferBytes, RESTART);
  */

//  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)){
//	  continue;
//  }
	 while(RDY_LOW==0){
		  continue;
	 }
	 RDY_LOW=0;

	  HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_ACTIVE_MODE_RR, I2C_MEMADD_SIZE_8BIT, transferBytes, 22, HAL_MAX_DELAY);


  /* Change the System Settings */
  /* Memory Map Position 0x50 - 0x5B */
  transferBytes[0] = SYSTEM_CONTROL_0;
  transferBytes[1] = SYSTEM_CONTROL_1;
  transferBytes[2] = CONFIG_SETTINGS0;
  transferBytes[3] = CONFIG_SETTINGS1;
  transferBytes[4] = OTHER_SETTINGS_0;
  transferBytes[5] = OTHER_SETTINGS_1;
  transferBytes[6] = TRACKPAD_TOUCH_SET_THRESHOLD;
  transferBytes[7] = TRACKPAD_TOUCH_CLEAR_THRESHOLD;
  transferBytes[8] = ALP_THRESHOLD_0;
  transferBytes[9] = ALP_THRESHOLD_1;
  transferBytes[10] = OPEN_0_0;
  transferBytes[11] = OPEN_0_1;
  transferBytes[12] = ALP_SET_DEBOUNCE;
  transferBytes[13] = ALP_CLEAR_DEBOUNCE;
  transferBytes[14] = OPEN_1_0;
  transferBytes[15] = OPEN_1_1;
  transferBytes[16] = TP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
  transferBytes[17] = TP_CONVERSION_FREQUENCY_FRACTION_VALUE;
  transferBytes[18] = ALP_CONVERSION_FREQUENCY_UP_PASS_LENGTH;
  transferBytes[19] = ALP_CONVERSION_FREQUENCY_FRACTION_VALUE;
  transferBytes[20] = TRACKPAD_HARDWARE_SETTINGS_0;
  transferBytes[21] = TRACKPAD_HARDWARE_SETTINGS_1;
  transferBytes[22] = ALP_HARDWARE_SETTINGS_0;
  transferBytes[23] = ALP_HARDWARE_SETTINGS_1;

  /*
  writeRandomBytes(IQS7211A_MM_SYSTEM_CONTROL, 24, transferBytes, RESTART);
  */

	 while(RDY_LOW==0){
		  continue;
	 }
	 RDY_LOW=0;
  HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_SYSTEM_CONTROL, I2C_MEMADD_SIZE_8BIT, transferBytes, 24, HAL_MAX_DELAY);

//  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)){
// 	  continue;
//   }


  /* Change the Trackpad Settings */
  /* Memory Map Position 0x60 - 0x69 */
  transferBytes[0] = TRACKPAD_SETTINGS_0_0;
  transferBytes[1] = TRACKPAD_SETTINGS_0_1;
  transferBytes[2] = TRACKPAD_SETTINGS_1_0;
  transferBytes[3] = TRACKPAD_SETTINGS_1_1;
  transferBytes[4] = X_RESOLUTION_0;
  transferBytes[5] = X_RESOLUTION_1;
  transferBytes[6] = Y_RESOLUTION_0;
  transferBytes[7] = Y_RESOLUTION_1;
  transferBytes[8] = XY_DYNAMIC_FILTER_BOTTOM_SPEED_0;
  transferBytes[9] = XY_DYNAMIC_FILTER_BOTTOM_SPEED_1;
  transferBytes[10] = XY_DYNAMIC_FILTER_TOP_SPEED_0;
  transferBytes[11] = XY_DYNAMIC_FILTER_TOP_SPEED_1;
  transferBytes[12] = XY_DYNAMIC_FILTER_BOTTOM_BETA;
  transferBytes[13] = XY_DYNAMIC_FILTER_STATIC_FILTER_BETA;
  transferBytes[14] = STATIONARY_TOUCH_MOV_THRESHOLD;
  transferBytes[15] = FINGER_SPLIT_FACTOR;
  transferBytes[16] = X_TRIM_VALUE_0;
  transferBytes[17] = X_TRIM_VALUE_1;
  transferBytes[18] = Y_TRIM_VALUE_0;
  transferBytes[19] = Y_TRIM_VALUE_1;

  /*
  writeRandomBytes(IQS7211A_MM_TP_SETTINGS_0, 20, transferBytes, RESTART);
  */

	 while(RDY_LOW==0){
		  continue;
	 }
	 RDY_LOW=0;

  HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_TP_SETTINGS_0, I2C_MEMADD_SIZE_8BIT, transferBytes, 20, HAL_MAX_DELAY);

//  while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)){
// 	  continue;
//   }



  /* Change the ALP Settings */
  /* Memory Map Position 0x70 - 0x74 */
  transferBytes[0] = ALP_COUNT_FILTER_BETA_0;
  transferBytes[1] = OPEN_0;
  transferBytes[2] = ALP_LTA_BETA_LP1;
  transferBytes[3] = ALP_LTA_BETA_LP2;
  transferBytes[4] = ALP_SETUP_0;
  transferBytes[5] = ALP_SETUP_1;
  transferBytes[6] = ALP_TX_ENABLE_0;
  transferBytes[7] = ALP_TX_ENABLE_1;

  /* Change the Settings Version Numbers */
  /* Memory Map Position 0x74 - 0x75 */
  transferBytes[8] = MINOR_VERSION;
  transferBytes[9] = MAJOR_VERSION;

  /*
  writeRandomBytes(IQS7211A_MM_ALP_COUNT_FILTER_BETA, 10, transferBytes, RESTART);
  */
	 while(RDY_LOW==0){
		  continue;
	 }
	 RDY_LOW=0;
	 HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_ALP_COUNT_FILTER_BETA, I2C_MEMADD_SIZE_8BIT, transferBytes, 10, HAL_MAX_DELAY);



  /* Change the Gesture Settings */
  /* Memory Map Position 0x80 - 0x8F */
  transferBytes[0] = GESTURE_ENABLE_0;
  transferBytes[1] = GESTURE_ENABLE_1;
  transferBytes[2] = TAP_TIME_0;
  transferBytes[3] = TAP_TIME_1;
  transferBytes[4] = TAP_DISTANCE_0;
  transferBytes[5] = TAP_DISTANCE_1;
  transferBytes[6] = HOLD_TIME_0;
  transferBytes[7] = HOLD_TIME_1;
  transferBytes[8] = SWIPE_TIME_0;
  transferBytes[9] = SWIPE_TIME_1;
  transferBytes[10] = SWIPE_X_DISTANCE_0;
  transferBytes[11] = SWIPE_X_DISTANCE_1;
  transferBytes[12] = SWIPE_Y_DISTANCE_0;
  transferBytes[13] = SWIPE_Y_DISTANCE_1;
  transferBytes[14] = SWIPE_ANGLE_0;
  transferBytes[15] = GESTURE_OPEN_0;

  /*
  writeRandomBytes(IQS7211A_MM_GESTURE_ENABLE, 16, transferBytes, RESTART);
  */

  HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_GESTURE_ENABLE, I2C_MEMADD_SIZE_8BIT, transferBytes, 16, HAL_MAX_DELAY);

	 while(RDY_LOW==0){
		  continue;
	 }
	 RDY_LOW=0;

  /* Change the RxTx Mapping */
  /* Memory Map Position 0x90 - 0x9C */
  transferBytes[0] = RX_TX_MAP_0;
  transferBytes[1] = RX_TX_MAP_1;
  transferBytes[2] = RX_TX_MAP_2;
  transferBytes[3] = RX_TX_MAP_3;
  transferBytes[4] = RX_TX_MAP_4;
  transferBytes[5] = RX_TX_MAP_5;
  transferBytes[6] = RX_TX_MAP_6;
  transferBytes[7] = RX_TX_MAP_7;
  transferBytes[8] = RX_TX_MAP_8;
  transferBytes[9] = RX_TX_MAP_9;
  transferBytes[10] = RX_TX_MAP_10;
  transferBytes[11] = RX_TX_MAP_11;
  transferBytes[12] = RX_TX_MAP_12;

  /*
  writeRandomBytes(IQS7211A_MM_RXTX_MAPPING_1_0, 13, transferBytes, RESTART);
  */

  HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_RXTX_MAPPING_1_0, I2C_MEMADD_SIZE_8BIT, transferBytes, 13, HAL_MAX_DELAY);

	 while(RDY_LOW==0){
		  continue;
	 }
	 RDY_LOW=0;

  /* Change the Allocation of channels into cycles 0-9 */
  /* Memory Map Position 0xA0 - 0xBD */
  transferBytes[0] = PLACEHOLDER_0;
  transferBytes[1] = CH_1_CYCLE_0;
  transferBytes[2] = CH_2_CYCLE_0;
  transferBytes[3] = PLACEHOLDER_1;
  transferBytes[4] = CH_1_CYCLE_1;
  transferBytes[5] = CH_2_CYCLE_1;
  transferBytes[6] = PLACEHOLDER_2;
  transferBytes[7] = CH_1_CYCLE_2;
  transferBytes[8] = CH_2_CYCLE_2;
  transferBytes[9] = PLACEHOLDER_3;
  transferBytes[10] = CH_1_CYCLE_3;
  transferBytes[11] = CH_2_CYCLE_3;
  transferBytes[12] = PLACEHOLDER_4;
  transferBytes[13] = CH_1_CYCLE_4;
  transferBytes[14] = CH_2_CYCLE_4;
  transferBytes[15] = PLACEHOLDER_5;
  transferBytes[16] = CH_1_CYCLE_5;
  transferBytes[17] = CH_2_CYCLE_5;
  transferBytes[18] = PLACEHOLDER_6;
  transferBytes[19] = CH_1_CYCLE_6;
  transferBytes[20] = CH_2_CYCLE_6;
  transferBytes[21] = PLACEHOLDER_7;
  transferBytes[22] = CH_1_CYCLE_7;
  transferBytes[23] = CH_2_CYCLE_7;
  transferBytes[24] = PLACEHOLDER_8;
  transferBytes[25] = CH_1_CYCLE_8;
  transferBytes[26] = CH_2_CYCLE_8;
  transferBytes[27] = PLACEHOLDER_9;
  transferBytes[28] = CH_1_CYCLE_9;
  transferBytes[29] = CH_2_CYCLE_9;

  /*
  writeRandomBytes(IQS7211A_MM_CYCLE_SETUP_0_9, 30, transferBytes, RESTART);
  */


	 while(RDY_LOW==0){
		  continue;
	 }
	 RDY_LOW=0;


  HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_CYCLE_SETUP_0_9, I2C_MEMADD_SIZE_8BIT, transferBytes, 30, HAL_MAX_DELAY);

  /* Change the Allocation of channels into cycles 10-17 */
  /* Memory Map Position 0xB0 - 0xCA */
  transferBytes[0] = PLACEHOLDER_10;
  transferBytes[1] = CH_1_CYCLE_10;
  transferBytes[2] = CH_2_CYCLE_10;
  transferBytes[3] = PLACEHOLDER_11;
  transferBytes[4] = CH_1_CYCLE_11;
  transferBytes[5] = CH_2_CYCLE_11;
  transferBytes[6] = PLACEHOLDER_12;
  transferBytes[7] = CH_1_CYCLE_12;
  transferBytes[8] = CH_2_CYCLE_12;
  transferBytes[9] = PLACEHOLDER_13;
  transferBytes[10] = CH_1_CYCLE_13;
  transferBytes[11] = CH_2_CYCLE_13;
  transferBytes[12] = PLACEHOLDER_14;
  transferBytes[13] = CH_1_CYCLE_14;
  transferBytes[14] = CH_2_CYCLE_14;
  transferBytes[15] = PLACEHOLDER_15;
  transferBytes[16] = CH_1_CYCLE_15;
  transferBytes[17] = CH_2_CYCLE_15;
  transferBytes[18] = PLACEHOLDER_16;
  transferBytes[19] = CH_1_CYCLE_16;
  transferBytes[20] = CH_2_CYCLE_16;
  transferBytes[21] = PLACEHOLDER_17;
  transferBytes[22] = CH_1_CYCLE_17;
  transferBytes[23] = CH_2_CYCLE_17;

  /*
  writeRandomBytes(IQS7211A_MM_CYCLE_SETUP_10_17, 24, transferBytes, stopOrRestart);
  */

  HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_CYCLE_SETUP_10_17, I2C_MEMADD_SIZE_8BIT, transferBytes, 24, HAL_MAX_DELAY);

	 while(RDY_LOW==0){
		  continue;
	 }
	 RDY_LOW=0;

}
void disableCommsReqEn()
{
  uint8_t transferBytes[2]; // The array which will hold the bytes which are transferred.

	 while(RDY_LOW==0){
		 continue;
	 }
	 RDY_LOW=0;

  // First read the bytes at the memory address so that they can be preserved.
  HAL_I2C_Mem_Read(&hi2c1, (0x56<<1), IQS7211A_MM_CONFIG_SETTINGS, 1, transferBytes, 2, HAL_MAX_DELAY);
  // Set the TP_EVENT_BIT in CONFIG_SETTINGS
  transferBytes[0] |= 0x10;
  // Write the bytes back to the device
	 while(RDY_LOW==0){
		 continue;
	 }
	 RDY_LOW=0;
  HAL_I2C_Mem_Write(&hi2c1, (0x56<<1), IQS7211A_MM_CONFIG_SETTINGS, 1, transferBytes, 2, HAL_MAX_DELAY);
}

uint16_t getProductNum()
{
	uint8_t transferBytes[2];
	uint8_t prodNumLow = 0;
	uint8_t prodNumHigh = 0;
	uint16_t prodNumReturn = 0;

//	HAL_I2C_Mem_Read_DMA(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size)

	 while(RDY_LOW==0){
		 continue;
	 }
	 RDY_LOW=0;
	HAL_I2C_Mem_Read(&hi2c1, (0x56<<1), IQS7211A_MM_PROD_NUM, I2C_MEMADD_SIZE_8BIT, transferBytes, 2, HAL_MAX_DELAY);
	prodNumLow = transferBytes[0];
	prodNumHigh = transferBytes[1];
	prodNumReturn = (uint16_t)(prodNumLow);
	prodNumReturn |= (uint16_t)(prodNumHigh<<8);
  // Return the product number value.
  return prodNumReturn;
}
