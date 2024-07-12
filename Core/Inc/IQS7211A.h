/**
  ************************************************************************************
  * @file    IQS7211A.h
  * @author  Azoteq
  * @version V1.0.0
  * @date    2021-07-06
  * @brief   This file contains the header information for an IQS7211A Arduino library.
  *          The goal of the library is to provide easy functionality for 
  *          initializing and using the Azoteq IQS7211A capacitive touch device.
  ************************************************************************************
  ************************************************************************************
  * @attention  Makes use of the following standard Arduino libraries:
  *       - Arduino.h   -> included in IQS7211A.h, comes standard with Arduino
  *       - Wire.h      -> Included in IQS7211A.h, comes standard with Arduino
  *
  ************************************************************************************
  */

#ifndef IQS7211A_h
#define IQS7211A_h

// Include Files
//#include "Arduino.h"
//#include "Wire.h"
#include "iqs7211a_addresses.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"

//FUNCS

void acknowledgeReset();
void TP_ReATI();
void Trackpad_writeMM();
uint16_t getProductNum();
void disableCommsReqEn();
void readRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[]);
void writeRandomBytes(uint8_t memoryAddress, uint8_t numBytes, uint8_t bytesArray[]);
uint8_t waitForReady(uint16_t readyPin, GPIO_TypeDef *readyPort);

void SW_Reset();
void UpdateGestures();
void ENABLE_GESTURE_EVENT();
void ENABLE_Gestures();
void UpdateAbsCoordinates();
uint8_t CheckReset(void);
void ENABLE_TP_EVENT();


//UTILITY BIT
#define TP_REATI_BIT                    0x20
#endif // IQS7211A_h  
