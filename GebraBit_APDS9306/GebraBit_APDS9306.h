/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Sepehr Azimi
 * ________________________________________________________________________________________________________
 */
#ifndef	__APDS9306_H__
#define	__APDS9306_H__
#include "arduino.h"
#include "Wire.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

/************************************************
 *              USER REGISTER MAP               *
 ***********************************************/ 
#define APDS9306_MAIN_CTRL  					0x00
#define APDS9306_ALS_MEAS_RATE  			0x04
#define APDS9306_ALS_GAIN 						0x05
#define APDS9306_PART_ID 							0x06
#define APDS9306_MAIN_STATUS 					0x07
#define APDS9306_CLEAR_DATA_0 				0x0A
#define APDS9306_CLEAR_DATA_1 				0x0B
#define APDS9306_CLEAR_DATA_2 				0x0C
#define APDS9306_ALS_DATA_0 					0x0D
#define APDS9306_ALS_DATA_1 					0x0E
#define APDS9306_ALS_DATA_2 					0x0F
#define APDS9306_INT_CFG 							0x19
#define APDS9306_INT_PERSISTENCE 			0x1A
#define APDS9306_ALS_THRES_UP_0 			0x21
#define APDS9306_ALS_THRES_UP_1 			0x22
#define APDS9306_ALS_THRES_UP_2 			0x23
#define APDS9306_ALS_THRES_LOW_0 			0x24
#define APDS9306_ALS_THRES_LOW_1 			0x25
#define APDS9306_ALS_THRES_LOW_2 			0x26
#define APDS9306_ALS_THRES_VAR 				0x27
#define APDS9306_I2C		              &hi2c1	
#define APDS9306_ADDRESS 							0x52	
/*----------------------------------------------*
 *           USER REGISTER MAP End              *
 *----------------------------------------------*/ 
 /************************************************
 *         MSB Bit Start Location Begin         *
 ***********************************************/ 
#define START_MSB_BIT_AT_0                    0
#define START_MSB_BIT_AT_1                    1
#define START_MSB_BIT_AT_2                    2
#define START_MSB_BIT_AT_3                    3
#define START_MSB_BIT_AT_4                    4
#define START_MSB_BIT_AT_5                    5
#define START_MSB_BIT_AT_6                    6
#define START_MSB_BIT_AT_7                    7
/*----------------------------------------------*
 *        MSB Bit Start Location End            *
 *----------------------------------------------*/ 
/************************************************
 *          Bit Field Length Begin              *
 ***********************************************/ 
#define BIT_LENGTH_1                          1
#define BIT_LENGTH_2                          2
#define BIT_LENGTH_3                          3
#define BIT_LENGTH_4                          4
#define BIT_LENGTH_5                          5
#define BIT_LENGTH_6                          6
#define BIT_LENGTH_7                          7
#define BIT_LENGTH_8                          8
/*----------------------------------------------*
 *          Bit Field Length End                *
 *----------------------------------------------*/
#define REGISTER_DATA_BUFFER_SIZE                6
/**************************************************
 *     Values For Disable And Enable Functions    *
 **************************************************/ 
typedef enum Ability
{  
	Disable = 0     ,                      
	Enable     
}APDS9306_Ability;    
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum 
{  
	FAILED = 0     ,                      
	DONE     
}APDS9306_Reset_Status;
/*************************************************
 *       Values For Interrupt Channel            *
 **************************************************/ 
typedef enum Interrupt_Channel 
{  
	CLEAR_CHANNEL = 0     ,                      
	ALS_CHANNEL     
}APDS9306_Interrupt_Channel;

/*************************************************
 *      			 Values For ALS Gain               *
 **************************************************/ 
typedef enum ALS_Gain 
{
  ALS_GAIN_1X  = 0,
  ALS_GAIN_3X  = 1,
  ALS_GAIN_6X  = 2,
  ALS_GAIN_9X  = 3,
  ALS_GAIN_18X = 4,
} APDS9306_ALS_Gain;

/*************************************************
 *         Values For ALS Resolution             *
 **************************************************/ 
typedef enum Resolution
{
  _20_BIT_400_mS,
  _19_BIT_200_mS,
  _18_BIT_100_mS,
  _17_BIT_50_mS,
  _16_BIT_25_mS,
  _13_BIT_3P125_mS
} APDS9306_ALS_Resolution;
/*************************************************
 *         Values For Measurement Rate           *
 **************************************************/ 
typedef enum Measurement_Rate
{
	ALS_MEASRATE_25_mS,
  ALS_MEASRATE_50_mS,
  ALS_MEASRATE_100_mS,
  ALS_MEASRATE_200_mS,
  ALS_MEASRATE_500_mS,
  ALS_MEASRATE_1000_mS,
  ALS_MEASRATE_2000_mS,
} APDS9306_Measurement_Rate;
/*************************************************
 *           Values For Power Issues             *
 **************************************************/ 
typedef enum Power_Status 
{  
	NO_POWER_ISSUE = 0     ,                      
	POWER_ISSUE     
}APDS9306_Power_Status;
/*************************************************
 *         Values For Interrupt Status           *
 **************************************************/ 
typedef enum Interrupt_Status 
{  
	INTERRUPT_NOT_FULFILLED = 0     ,                      
	INTERRUPT_FULFILLED     
}APDS9306_Interrupt_Status;
/*************************************************
 *           Values For Data Status              *
 **************************************************/ 
typedef enum Data_Status 
{  
	OLD_DATA = 0     ,                      
	NEW_DATA     
}APDS9306_Data_Status;
/*************************************************
 *           Values For Interrupt Mode           *
 **************************************************/ 
typedef enum Interrupt_Mode 
{  
	ALS_THRESHOLD_INTERRUPT = 0     ,                      
	ALS_VARIATION_INTERRUPT     
}APDS9306_Interrupt_Mode;
/**************************************************************
 *       				Values For Interrupt Persist    					    *
 **************************************************************/ 
typedef enum Interrupt_Persist
{
  EVERY_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_2_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_3_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_4_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_5_ALS_VALUE_OUT_OF_THR_RANGE,
  CONSECUTIVE_6_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_7_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_8_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_9_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_10_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_11_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_12_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_13_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_14_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_15_ALS_VALUE_OUT_OF_THR_RANGE,
	CONSECUTIVE_16_ALS_VALUE_OUT_OF_THR_RANGE,
} APDS9306_Interrupt_Persist;

 /*************************************************
 *  Defining APDS9306 Register & Data As Struct   *
 **************************************************/
typedef	struct APDS9306
{
	  uint8_t                         Register_Cache;
	  uint8_t						  PART_ID;
	  APDS9306_Reset_Status			  RESET;
	  APDS9306_Ability                ALS;
	  APDS9306_ALS_Gain               ALS_GAIN;
	  float							  ALS_GAIN_VALUE;
	  APDS9306_Measurement_Rate       MEASUREMENT_RATE;
	  APDS9306_ALS_Resolution         ALS_RESOLUTION;
	  float							  ALS_RESOLUTION_TIME;
      APDS9306_Power_Status           POWER_STATUS;
	  APDS9306_Data_Status            DATA;
	  APDS9306_Ability                INTERRUPT;
      APDS9306_Interrupt_Channel      INTERRUPT_CHANNEL;
	  APDS9306_Interrupt_Mode         INTERRUPT_MODE;
      APDS9306_Interrupt_Persist      INTERRUPT_PERSIST;
	  APDS9306_Interrupt_Status		  INTERRRUPT_STATUS;
	  uint32_t                        INTERRUPT_UPPER_THRESHOLD;
	  uint32_t                        INTERRUPT_LOWER_THRESHOLD;
	  uint8_t 						  REGISTER_DATA[REGISTER_DATA_BUFFER_SIZE];
	  uint32_t               		  CLEAR_DATA;
	  uint32_t               		  ALS_DATA;
      float 						  LUMINOSITY;
}GebraBit_APDS9306;
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/********************************************************
 *  Declare Read&Write APDS9306 Register Values Functions *
 ********************************************************/
extern void GB_APDS9306_Read_Reg_Data(uint8_t regAddr,  uint8_t *data)	;
extern void GB_APDS9306_Burst_Read(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity);
extern void GB_APDS9306_Read_Reg_Bits (uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t* data);	
extern void GB_APDS9306_Write_Command( uint8_t cmd);
extern void GB_APDS9306_Write_Reg_Data(uint8_t regAddr,  uint8_t data)	;
extern void GB_APDS9306_Burst_Write(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)								;
extern void GB_APDS9306_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data);
/********************************************************
 *       Declare APDS9306 Configuration Functions         *
 ********************************************************/
extern void GB_APDS9306_Soft_Reset ( GebraBit_APDS9306 * APDS9306 )  ;
extern void GB_APDS9306_ALS ( GebraBit_APDS9306 * APDS9306 , APDS9306_Ability als );
extern void GB_APDS9306_ALS_Gain ( GebraBit_APDS9306 * APDS9306 , APDS9306_ALS_Gain gain )  ;
extern void GB_APDS9306_Measurement_Repeat_Rate ( GebraBit_APDS9306 * APDS9306 , APDS9306_Measurement_Rate rate )  ;
extern void GB_APDS9306_ALS_Resolution ( GebraBit_APDS9306 * APDS9306 , APDS9306_ALS_Resolution res ) ;
extern void GB_APDS9306_Read_Part_ID ( GebraBit_APDS9306 * APDS9306  )  ;
extern void GB_APDS9306_Read_STATUS ( GebraBit_APDS9306 * APDS9306 )   ;
extern void GB_APDS9306_Interrupt_Channel ( GebraBit_APDS9306 * APDS9306 , APDS9306_Interrupt_Channel intr );
extern void GB_APDS9306_Interrupt_Mode ( GebraBit_APDS9306 * APDS9306 , APDS9306_Interrupt_Mode mode )  ;
extern void GB_APDS9306_Interrupt( GebraBit_APDS9306 * APDS9306 , APDS9306_Ability intpt )  ;
extern void GB_APDS9306_Interrupt_Persist ( GebraBit_APDS9306 * APDS9306 , APDS9306_Interrupt_Persist persist )  ;
extern void GB_APDS9306_Interrupt_Upper_Threshold ( GebraBit_APDS9306 * APDS9306 , uint32_t upthr ) ;
extern void GB_APDS9306_Interrupt_Lower_Threshold ( GebraBit_APDS9306 * APDS9306 , uint16_t lothr )  ;
extern void GB_APDS9306_initialize( GebraBit_APDS9306 * APDS9306 )  ;
extern void GB_APDS9306_Configuration(GebraBit_APDS9306 * APDS9306)  ;
extern void GB_APDS9306_Get_Raw_Data(GebraBit_APDS9306 * APDS9306);
extern void GB_APDS9306_Luminosity_Reading(GebraBit_APDS9306 * APDS9306);
extern void GB_APDS9306_Get_Data(GebraBit_APDS9306 * APDS9306);
#endif
