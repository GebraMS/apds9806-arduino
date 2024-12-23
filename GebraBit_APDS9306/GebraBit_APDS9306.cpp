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
#include "GebraBit_APDS9306.h"

/*========================================================================================================================================= 
 * @brief     Read  data from  spacial register address.
 * @param     regAddr Register Address of APDS9306 that reading data from this address
 * @param     data    Pointer to Variable that data is saved .
 * @return    None
 ========================================================================================================================================*/
void GB_APDS9306_Read_Reg_Data(uint8_t regAddr,  uint8_t *data)																			/*		Read Burst Data From Register			*/
{
	Wire.beginTransmission(APDS9306_ADDRESS);
    Wire.write(regAddr); 
    Wire.endTransmission(false); 
    Wire.requestFrom((uint8_t)APDS9306_ADDRESS, (uint8_t)1);
	delay(15);
    if (Wire.available()) {
        *data = Wire.read(); 
    }
}
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of APDS9306 that reading multiple data start from this address
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     byteQuantity Quantity of data that we want to read .
 * @return    None
 ========================================================================================================================================*/
void GB_APDS9306_Burst_Read(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)																			/*		Read Burst Data From Register			*/
{
	Wire.beginTransmission(APDS9306_ADDRESS);
    Wire.write(regAddr); 
    Wire.endTransmission(false); 
    Wire.requestFrom((uint8_t)APDS9306_ADDRESS, (uint8_t)byteQuantity); 
	delay(15);
    for (uint16_t i = 0; i < byteQuantity; i++) {
        if (Wire.available()) {
            data[i] = Wire.read();
        }
    }
}
/*========================================================================================================================================= 
 * @brief     Read data from spacial bits of a register.
 * @param     regAddr     Register Address of APDS9306 .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    status      Return status
 ========================================================================================================================================*/
void GB_APDS9306_Read_Reg_Bits (uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t* data)
{
	uint8_t tempData = 0;
	GB_APDS9306_Read_Reg_Data( regAddr, &tempData);
	uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); //formula for making a broom of 1&0 for gathering desired bits
	tempData &= mask; // zero all non-important bits in data
	tempData >>= (start_bit - len + 1); //shift data to zero position
	*data = tempData;
}
/*=========================================================================================================================================
 * @param     cmd    Command that will be writen 
 * @return    status    Return status
 ========================================================================================================================================*/
void GB_APDS9306_Write_Command( uint8_t cmd)
{
	uint8_t TBuff[1];
	TBuff[0]=cmd;
	Wire.beginTransmission(APDS9306_ADDRESS);
	Wire.write(cmd); 
    Wire.endTransmission(); 
}
/*========================================================================================================================================= 
 * @brief     Write  data to  spacial register address.
 * @param     regAddr First Register Address of APDS9306 that reading multiple data start from this address
 * @param     data    Variable that to be written .
 * @return    None
 ========================================================================================================================================*/
void GB_APDS9306_Write_Reg_Data(uint8_t regAddr,  uint8_t data)																			/*		Read Burst Data From Register			*/
{
	Wire.beginTransmission(APDS9306_ADDRESS);
    Wire.write(regAddr); 
    Wire.write(data); 
    Wire.endTransmission();
}
/*========================================================================================================================================= 
 * @brief     Write multiple data from first spacial register address.
 * @param     regAddr First Register Address of APDS9306 that  multiple data to be written start from this address
 * @param     data    Pointer to multiple data Variable that to be written.
 * @param     byteQuantity Quantity of data that we want to Write .
 * @return    None
 ========================================================================================================================================*/
void GB_APDS9306_Burst_Write(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)																			/*		Read Burst Data From Register			*/
{
	Wire.beginTransmission(APDS9306_ADDRESS);
    Wire.write(regAddr);
	delay(15);
	for (uint16_t i = 0; i < byteQuantity; i++){
		Wire.write(data[i]);
	}
    Wire.endTransmission();
}
/*=========================================================================================================================================
 * @brief     Write data to spacial bits of a register.
 * @param     regAddr     Register Address of APDS9306 .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to write(1 to 8) 
 * @param     data        Value that will be writen to register bits .
 * @return    status      Return status
 ========================================================================================================================================*/
void GB_APDS9306_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data)
{
	uint8_t tempData = 0;
	GB_APDS9306_Read_Reg_Data( regAddr, &tempData) ;	
	uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
	data <<= (start_bit - len + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tempData &= ~(mask); // zero all important bits in existing byte
	tempData |= data; // combine data with existing byte
	GB_APDS9306_Write_Reg_Data(regAddr,  tempData);
}
/*=========================================================================================================================================
 * @brief     Reset APDS9306
 * @param     APDS9306   APDS9306 Struct RESET Variable.
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_Soft_Reset ( GebraBit_APDS9306 * APDS9306 )  
{
	GB_APDS9306_Write_Reg_Bits(APDS9306_MAIN_CTRL, START_MSB_BIT_AT_4, BIT_LENGTH_1, 1);
	delay(100);
	GB_APDS9306_Read_STATUS( APDS9306 );
	APDS9306->RESET = DONE ;
}
/*=========================================================================================================================================
 * @brief     Enable Or Disable ALS
 * @param     APDS9306   APDS9306 Struct ALS variable
 * @param     als        Value is from APDS9306_Ability Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_ALS ( GebraBit_APDS9306 * APDS9306 , APDS9306_Ability als ) 
{
 GB_APDS9306_Write_Reg_Bits(APDS9306_MAIN_CTRL, START_MSB_BIT_AT_1, BIT_LENGTH_1, als);
 APDS9306->ALS = als ;
}
/*=========================================================================================================================================
 * @brief     Set ALS Gain
 * @param     APDS9306   APDS9306 Struct ALS_GAIN variable & ALS_GAIN_VALUE variable
 * @param     gain        Value is from APDS9306_ALS_Gain Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_ALS_Gain ( GebraBit_APDS9306 * APDS9306 , APDS9306_ALS_Gain gain ) 
{
 GB_APDS9306_Write_Reg_Bits(APDS9306_ALS_GAIN, START_MSB_BIT_AT_2, BIT_LENGTH_3, gain);
 APDS9306->ALS_GAIN = gain ;
 	switch(APDS9306->ALS_GAIN)
	 {
	  case ALS_GAIN_1X:
		APDS9306->ALS_GAIN_VALUE = 1.0f ;
    break;
		case ALS_GAIN_3X:
		APDS9306->ALS_GAIN_VALUE = 3.0f ;
    break;	
		case ALS_GAIN_6X:
		APDS9306->ALS_GAIN_VALUE = 6.0f ;
    break;	
		case ALS_GAIN_9X:
		APDS9306->ALS_GAIN_VALUE = 9.0f ;
    break;
		case ALS_GAIN_18X:
		APDS9306->ALS_GAIN_VALUE = 18.0f ;
    break;			
	 }
}
/*=========================================================================================================================================
 * @brief     Set Measurement Repeat Rate
 * @param     APDS9306   APDS9306 Struct MEASUREMENT_RATE variable 
 * @param     rate       Value is from APDS9306_Measurement_Rate Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_Measurement_Repeat_Rate ( GebraBit_APDS9306 * APDS9306 , APDS9306_Measurement_Rate rate ) 
{
 GB_APDS9306_Write_Reg_Bits(APDS9306_ALS_MEAS_RATE, START_MSB_BIT_AT_2, BIT_LENGTH_3, rate);
 APDS9306->MEASUREMENT_RATE = rate ;
}
/*=========================================================================================================================================
 * @brief     Set ALS Resolution
 * @param     APDS9306   APDS9306 Struct ALS_RESOLUTION variable & ALS_RESOLUTION_TIME variable
 * @param     res       Value is from APDS9306_ALS_Resolution Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_ALS_Resolution ( GebraBit_APDS9306 * APDS9306 , APDS9306_ALS_Resolution res ) 
{
 GB_APDS9306_Write_Reg_Bits(APDS9306_ALS_MEAS_RATE, START_MSB_BIT_AT_6, BIT_LENGTH_3, res);
 APDS9306->ALS_RESOLUTION = res ;
 switch(APDS9306->ALS_RESOLUTION)
	 {
	  case _20_BIT_400_mS:
		APDS9306->ALS_RESOLUTION_TIME = 400.0f ;
    break;
		case _19_BIT_200_mS:
		APDS9306->ALS_RESOLUTION_TIME = 200.0f ;
    break;	
		case _18_BIT_100_mS:
		APDS9306->ALS_RESOLUTION_TIME = 100.0f ;
    break;	
		case _17_BIT_50_mS: 
		APDS9306->ALS_RESOLUTION_TIME = 50.0f ;
    break;
		case _16_BIT_25_mS:
		APDS9306->ALS_RESOLUTION_TIME = 25.0f ;
    break;	
		case _13_BIT_3P125_mS:
		APDS9306->ALS_RESOLUTION_TIME = 3.125f ;
    break;
	 }
}
/*=========================================================================================================================================
 * @brief     Read APDS9306 Part ID
 * @param     APDS9306   APDS9306 Struct PART_ID variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_Read_Part_ID ( GebraBit_APDS9306 * APDS9306  ) 
{
 GB_APDS9306_Read_Reg_Data(APDS9306_PART_ID, &APDS9306->PART_ID);
}
/*=========================================================================================================================================
 * @brief     Read APDS9306 STATUS
 * @param     APDS9306   APDS9306 Struct POWER_STATUS variable  & INTERRRUPT_STATUS variable  & DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_Read_STATUS ( GebraBit_APDS9306 * APDS9306 ) 
{
 uint8_t status;
 GB_APDS9306_Read_Reg_Data(APDS9306_MAIN_STATUS, &status);
 APDS9306->POWER_STATUS = (status & 0x20)>>5  ;
 APDS9306->INTERRRUPT_STATUS = (status & 0x10)>>4  ;
 APDS9306->DATA = (status & 0x08)>>3  ;
}
/*=========================================================================================================================================
 * @brief     Enable Or Disable Interrupt
 * @param     APDS9306   APDS9306 Struct INTERRUPT variable
 * @param     intpt        Value is from APDS9306_Ability Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_Interrupt( GebraBit_APDS9306 * APDS9306 , APDS9306_Ability intpt ) 
{
 GB_APDS9306_Write_Reg_Bits(APDS9306_INT_CFG, START_MSB_BIT_AT_2, BIT_LENGTH_1, intpt);
 APDS9306->INTERRUPT = intpt ;
}
/*=========================================================================================================================================
 * @brief     Select Interrupt Channel
 * @param     APDS9306   APDS9306 Struct INTERRUPT_CHANNEL variable
 * @param     intr        Value is from APDS9306_Interrupt_Channel Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_Interrupt_Channel ( GebraBit_APDS9306 * APDS9306 , APDS9306_Interrupt_Channel intr ) 
{
 GB_APDS9306_Write_Reg_Bits(APDS9306_INT_CFG, START_MSB_BIT_AT_5, BIT_LENGTH_2, intr);
 APDS9306->INTERRUPT_CHANNEL = intr ;
}
/*=========================================================================================================================================
 * @brief     Select Interrupt Mode
 * @param     APDS9306   APDS9306 Struct INTERRUPT_MODE variable
 * @param     mode        Value is from APDS9306_Interrupt_Mode Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_Interrupt_Mode ( GebraBit_APDS9306 * APDS9306 , APDS9306_Interrupt_Mode mode ) 
{
 GB_APDS9306_Write_Reg_Bits(APDS9306_INT_CFG, START_MSB_BIT_AT_3, BIT_LENGTH_1, mode);
 APDS9306->INTERRUPT_MODE = mode ;
}
/*=========================================================================================================================================
 * @brief     Select Interrupt Persist
 * @param     APDS9306   APDS9306 Struct INTERRUPT_PERSIST variable
 * @param     persist    Value is from APDS9306_Interrupt_Persist Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_Interrupt_Persist ( GebraBit_APDS9306 * APDS9306 , APDS9306_Interrupt_Persist persist ) 
{
 GB_APDS9306_Write_Reg_Bits(APDS9306_INT_PERSISTENCE, START_MSB_BIT_AT_7, BIT_LENGTH_4, persist);
 APDS9306->INTERRUPT_PERSIST = persist ;
}
/*
M403Z 
*/
/*=========================================================================================================================================
 * @brief     Select Interrupt Upper Threshold
 * @param     APDS9306   APDS9306 Struct INTERRUPT_UPPER_THRESHOLD variable
 * @param     upthr      Value for Upper Threshold 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_Interrupt_Upper_Threshold ( GebraBit_APDS9306 * APDS9306 , uint32_t upthr ) 
{
 GB_APDS9306_Write_Reg_Data(APDS9306_ALS_THRES_UP_0, (uint8_t) (upthr&0x000000FF));
 GB_APDS9306_Write_Reg_Data(APDS9306_ALS_THRES_UP_1, (uint8_t)((upthr&0x0000FF00)>>8));
 GB_APDS9306_Write_Reg_Data(APDS9306_ALS_THRES_UP_2, (uint8_t)((upthr&0x000F0000)>>19));
 APDS9306->INTERRUPT_UPPER_THRESHOLD = upthr ;
}
/*=========================================================================================================================================
 * @brief     Select Interrupt Lower Threshold
 * @param     APDS9306   APDS9306 Struct INTERRUPT_LOWER_THRESHOLD variable
 * @param     lothr      Value for Lower Threshold 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_APDS9306_Interrupt_Lower_Threshold ( GebraBit_APDS9306 * APDS9306 , uint16_t lothr ) 
{
 GB_APDS9306_Write_Reg_Data(APDS9306_ALS_THRES_LOW_0, (uint8_t) (lothr&0x000000FF));
 GB_APDS9306_Write_Reg_Data(APDS9306_ALS_THRES_LOW_1, (uint8_t)((lothr&0x0000FF00)>>8));
 GB_APDS9306_Write_Reg_Data(APDS9306_ALS_THRES_LOW_2, (uint8_t)((lothr&0x000F0000)>>19));
 APDS9306->INTERRUPT_LOWER_THRESHOLD = lothr ;
}
/*=========================================================================================================================================
 * @brief     initialize APDS9306
 * @param     APDS9306     APDS9306 Struct 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_APDS9306_initialize( GebraBit_APDS9306 * APDS9306 )
{
  GB_APDS9306_Soft_Reset   ( APDS9306 ) ;
  GB_APDS9306_Read_Part_ID ( APDS9306 ) ;
//	GB_APDS9306_Interrupt_Upper_Threshold( APDS9306 , 40000);
//	GB_APDS9306_Interrupt_Lower_Threshold( APDS9306 , 30000);
	GB_APDS9306_Interrupt_Mode( APDS9306 , ALS_THRESHOLD_INTERRUPT ) ;
	GB_APDS9306_Interrupt_Channel (  APDS9306 , CLEAR_CHANNEL   );
	GB_APDS9306_Interrupt_Persist( APDS9306 , CONSECUTIVE_5_ALS_VALUE_OUT_OF_THR_RANGE ) ;
	GB_APDS9306_Interrupt( APDS9306 ,  Disable )  ;
}
/*=========================================================================================================================================
 * @brief     Configure APDS9306
 * @param     APDS9306  Configure APDS9306 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_APDS9306_Configuration(GebraBit_APDS9306 * APDS9306)
{
	GB_APDS9306_ALS_Resolution( APDS9306 , _18_BIT_100_mS ) ;
	GB_APDS9306_Measurement_Repeat_Rate( APDS9306 , ALS_MEASRATE_100_mS ) ;
	GB_APDS9306_ALS_Gain( APDS9306 , ALS_GAIN_3X ) ;
	GB_APDS9306_ALS ( APDS9306 ,  Enable );
}
/*=========================================================================================================================================
 * @brief     Get ADC Raw Data  from Register 
 * @param     APDS9306  store Raw Data Of ADC in GebraBit_APDS9306 Struct CLEAR_DATA & ALS_DATA 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_APDS9306_Get_Raw_Data(GebraBit_APDS9306 * APDS9306)
{
	GB_APDS9306_Read_STATUS( APDS9306 );
	if ( APDS9306->DATA== NEW_DATA ) 
	{    
   GB_APDS9306_Burst_Read( APDS9306_CLEAR_DATA_0 , APDS9306->REGISTER_DATA , REGISTER_DATA_BUFFER_SIZE);
	 APDS9306->CLEAR_DATA = ((uint32_t)APDS9306->REGISTER_DATA[2]<<16)|((uint32_t)APDS9306->REGISTER_DATA[1]<<8)|((uint32_t)APDS9306->REGISTER_DATA[0])  ;
   APDS9306->ALS_DATA =   ((uint32_t)APDS9306->REGISTER_DATA[5]<<16)|((uint32_t)APDS9306->REGISTER_DATA[4]<<8)|((uint32_t)APDS9306->REGISTER_DATA[3])  ;
	}
}  
/*=========================================================================================================================================
 * @brief     Calculate Intensity Of Light
 * @param     APDS9306  Staruct LUMINOSITY Variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_APDS9306_Luminosity_Reading(GebraBit_APDS9306 * APDS9306)
{
  APDS9306->LUMINOSITY =(APDS9306->ALS_DATA / APDS9306->ALS_GAIN_VALUE) * (100.0f / APDS9306->ALS_RESOLUTION_TIME); 
} 
/*=========================================================================================================================================
 * @brief     Get Data From Sensor 
 * @param     APDS9306       GebraBit_APDS9306 Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_APDS9306_Get_Data(GebraBit_APDS9306 * APDS9306)
{
  GB_APDS9306_Get_Raw_Data( APDS9306 );
	GB_APDS9306_Luminosity_Reading(APDS9306);
}
/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/
//	GB_APDS9306_Write_Reg_Data(APDS9306_ALS_THRES_UP_0, 0x0A)	;
//	GB_APDS9306_Write_Reg_Data(APDS9306_ALS_THRES_UP_1, 0x0B)	;
//	GB_APDS9306_Write_Reg_Data(APDS9306_ALS_THRES_UP_2, 0x0C)	;
//  GB_APDS9306_Read_Reg_Data (APDS9306_ALS_MEAS_RATE,&APDS9306_Module.Register_Cache);
//	GB_APDS9306_Read_Reg_Data (APDS9306_ALS_GAIN,&APDS9306_Module.Register_Cache);
//	GB_APDS9306_Read_Reg_Data (APDS9306_PART_ID,&APDS9306_Module.Register_Cache);
//	GB_APDS9306_Read_Reg_Data (APDS9306_MAIN_STATUS,&APDS9306_Module.Register_Cache);
//	GB_APDS9306_Read_Reg_Data (APDS9306_INT_CFG,&APDS9306_Module.Register_Cache);
//	HAL_I2C_Mem_Write(APDS9306_I2C,APDS9306_WRITE_ADDRESS,APDS9306_ALS_THRES_UP_0,1,data,3,200);
//	GB_APDS9306_Burst_Read( APDS9306_ALS_THRES_UP_0 , APDS9306_Module.REGISTER_DATA , 3);