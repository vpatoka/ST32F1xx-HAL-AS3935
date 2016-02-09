/**
  ******************************************************************************
  * @file    AS3935_I2C.h
  * @author  V. Patoka
  * @version V0.0.1 alpha
  * @date    31-January-2016
  * @brief   This file includes the driver for AS3935 Franklin Lightning Sensor 
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 Patoka Consulting Inc</center></h2>
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

  **************************************************************************
  ADDITIONAL NOTES:
  This file contains functions to interface with the AS3935 Franklin 
  Lightning Sensor manufactured by AMS. Originally designed for application
  on the Arduino Uno platform.


  **************************************************************************
  APPLICATION SPECIFIC NOTES (READ THIS!!!):
  - This file configures then runs a program on an STM32 to interface with
    an AS3935 Franklin Lightning Sensor manufactured by AMS.
     - Configure STM32
     - Perform setup for AS3935 chip
       --- capacitance registers for tuning (based on cal value provided)
       --- configurations for your application specifics (indoor/outdoor, etc)
     - read status/info from sensor
     - Write formatted information to serial port
  - Set configs for your specific needs using the #defines for wiring, and
    review the setup() function for other settings (indoor/outdoor, for example)
  - I2C specific note: This example uses the I2C interface 

	The device addresses for the AS3935 in read or write mode are defined by:
	0-0-0-0-0-a1-a0-0: write mode device address (DW)
	0-0-0-0-0-a1-a0-1: read mode device address (DR)
	Where a0 and a1 are defined by the pins 5 (ADD0) and 6 (ADD1)
  
  Circuit:
      -->  SEN-39001: AS3935 Breakout
     SDA     -->  MOSI/SDA   (SDA is PB7)
     SCL     -->  SCK/SCL    (SCL is PB6)
     SI:     -->  SI (select interface; GND=SPI, VDD=I2C)
     IRQ:    -->  IRQ (pin. PB5)
     GND:    GND        ''       -->  CS (pull CS to ground even though it's not used)
     GND:    GND        ''       -->  GND
     VC:     5V         ''       -->  STM I/O is tolerant to 5/3.3V, so power board from 5V. 


**************************************************************************/


#ifndef AS3935_I2C_h
#define AS3935_I2C_h

#include "stm32f1xx_hal.h"

#define MAX_PF_CAP		120
#define TUNE_CAP_DIV		8

#define AS3935_CAPACITANCE   56       // <-- SET THIS VALUE TO THE NUMBER LISTED ON YOUR BOARD [56]

// defines for general chip settings
#define AS3935_INDOORS       0
#define AS3935_OUTDOORS      1
#define AS3935_DIST_DIS      0
#define AS3935_DIST_EN       1


/**
 * Provides possible return values for the functions
 */
typedef enum{
	AS3935_OK,		/**< Everything went OK */
	AS3935_ERROR		/**< An error occured */
} AS3935_RESULT;

/** @def AS3935_I2C_ADDRESS and I2C TIMEOUT
 */
#define	AS3935_I2C_ADDRESS   0x06		// x03 - standard PWF SEN-39001-R01 config, However, it is 0-0-0-0-0-a0-a1-0, which is 0x06, not 0x03
#define AS3935_I2C_TIMEOUT   1000

/**
 * AS3935 handle structure which wraps all the necessary variables together in
 * order to simplify the communication with the chip
 */
typedef struct{
	uint8_t				as3935_i2c_address;	/**< address of the chip you want to communicate with */
	uint32_t			as3935_i2c_timeout;	/**< timeout value for the communication in milliseconds */
	I2C_HandleTypeDef 		i2c;			/**< I2C_HandleTypeDef structure */
	void				(*errorCallback)(AS3935_RESULT);
} AS3935_HandleTypeDef;

extern void Error_Handler(void);

/**
 * Initializes the I2C for communication
 * @param	handle - a pointer to the AS3935 handle
 * @return	whether the function was successful or not
 */
AS3935_RESULT AS3935_Init(AS3935_HandleTypeDef* handle);

/**
 * Deinitializes the I2C
 * @param	handle - a pointer to the AS3935 handle
 * @return	whether the function was successful or not
 */
AS3935_RESULT AS3935_DeInit(AS3935_HandleTypeDef* handle);

/**
 * Writes a given value to the port of AS3935
 * @param	handle - a pointer to the AS3935 handle
 * @param	val - a value to be written to the port
 * @return	whether the function was successful or not
 */
AS3935_RESULT AS3935_Write(AS3935_HandleTypeDef* handle, uint8_t reg, uint8_t DataMask, uint8_t RegData);

/**
 * Reads the current state of the port of AS3935
 * @param	handle - a pointer to the AS3935 handle
 * @param	val - a pointer to the variable that will be assigned a value from the chip
 * @return	whether the function was successful or not
 */
uint8_t AS3935_Read(AS3935_HandleTypeDef* handle, uint8_t reg);

// --------------

AS3935_RESULT AS3935_WriteReg(AS3935_HandleTypeDef* handle, uint8_t reg, uint8_t val) ;
AS3935_RESULT AS3935_CheckUp(AS3935_HandleTypeDef* handle);
void AS3935_Delay(uint16_t ms);
void AS3935_Reset(AS3935_HandleTypeDef* handle);
void AS3935_CalRCO(AS3935_HandleTypeDef* handle);
void AS3935_PowerUp(AS3935_HandleTypeDef* handle);
void AS3935_PowerDown(AS3935_HandleTypeDef* handle);
void AS3935_DisturberEn(AS3935_HandleTypeDef* handle);
void AS3935_DisturberDis(AS3935_HandleTypeDef* handle);
void AS3935_SetIRQ_Output_Source(AS3935_HandleTypeDef* handle, uint8_t irq_select);
void AS3935_SetTuningCaps(AS3935_HandleTypeDef* handle, uint8_t cap_val);
uint8_t AS3935_GetInterruptSrc(AS3935_HandleTypeDef* handle);
uint8_t AS3935_GetLightningDistKm(AS3935_HandleTypeDef* handle);
uint32_t AS3935_GetStrikeEnergyRaw(AS3935_HandleTypeDef* handle);
uint8_t AS3935_SetMinStrikes(AS3935_HandleTypeDef* handle, uint8_t min_strk);
void AS3935_SetIndoors(AS3935_HandleTypeDef* handle);
void AS3935_SetOutdoors(AS3935_HandleTypeDef* handle);
void AS3935_ClearStatistics(AS3935_HandleTypeDef* handle);
uint8_t AS3935_GetNoiseFloorLvl(AS3935_HandleTypeDef* handle);
void AS3935_SetNoiseFloorLvl(AS3935_HandleTypeDef* handle, uint8_t nf_sel);
uint8_t AS3935_GetWatchdogThreshold(AS3935_HandleTypeDef* handle);
void AS3935_SetWatchdogThreshold(AS3935_HandleTypeDef* handle, uint8_t wdth);
uint8_t AS3935_GetSpikeRejection(AS3935_HandleTypeDef* handle);
void AS3935_SetSpikeRejection(AS3935_HandleTypeDef* handle, uint8_t srej);
void AS3935_SetLCO_FDIV(AS3935_HandleTypeDef* handle, uint8_t fdiv);
void AS3935_PrintAllRegs(AS3935_HandleTypeDef* handle);
void AS3935_ManualCal(AS3935_HandleTypeDef* handle, uint8_t capacitance, uint8_t location, uint8_t disturber);
uint8_t AS3935_AutoCal(AS3935_HandleTypeDef* handle);




#endif
