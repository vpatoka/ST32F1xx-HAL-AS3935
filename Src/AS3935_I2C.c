/**
  ******************************************************************************
  * @file    AS3935_I2C.c 
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

  Detects both cloud-to-ground and cloud-to-cloud flashes
  Estimates distance to lightning strikes between 1km and 40km in 14 steps
  Embedded ‘man made’ disturbance rejection algorithm
  Lightning detection thresholds are programmable (outdoor vs indoor, for example)
  Wide supply voltage range: 2.4V – 5.5V

  Notes from AS3935 datasheet:
  Register Map: Page 11, detailed version on Page 12

  The SEN-39001 can be used with either I2C or SPI without modification, though pull-up resistors (10k) are
  populated on the board by default. This doesn't seem to hinder operation at room temperature, but may
  have a negative impact on performance in different operating situations. It may be necessary to remove
  these resistors for your specific app. 

  Connections should be made per the 'Circuit' description. This configuration is the setup that has been
  tested by Playing With Fusion engineers.



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
  
  Circuit:
      -->  SEN-39001: AS3935 Breakout
     SDA     -->  MOSI/SDA   (SDA is PB7)
     SCL     -->  SCK/SCL    (SCL is PB6)
     SI:     -->  SI (select interface; GND=SPI, VDD=I2C)
     IRQ:    -->  IRQ (pin. PA8, CH1, TIM1)
     GND:    GND        ''       -->  CS (pull CS to ground even though it's not used)
     GND:    GND        ''       -->  GND
     VC:     5V         ''       -->  STM I/O is tolerant to 5/3.3V, so power board from 5V. 


**************************************************************************/
#include "AS3935_I2C.h"



extern I2C_HandleTypeDef hi2c2;

/* Captured Values */
extern __IO uint32_t               uwIC2Value1;
extern __IO uint32_t               uwIC2Value2;
extern __IO uint32_t               uwDiffCapture;
extern __IO uint8_t                uhCaptureIndex;
extern __IO uint8_t                uhLevent;


// AS3935 Init
AS3935_RESULT AS3935_Init(AS3935_HandleTypeDef* handle) 
{

	if (handle->i2c.State == HAL_I2C_STATE_RESET) {
		handle->i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
		handle->i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
		handle->i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
		handle->i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
		handle->i2c.Init.OwnAddress1 = 0;
		handle->i2c.Init.OwnAddress2 = 0;
		if (HAL_I2C_Init(&handle->i2c) != HAL_OK) {
			handle->errorCallback(AS3935_ERROR);
			return AS3935_ERROR;
		}
	}
	return AS3935_OK;
}


// AS3935 De-Init
AS3935_RESULT AS3935_DeInit(AS3935_HandleTypeDef* handle) 
{
	HAL_I2C_DeInit(&hi2c2);
	return AS3935_OK;
}


/**
  * @brief  Read an amount of data in no-blocking mode with Interrupt from a specific memory address
  * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress: Target device address
  * @param  MemAddress: Internal memory address
  * @param  MemAddSize: Size of internal memory address
  * @param  pData: Pointer to data buffer
  * @param  Size: Amount of data to be sent
  * @retval HAL status
  */
uint8_t AS3935_Read(AS3935_HandleTypeDef* handle, uint8_t reg) 
{
	uint8_t val = 0x00;

	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
	while(HAL_I2C_Mem_Read_IT(&hi2c2, 
			(uint16_t) handle->as3935_i2c_address, (uint16_t)reg, 
			I2C_MEMADD_SIZE_8BIT, &val, 0x0001) != HAL_OK) {
    		if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF) {
			handle->errorCallback(AS3935_ERROR);
		      	Error_Handler();
    		}
  	}
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
	return val;
}



// AS3935 Write Partial Data To Register
AS3935_RESULT AS3935_Write(AS3935_HandleTypeDef* handle, uint8_t reg, uint8_t DataMask, uint8_t RegData) 
{
	uint8_t OrigRegData;
	uint8_t NewRegData;

	OrigRegData = AS3935_Read(handle, reg);
	NewRegData = ((OrigRegData & ~DataMask) | (RegData & DataMask));

	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
	while(HAL_I2C_Mem_Write_IT(&hi2c2, 
			(uint16_t)handle->as3935_i2c_address, (uint16_t)reg, 
			I2C_MEMADD_SIZE_8BIT, &NewRegData, 0x0001) != HAL_OK) {
    		if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF) {
			handle->errorCallback(AS3935_ERROR);
		      	Error_Handler();
    		}
  	}

	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
	return AS3935_OK;
}


// AS3935 Write Full Data To Register
AS3935_RESULT AS3935_WriteReg(AS3935_HandleTypeDef* handle, uint8_t reg, uint8_t val) 
{
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
	while(HAL_I2C_Mem_Write_IT(&hi2c2, 
			(uint16_t) handle->as3935_i2c_address, (uint16_t)reg, 
			I2C_MEMADD_SIZE_8BIT, &val, 0x0001) != HAL_OK) {
    		if (HAL_I2C_GetError(&hi2c2) != HAL_I2C_ERROR_AF) {
			handle->errorCallback(AS3935_ERROR);
		      	Error_Handler();
    		}
  	}
	while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);
	return AS3935_OK;
}


void AS3935_Delay(uint16_t ms)
{
	HAL_Delay(ms);
}

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void AS3935_Reset(AS3935_HandleTypeDef* handle)
{
	// run PRESET_DEFAULT Direct Command to set all registers in default state
	AS3935_WriteReg(handle, (uint8_t)0x3C, (uint8_t)0x96);
	AS3935_Delay(2);					// wait 2ms to complete
}

void AS3935_CalRCO(AS3935_HandleTypeDef* handle)
{
	// run ALIB_RCO Direct Command to cal internal RCO
	AS3935_WriteReg(handle, (uint8_t)0x3D, (uint8_t)0x96);
	AS3935_Delay(2);					// wait 2ms to complete
}


/*
If the AS3935 is set in power-down mode, the TRCO needs to be recalibrated using the following procedure:
1. Send Direct command CALIB_RCO
2. Modify REG0x08[5] = 1
3. Wait 2ms
4. Modify REG0x08[5] = 0
*/
void AS3935_PowerUp(AS3935_HandleTypeDef* handle)
{
	// power-up sequence based on datasheet, pg 23/27
	// register 0x00, PWD bit: 0 (clears PWD)
	AS3935_Write(handle, 0x00, 0x01, 0x00);

	AS3935_CalRCO(handle);

	AS3935_Write(handle, 0x08, 0x20, 0x20);              // set DISP_SRCO to 1

	AS3935_Delay(2);					// wait 2ms to complete

	AS3935_Write(handle, 0x08, 0x20, 0x00);              // set DISP_SRCO to 0
	printf("# AS3935: Powered up\n\r");
}


void AS3935_PowerDown(AS3935_HandleTypeDef* handle)
{
	// register 0x00, PWD bit: 0 (sets PWD)
	AS3935_Write(handle, 0x00, 0x01, 0x01);              // set DISP_SRCO to 0
	printf("# AS3935: Powered down\n\r");
}


void AS3935_DisturberEn(AS3935_HandleTypeDef* handle)
{
	// register 0x03, PWD bit: 5 (sets MASK_DIST)
	AS3935_Write(handle, 0x03, 0x20, 0x00);
	printf("# AS3935: Disturber detection enabled\n\r");
}


void AS3935_DisturberDis(AS3935_HandleTypeDef* handle)
{
	// register 0x03, PWD bit: 5 (sets MASK_DIST)
	AS3935_Write(handle, 0x03, 0x20, 0x20);

}

/*
The clock generation is based on two different RC oscillators: a system RCO (SRCO) and a timer RCO (TRCO). The SRCO will run at about
1.1MHz and provides the main clock for the whole digital part. The TRCO is a low power low frequency oscillator and runs at 32.768 kHz.
Frequency variations in these two oscillators, due to temperature change, are automatically compensated.
*/
void AS3935_SetIRQ_Output_Source(AS3935_HandleTypeDef* handle, uint8_t irq_select)
{
	// set interrupt source - what to displlay on IRQ pin
	// reg 0x08, bits 5 (TRCO), 6 (SRCO), 7 (LCO)
	// only one should be set at once, I think
	// 0 = NONE, 1 = TRCO, 2 = SRCO, 3 = LCO
	
	if(1 == irq_select) 
	{
		AS3935_Write(handle, 0x08, 0xe0, 0x20);              // set only TRCO bit
	}
	else if(2 == irq_select)
	{
		AS3935_Write(handle, 0x08, 0xe0, 0x40);              // set only SRCO bit
	}                                                               
	else if(3 == irq_select)
	{
		AS3935_Write(handle, 0x08, 0xe0, 0x80);              // set only LCO bit
	}                                                               
	else // assume 0
	{
		AS3935_Write(handle, 0x08, 0xe0, 0x00);              // clear IRQ pin display bits
	}
	
}

// Setup value for the Capacitor 
// Assume only numbers divisible by 8 (because that's all the chip supports)
void AS3935_SetTuningCaps(AS3935_HandleTypeDef* handle, uint8_t cap_val)
{
	if(120 < cap_val) 	// cap_value out of range, assume highest capacitance
		cap_val = 0x0f;
	else
		cap_val = (cap_val >> 3);               // translate value to bits

	AS3935_Write(handle, 0x08, 0x0f, cap_val);      // set capacitance bits
	AS3935_Delay(250);
	printf("# AS3935: Capacitance set to %u, Reading it as 8 x %d\n\r", (cap_val<<3), (AS3935_Read(handle, 0x08) & 0x0F) );
}


uint8_t AS3935_GetInterruptSrc(AS3935_HandleTypeDef* handle)
{
	// definition of interrupt data on table 18 of datasheet
	// for this function:
	// 0 = unknown src, 1 = lightning detected, 2 = disturber, 3 = Noise level too high
	AS3935_Delay(10);					// wait 3ms before reading (min 2ms per pg 22 of datasheet)
	uint8_t int_src = (AS3935_Read(handle, 0x03) & 0x0F);	// read register, get rid of non-interrupt data
	if(0x08 == int_src) 
	{
		return 1;					// lightning caused interrupt
	}
	else if(0x04 == int_src)
	{
		return 2;					// disturber detected
	}
	else if(0x01 == int_src)
	{
		return 3;					// Noise level too high
	}
	else{return 0;}					// interrupt result not expected
	
}


uint8_t AS3935_GetLightningDistKm(AS3935_HandleTypeDef* handle)
{
	uint8_t strike_dist = (AS3935_Read(handle, 0x07) & 0x3F);	// read register, get rid of non-distance data
	return strike_dist;
}


uint32_t AS3935_GetStrikeEnergyRaw(AS3935_HandleTypeDef* handle)
{
	uint32_t nrgy_raw = ((AS3935_Read(handle, 0x06) & 0x1F) <<8);				// MMSB, shift 8  bits left, make room for MSB
	nrgy_raw |= (AS3935_Read(handle, 0x05));						// read MSB
	nrgy_raw <<= 8;										// shift 8 bits left, make room for LSB
	nrgy_raw |= (AS3935_Read(handle, 0x04));						// read LSB, add to others
	
	return nrgy_raw;
}


uint8_t AS3935_SetMinStrikes(AS3935_HandleTypeDef* handle, uint8_t min_strk)
{
	// This function sets min strikes to the closest available number, rounding to the floor, 
	// where necessary, then returns the physical value that was set. Options are 1, 5, 9 or 16 strikes.
	// see pg 22 of the datasheet for more info (#strikes in 17 min)
	if(5 > min_strk)
	{
		AS3935_Write(handle, 0x02, 0x30, 0x00);
		return 1;
	}
	else if(9 > min_strk)
	{
		AS3935_Write(handle, 0x02, 0x30, 0x10);
		return 5;
	}
	else if(16 > min_strk)
	{
		AS3935_Write(handle, 0x02, 0x30, 0x20);
		return 9;
	}
	else
	{
		AS3935_Write(handle, 0x02, 0x30, 0x30);
		return 16;
	}
}


void AS3935_SetIndoors(AS3935_HandleTypeDef* handle)
{
	// AFE settings addres 0x00, bits 5:1 (10010, based on datasheet, pg 19, table 15)
	// this is the default setting at power-up (AS3935 datasheet, table 9)
	AS3935_Write(handle, 0x00, 0x3e, 0x24);
	printf("# AS3935: Set up for indoor operation\n\r");
}


void AS3935_SetOutdoors(AS3935_HandleTypeDef* handle)
{
	// AFE settings addres 0x00, bits 5:1 (01110, based on datasheet, pg 19, table 15)
	AS3935_Write(handle, 0x00, 0x3e, 0x1c);
	printf("# AS3935: Set up for outdoor operation\n\r");
}


void AS3935_ClearStatistics(AS3935_HandleTypeDef* handle)
{
	// clear is accomplished by toggling CL_STAT bit 'high-low-high' (then set low to move on)
	AS3935_Write(handle, 0x02, 0x40, 0x40);		// high

	AS3935_Write(handle, 0x02, 0x40, 0x00);		// low

	AS3935_Write(handle, 0x02, 0x40, 0x40);		// high
}


uint8_t AS3935_GetNoiseFloorLvl(AS3935_HandleTypeDef* handle)
{
	// NF settings addres 0x01, bits 6:4
	// default setting of 010 at startup (datasheet, table 9)
	uint8_t reg_raw = AS3935_Read(handle, 0x01);		// read register 0x01
	return ((reg_raw & 0x70)>>4);				// should return value from 0-7, see table 16 for info
}


void AS3935_SetNoiseFloorLvl(AS3935_HandleTypeDef* handle, uint8_t nf_sel)
{
	// NF settings addres 0x01, bits 6:4
	// default setting of 010 at startup (datasheet, table 9)
	if(7 >= nf_sel)								// nf_sel within expected range
	{
		AS3935_Write(handle, 0x01, 0x70, ((nf_sel & 0x07)<<4));
	}
	else
	{											// out of range, set to default (power-up value 010)
		AS3935_Write(handle, 0x01, 0x70, 0x20);
	}
}


uint8_t AS3935_GetWatchdogThreshold(AS3935_HandleTypeDef* handle)
{
	// This function is used to read WDTH. It is used to increase robustness to disturbers,
	// though will make detection less efficient (see page 19, Fig 20 of datasheet)
	// WDTH register: add 0x01, bits 3:0
	// default value of 0001
	// values should only be between 0x00 and 0x0F (0 and 7)
	uint8_t reg_raw = AS3935_Read(handle, 0x01);
	return (reg_raw & 0x0F);
}


void AS3935_SetWatchdogThreshold(AS3935_HandleTypeDef* handle, uint8_t wdth)
{
	// This function is used to modify WDTH. It is used to increase robustness to disturbers,
	// though will make detection less efficient (see page 19, Fig 20 of datasheet)
	// WDTH register: add 0x01, bits 3:0
	// default value of 0001
	// values should only be between 0x00 and 0x0F (0 and 7)
	AS3935_Write(handle, 0x01, 0x0f, (wdth & 0x0F));
}


uint8_t AS3935_GetSpikeRejection(AS3935_HandleTypeDef* handle)
{
	// This function is used to read SREJ (spike rejection). Similar to the Watchdog threshold,
	// it is used to make the system more robust to disturbers, though will make general detection
	// less efficient (see page 20-21, especially Fig 21 of datasheet)
	// SREJ register: add 0x02, bits 3:0
	// default value of 0010
	// values should only be between 0x00 and 0x0F (0 and 7)
	uint8_t reg_raw = AS3935_Read(handle, 0x02);
	return (reg_raw & 0x0F);
}


void AS3935_SetSpikeRejection(AS3935_HandleTypeDef* handle, uint8_t srej)
{
	// This function is used to modify SREJ (spike rejection). Similar to the Watchdog threshold,
	// it is used to make the system more robust to disturbers, though will make general detection
	// less efficient (see page 20-21, especially Fig 21 of datasheet)
	// WDTH register: add 0x02, bits 3:0
	// default value of 0010
	// values should only be between 0x00 and 0x0F (0 and 7)
	AS3935_Write(handle, 0x02, 0x0f, (srej & 0x0F));
}


void AS3935_SetLCO_FDIV(AS3935_HandleTypeDef* handle, uint8_t fdiv)
{
	// This function sets LCO_FDIV register. This is useful in the tuning of the antenna
	// LCO_FDIV register: add 0x03, bits 7:6
	// default value: 00
	// set 0, 1, 2 or 3 for ratios of 16, 32, 64 and 128, respectively. 
	// See pg 23, Table 20 for more info.
	AS3935_Write(handle, 0x03, 0xc0, ((fdiv & 0x03) << 5));
}


void AS3935_PrintAllRegs(AS3935_HandleTypeDef* handle)
{
	printf("# AS3935: Reg 0x00:0x%X\n\r", AS3935_Read(handle, 0x00) );
	printf("#         Reg 0x01:0x%X\n\r", AS3935_Read(handle, 0x01) );
	printf("#         Reg 0x02:0x%X\n\r", AS3935_Read(handle, 0x02) );
	printf("#         Reg 0x03:0x%X\n\r", AS3935_Read(handle, 0x03) );
	printf("#         Reg 0x04:0x%X\n\r", AS3935_Read(handle, 0x04) );
	printf("#         Reg 0x05:0x%X\n\r", AS3935_Read(handle, 0x05) );
	printf("#         Reg 0x06:0x%X\n\r", AS3935_Read(handle, 0x06) );
	printf("#         Reg 0x07:0x%X\n\r", AS3935_Read(handle, 0x07) );
	printf("#         Reg 0x08:0x%X\n\r", AS3935_Read(handle, 0x08) );

	uint32_t nrgy_val = AS3935_GetStrikeEnergyRaw(handle);
	printf("# AS3935: Strike Energy: 0x%X\n\r",  nrgy_val);
}


void AS3935_ManualCal(AS3935_HandleTypeDef* handle, uint8_t capacitance, uint8_t location, uint8_t disturber)
{
	// start by powering up
	AS3935_PowerUp(handle);
	
	// indoors/outdoors next...
	if(1 == location)							// set outdoors if 1
	{
		AS3935_SetOutdoors(handle);
	}
	else										// set indoors if anything but 1
	{
		AS3935_SetIndoors(handle);
	}
	
	// disturber cal
	if(0 == disturber)							// disabled if 0
	{				
		AS3935_DisturberDis(handle);
	}
	else										// enabled if anything but 0
	{
		AS3935_DisturberEn(handle);
	}
	
	AS3935_Delay(5);

	AS3935_SetIRQ_Output_Source(handle, 0);
	
	AS3935_Delay(500);
	// capacitance first... directly write value here
	AS3935_SetTuningCaps(handle, capacitance);
	AS3935_Delay(100);

	// Force to recalibrate frequencies	
	AS3935_CalRCO(handle);
	AS3935_Write(handle, 0x08, 0x20, 0x20);              // set DISP_SRCO to 1
	AS3935_Delay(2);				     // wait 2ms to complete
	AS3935_Write(handle, 0x08, 0x20, 0x00);              // set DISP_SRCO to 0

	printf("# AS3935: Manual calibration complete\n\r");
}


// Check the default values from LUT. Raise the flag if something wrong.
AS3935_RESULT AS3935_CheckUp(AS3935_HandleTypeDef* handle)
{
	uint8_t a = AS3935_Read(handle, 0x31); // Read 0x81 from AS3935
	uint8_t b = AS3935_Read(handle, 0x2e); // Read 0x56 from AS3935
	

	if( (a-b) != 0x2b) {	
		printf("# AS3935: Communication Error, 0x31:0x%X - 0x2e:0x%X\n\r", a, b );
		return AS3935_ERROR;
	} else
		return AS3935_OK;
}

// Auto-Calibration routine
// It will measure the Resonance Freq and tune up capacitor to find the best value
// This Routine using TIM1/CH1 capture mode
uint8_t AS3935_AutoCal(AS3935_HandleTypeDef* handle)
{
    uint32_t uwFrequency;
    uint8_t i, j, bestTuning;
    int32_t errorVal = 0;
    int32_t bestError = 31250; // Worth case

    AS3935_PowerUp(handle);
    AS3935_Reset(handle);
    HAL_Delay(50);
    AS3935_SetLCO_FDIV(handle, 0x00); // This is default and can't be lower than 16. FREQ / 16 --> 500000/16 = 31250.0 (Res Freq / Div factor)
    HAL_Delay(100);
    printf("# AS3935: Capacitor Tuning in progress: ==");
    for (i = 0; i < 16; ++i) {
        // Set tuning and enable LCO output on INT pin
        AS3935_WriteReg(handle, 0x08, (0x80 | i)); 
        AS3935_Delay(100);	// Just in case, wait to stabilize
        for(j=0; j<255; j++) {
            AS3935_Delay(25);
            if(!j)
                uwFrequency = (HAL_RCC_GetPCLK2Freq() / uwDiffCapture);   // 72,000,000 for STM32F1xx for TIM1, since TIM1 is on PCLK2/APB2 BUS
            else {
                uwFrequency += (HAL_RCC_GetPCLK2Freq() / uwDiffCapture);  // Runing Average
                uwFrequency = (uwFrequency >> 1);                         // for 255 samples
            }
        }
        errorVal = 31250 - uwFrequency; // 500000/16 = 31250.0 (Res Freq / Div factor)
        if (errorVal < 0) errorVal = -errorVal;
        if (errorVal < bestError) {
            bestError = errorVal;
            bestTuning = (i<<3);
        }
	printf("==");

    }

    // Power it down, before calibration. Register 0x00, PWD bit: 0 (sets PWD)
    AS3935_Write(handle, 0x00, 0x01, 0x01);              // set DISP_SRCO to 0
    AS3935_Delay(200);
    printf("\n\r# AS3935: Best Capacitor Value calculated as %u\n\r", bestTuning);
    return (bestTuning);
}


// a nice function would be to read the last 'x' strike data values.... 
