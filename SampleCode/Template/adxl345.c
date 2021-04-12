/***************************************************************************//**
 *   @file   ADXL345.c
 *   @brief  Implementation of ADXL345 Driver.
 *   @author ATofan (alexandru.tofan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: $WCREV$
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/
#include <stdio.h>
#include "adxl345.h"

#include "i2c_driver.h"
#include <math.h>
#include	"project_config.h"


#define PI (float)3.14159265f

float RollAng = 0.0f, PitchAng = 0.0f, YawAng = 0.0f;


/******************************************************************************
* @brief Writes to a ADXL345 Internal Register.
*
* @param addr 	- Register address.
* @param txData - Data to be sent.
*
* @return None.
******************************************************************************/
void ADXL345_WriteReg(unsigned char RegAddr, unsigned char txData)
{
	I2C_WriteData(ADXL345_DEVADDR_8BIT ,RegAddr , txData ,1);
}

/******************************************************************************
* @brief Reads from a ADXL345 Internal Register.
*
* @param addr 	- Register address.
*
* @return rxData - Data read from ADXL345.
******************************************************************************/
int ADXL345_ReadReg(unsigned char RegAddr)
{
	unsigned char rxData = 0;

	I2C_ReadData(ADXL345_DEVADDR_8BIT ,RegAddr ,&rxData ,1);	

	return(rxData);
}

/******************************************************************************
* @brief Burst reads from a ADXL345 Internal Register.
*
* @param addr 	- First Register address.
*
* @return rxData - Data read from ADXL345.
******************************************************************************/
int ADXL345_BurstReadReg(unsigned char addr)
{
	unsigned char rxData[2] ={0};

	I2C_ReadData(ADXL345_DEVADDR_8BIT ,addr ,&rxData[0] ,2);	

	return (int)( (rxData[1] << 8) | rxData[0]);
}

void setRate(double rate)
{
	unsigned char rxValue = 0;	
    unsigned char _s;
    int v = (int)(rate / 6.25);
    int r = 0;
	
    while (v >>= 1) 
	{
        r++;
    }
    if (r <= 9) 
	{
        rxValue = ADXL345_ReadReg(ADXL345_BW_RATE);
        _s = (r + 6) | (rxValue & 0xF0);		//B11110000
        ADXL345_WriteReg(ADXL345_BW_RATE, _s);
    }
}

void setMode(unsigned char mode)
{
	unsigned char rxValue = 0;
	
	rxValue = ADXL345_ReadReg(ADXL345_FIFO_CTL);

    rxValue &= ~0xC0; 															//0b11000000 , clearing bit 6 and 7
    rxValue |= (mode << 6); 													//setting op mode
    
    ADXL345_WriteReg(ADXL345_FIFO_CTL, rxValue);	
}

unsigned char getMode(void)
{
	unsigned char rxValue = 0;
	
	rxValue = ADXL345_ReadReg(ADXL345_FIFO_CTL);

    rxValue &= 0xC0; 			//0b11000000 , masking bit 6 and 7
    rxValue = (rxValue >> 6); 					//setting op mode 

	return rxValue;	
}

void setWatermark(unsigned char mode)
{
	unsigned char rxValue = 0;
	unsigned char txValue = 0;
	
	rxValue = ADXL345_ReadReg(ADXL345_FIFO_CTL);

    rxValue &= 0xE0; 															//0b11100000 , clearing bit 0 to 4
    txValue = mode & (0x1F);														//0b00011111 , clearing highest 3 bits in waterlevel
    rxValue |= txValue; 															//setting waterlevel in operationmode register
	
    ADXL345_WriteReg(ADXL345_FIFO_CTL, rxValue);		
}

/******************************************************************************
* @brief Initialize ADXL345.
*
* @param None.
*
* @return None.
******************************************************************************/
void ADXL345_Init(void)
{
	unsigned char temp = 0;
	int x = 0;
	int y = 0;
	int z = 0;
	
	//Turning on the ADXL345
	ADXL345_WriteReg(ADXL345_POWER_CTL, ADXL345_Measure); 

	
    //set activity/ inactivity thresholds (0-255)
	ADXL345_WriteReg(ADXL345_THRESH_ACT, 75); 									//62.5mg per increment
	ADXL345_WriteReg(ADXL345_THRESH_INACT, 75); 								//62.5mg per increment
	ADXL345_WriteReg(ADXL345_TIME_INACT, 10); 									// how many seconds of no activity is inactive?
	
    //look of activity movement on this axes - 1 == on; 0 == off
	ADXL345_WriteReg(ADXL345_ACT_INACT_CTL, ((ADXL345_ACT_X_en)|(ADXL345_ACT_Y_en) |(ADXL345_ACT_Z_en)));

    //look of inactivity movement on this axes - 1 == on; 0 == off
	ADXL345_WriteReg(ADXL345_ACT_INACT_CTL, ((ADXL345_INACT_X_en)|(ADXL345_INACT_Y_en) |(ADXL345_INACT_Z_en)));

    //look of tap movement on this axes - 1 == on; 0 == off
	ADXL345_WriteReg(ADXL345_TAP_AXES, ((ADXL345_TAP_X_en)|(ADXL345_TAP_Y_en) |(ADXL345_TAP_Z_en)));		
//	ADXL345_WriteReg(ADXL345_TAP_AXES, (ADXL345_TAP_Z_en));
	
    //set values for what is a tap, and what is a double tap (0-255)
	ADXL345_WriteReg(ADXL345_THRESH_TAP, 50);									//62.5mg per increment
	ADXL345_WriteReg(ADXL345_DUR, 15); 										//625us per increment
	ADXL345_WriteReg(ADXL345_Latent, 80); 										//1.25ms per increment
	ADXL345_WriteReg(ADXL345_Window, 200); 									//1.25ms per increment

    //set values for what is considered freefall (0-255)
	ADXL345_WriteReg(ADXL345_THRESH_FF, 7); 									//(5 - 9) recommended - 62.5mg per increment
	ADXL345_WriteReg(ADXL345_TIME_FF, 45); 										//(20 - 70) recommended - 5ms per increment 

    //setting all interrupts to take place on int pin 1

	ADXL345_WriteReg(ADXL345_INT_MAP, (ADXL345_INT1_PIN<<ADXL345_SINGLE_TAP)|
										(ADXL345_INT1_PIN<<ADXL345_DOUBLE_TAP)|
										(ADXL345_INT1_PIN<<ADXL345_FREE_FALL)|
										(ADXL345_INT1_PIN<<ADXL345_Activity)|
										(ADXL345_INT1_PIN<<ADXL345_Inactivity));

	ADXL345_WriteReg(ADXL345_INT_ENABLE, (ADXL345_REG_ENABLE<<ADXL345_SINGLE_TAP)|
										(ADXL345_REG_ENABLE<<ADXL345_DOUBLE_TAP)|
										(ADXL345_REG_ENABLE<<ADXL345_FREE_FALL)|
										(ADXL345_REG_ENABLE<<ADXL345_Activity)|
										(ADXL345_REG_ENABLE<<ADXL345_Inactivity));								

	setRate(6.25);
//	ADXL345_WriteReg(ADXL345_BW_RATE, ADXL345_RATE(ADXL345_RATE_BW_100)); 	// Data update rate = 100 Hz	

    //setting device into FIFO mode
    setMode(ADXL345_FIFO_FIFO);
	temp = getMode();
	printf("%s : 0x%2X\r\n" , __FUNCTION__ , temp);

    //set watermark for Watermark interrupt
	setWatermark(30);

	// Data format = +- 16g, Full Resolution
	ADXL345_WriteReg(ADXL345_DATA_FORMAT, (ADXL345_FULL_RES)|(ADXL345_Range(ADXL345_RANGE_PM_16G)));


	ADXL345_ReadAllAxes(&x , &y , &z);
	ADXL345_DisplayAllAxes();

}

/******************************************************************************
* @brief Display g force for desired axis.
*
* @param axis - Which axis to read.
*
* @return None.
******************************************************************************/
float ADXL345_Display_G_Force(char axis)
{
	int 	gForce 		= 0;
	int 	gForceCalc 	= 0;
	float 	value 	= 0;
	int 	whole 		= 0;
	int 	thousands	= 0;

	// Select which axis to read
	switch(axis)
	{
		case 'x':
			gForce = ADXL345_BurstReadReg(ADXL345_DATA_X0);
			printf("[X = ");
			break;
		case 'y':
			gForce = ADXL345_BurstReadReg(ADXL345_DATA_Y0);
			printf("[Y = ");
			break;
		case 'z':
			gForce = ADXL345_BurstReadReg(ADXL345_DATA_Z0);
			printf("[Z = ");
			break;
		default:
			break;
	}

	// If read result is negative, apply padding with 0xFFFFF000
	if((gForce & 0x1000) == 0x1000)
	{
		gForceCalc = 0xFFFFF000 | gForce;
	}
	// If read result is positive, do nothing to modify it
	else
	{
		gForceCalc = gForce & 0x0FFF;
	}

	// Apply HEX to Accel data processing
	value = ((float)gForceCalc * 0.004);

	// Prepare and display data
	// If positive, leave as is
	if(gForceCalc >= 0)
	{
		whole = value;
		thousands = (value - whole) * 1000;

		if(thousands > 99)
		{
			printf("+%d.%3d] ", whole, thousands);
		}
		else if(thousands > 9)
		{
			printf("+%d.0%2d] ", whole, thousands);
		}
		else
		{
			printf("+%d.00%1d] ", whole, thousands);
		}
	}
	// If negative, multiply by (-1) for proper display
	else
	{
		value = value * (-1);
		whole = value;
		thousands = (value - whole) * 1000;

		if(thousands > 99)
		{
			printf("-%d.%3d] ", whole, thousands);
		}
		else if(thousands > 9)
		{
			printf("-%d.0%2d] ", whole, thousands);
		}
		else
		{
			printf("-%d.00%1d] ", whole, thousands);
		}
	}

	return value;
}

/******************************************************************************
* @brief Display all axes.
*
* @param None.
*
* @return None.
******************************************************************************/
void ADXL345_DisplayAllAxes(void)
{
	ADXL345_Display_G_Force('x');
	ADXL345_Display_G_Force('y');
	ADXL345_Display_G_Force('z');
	printf("\n\r");
}

/******************************************************************************
* @brief Read all axes.
*
* @param None.
*
* @return None.
******************************************************************************/
void ADXL345_ReadAllAxes(int* Data_X , int* Data_Y ,int* Data_Z)
{
	*Data_X = ADXL345_BurstReadReg(ADXL345_DATA_X0);
	*Data_Y = ADXL345_BurstReadReg(ADXL345_DATA_Y0);
	*Data_Z = ADXL345_BurstReadReg(ADXL345_DATA_Z0);
}



/******************************************************************************
* @brief External interrupt function.
*
* @param None.
*
* @return None.
******************************************************************************/
void ADIExtIntrFunction(void)
{
	unsigned char intSource  = 0;
	unsigned char intEnabled = 0;
	int x = 0;
	int y = 0;
	int z = 0;
	
	// Read ADXL345 Interrupt Source
	intSource = ADXL345_ReadReg(ADXL345_INT_SOURCE);
	// Read ADXL345 Enabled Interrupts
	intEnabled = ADXL345_ReadReg(ADXL345_INT_ENABLE);

	// If Interrupt Enabled and Triggered is Single Tap
	if((intSource & (1<<ADXL345_SINGLE_TAP)) && (intEnabled & (1<<ADXL345_SINGLE_TAP)))
	{

	}
	// If Interrupt Enabled and Triggered is Double Tap
	if((intSource & (1<<ADXL345_DOUBLE_TAP)) && (intEnabled & (1<<ADXL345_DOUBLE_TAP)))
	{

	}

    // Clear interrupts by reading all axes registers
	ADXL345_ReadAllAxes(&x , &y , &z);

}


void Angle_Calculate(void)
{  
//	float s1 = 0.0f;
//	float s2 = 0.0f;	
	static float filteredXg = 0.0f;
	static float filteredYg = 0.0f;
	static float filteredZg = 0.0f;
	const float alpha = 0.5f;

	float multipleXg = 0.0f;
	float multipleYg = 0.0f;
	float multipleZg = 0.0f;

	float ax = 0.0f;
	float ay = 0.0f;
	float az = 0.0f;

	ax = ADXL345_Display_G_Force('x');
	ay = ADXL345_Display_G_Force('y');
	az = ADXL345_Display_G_Force('z');
	

	/*
		https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data
	*/

	#if 0
	s1 = sqrt((float)((ay *ay )+(az *az )));
	s2 = sqrt((float)((ax *ax )+(az *az )));

	PitchAng = atan(ax /s1)*180/PI;
	RollAng = atan(ay /s2)*180/PI;
	YawAng = atan(az /s2)*180/PI;

//	PitchAng = atan(ax /s1)*57.295779;
//	RollAng = atan(ay /s2)*57.295779;
	
	#else
    //Low Pass Filter
    filteredXg = ax * alpha + (filteredXg * (1.0f - alpha));
    filteredYg = ay * alpha + (filteredYg * (1.0f - alpha));
    filteredZg = az * alpha + (filteredZg * (1.0f - alpha));

	multipleXg = ax*ax;
	multipleYg = ay*ay;
	multipleZg = az*az;
	
	PitchAng = 180 * atan (ax/sqrt(multipleYg + multipleZg))/PI;
	RollAng = 180 * atan (ay/sqrt(multipleXg + multipleZg))/PI;
	YawAng = 180 * atan (az/sqrt(multipleXg + multipleZg))/PI;

//    PitchAng = (atan2(-ax, sqrt(ay*ay + az*az))*180.0)/PI;	
//    RollAng  = (atan2(ay, az)*180.0)/PI;
	#endif
	

	#if (_debug_log_ADXL345_ == 1)	//debug

	if (is_flag_set(flag_switch_display))
	{
		printf("Acc:%8.3lf,%8.3lf,%8.3lf,",filteredXg ,filteredYg ,filteredZg );

		printf("\r\n");
	}
	else
	{		
		printf("Pitch:%8.3lf,",PitchAng);
		printf("Roll:%8.3lf,",RollAng);
		printf("Yaw:%8.3lf,",YawAng);
		
		printf("\r\n");
	}
	
	#endif
}



