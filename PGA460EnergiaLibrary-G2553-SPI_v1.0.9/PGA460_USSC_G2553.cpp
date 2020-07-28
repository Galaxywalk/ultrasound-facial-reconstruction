/*
	PGA460_USSC.cpp
	
	BSD 2-clause "Simplified" License
	Copyright (c) 2017, Texas Instruments
	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the above copyright notice, this
	   list of conditions and the following disclaimer.
	2. Redistributions in binary form must reproduce the above copyright notice,
	   this list of conditions and the following disclaimer in the documentation
	   and/or other materials provided with the distribution.

	THIS SOFTWARE IS PROVIDED BY TEXAS INSTRUMENTS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
	DISCLAIMED. IN NO EVENT SHALL TEXAS INSTRUMENTS BE LIABLE FOR
	ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
	SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	The views and conclusions contained in the software and documentation are those
	of the authors and should not be interpreted as representing official policies,
	either expressed or implied, of the FreeBSD Project.
	
	Last Updated: Nov 2017
	By: A. Whitehead <make@energia.nu>
*/

#include "PGA460_USSC_G2553.h"
#include "PGA460_SPI_G2553.h"
#include "Energia.h"

/*------------------------------------------------- Global Variables -----
 |  Global Variables
 |
 |  Purpose:  Variables shared throughout the PGA460_USSC.cpp functions
 *-------------------------------------------------------------------*/
#pragma region globals
// Pin mapping of BOOSTXL-PGA460 to LaunchPad by pin name
	#define MEM_SOMI 9
	#define MEM_SIMO 10
	#define COM_SEL 17
	#define COM_PD 18
	#define SPI_CS 33
	#define SCLK_CLK 34
	#define MEM_HOLD 36
	#define MEM_CS 37
	#define DS1_LED 38
	#define F_DIAG_LED 39
	#define V_DIAG_LED 40

// Define UART commands by name
	// Single Address
		byte P1BL = 0x00;
		byte P2BL = 0x01;
		byte P1LO = 0x02;
		byte P2LO = 0x03;
		byte TNLM = 0x04;
		byte UMR = 0x05;
		byte TNLR = 0x06;
		byte TEDD = 0x07;
		byte SD = 0x08;
		byte SRR = 0x09; 
		byte SRW = 0x0A;
		byte EEBR = 0x0B;
		byte EEBW = 0x0C;
		byte TVGBR = 0x0D;
		byte TVGBW = 0x0E;
		byte THRBR = 0x0F;
		byte THRBW = 0x10; 
	//Broadcast
		byte BC_P1BL = 0x11;
		byte BC_P2BL = 0x12;
		byte BC_P1LO = 0x13;
		byte BC_P2LO = 0x14;
		byte BC_TNLM = 0x15;
		byte BC_RW = 0x16;
		byte BC_EEBW = 0x17;
		byte BC_TVGBW = 0x18;
		byte BC_THRBW = 0x19;
		//CMDs 26-31 are reserved

// List user registers by name with default settings from TI factory
	byte USER_DATA1 = 0x00;
	byte USER_DATA2 = 0x00;
	byte USER_DATA3 = 0x00;
	byte USER_DATA4 = 0x00;
	byte USER_DATA5 = 0x00;
	byte USER_DATA6 = 0x00;
	byte USER_DATA7 = 0x00;
	byte USER_DATA8 = 0x00;
	byte USER_DATA9 = 0x00;
	byte USER_DATA10 = 0x00;
	byte USER_DATA11 = 0x00;
	byte USER_DATA12 = 0x00;
	byte USER_DATA13 = 0x00;
	byte USER_DATA14 = 0x00;
	byte USER_DATA15 = 0x00;
	byte USER_DATA16 = 0x00;
	byte USER_DATA17 = 0x00;
	byte USER_DATA18 = 0x00;
	byte USER_DATA19 = 0x00;
	byte USER_DATA20 = 0x00;
	byte TVGAIN0 = 0xAF;
	byte TVGAIN1 = 0xFF;
	byte TVGAIN2 = 0xFF;
	byte TVGAIN3 = 0x2D;
	byte TVGAIN4 = 0x68;
	byte TVGAIN5 = 0x36;
	byte TVGAIN6 = 0xFC;
	byte INIT_GAIN = 0xC0;
	byte FREQUENCY  = 0x8C;
	byte DEADTIME = 0x00;
	byte PULSE_P1 = 0x01;
	byte PULSE_P2 = 0x12;
	byte CURR_LIM_P1 = 0x47;
	byte CURR_LIM_P2 = 0xFF;
	byte REC_LENGTH = 0x1C;
	byte FREQ_DIAG = 0x00;
	byte SAT_FDIAG_TH = 0xEE;
	byte FVOLT_DEC = 0x7C;
	byte DECPL_TEMP = 0x0A;
	byte DSP_SCALE = 0x00;
	byte TEMP_TRIM = 0x00;
	byte P1_GAIN_CTRL = 0x00;
	byte P2_GAIN_CTRL = 0x00;
	byte EE_CRC = 0xFF;
	byte EE_CNTRL = 0x00;
	byte P1_THR_0 = 0x88;
	byte P1_THR_1 = 0x88;
	byte P1_THR_2 = 0x88;
	byte P1_THR_3 = 0x88;
	byte P1_THR_4 = 0x88;
	byte P1_THR_5 = 0x88;
	byte P1_THR_6 = 0x84;
	byte P1_THR_7 = 0x21;
	byte P1_THR_8 = 0x08;
	byte P1_THR_9 = 0x42;
	byte P1_THR_10 = 0x10;
	byte P1_THR_11 = 0x80;
	byte P1_THR_12 = 0x80;
	byte P1_THR_13 = 0x80;
	byte P1_THR_14 = 0x80;
	byte P1_THR_15 = 0x80;
	byte P2_THR_0 = 0x88;
	byte P2_THR_1 = 0x88;
	byte P2_THR_2 = 0x88;
	byte P2_THR_3 = 0x88;
	byte P2_THR_4 = 0x88;
	byte P2_THR_5 = 0x88;
	byte P2_THR_6 = 0x84;
	byte P2_THR_7 = 0x21;
	byte P2_THR_8 = 0x08;
	byte P2_THR_9 = 0x42;
	byte P2_THR_10 = 0x10;
	byte P2_THR_11 = 0x80;
	byte P2_THR_12 = 0x80;
	byte P2_THR_13 = 0x80;
	byte P2_THR_14 = 0x80;
	byte P2_THR_15 = 0x80;

// Miscellaneous variables; (+) indicates OWU transmitted byte offset
	byte checksum = 0x00; 			// UART checksum value	
	byte ChecksumInput[44]; 		// data byte array for checksum calculator
	byte ultraMeasResult[34]; 	// data byte array for cmd5 and tciB+L return
	byte diagMeasResult[5]; 		// data byte array for cmd8 and index1 return
	byte tempNoiseMeasResult[4]; 	// data byte array for cmd6 and index0&1 return
	byte echoDataDump[130]; 		// data byte array for cmd7 and index12 return
	byte tempOrNoise = 0; 			// data byte to determine if temp or noise measurement is to be performed
	byte bulkThr[34];				// data byte array for bulk threhsold commands
	//UART & OWU exclusive variables
		byte syncByte = 0x55; 		// data byte for Sync field set UART baud rate of PGA460
		byte regAddr = 0x00; 		// data byte for Register Address
		byte regData = 0x00; 		// data byte for Register Data
		byte uartAddr = 0; 			// PGA460 UART device address (0-7). '0' is factory default address
		byte numObj = 1; 			// number of objects to detect
	//SPI exclusive variables
		byte misoBuf[131]; 				// SPI MISO receive data buffer for all commands	
#pragma endregion globals

/*------------------------------------------------- PGA460 Top Level -----
 |  PGA460 Top Level Scope Resolution Operator
 |
 | Use the double colon operator (::) to qualify a C++ member function, a top
 | level function, or a variable with global scope with:
 | • An overloaded name (same name used with different argument types)
 | • An ambiguous name (same name used in different classes)
 *-------------------------------------------------------------------*/
pga460::pga460(){}

/*------------------------------------------------- initBoostXLPGA460 -----
 |  Function initBoostXLPGA460
 |
 |  Purpose:  Configure the master communication mode and BOOSTXL-PGA460 hardware to operate in UART, TCI, or OWU mode.
 |  Configures master serial baud rate for UART/OWU modes. Updates UART address based on sketch input.
 |
 |  Parameters:
 |		mode (IN) -- sets communicaiton mode. 
 |			0=UART 
 |			1=TCI 
 |			2=OWU 
 |			3-SPI (Synchronous Mode)
 |			4 = Not Used
 |			5 = Not Used
 |			6=Bus_Demo_Bulk_TVG_or_Threshold_Broadcast_is_True
 |			7=Bus_Demo_UART_Mode
 |			8=Bus_Demo_OWU_One_Time_Setup
 |			9=Bus_Demo_OWU_Mode
 | 		baud (IN) -- PGA460 accepts a baud rate of 9600 to 115.2k bps
 | 		uartAddrUpdate (IN) -- PGA460 address range from 0 to 7
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::initBoostXLPGA460(byte mode, uint32_t baud, byte uartAddrUpdate)
{
	// check for valid UART address
	if (uartAddrUpdate > 7)
	{
		uartAddrUpdate = 0; // default to '0'
		Serial.println("ERROR - Invalid UART Address!");
	}
	// globally update target PGA460 UART address and commands	
	if (uartAddr != uartAddrUpdate)
	{
		// Update commands to account for new UART addr
		  // Single Address
		   P1BL = 0x00 + (uartAddrUpdate << 5);	   
		   P2BL = 0x01 + (uartAddrUpdate << 5);
		   P1LO = 0x02 + (uartAddrUpdate << 5);
		   P2LO = 0x03 + (uartAddrUpdate << 5);
		   TNLM = 0x04 + (uartAddrUpdate << 5);
		   UMR = 0x05 + (uartAddrUpdate << 5);
		   TNLR = 0x06 + (uartAddrUpdate << 5);
		   TEDD = 0x07 + (uartAddrUpdate << 5);
		   SD = 0x08 + (uartAddrUpdate << 5);
		   SRR = 0x09 + (uartAddrUpdate << 5); 
		   SRW = 0x0A + (uartAddrUpdate << 5);
		   EEBR = 0x0B + (uartAddrUpdate << 5);
		   EEBW = 0x0C + (uartAddrUpdate << 5);
		   TVGBR = 0x0D + (uartAddrUpdate << 5);
		   TVGBW = 0x0E + (uartAddrUpdate << 5);
		   THRBR = 0x0F + (uartAddrUpdate << 5);
		   THRBW = 0x10 + (uartAddrUpdate << 5); 
	}
	uartAddr = uartAddrUpdate;
	
	// turn on LP's Red LED to indicate code has started to run
	pinMode(RED_LED, OUTPUT); digitalWrite(RED_LED, HIGH);

	// turn off BOOSTXL-PGA460's diagnostic LEDs
	//pinMode(DS1_LED, OUTPUT); digitalWrite(DS1_LED, LOW);
	//pinMode(F_DIAG_LED, OUTPUT); digitalWrite(F_DIAG_LED, LOW);
	//pinMode(V_DIAG_LED, OUTPUT); digitalWrite(V_DIAG_LED, LOW);

	// set communication mode flag

		// disable synchronous mode dump to external memory
		//pinMode(MEM_HOLD, OUTPUT);  digitalWrite(MEM_HOLD, HIGH);
		//pinMode(MEM_CS, OUTPUT);  digitalWrite(MEM_CS, HIGH);

		usscSPIVL.begin();            	// start the SPI for BOOSTXL-PGA460 library
		usscSPIVL.setBitOrder(0);     	// set bit order to LSB first
		//In this mode the USART interface acts as a serial-shift register with data set on the rising edge of the clock and sampled on the falling edge of the clock.
		usscSPIVL.setDataMode(2);     	// set the data mode for clock is High when inactive (CPOL=1) & data is valid on clock leading edge (CPHA = 0) (SPI_MODE2)
		usscSPIVL.setClockDivider(baud); 	// set clock divider (16MHz master)
		//Serial.begin(9600); 			// initialize COM UART serial channel

		//// manually config chip select for debug (not used by PGA460)
		pinMode(SPI_CS, OUTPUT); digitalWrite(SPI_CS, HIGH);
		
		// enable PGA460 SPI communication mode
		//pinMode(COM_SEL, OUTPUT);  digitalWrite(COM_SEL, HIGH);

	
	// Visibly show initilization status
        /*
		digitalWrite(GREEN_LED, HIGH);
		for(int loops = 0; loops < 5; loops++)
		{
			digitalWrite(GREEN_LED, HIGH); 
			delay(200);
			digitalWrite(GREEN_LED, LOW);
			delay(200);
		}	
		*/
	
	return;
}

/*------------------------------------------------- defaultPGA460 -----
 |  Function defaultPGA460
 |
 |  Purpose:  Updates user EEPROM values, and performs bulk EEPROM write.
 |
 |  Parameters:
 |		xdcr (IN) -- updates user EEPROM based on predefined listing for a specific transducer.
 |			Modify existing case statements, or append additional case-statement for custom user EEPROM configurations.
 |			• 0 = Murata MA58MF14-7N
 |			• 1 = Murata MA40H1S-R
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::defaultPGA460(byte xdcr)
{
		   USER_DATA1 = 0x00;
		   USER_DATA2 = 0x00;
		   USER_DATA3 = 0x00;
		   USER_DATA4 = 0x00;
		   USER_DATA5 = 0x00;
		   USER_DATA6 = 0x00;
		   USER_DATA7 = 0x00;
		   USER_DATA8 = 0x00;
		   USER_DATA9 = 0x00;
		   USER_DATA10 = 0x00;
		   USER_DATA11 = 0x00;
		   USER_DATA12 = 0x00;
		   USER_DATA13 = 0x00;
		   USER_DATA14 = 0x00;
		   USER_DATA15 = 0x00;
		   USER_DATA16 = 0x00;
		   USER_DATA17 = 0x00;
		   USER_DATA18 = 0x00;
		   USER_DATA19 = 0x00;
		   USER_DATA20 = 0x00;
		   TVGAIN0 = 0xAA;
		   TVGAIN1 = 0xAA;
		   TVGAIN2 = 0xAA;
		   TVGAIN3 = 0x82;
		   TVGAIN4 = 0x08;
		   TVGAIN5 = 0x20;
		   TVGAIN6 = 0x80;
		   INIT_GAIN = 0x60;
		   FREQUENCY  = 0x8F;
		   DEADTIME = 0xA0;
		   PULSE_P1 = 0x04;
		   PULSE_P2 = 0x10;
		   CURR_LIM_P1 = 0x55;
		   CURR_LIM_P2 = 0x55;
		   REC_LENGTH = 0x19;
		   FREQ_DIAG = 0x33;
		   SAT_FDIAG_TH = 0xEE;
		   FVOLT_DEC = 0x7C;
		   DECPL_TEMP = 0x4F;
		   DSP_SCALE = 0x00;
		   TEMP_TRIM = 0x00;
		   P1_GAIN_CTRL = 0x09;
		   P2_GAIN_CTRL = 0x09;	
		
	
			byte buf12[46] = {syncByte, EEBW, USER_DATA1, USER_DATA2, USER_DATA3, USER_DATA4, USER_DATA5, USER_DATA6,
				USER_DATA7, USER_DATA8, USER_DATA9, USER_DATA10, USER_DATA11, USER_DATA12, USER_DATA13, USER_DATA14, 
				USER_DATA15,USER_DATA16,USER_DATA17,USER_DATA18,USER_DATA19,USER_DATA20,
				TVGAIN0,TVGAIN1,TVGAIN2,TVGAIN3,TVGAIN4,TVGAIN5,TVGAIN6,INIT_GAIN,FREQUENCY,DEADTIME,
				PULSE_P1,PULSE_P2,CURR_LIM_P1,CURR_LIM_P2,REC_LENGTH,FREQ_DIAG,SAT_FDIAG_TH,FVOLT_DEC,DECPL_TEMP,
				DSP_SCALE,TEMP_TRIM,P1_GAIN_CTRL,P2_GAIN_CTRL,calcChecksum(EEBW)};
		

				spiTransfer(buf12, sizeof(buf12));

			delay(50);
			
			// Update targeted UART_ADDR to address defined in EEPROM bulk switch-case
			byte uartAddrUpdate = (PULSE_P2 >> 5) & 0x07;
			if (uartAddr != uartAddrUpdate)
			{
				// Update commands to account for new UART addr
				  // Single Address
				   P1BL = 0x00 + (uartAddrUpdate << 5);	   
				   P2BL = 0x01 + (uartAddrUpdate << 5);
				   P1LO = 0x02 + (uartAddrUpdate << 5);
				   P2LO = 0x03 + (uartAddrUpdate << 5);
				   TNLM = 0x04 + (uartAddrUpdate << 5);
				   UMR = 0x05 + (uartAddrUpdate << 5);
				   TNLR = 0x06 + (uartAddrUpdate << 5);
				   TEDD = 0x07 + (uartAddrUpdate << 5);
				   SD = 0x08 + (uartAddrUpdate << 5);
				   SRR = 0x09 + (uartAddrUpdate << 5); 
				   SRW = 0x0A + (uartAddrUpdate << 5);
				   EEBR = 0x0B + (uartAddrUpdate << 5);
				   EEBW = 0x0C + (uartAddrUpdate << 5);
				   TVGBR = 0x0D + (uartAddrUpdate << 5);
				   TVGBW = 0x0E + (uartAddrUpdate << 5);
				   THRBR = 0x0F + (uartAddrUpdate << 5);
				   THRBW = 0x10 + (uartAddrUpdate << 5);				
			}
			uartAddr = uartAddrUpdate;

	return;
}

/*------------------------------------------------- initThresholds -----
 |  Function initThresholds
 |
 |  Purpose:  Updates threshold mapping for both presets, and performs bulk threshold write
 |
 |  Parameters:
 |		thr (IN) -- updates all threshold levels to a fixed level based on specific percentage of the maximum level. 
 |			All times are mid-code (1.4ms intervals).
 |			Modify existing case statements, or append additional case-statement for custom user threshold configurations.
 |			• 0 = 25% Levels 64 of 255
 |			• 1 = 50% Levels 128 of 255
 |			• 2 = 75% Levels 192 of 255
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::initThresholds(byte thr)
{
		   P1_THR_0 = 0x88;
		   P1_THR_1 = 0x88;
		   P1_THR_2 = 0x88;
		   P1_THR_3 = 0x88;
		   P1_THR_4 = 0x88;
		   P1_THR_5 = 0x88;
		   P1_THR_6 = 0x84;
		   P1_THR_7 = 0x21;
		   P1_THR_8 = 0x08;
		   P1_THR_9 = 0x42;
		   P1_THR_10 = 0x10;
		   P1_THR_11 = 0x80;
		   P1_THR_12 = 0x80;
		   P1_THR_13 = 0x80;
		   P1_THR_14 = 0x80;
		   P1_THR_15 = 0x00;
		   P2_THR_0 = 0x88;
		   P2_THR_1 = 0x88;
		   P2_THR_2 = 0x88;
		   P2_THR_3 = 0x88;
		   P2_THR_4 = 0x88;
		   P2_THR_5 = 0x88;
		   P2_THR_6 = 0x84;
		   P2_THR_7 = 0x21;
		   P2_THR_8 = 0x08;
		   P2_THR_9 = 0x42;
		   P2_THR_10 = 0x10;
		   P2_THR_11 = 0x80;
		   P2_THR_12 = 0x80;
		   P2_THR_13 = 0x80;
		   P2_THR_14 = 0x80;
		   P2_THR_15 = 0x00;		
		
		byte buf16[35] = {syncByte, THRBW, P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4, P1_THR_5, P1_THR_6,
			  P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10, P1_THR_11, P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15,
			  P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5, P2_THR_6, 
			  P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10, P2_THR_11, P2_THR_12, P2_THR_13, P2_THR_14, P2_THR_15,
			  calcChecksum(THRBW)};

			spiTransfer(buf16, sizeof(buf16));
	
	delay(100);
	return;
}

/*------------------------------------------------- initTVG -----
 |  Function initTVG
 |
 |  Purpose:  Updates time varying gain (TVG) range and mapping, and performs bulk TVG write
 |
 |  Parameters:
 |		agr (IN) -- updates the analog gain range for the TVG.
 |			• 0 = 32-64dB
 |			• 1 = 46-78dB
 |			• 2 = 52-84dB
 |			• 3 = 58-90dB
 |		tvg (IN) -- updates all TVG levels to a fixed level based on specific percentage of the maximum level. 
 |			All times are mid-code (2.4ms intervals).
 |			Modify existing case statements, or append additional case-statement for custom user TVG configurations
 |			• 0 = 25% Levels of range
 |			• 1 = 50% Levels of range
 |			• 2 = 75% Levels of range
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::initTVG(byte agr, byte tvg)
{
	byte gain_range = 0x4F;
	// set AFE gain range
		byte buf10[5] = {syncByte, SRW, regAddr, regData, calcChecksum(SRW)};
			spiTransfer(buf10, sizeof(buf10));

	
	//Set fixed AFE gain value

		   TVGAIN0 = 0x88;
		   TVGAIN1 = 0x88;
		   TVGAIN2 = 0x88;
		   TVGAIN3 = 0x82;
		   TVGAIN4 = 0x08;
		   TVGAIN5 = 0x20;
		   TVGAIN6 = 0x80;	


		byte buf14[10] = {syncByte, TVGBW, TVGAIN0, TVGAIN1, TVGAIN2, TVGAIN3, TVGAIN4, TVGAIN5, TVGAIN6, calcChecksum(TVGBW)};

			spiTransfer(buf14, sizeof(buf14));
	
	return;
}

/*------------------------------------------------- ultrasonicCmd -----
 |  Function ultrasonicCmd
 |
 |  Purpose:  Issues a burst-and-listen or listen-only command based on the number of objects to be detected.
 |
 |  Parameters:
 |		cmd (IN) -- determines which preset command is run
 |			• 0 = Preset 1 Burst + Listen command
 |			• 1 = Preset 2 Burst + Listen command
 |			• 2 = Preset 1 Listen Only command
 |			• 3 = Preset 2 Listen Only command
 |		numObjUpdate (IN) -- PGA460 can capture time-of-flight, width, and amplitude for 1 to 8 objects. 
 |			TCI is limited to time-of-flight measurement data only.
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::ultrasonicCmd(byte cmd, byte numObjUpdate)
{	
	numObj = numObjUpdate; // number of objects to detect
	byte bufCmd[4] = {syncByte, 0xFF, numObj, 0xFF}; // prepare bufCmd with 0xFF placeholders
	
	switch (cmd)
	{
		// SINGLE ADDRESS		
		case 0: // Send Preset 1 Burst + Listen command
		{			
			bufCmd[1] = P1BL;
			bufCmd[3] = calcChecksum(P1BL);
			break;
		}
		case 1: // Send Preset 2 Burst + Listen command
		{			
			bufCmd[1] = P2BL;
			bufCmd[3] = calcChecksum(P2BL);
			break;
		}	
		case 2: // Send Preset 1 Listen Only command
		{			
			bufCmd[1] = P1LO;
			bufCmd[3] = calcChecksum(P1LO);
			break;
		}
		case 3: // Send Preset 2 Listen Only command
		{			
			bufCmd[1] = P2LO;
			bufCmd[3] = calcChecksum(P2LO);
			break;
		}	
		
		// BROADCAST
		case 17: // Send Preset 1 Burst + Listen Broadcast command
		{			
			bufCmd[1] = BC_P1BL;
			bufCmd[3] = calcChecksum(BC_P1BL);
			break;
		}
		case 18: // Send Preset 2 Burst + Listen Broadcast command
		{			
			bufCmd[1] = BC_P2BL;
			bufCmd[3] = calcChecksum(BC_P2BL);
			break;
		}	
		case 19: // Send Preset 1 Listen Only Broadcast command
		{			
			bufCmd[1] = BC_P1LO;
			bufCmd[3] = calcChecksum(BC_P1LO);
			break;
		}
		case 20: // Send Preset 2 Listen Only Broadcast command
		{			
			bufCmd[1] = BC_P2LO;
			bufCmd[3] = calcChecksum(BC_P2LO);
			break;
		}		
		
		default: return;	
	}		

			spiTransfer(bufCmd, sizeof(bufCmd));
	
	delay(70); // maximum record length is 65ms, so delay with margin
	return;
}

/*------------------------------------------------- pullUltrasonicMeasResult -----
 |  Function pullUltrasonicMeasResult
 |
 |  Purpose:  Read the ultrasonic measurement result data based on the last busrt and/or listen command issued.
 |	Only applicable to UART and OWU modes.
 |
 |  Parameters:
 |		busDemo (IN) -- When true, do not print error message for a failed reading when running bus demo
 |
 |  Returns:  If measurement data successfully read, return true.
 *-------------------------------------------------------------------*/
bool pga460::pullUltrasonicMeasResult(bool busDemo)
{

		
		memset(ultraMeasResult, 0, sizeof(ultraMeasResult));
		
		
		byte buf5[3] = {syncByte, UMR, calcChecksum(UMR)};

			spiTransfer(buf5, sizeof(buf5));


			// MOSI transmit 0xFF to pull MISO return data
			spiMosiIdle(numObj*4+1);
			
			// copy MISO global array data to local array based on number of objects
			for(int n=0; n<((2+(numObj*4))+0); n++)
			{			
			   ultraMeasResult[n+1] = misoBuf[n];	   
			}
		
	return true;
}

/*------------------------------------------------- printUltrasonicMeasResult -----
 |  Function printUltrasonicMeasResult
 |
 |  Purpose:  Converts time-of-flight readout to distance in meters. 
 |		Width and amplitude data only available in UART or OWU mode.
 |
 |  Parameters:
 |		umr (IN) -- Ultrasonic measurement result look-up selector:
 |				Distance (m)	Width	Amplitude
 |				--------------------------------
 |			Obj1		0		1		2
 |			Obj2		3		4		5
 |			Obj3		6		7		8
 |			Obj4		9		10		11
 |			Obj5		12		13		14
 |			Obj6		15		16		17
 |			Obj7		18		19		20
 |			Obj8		21		22		23
 |
 |  Returns:  double representation of distance (m), width (us), or amplitude (8-bit)
 *-------------------------------------------------------------------*/
double pga460::printUltrasonicMeasResult(byte umr)
{
	int speedSound = 343; // speed of sound in air at room temperature
	pga460::printUltrasonicMeasResultExt(umr, speedSound);
}
byte pga460::printUltrasonicMeasResultRaw(byte umr)
{
	return ultraMeasResult[umr];
}
double pga460::printUltrasonicMeasResultExt(byte umr, int speedSound)
{
	double objReturn = 0;
	uint16_t objDist = 0;
	uint16_t objWidth = 0;
	uint16_t objAmp = 0;
	
	switch (umr)
	{
		case 0: //Obj1 Distance (m)
		{
			objDist = (ultraMeasResult[1]<<8) + ultraMeasResult[2];
			objReturn = (objDist/2*0.000001*speedSound) ;
			break;
		}
		case 1: //Obj1 Width (us)
		{
			objWidth = ultraMeasResult[3];
			objReturn= objWidth * 16;
			break;
		}
		case 2: //Obj1 Peak Amplitude
		{
			objAmp = ultraMeasResult[4];
			objReturn= objAmp;
			break;
		}
		
		case 3: //Obj2 Distance (m)
		{
			objDist = (ultraMeasResult[5]<<8) + ultraMeasResult[6];
			objReturn = (objDist/2*0.000001*speedSound) ;
			break;
		}
		case 4: //Obj2 Width (us)
		{
			objWidth = ultraMeasResult[7];
			objReturn= objWidth * 16;
			break;
		}
		case 5: //Obj2 Peak Amplitude
		{
			objAmp = ultraMeasResult[8];
			objReturn= objAmp;
			break;
		}
		
		case 6: //Obj3 Distance (m)
		{
			objDist = (ultraMeasResult[9]<<8) + ultraMeasResult[10];
			objReturn = (objDist/2*0.000001*speedSound) ;
			break;
		}
		case 7: //Obj3 Width (us)
		{
			objWidth = ultraMeasResult[11];
			objReturn= objWidth * 16;
			break;
		}
		case 8: //Obj3 Peak Amplitude
		{
			objAmp = ultraMeasResult[12];
			objReturn= objAmp;
			break;
		}
		case 9: //Obj4 Distance (m)
		{
			objDist = (ultraMeasResult[13]<<8) + ultraMeasResult[14];
			objReturn = (objDist/2*0.000001*speedSound) ;
			break;
		}
		case 10: //Obj4 Width (us)
		{
			objWidth = ultraMeasResult[15];
			objReturn= objWidth * 16;
			break;
		}
		case 11: //Obj4 Peak Amplitude
		{
			objAmp = ultraMeasResult[16];
			objReturn= objAmp;
			break;
		}
		case 12: //Obj5 Distance (m)
		{
			objDist = (ultraMeasResult[17]<<8) + ultraMeasResult[18];
			objReturn = (objDist/2*0.000001*speedSound) ;
			break;
		}
		case 13: //Obj5 Width (us)
		{
			objWidth = ultraMeasResult[19];
			objReturn= objWidth * 16;
			break;
		}
		case 14: //Obj5 Peak Amplitude
		{
			objAmp = ultraMeasResult[20];
			objReturn= objAmp;
			break;
		}
		case 15: //Obj6 Distance (m)
		{
			objDist = (ultraMeasResult[21]<<8) + ultraMeasResult[22];
			objReturn = (objDist/2*0.000001*speedSound) ;
			break;
		}
		case 16: //Obj6 Width (us)
		{
			objWidth = ultraMeasResult[23];
			objReturn= objWidth * 16;
			break;
		}
		case 17: //Obj6 Peak Amplitude
		{
			objAmp = ultraMeasResult[24];
			objReturn= objAmp;
			break;
		}
		case 18: //Obj7 Distance (m)
		{
			objDist = (ultraMeasResult[25]<<8) + ultraMeasResult[26];
			objReturn = (objDist/2*0.000001*speedSound) ;
			break;
		}
		case 19: //Obj7 Width (us)
		{
			objWidth = ultraMeasResult[27];
			objReturn= objWidth * 16;
			break;
		}
		case 20: //Obj7 Peak Amplitude
		{
			objAmp = ultraMeasResult[28];
			objReturn= objAmp;
			break;
		}
		case 21: //Obj8 Distance (m)
		{
			objDist = (ultraMeasResult[29]<<8) + ultraMeasResult[30];
			objReturn = (objDist/2*0.000001*speedSound) ;
			break;
		}
		case 22: //Obj8 Width (us)
		{
			objWidth = ultraMeasResult[31];
			objReturn= objWidth * 16;
			break;
		}
		case 23: //Obj8 Peak Amplitude
		{
			objAmp = ultraMeasResult[32];
			objReturn= objAmp;
			break;
		}		
		default: Serial.println("ERROR - Invalid object result!"); break;
	}	
	return objReturn;
}

/*------------------------------------------------- runEchoDataDump -----
 |  Function runEchoDataDump
 |
 |  Purpose:  Runs a preset 1 or 2 burst and or listen command to capture 128 bytes of echo data dump.
 |		Toggle echo data dump enable bit to enable/disable echo data dump mode.
 |
 |  Parameters:
 |		preset (IN) -- determines which preset command is run:
 |			• 0 = Preset 1 Burst + Listen command
 |			• 1 = Preset 2 Burst + Listen command
 |			• 2 = Preset 1 Listen Only command
 |			• 3 = Preset 2 Listen Only command
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::runEchoDataDump(byte preset)
{

		// enable Echo Data Dump bit
		regAddr = 0x40;
		regData = 0x80;
		byte buf10[5] = {syncByte, SRW, regAddr, regData, calcChecksum(SRW)};
			spiTransfer(buf10, sizeof(buf10));
		delay(10);
		
		// run preset 1 or 2 burst and or listen command
		pga460::ultrasonicCmd(preset, 1);	

		// disbale Echo Data Dump bit
		regData = 0x00;
		buf10[3] = regData;
		buf10[4] = calcChecksum(SRW);

			spiTransfer(buf10, sizeof(buf10));

	
	return;
}

/*------------------------------------------------- pullEchoDataDump -----
 |  Function pullEchoDataDump
 |
 |  Purpose:  Read out 128 bytes of echo data dump (EDD) from latest burst and or listen command. 
 |		For UART and OWU, readout individual echo data dump register values, instead in bulk.
 |		For TCI, perform index 12 read of all echo data dump values in bulk.
 |		TODO: Enable UART and OWU cmd7 transducer echo data dump bulk read.
 |
 |  Parameters:
 |		element (IN) -- element from the 128 byte EDD memory
 |
 |  Returns:  byte representation of EDD element value
 *-------------------------------------------------------------------*/
byte pga460::pullEchoDataDump(byte element)
{
	

		// run read bulk transducer echo data dump command on first iteration
		if (element == 0)
		{
			byte buf7[3] = {syncByte, TEDD, calcChecksum(TEDD)};
			spiTransfer(buf7, sizeof(buf7));
			// MOSI transmit 0xFF to pull MISO return data
			spiMosiIdle(129);
			//// copy MISO global array data to local array based on number of objects
			//for(int n=0; n<129; n++)
			//{			
			//   localArry[n] = misoBuf[n];	   
			//}
		}
		return misoBuf[element];
	
}

/*------------------------------------------------- runDiagnostics -----
 |  Function runDiagnostics
 |
 |  Purpose:  Runs a burst+listen command to capture frequency, decay, and voltage diagnostic.
 |		Runs a listen-only command to capture noise level.
 |		Captures die temperature of PGA460 device.
 |		Converts raw diagnostics to comprehensive units
 |
 |  Parameters:
 |		run (IN) -- issue a preset 1 burst-and-listen command
 |		diag (IN) -- diagnostic value to return:
 |			• 0 = frequency diagnostic (kHz)
 |			• 1 = decay period diagnostic (us)
 |			• 2 = die temperature (degC)
 |			• 3 = noise level (8bit)
 |
 |  Returns:  double representation of last captured diagnostic
 *-------------------------------------------------------------------*/
 
double pga460::runDiagnostics(byte run, byte diag)
{
	double diagReturn = 0;
	int elementOffset = 0; //Only non-zero for OWU mode.
	

			
		if (run == 1) // issue  P1 burst+listen, and run system diagnostics command to get latest results
		{
			// run burst+listen command at least once for proper diagnostic analysis
			pga460::ultrasonicCmd(0, 1);	// always run preset 1 (short distance) burst+listen for 1 object for system diagnostic
			
			
			delay(100); // record time length maximum of 65ms, so add margin
			
			byte buf8[3] = {syncByte, SD, calcChecksum(SD)};

				spiTransfer(buf8, sizeof(buf8));
				// MOSI transmit 0xFF to pull MISO return data
				spiMosiIdle(3);
				for(int n=0; n<2; n++)
				{
				   diagMeasResult[n] = misoBuf[n];
				}
			
		}
		
		if (diag == 2) //run temperature measurement
		{
			tempOrNoise = 0; // temp meas
			byte buf4[4] = {syncByte, TNLM, tempOrNoise, calcChecksum(TNLM)}; 

				spiTransfer(buf4, sizeof(buf4));
			
			byte buf6[3] = {syncByte, TNLR, calcChecksum(TNLR)};

				spiTransfer(buf6, sizeof(buf6));
				spiMosiIdle(3);			
			
			delay(100);		
		}
			
		if (diag == 3) // run noise level meas
		{
			tempOrNoise = 1; // noise meas
			byte buf4[4] = {syncByte, TNLM, tempOrNoise, calcChecksum(TNLM)};

				spiTransfer(buf4, sizeof(buf4));

			delay(10);
			
			byte buf6[3] = {syncByte, TNLR, calcChecksum(TNLR)}; //serial transmit master data to read temperature and noise results
				spiTransfer(buf6, sizeof(buf6));
				spiMosiIdle(3);
			
			delay(100);
		}
			

			for(int n=0; n<2; n++)
			{
			   tempNoiseMeasResult[n] = misoBuf[n];
			}
		
		
		// if SPI mode, do not apply array offset
			elementOffset = -1;

			
	

	
	delay(100);
		
	switch (diag)
	{
		case 0: // convert to transducer frequency in kHz
			{
				diagReturn = (1 / (diagMeasResult[1+elementOffset] * 0.0000005)) / 1000;
			}
			break;
		case 1: // convert to decay period time in us
			{
				diagReturn = diagMeasResult[2+elementOffset] * 16;
			}
			break;
		case 2: //convert to temperature in degC
			{
				diagReturn = (tempNoiseMeasResult[1+elementOffset] - 64) / 1.5;
			}
			break;
		case 3: //noise floor level
			{
				diagReturn = tempNoiseMeasResult[2+elementOffset];
			}
			break;
		default: break;
	}
	
	return diagReturn;
}


/*------------------------------------------------- burnEEPROM -----
 |  Function burnEEPROM
 |
 |  Purpose:  Burns the EEPROM to preserve the working/shadow register values to EEPROM after power
 |		cycling the PGA460 device. Returns EE_PGRM_OK bit to determine if EEPROM burn was successful.
 |
 |  Parameters:
 |		none
 |
 |  Returns:  bool representation of EEPROM program success
 *-------------------------------------------------------------------*/
bool pga460::burnEEPROM()
{
	byte burnStat = 0;
	byte temp = 0;
	bool burnSuccess = false;

			
		// Write "0xD" to EE_UNLCK to unlock EEPROM, and '0' to EEPRGM bit at EE_CNTRL register
		regAddr = 0x40; //EE_CNTRL
		regData = 0x68;
		byte buf10[5] = {syncByte, SRW, regAddr, regData, calcChecksum(SRW)};

			spiTransfer(buf10, sizeof(buf10));
		
		delay(1);
		
		// Write "0xD" to EE_UNLCK to unlock EEPROM, and '1' to EEPRGM bit at EE_CNTRL register
		regAddr = 0x40; //EE_CNTRL
		regData = 0x69;
		buf10[2] = regAddr;
		buf10[3] = regData;
		buf10[4] = calcChecksum(SRW);

			spiTransfer(buf10, sizeof(buf10));
		delay(1000);
		
		
		// Read back EEPROM program status
		regAddr = 0x40; //EE_CNTRL
		byte buf9[4] = {syncByte, SRR, regAddr, calcChecksum(SRR)};

			spiTransfer(buf9, sizeof(buf9));


			spiMosiIdle(3);
			burnStat = misoBuf[1];

	
	
	if((burnStat & 0x04) == 0x04){burnSuccess = true;} // check if EE_PGRM_OK bit is '1'
	
	return burnSuccess;
}

/*------------------------------------------------- broadcast -----
 |  Function broadcast
 |
 |  Purpose:  Send a broadcast command to bulk write the user EEPROM, TVG, and/or Threshold values for all devices, regardless of UART_ADDR.
 |		Placehold for user EEPROM broadcast available. Note, all devices will update to the same UART_ADDR in user EEPROM broadcast command.
 |		This function is not applicable to TCI mode.
 |
 |  Parameters:
 |		eeBulk (IN) -- if true, broadcast user EEPROM
 |		tvgBulk (IN) -- if true, broadcast TVG
 |		thrBulk (IN) -- if true, broadcast Threshold
 |
 |  Returns: none
 *-------------------------------------------------------------------*/
void pga460::broadcast(bool eeBulk, bool tvgBulk, bool thrBulk)
{

	// TVG broadcast command:
	if (tvgBulk == true)
	{
		byte buf24[10] = {syncByte, BC_TVGBW, TVGAIN0, TVGAIN1, TVGAIN2, TVGAIN3, TVGAIN4, TVGAIN5, TVGAIN6, calcChecksum(BC_TVGBW)};
			spiTransfer(buf24, sizeof(buf24));
		delay(10);
	}
	
	// Threshold broadcast command:
	if (thrBulk == true)
	{
		byte buf25[35] = {syncByte, BC_THRBW, P1_THR_0, P1_THR_1, P1_THR_2, P1_THR_3, P1_THR_4, P1_THR_5, P1_THR_6,
		  P1_THR_7, P1_THR_8, P1_THR_9, P1_THR_10, P1_THR_11, P1_THR_12, P1_THR_13, P1_THR_14, P1_THR_15,
		  P2_THR_0, P2_THR_1, P2_THR_2, P2_THR_3, P2_THR_4, P2_THR_5, P2_THR_6, 
		  P2_THR_7, P2_THR_8, P2_THR_9, P2_THR_10, P2_THR_11, P2_THR_12, P2_THR_13, P2_THR_14, P2_THR_15,
		  calcChecksum(BC_THRBW)};

			spiTransfer(buf25, sizeof(buf25));
		delay(10);		
	}
	
	// User EEPROM broadcast command (placeholder):
	if (eeBulk == true)
	{
		byte buf23[46] = {syncByte, BC_EEBW, USER_DATA1, USER_DATA2, USER_DATA3, USER_DATA4, USER_DATA5, USER_DATA6,
			USER_DATA7, USER_DATA8, USER_DATA9, USER_DATA10, USER_DATA11, USER_DATA12, USER_DATA13, USER_DATA14, 
			USER_DATA15,USER_DATA16,USER_DATA17,USER_DATA18,USER_DATA19,USER_DATA20,
			TVGAIN0,TVGAIN1,TVGAIN2,TVGAIN3,TVGAIN4,TVGAIN5,TVGAIN6,INIT_GAIN,FREQUENCY,DEADTIME,
			PULSE_P1,PULSE_P2,CURR_LIM_P1,CURR_LIM_P2,REC_LENGTH,FREQ_DIAG,SAT_FDIAG_TH,FVOLT_DEC,DECPL_TEMP,
			DSP_SCALE,TEMP_TRIM,P1_GAIN_CTRL,P2_GAIN_CTRL,calcChecksum(BC_EEBW)};

			spiTransfer(buf23, sizeof(buf23));

		delay(50);
	}
	
	return;
}


/*------------------------------------------------- calcChecksum -----
 |  Function calcChecksum
 |
 |  Purpose:  Calculates the UART checksum value based on the selected command and the user EERPOM values associated with the command
 |		This function is not applicable to TCI mode. 
 |
 |  Parameters:
 |		cmd (IN) -- the UART command for which the checksum should be calculated for
 |
 |  Returns: byte representation of calculated checksum value
 *-------------------------------------------------------------------*/
byte pga460::calcChecksum(byte cmd)
{
	int checksumLoops = 0;
	
	cmd = cmd & 0x001F; // zero-mask command address of cmd to select correct switch-case statement
	
	switch(cmd)
	{
		case 0 : //P1BL
		case 1 : //P2BL
		case 2 : //P1LO
		case 3 : //P2LO
		case 17 : //BC_P1BL
		case 18 : //BC_P2BL
		case 19 : //BC_P1LO
		case 20 : //BC_P2LO
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = numObj;
			checksumLoops = 2;
		break;
		case 4 : //TNLM
		case 21 : //TNLM
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = tempOrNoise;
			checksumLoops = 2;
		break;
		case 5 : //UMR
		case 6 : //TNLR
		case 7 : //TEDD
		case 8 : //SD
		case 11 : //EEBR
		case 13 : //TVGBR
		case 15 : //THRBR
			ChecksumInput[0] = cmd;
			checksumLoops = 1;
		break;
		case 9 : //RR
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = regAddr;
			checksumLoops = 2;
		break;
		case 10 : //RW
		case 22 : //BC_RW
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = regAddr;
			ChecksumInput[2] = regData;
			checksumLoops = 3;
		break;
		case 14 : //TVGBW
		case 24 : //BC_TVGBW
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = TVGAIN0;
			ChecksumInput[2] = TVGAIN1;
			ChecksumInput[3] = TVGAIN2;
			ChecksumInput[4] = TVGAIN3;
			ChecksumInput[5] = TVGAIN4;
			ChecksumInput[6] = TVGAIN5;
			ChecksumInput[7] = TVGAIN6;
			checksumLoops = 8;
		break;
		case 16 : //THRBW
		case 25 : //BC_THRBW
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = P1_THR_0;
			ChecksumInput[2] = P1_THR_1;
			ChecksumInput[3] = P1_THR_2;
			ChecksumInput[4] = P1_THR_3;
			ChecksumInput[5] = P1_THR_4;
			ChecksumInput[6] = P1_THR_5;
			ChecksumInput[7] = P1_THR_6;
			ChecksumInput[8] = P1_THR_7;
			ChecksumInput[9] = P1_THR_8;
			ChecksumInput[10] = P1_THR_9;
			ChecksumInput[11] = P1_THR_10;
			ChecksumInput[12] = P1_THR_11;
			ChecksumInput[13] = P1_THR_12;
			ChecksumInput[14] = P1_THR_13;
			ChecksumInput[15] = P1_THR_14;
			ChecksumInput[16] = P1_THR_15;
			ChecksumInput[17] = P2_THR_0;
			ChecksumInput[18] = P2_THR_1;
			ChecksumInput[19] = P2_THR_2;
			ChecksumInput[20] = P2_THR_3;
			ChecksumInput[21] = P2_THR_4;
			ChecksumInput[22] = P2_THR_5;
			ChecksumInput[23] = P2_THR_6;
			ChecksumInput[24] = P2_THR_7;
			ChecksumInput[25] = P2_THR_8;
			ChecksumInput[26] = P2_THR_9;
			ChecksumInput[27] = P2_THR_10;
			ChecksumInput[28] = P2_THR_11;
			ChecksumInput[29] = P2_THR_12;
			ChecksumInput[30] = P2_THR_13;
			ChecksumInput[31] = P2_THR_14;
			ChecksumInput[32] = P2_THR_15;
			checksumLoops = 33;
		break;
		case 12 : //EEBW
		case 23 : //BC_EEBW
			ChecksumInput[0] = cmd;
			ChecksumInput[1] = USER_DATA1;
			ChecksumInput[2] = USER_DATA2;
			ChecksumInput[3] = USER_DATA3;
			ChecksumInput[4] = USER_DATA4;
			ChecksumInput[5] = USER_DATA5;
			ChecksumInput[6] = USER_DATA6;
			ChecksumInput[7] = USER_DATA7;
			ChecksumInput[8] = USER_DATA8;
			ChecksumInput[9] = USER_DATA9;
			ChecksumInput[10] = USER_DATA10;
			ChecksumInput[11] = USER_DATA11;
			ChecksumInput[12] = USER_DATA12;
			ChecksumInput[13] = USER_DATA13;
			ChecksumInput[14] = USER_DATA14;
			ChecksumInput[15] = USER_DATA15;
			ChecksumInput[16] = USER_DATA16;
			ChecksumInput[17] = USER_DATA17;
			ChecksumInput[18] = USER_DATA18;
			ChecksumInput[19] = USER_DATA19;
			ChecksumInput[20] = USER_DATA20;
			ChecksumInput[21] = TVGAIN0;
			ChecksumInput[22] = TVGAIN1;
			ChecksumInput[23] = TVGAIN2;
			ChecksumInput[24] = TVGAIN3;
			ChecksumInput[25] = TVGAIN4;
			ChecksumInput[26] = TVGAIN5;
			ChecksumInput[27] = TVGAIN6;
			ChecksumInput[28] = INIT_GAIN;
			ChecksumInput[29] = FREQUENCY;
			ChecksumInput[30] = DEADTIME;
			ChecksumInput[31] = PULSE_P1;
			ChecksumInput[32] = PULSE_P2;
			ChecksumInput[33] = CURR_LIM_P1;
			ChecksumInput[34] = CURR_LIM_P2;
			ChecksumInput[35] = REC_LENGTH;
			ChecksumInput[36] = FREQ_DIAG;
			ChecksumInput[37] = SAT_FDIAG_TH;
			ChecksumInput[38] = FVOLT_DEC;
			ChecksumInput[39] = DECPL_TEMP;
			ChecksumInput[40] = DSP_SCALE;
			ChecksumInput[41] = TEMP_TRIM;
			ChecksumInput[42] = P1_GAIN_CTRL;
			ChecksumInput[43] = P2_GAIN_CTRL;
			checksumLoops = 44;
		break;
		default: break;
	}

	if (ChecksumInput[0]<17) //only re-append command address for non-broadcast commands.
	{
		ChecksumInput[0] = ChecksumInput[0] + (uartAddr << 5);
	}
	
	uint16_t carry = 0;

	for (int i = 0; i < checksumLoops; i++)
	{
		if ((ChecksumInput[i] + carry) < carry)
		{
			carry = carry + ChecksumInput[i] + 1;
		}
		else
		{
			carry = carry + ChecksumInput[i];
		}

		if (carry > 0xFF)
		{
		  carry = carry - 255;
		}
	}
	
	carry = (~carry & 0x00FF);
	return carry;
}

/*------------------------------------------------- spiTransfer -----
 |  Function spiTransfer
 |
 |  Purpose:  Transfers one byte over the SPI bus, both sending and receiving. 
 |			Captures MISO data in global byte-array.
 |
 |  Parameters:
 |		mosi (IN) -- MOSI data byte array to transmit over SPI
 |		size (IN) -- size of MOSI data byte array
 |
 |  Returns: byte representation of calculated checksum value
 *-------------------------------------------------------------------*/
void pga460::spiTransfer(byte* mosi, byte size )
{
	memset(misoBuf, 0x00, sizeof(misoBuf)); // idle-low receive buffer data
	
	for (int i = 0; i<size; i++)
	{
		digitalWrite(SPI_CS, LOW);
		misoBuf[i] = usscSPIVL.transfer(mosi[i]);
		digitalWrite(SPI_CS, HIGH);
	}
	return;
}

/*------------------------------------------------- spiMosiIdle-----
 |  Function spiMosiIdle
 |
 |  Purpose:  Forces MOSI of 0xFF to idle high master output, while
 |			MISO pin returns valid data.
 |
 |  Parameters:
 |		size (IN) -- number of MISO data bytes expected from slave
 |
 |  Returns: none
 *-------------------------------------------------------------------*/
void pga460::spiMosiIdle(byte size)
{		
	//memset(misoBuf, 0x00, sizeof(misoBuf)); // idle-low receive buffer data
	
	for (int i = 0; i<size; i++)
	{
		digitalWrite(SPI_CS, LOW);
		misoBuf[i] = usscSPIVL.transfer(0xFF);
		digitalWrite(SPI_CS, HIGH);
	}
	return;
}

/*------------------------------------------------- toggleLEDs -----
 |  Function toggleLEDs
 |
 |  Purpose:  Set the BOOSTXL-PGA460 diagnostic LED state to ON or OFF.
 |
 |  Parameters:
 |		ds1State (IN) -- state of BOOSTXL-PGA460 RED LED populated at D9
 |		fdiagState (IN) -- state of BOOSTXL-PGA460 RED LED populated at D8
 |		vdiagate (IN) -- state of BOOSTXL-PGA460 RED LED populated at D7
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::toggleLEDs(bool ds1State, bool fdiagState, bool vdiagState)
{
	digitalWrite(DS1_LED, ds1State); digitalWrite(F_DIAG_LED, fdiagState); digitalWrite(V_DIAG_LED, vdiagState);
	return;
}

/*------------------------------------------------- registerRead -----
 |  Function registerRead
 |
 |  Purpose:  Read single register data from PGA460
 |
 |  Parameters:
 |		addr (IN) -- PGA460 register address to read data from
 |
 |  Returns:  8-bit data read from register
 *-------------------------------------------------------------------*/
byte pga460::registerRead(byte addr)
{
	byte data = 0x00;
	byte temp = 0;

	
	
	regAddr = addr;
	byte buf9[4] = {syncByte, SRR, regAddr, calcChecksum(SRR)};

		spiTransfer(buf9, sizeof(buf9));

	delay(10);

		spiMosiIdle(3);
		data = misoBuf[1];
	
	return data;
}
	
/*------------------------------------------------- registerWrite -----
 |  Function registerWrite
 |
 |  Purpose:  Write single register data to PGA460
 |
 |  Parameters:
 |		addr (IN) -- PGA460 register address to write data to
 |		data (IN) -- 8-bit data value to write into register
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
byte pga460::registerWrite(byte addr, byte data)
{
	regAddr = addr;
	regData = data;	
	byte buf10[5] = {syncByte, SRW, regAddr, regData, calcChecksum(SRW)};
		spiTransfer(buf10, sizeof(buf10));
	delay(10);
}



/*------------------------------------------------- thresholdBulkWrite -----
 |  Function thresholdBulkWrite
 |
 |  Purpose:  Bulk write to all threshold registers
 |
 |  Parameters:
 |		p1ThrMap (IN) -- data byte array for 16 bytes of Preset 1 threhsold data
  |		p2ThrMap (IN) -- data byte array for 16 bytes of Preset 2 threhsold data
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::thresholdBulkWrite(byte *p1ThrMap, byte *p2ThrMap)
{

	//bulk write new threshold values

		byte buf16[35] = {syncByte,	THRBW, p1ThrMap[0], p1ThrMap[1], p1ThrMap[2], p1ThrMap[3], p1ThrMap[4], p1ThrMap[5],
			p1ThrMap[6], p1ThrMap[7], p1ThrMap[8], p1ThrMap[9], p1ThrMap[10], p1ThrMap[11], p1ThrMap[12],
			p1ThrMap[13], p1ThrMap[14], p1ThrMap[15],
			p2ThrMap[0], p2ThrMap[1], p2ThrMap[2], p2ThrMap[3], p2ThrMap[4], p2ThrMap[5],
			p2ThrMap[6], p2ThrMap[7], p2ThrMap[8], p2ThrMap[9], p2ThrMap[10], p2ThrMap[11], p2ThrMap[12],
			p2ThrMap[13], p2ThrMap[14], p2ThrMap[15],
			calcChecksum(THRBW)};

			spiTransfer(buf16, sizeof(buf16));

		
	
	delay(100);
	return;

}

/*------------------------------------------------- eepromThreshold -----
 |  Function eepromThreshold
 |
 |  Purpose:  Copy a single preset's threshold times and levels 
 |  			to USER_DATA1-16 in EEPROM
 |
 |  Parameters:
 |		preset (IN) -- preset's threshold to copy
 |		saveLoad (IN) -- when false, copy threshold to EEPROM;
 |					when true, copy threshold from EEPROM
 |
 |  Returns:  none
 *-------------------------------------------------------------------*/
void pga460::eepromThreshold(byte preset, bool saveLoad)
{
	byte presetOffset = 0;
	byte addr = 0x5F; // beginning of threshold memory space
	
	if (saveLoad == false) // save thr
	{
		//Preset 2 advances 16 address bytes
		if (preset == 2 || preset == 4) 
		{
			presetOffset = 16;
		}
		
		for (int n = 0; n<16; n++)
		{
			bulkThr[n + presetOffset] = registerRead(addr + presetOffset);
			// write threshold values into USER_DATA1-16
			registerWrite(n, bulkThr[n + presetOffset]);
			addr++;
		}
	}
	else // load thr
	{
		//Preset 2 advances 16 address bytes
		if (preset == 2 || preset == 4) //Preset 2 advances 16 address bytes
		{
			presetOffset = 16;
		}
		
		// copy USER_DATA1-16 into selected preset threhsold space
		for (int n = 0; n<16; n++)
		{
			bulkThr[n + presetOffset] = registerRead(n);
			// bulk write to threshold
			registerWrite(addr + presetOffset, bulkThr[n + presetOffset]);
			addr++;
		}
	}
}

/*------------------------------------------------- FUNCTION_NAME -----
 |  Function FUNCTION_NAME
 |
 |  Purpose:  EXPLAIN WHAT THIS FUNCTION DOES TO SUPPORT THE CORRECT
 |      OPERATION OF THE PROGRAM, AND HOW IT DOES IT.
 |
 |  Parameters:
 |      parameter_name (IN, OUT, or IN/OUT) -- EXPLANATION OF THE
 |              PURPOSE OF THIS PARAMETER TO THE FUNCTION.
 |                      (REPEAT THIS FOR ALL FORMAL PARAMETERS OF
 |                       THIS FUNCTION.
 |                       IN = USED TO PASS DATA INTO THIS FUNCTION,
 |                       OUT = USED TO PASS DATA OUT OF THIS FUNCTION
 |                       IN/OUT = USED FOR BOTH PURPOSES.)
 |
 |  Returns:  IF THIS FUNCTION SENDS BACK A VALUE VIA THE RETURN
 |      MECHANISM, DESCRIBE THE PURPOSE OF THAT VALUE HERE.
 *-------------------------------------------------------------------*/
