/**
  CCP5 Generated Driver File

  @Company
    Microchip Technology Inc.

  @File Name
    ccp5.c

  @Summary
    This is the generated driver implementation file for the CCP5 driver using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  @Description
    This source file provides implementations for driver APIs for CCP5.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.65.2
        Device            :  PIC18F26K22
        Driver Version    :  2.01
    The generated drivers are tested against the following:
        Compiler          :  XC8 1.45
         MPLAB 	          :  MPLAB X 4.15
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

/**
  Section: Included Files
*/

#include <xc.h>
#include "ccp5.h"

/**
  Section: Capture Module APIs:
*/

void CCP5_Initialize(void)
{
    // Set the CCP5 to the options selected in the User Interface
	
	// CCP5M Rising edge; DC5B 0; 
	CCP5CON = 0x05;    
	
	// CCPR5L 0; 
	CCPR5L = 0x00;    
	
	// CCPR5H 0; 
	CCPR5H = 0x00;    

	// Selecting Timer 3
	CCPTMRS1bits.C5TSEL = 0x1;
    
    // Clear the CCP5 interrupt flag
    PIR4bits.CCP5IF = 0;

    // Enable the CCP5 interrupt
    PIE4bits.CCP5IE = 1;
}

void CCP5_CaptureISR(void)
{
    CCP_PERIOD_REG_T module;

    // Clear the CCP5 interrupt flag
    PIR4bits.CCP5IF = 0;
    
    // Copy captured value.
    module.ccpr5l = CCPR5L;
    module.ccpr5h = CCPR5H;
    
    // Return 16bit captured value
    CCP5_CallBack(module.ccpr5_16Bit);
}


/**
 End of File
*/