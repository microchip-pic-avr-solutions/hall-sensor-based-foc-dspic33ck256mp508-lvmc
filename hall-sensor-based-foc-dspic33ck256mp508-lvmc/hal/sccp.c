/*******************************************************************************
  Timer Configuration Routine source File.

  File Name:
    sccp.c

  Summary:
    This file includes subroutine for initializing Timer Reference for Speed Calculation.

  Description:
    Definitions in the file are for dsPIC33CK256MP508 External OP-AMP PIM
    plugged onto Motor Control Development board from Microchip.

*******************************************************************************/
/*******************************************************************************
* Copyright (c) 2019 released Microchip Technology Inc.  All rights reserved.
*
* SOFTWARE LICENSE AGREEMENT:
* 
* Microchip Technology Incorporated ("Microchip") retains all ownership and
* intellectual property rights in the code accompanying this message and in all
* derivatives hereto.  You may use this code, and any derivatives created by
* any person or entity by or on your behalf, exclusively with Microchip's
* proprietary products.  Your acceptance and/or use of this code constitutes
* agreement to the terms and conditions of this notice.
*
* CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO
* WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
* TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S
* PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.
*
* YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE,
* WHETHER IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF
* STATUTORY DUTY),STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE,
* FOR ANY INDIRECT, SPECIAL,PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL
* LOSS, DAMAGE, FOR COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE CODE,
* HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR
* THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT ALLOWABLE BY LAW,
* MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO THIS CODE,
* SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO
* HAVE THIS CODE DEVELOPED.
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>
#include "clock.h"
// *****************************************************************************
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************
void Init_SCCP4(void);
// *****************************************************************************
/* Function:
    void Init_SCCP4(void)

  Summary:
    Routine to initialize SCCP4 in timer mode

  Description:
    Function to find the time difference for speed calculation

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void Init_SCCP4(void)
{          
    CCP4CON1Lbits.CCSEL = 0;     // Set SCCP4 operating OFF
    CCP4CON1Lbits.T32 = 0;       // Set timebase width (16-bit = 0)
    CCP4CON1Lbits.MOD = 0b0000;  // Set mode to 16/32 bit timer mode features to Output Timer Mode
    CCP4CON1Hbits.SYNC = 0b00000;// No external synchronization; timer rolls over at FFFFh or matches with the Timer Period register
    CCP4CON1Lbits.TMRSYNC = 0;   // Set timebase synchronization (Synchronized)
    CCP4CON1Lbits.CLKSEL = 0b000;// Set the clock source (Tcy)
    CCP4CON1Lbits.TMRPS = 0b00;  // Set the clock pre-scaler (1:64)
    CCP4CON1Hbits.TRIGEN = 0;    // Set Sync/Triggered mode (Synchronous)
    
    CCP4TMRL = 0x0000;           // Initialize timer prior to enable module.
    CCP4TMRH = 0x0000;           // Initialize timer prior to enable module.
   
    IPC10bits.CCT4IP = 6;        // Interrupt Priority set
    IFS2bits.CCT4IF = 0;         // Clear Interrupt flag
    IEC2bits.CCT4IE = 0;         // Disable Interrupt
    CCP4CON1Lbits.CCPON = 0;     // Disable CCP/input capture
}