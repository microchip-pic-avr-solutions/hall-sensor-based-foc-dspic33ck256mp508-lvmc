/*******************************************************************************
  Timer Configuration Routine Header File

  File Name:
    sccp.h

  Summary:
    This header file lists SCCP Configuration related functions and definitions.

  Description:
    Definitions in the file are for dsPIC33CK256MP508 MC PIM plugged onto
    Motor Control Development board from Microchip

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

#ifndef _TIMER_H
#define _TIMER_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
     
 // *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>
// *****************************************************************************
// *****************************************************************************
// Section: Constants
   /** The Timer Prescaler Value set to 1:64 */
#define	TIMER_PRESCALER     64     
// *****************************************************************************
// Section: Functions
// *****************************************************************************
// *****************************************************************************
void Init_Timer(void);
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
    
// <editor-fold defaultstate="expanded" desc="TYPE DEFINITIONS ">  
        
/** TIMER Clock Pre-scalers */
typedef enum tagTIMER_CLOCK_PRESCALER
{ 
    /** TMRPS<1:0>: CCPx Time Base Prescale Select bits
        0b11 = 1:256 , 0b10 = 1:64 ,0b01 = 1:8 0b00 = 1:1                     */
    TIMER_CLOCK_PRESCALER_256    = 3,
    TIMER_CLOCK_PRESCALER_64     = 2,
    TIMER_CLOCK_PRESCALER_8      = 1,
    TIMER_CLOCK_PRESCALER_1      = 0,
            
}TIMER_CLOCK_PRESCALER_TYPE;

// </editor-fold> 
/**
 * Sets the TImer1 Input Clock Select bits.
 * @example
 * <code>
 * TIMER1_InputClockSet();
 * </code>
 */
inline static void TIMER_SetTimerPrescaler(uint16_t timerPrescaler)
{
    if(timerPrescaler == 256)
    {
        T1CONbits.TCKPS = TIMER_CLOCK_PRESCALER_256;
    }
    else if(timerPrescaler == 64)
    {
        T1CONbits.TCKPS = TIMER_CLOCK_PRESCALER_64;
    }
    else if(timerPrescaler == 8)
    {
        T1CONbits.TCKPS = TIMER_CLOCK_PRESCALER_8;
    }
    else if(timerPrescaler == 1)
    {
        T1CONbits.TCKPS = TIMER_CLOCK_PRESCALER_1;
    }
    
}
inline static void  TIMER_SetTimerPeriod(uint16_t period)
{
    PR1 = period;
}

inline static void Timer_Start()
{
    T1CONbits.TON = 1;
}       

#endif        
