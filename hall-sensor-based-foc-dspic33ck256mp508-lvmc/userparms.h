/*******************************************************************************
* Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.
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
#ifndef USERPARMS_H
#define USERPARMS_H

#ifdef __cplusplus
extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>
#include "general.h"
#include "timer.h"
#include "pwm.h"

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
/* open loop continuous functioning */
#undef OPEN_LOOP_FUNCTIONING

/* Definition for torque mode - for a separate tuning of the current PI
controllers, tuning mode will disable the speed PI controller */
#undef TORQUE_MODE
    
#define INTERNAL_OPAMP_CONFIG    

/****************************** Motor Parameters ******************************/
/********************  support xls file definitions begin *********************/
/* The following values are given in the xls attached file */
    
    
/*Update the following motor tuning parameters while using LVMC build configuration*/
    
/* Motor's number of pole pairs */
#define NOPOLESPAIRS 5
/* Nominal speed of the motor in RPM */
#define NOMINAL_SPEED_RPM    2000 
/* Maximum speed of the motor in RPM - given by the motor's manufacturer */
#define MAXIMUM_SPEED_RPM    3500 

/* The following values are given in the xls attached file */
#define NORM_CURRENT_CONST     0.000671
/* normalized ls/dt value */
#define NORM_LSDTBASE 8129
/* normalized rs value */
#define NORM_RS  9044
/* the calculation of Rs gives a value exceeding the Q15 range so,
 the normalized value is further divided by 2 to fit the 32768 limit
 this is taken care in the estim.c where the value is implied
 normalized inv kfi at base speed */
#define NORM_INVKFIBASE  7956
/* the calculation of InvKfi gives a value which not exceed the Q15 limit
   to assure that an increase of the term with 5 is possible in the lookup table
   for high flux weakening the normalized is initially divided by 2
   this is taken care in the estim.c where the value is implied
   normalized dt value */
#define NORM_DELTAT  1790

/* Limitation constants */
/* di = i(t1)-i(t2) limitation
 high speed limitation, for dt 50us 
 the value can be taken from attached xls file */
#define D_ILIMIT_HS 956
/* low speed limitation, for dt 8*50us */
#define D_ILIMIT_LS 4369

/**********************  support xls file definitions end *********************/

/* current transformation macro, used below */
#define NORM_CURRENT(current_real) (Q15(current_real/NORM_CURRENT_CONST/32768))

/* Open loop startup constants */

/* The following values depends on the PWM frequency,
 lock time is the time needed for motor's poles alignment 
before the open loop speed ramp up */
/* This number is: 20,000 is 1 second. */
#define LOCK_TIME 4000 
/* Open loop speed ramp up end value Value in RPM*/
#define END_SPEED_RPM 400 
/* Open loop acceleration */
#define OPENLOOP_RAMPSPEED_INCREASERATE 50
/* Open loop q current setup - */
#define Q_CURRENT_REF_OPENLOOP NORM_CURRENT(0.4)

/* Specify Over Current Limit - DC BUS */
#define Q15_OVER_CURRENT_THRESHOLD NORM_CURRENT(5.0)

/* Maximum motor speed converted into electrical speed */
#define MAXIMUMSPEED_ELECTR MAXIMUM_SPEED_RPM*NOPOLESPAIRS
/* Nominal motor speed converted into electrical speed */
#define NOMINALSPEED_ELECTR NOMINAL_SPEED_RPM*NOPOLESPAIRS

/* End speed converted to fit the startup ramp */
#define END_SPEED (END_SPEED_RPM * NOPOLESPAIRS * LOOPTIME_SEC * 65536 / 60.0)*1024
/* End speed of open loop ramp up converted into electrical speed */
#define ENDSPEED_ELECTR END_SPEED_RPM*NOPOLESPAIRS
    
/* In case of the potentiometer speed reference, a reference ramp
is needed for assuring the motor can follow the reference imposed /
minimum value accepted */
#define SPEEDREFRAMP   Q15(0.00003)  
/* The Speed Control Loop Executes every  SPEEDREFRAMP_COUNT */
#define SPEEDREFRAMP_COUNT   3  
    
/**  SPEED MULTIPLIER CALCULATION = ((FCY*60)/(TIMER_PRESCALER*6))
 * This is to calculate speed in electrical RPM  */
#define SPEED_MULTI     (unsigned long)((float)(FCY/(float)(TIMER_PRESCALER*6)))*(float)(60)    
/** PHASE INCREMENT MULTIPLIER = (FCY/(TIMER_PRESCALER*PWM_FREQUENCY))(65536/6)*/
#define PHASE_INC_MULTI    (unsigned long)((float)FCY/((float)(TIMER_PRESCALER)*(float)(PWMFREQUENCY_HZ))*(float)(65536/6))
    
// *****************************************************************************
/* PI controllers tuning values - */     
/* D Control Loop Coefficients */
#define D_CURRCNTR_PTERM       Q15(0.05)
#define D_CURRCNTR_ITERM       Q15(0.003)
#define D_CURRCNTR_CTERM       Q15(0.999)
#define D_CURRCNTR_OUTMAX      0x7FFF

/* Q Control Loop Coefficients */
#define Q_CURRCNTR_PTERM       Q15(0.05)
#define Q_CURRCNTR_ITERM       Q15(0.003)
#define Q_CURRCNTR_CTERM       Q15(0.999)
#define Q_CURRCNTR_OUTMAX      0x7FFF

/* Velocity Control Loop Coefficients */
#define SPEEDCNTR_PTERM        Q15(0.0286)
#define SPEEDCNTR_ITERM        Q15(0.000126)
#define SPEEDCNTR_CTERM        Q15(0.999)
#define SPEEDCNTR_OUTMAX       0x5000
    
#ifdef __cplusplus
}
#endif

#endif /* __USERPARMS_H */
