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

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
        


/* Definition for tuning - if active the speed reference is a ramp with a 
constant slope. The slope is determined by TUNING_DELAY_RAMPUP constant.
 the software ramp implementing the speed increase has a constant slope, 
 adjusted by the delay TUNING_DELAY_RAMPUP when the speed is incremented.
 The potentiometer speed reference is overwritten. The speed is          
 increased from 0 up to the END_SPEED_RPM in open loop – with the speed  
 increase typical to open loop, the transition to closed loop is done    
 and the software speed ramp reference is continued up to MAXIMUM_SPEED_RPM. */
#undef TUNING

/* if TUNING was define, then the ramp speed is needed: */
#ifdef TUNING
    /* the smaller the value, the quicker the ramp */
    #define TUNING_DELAY_RAMPUP   0xF      
#endif


/* open loop continuous functioning */
/* closed loop transition disabled  */
#undef OPEN_LOOP_FUNCTIONING

/* Definition for torque mode - for a separate tuning of the current PI
controllers, tuning mode will disable the speed PI controller */
#undef TORQUE_MODE
/* FOC with single shunt is enabled */
/* undef to work with dual Shunt  */    
#undef SINGLE_SHUNT     

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


/* Filters constants definitions  */
/* BEMF filter for d-q components @ low speeds */
#define KFILTER_ESDQ 1200
/* BEMF filter for d-q components @ high speed - Flux Weakening case */
#define KFILTER_ESDQ_FW 164
/* Estimated speed filter constatn */
#define KFILTER_VELESTIM 2*374


/* initial offset added to estimated value, 
 when transitioning from open loop to closed loop 
 the value represents 45deg and should satisfy both 
 open loop and closed loop functioning 
 normally this value should not be modified, but in 
 case of fine tuning of the transition, depending on 
 the load or the rotor moment of inertia */
#define INITOFFSET_TRANS_OPEN_CLSD 0x2000

/* current transformation macro, used below */
#define NORM_CURRENT(current_real) (Q15(current_real/NORM_CURRENT_CONST/32768))

/* Open loop startup constants */

/* The following values depends on the PWM frequency,
 lock time is the time needed for motor's poles alignment 
before the open loop speed ramp up */
/* This number is: 20,000 is 1 second. */
#define LOCK_TIME 4000 
/* Open loop speed ramp up end value Value in RPM*/
#define END_SPEED_RPM 500 
/* Open loop acceleration */
#define OPENLOOP_RAMPSPEED_INCREASERATE 10
/* Open loop q current setup - */
#define Q_CURRENT_REF_OPENLOOP NORM_CURRENT(1.0)

/* Specify Over Current Limit - DC BUS */
#define Q15_OVER_CURRENT_THRESHOLD NORM_CURRENT(3.0)

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
#define SPEEDCNTR_PTERM        Q15(0.05)
#define SPEEDCNTR_ITERM        Q15(0.001)
#define SPEEDCNTR_CTERM        Q15(0.999)
#define SPEEDCNTR_OUTMAX       0x5000
/******************************** Field Weakening *****************************/
/* Field Weakening constant for constant torque range 
   Flux reference value */
#define IDREF_BASESPEED         NORM_CURRENT(0.0)   

/*-------------------------------------------------------------
   IMPORTANT:--------------------------------------------------
  -------------------------------------------------------------
   In flux weakening of the surface mounted permanent magnets
   PMSMs the mechanical damage of the rotor and the
   demagnetization of the permanent magnets is possible if
   cautions measures are not taken or the motor’s producer
   specifications are not respected.
  -------------------------------------------------------------
   IMPORTANT:--------------------------------------------------
  -------------------------------------------------------------
   In flux weakening regime implementation, if the FOC is lost
   at high speed above the nominal value, the possibility of
   damaging the inverter is eminent. The reason is that the
   BEMF will have a greater value than the one that would be
   obtained for the nominal speed exceeding the DC bus voltage
   value and though the inverter’s power semiconductors and DC
   link capacitors would have to support it. Since the tuning
   proposed implies iterative coefficient corrections until
   the optimum functioning is achieved, the protection of the
   inverter with corresponding circuitry should be assured in
   case of stalling at high speeds.                            */

/* speed index is increase */
#define SPEED_INDEX_CONST 10                

/* the following values indicate the d-current variation with speed 
 please consult app note for details on tuning */
#define	IDREF_SPEED0	NORM_CURRENT(0)     /* up to 2800 RPM */
#define	IDREF_SPEED1	NORM_CURRENT(-0.7)  /* ~2950 RPM */
#define	IDREF_SPEED2	NORM_CURRENT(-0.9)  /* ~3110 RPM */
#define	IDREF_SPEED3	NORM_CURRENT(-1.0)  /* ~3270 RPM */
#define	IDREF_SPEED4	NORM_CURRENT(-1.4)  /* ~3430 RPM */
#define	IDREF_SPEED5	NORM_CURRENT(-1.7)  /* ~3600 RPM */
#define	IDREF_SPEED6	NORM_CURRENT(-2.0)  /* ~3750 RPM */
#define	IDREF_SPEED7	NORM_CURRENT(-2.1)  /* ~3910 RPM */
#define	IDREF_SPEED8	NORM_CURRENT(-2.2)  /* ~4070 RPM */
#define	IDREF_SPEED9	NORM_CURRENT(-2.25) /* ~4230 RPM */
#define	IDREF_SPEED10	NORM_CURRENT(-2.3)  /* ~4380 RPM */
#define	IDREF_SPEED11	NORM_CURRENT(-2.35) /* ~4550 RPM */
#define	IDREF_SPEED12	NORM_CURRENT(-2.4)  /* ~4700 RPM */
#define	IDREF_SPEED13	NORM_CURRENT(-2.45) /* ~4860 RPM */
#define	IDREF_SPEED14	NORM_CURRENT(-2.5)  /* ~5020 RPM */
#define	IDREF_SPEED15	NORM_CURRENT(-2.5)  /* ~5180 RPM */
#define	IDREF_SPEED16	NORM_CURRENT(-2.5)  /* ~5340 RPM */
#define	IDREF_SPEED17	NORM_CURRENT(-2.5)  /* ~5500 RPM */

/* the following values indicate the invKfi variation with speed 
 please consult app note for details on tuning */
#define	INVKFI_SPEED0           NORM_INVKFIBASE     /* up to 2800 RPM */
#define	INVKFI_SPEED1           8674        /* ~2950 RPM */
#define	INVKFI_SPEED2           9156        /* ~3110 RPM */
#define	INVKFI_SPEED3           9638        /* ~3270 RPM */
#define	INVKFI_SPEED4           10120       /* ~3430 RPM */
#define	INVKFI_SPEED5           10602       /* ~3600 RPM */
#define	INVKFI_SPEED6           11084       /* ~3750 RPM */
#define	INVKFI_SPEED7           11566       /* ~3910 RPM */
#define	INVKFI_SPEED8           12048       /* ~4070 RPM */
#define	INVKFI_SPEED9           12530       /* ~4230 RPM */
#define	INVKFI_SPEED10          13012       /* ~4380 RPM */
#define	INVKFI_SPEED11          13494       /* ~4550 RPM */
#define	INVKFI_SPEED12          13976       /* ~4700 RPM */
#define	INVKFI_SPEED13          14458       /* ~4860 RPM */
#define	INVKFI_SPEED14          14940       /* ~5020 RPM */
#define	INVKFI_SPEED15          15422       /* ~5180 RPM */
#define	INVKFI_SPEED16          15904       /* ~5340 RPM */
#define	INVKFI_SPEED17          16387       /* ~5500 RPM */



/* the following values indicate the Ls variation with speed 
 please consult app note for details on tuning */
#define LS_OVER2LS0_SPEED0      Q15(0.5)    /* up to 2800 RPM */
#define LS_OVER2LS0_SPEED1      Q15(0.45)   /* ~2950 RPM */
#define LS_OVER2LS0_SPEED2      Q15(0.4)    /* ~3110 RPM */
#define LS_OVER2LS0_SPEED3      Q15(0.4)    /* ~3270 RPM */
#define LS_OVER2LS0_SPEED4      Q15(0.35)   /* ~3430 RPM */
#define LS_OVER2LS0_SPEED5      Q15(0.35)   /* ~3600 RPM */
#define LS_OVER2LS0_SPEED6      Q15(0.34)   /* ~3750 RPM */
#define LS_OVER2LS0_SPEED7      Q15(0.34)   /* ~3910 RPM */
#define LS_OVER2LS0_SPEED8      Q15(0.33)   /* ~4070 RPM */
#define LS_OVER2LS0_SPEED9      Q15(0.33)   /* ~4230 RPM */
#define LS_OVER2LS0_SPEED10     Q15(0.32)   /* ~4380 RPM */
#define LS_OVER2LS0_SPEED11     Q15(0.32)   /* ~4550 RPM */
#define LS_OVER2LS0_SPEED12     Q15(0.31)   /* ~4700 RPM */
#define LS_OVER2LS0_SPEED13     Q15(0.30)   /* ~4860 RPM */
#define LS_OVER2LS0_SPEED14     Q15(0.29)   /* ~5020 RPM */
#define LS_OVER2LS0_SPEED15     Q15(0.28)   /* ~5180 RPM */
#define LS_OVER2LS0_SPEED16     Q15(0.27)   /* ~5340 RPM */
#define LS_OVER2LS0_SPEED17     Q15(0.26)   /* ~5500 RPM */


#ifdef __cplusplus
}
#endif

#endif /* __USERPARMS_H */
