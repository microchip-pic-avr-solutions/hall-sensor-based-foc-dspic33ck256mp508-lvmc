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

#include "fdweak.h"
#include "estim.h"
#include "userparms.h"
#include "general.h"

FDWEAK_PARM_T fdWeakParm;

#define FWONSPEED NOMINAL_SPEED_RPM*NOPOLESPAIRS
// *****************************************************************************

/* Function:
    InitFWParams()

  Summary:
    Initializes field weakening parameters

  Description:
    This routine Initializes field weakening structure variables

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitFWParams(void) 
{
    /* Field Weakening constant for constant torque range */
    /* Flux reference value */
    fdWeakParm.qIdRef = IDREF_BASESPEED;
    /* Start speed for Field weakening  */
    fdWeakParm.qFwOnSpeed = FWONSPEED ;

    /* Initialize magnetizing curve values */
    fdWeakParm.qFwCurve[0] = IDREF_SPEED0;
    fdWeakParm.qFwCurve[1] = IDREF_SPEED1;
    fdWeakParm.qFwCurve[2] = IDREF_SPEED2;
    fdWeakParm.qFwCurve[3] = IDREF_SPEED3;
    fdWeakParm.qFwCurve[4] = IDREF_SPEED4;
    fdWeakParm.qFwCurve[5] = IDREF_SPEED5;
    fdWeakParm.qFwCurve[6] = IDREF_SPEED6;
    fdWeakParm.qFwCurve[7] = IDREF_SPEED7;
    fdWeakParm.qFwCurve[8] = IDREF_SPEED8;
    fdWeakParm.qFwCurve[9] = IDREF_SPEED9;
    fdWeakParm.qFwCurve[10] = IDREF_SPEED10;
    fdWeakParm.qFwCurve[11] = IDREF_SPEED11;
    fdWeakParm.qFwCurve[12] = IDREF_SPEED12;
    fdWeakParm.qFwCurve[13] = IDREF_SPEED13;
    fdWeakParm.qFwCurve[14] = IDREF_SPEED14;
    fdWeakParm.qFwCurve[15] = IDREF_SPEED15;
    fdWeakParm.qFwCurve[16] = IDREF_SPEED16;
    fdWeakParm.qFwCurve[17] = IDREF_SPEED17;


    /* Initialize inverse Kfi curve values */
    fdWeakParm.qInvKFiCurve[0] = INVKFI_SPEED0;
    fdWeakParm.qInvKFiCurve[1] = INVKFI_SPEED1;
    fdWeakParm.qInvKFiCurve[2] = INVKFI_SPEED2;
    fdWeakParm.qInvKFiCurve[3] = INVKFI_SPEED3;
    fdWeakParm.qInvKFiCurve[4] = INVKFI_SPEED4;
    fdWeakParm.qInvKFiCurve[5] = INVKFI_SPEED5;
    fdWeakParm.qInvKFiCurve[6] = INVKFI_SPEED6;
    fdWeakParm.qInvKFiCurve[7] = INVKFI_SPEED7;
    fdWeakParm.qInvKFiCurve[8] = INVKFI_SPEED8;
    fdWeakParm.qInvKFiCurve[9] = INVKFI_SPEED9;
    fdWeakParm.qInvKFiCurve[10] = INVKFI_SPEED10;
    fdWeakParm.qInvKFiCurve[11] = INVKFI_SPEED11;
    fdWeakParm.qInvKFiCurve[12] = INVKFI_SPEED12;
    fdWeakParm.qInvKFiCurve[13] = INVKFI_SPEED13;
    fdWeakParm.qInvKFiCurve[14] = INVKFI_SPEED14;
    fdWeakParm.qInvKFiCurve[15] = INVKFI_SPEED15;
    fdWeakParm.qInvKFiCurve[16] = INVKFI_SPEED16;
    fdWeakParm.qInvKFiCurve[17] = INVKFI_SPEED17;

    /* Initialize Ls variation curve */
    fdWeakParm.qLsCurve[0] = LS_OVER2LS0_SPEED0;
    fdWeakParm.qLsCurve[1] = LS_OVER2LS0_SPEED1;
    fdWeakParm.qLsCurve[2] = LS_OVER2LS0_SPEED2;
    fdWeakParm.qLsCurve[3] = LS_OVER2LS0_SPEED3;
    fdWeakParm.qLsCurve[4] = LS_OVER2LS0_SPEED4;
    fdWeakParm.qLsCurve[5] = LS_OVER2LS0_SPEED5;
    fdWeakParm.qLsCurve[6] = LS_OVER2LS0_SPEED6;
    fdWeakParm.qLsCurve[7] = LS_OVER2LS0_SPEED7;
    fdWeakParm.qLsCurve[8] = LS_OVER2LS0_SPEED8;
    fdWeakParm.qLsCurve[9] = LS_OVER2LS0_SPEED9;
    fdWeakParm.qLsCurve[10] = LS_OVER2LS0_SPEED10;
    fdWeakParm.qLsCurve[11] = LS_OVER2LS0_SPEED11;
    fdWeakParm.qLsCurve[12] = LS_OVER2LS0_SPEED12;
    fdWeakParm.qLsCurve[13] = LS_OVER2LS0_SPEED13;
    fdWeakParm.qLsCurve[14] = LS_OVER2LS0_SPEED14;
    fdWeakParm.qLsCurve[15] = LS_OVER2LS0_SPEED15;
    fdWeakParm.qLsCurve[16] = LS_OVER2LS0_SPEED16;
    fdWeakParm.qLsCurve[17] = LS_OVER2LS0_SPEED17;

}
// *****************************************************************************

/* Function:
    FieldWeakening()

  Summary:
    Routine implements field weakening

  Description:
    Function calculates the Id reference based on the motor speed

  Precondition:
    None.

  Parameters:
    Motor Speed

  Returns:
    Id reference.

  Remarks:
    None.
 */
int16_t FieldWeakening(int16_t qMotorSpeed) 
{
    int16_t iTempInt1, iTempInt2;

    int16_t qInvKFi;
    int16_t qLsDt;

    /* LsDt value - for base speed */
    qLsDt = motorParm.qLsDtBase;

    /* If the speed is less than one for activating the FW */
    if (qMotorSpeed <= fdWeakParm.qFwOnSpeed) 
    {
        /* Set Idref as first value in magnetizing curve */
        fdWeakParm.qIdRef = fdWeakParm.qFwCurve[0];

        /* Adapt filter parameter */
        estimator.qKfilterEsdq = KFILTER_ESDQ;

        /* Inverse Kfi constant for base speed */
        qInvKFi = motorParm.qInvKFiBase;
    } 
    else 
    {
        /* Get the index parameter */
        /* Index in FW-Table */
        fdWeakParm.qIndex = (qMotorSpeed - fdWeakParm.qFwOnSpeed) >> SPEED_INDEX_CONST;

        iTempInt1 = fdWeakParm.qFwCurve[fdWeakParm.qIndex] -
                    fdWeakParm.qFwCurve[fdWeakParm.qIndex + 1];
        iTempInt2 = (fdWeakParm.qIndex << SPEED_INDEX_CONST) +
                    fdWeakParm.qFwOnSpeed;
        iTempInt2 = qMotorSpeed - iTempInt2;

        /* Interpolation between two results from the Table */
        fdWeakParm.qIdRef = fdWeakParm.qFwCurve[fdWeakParm.qIndex]-
                (int16_t) (__builtin_mulss(iTempInt1, iTempInt2) >> SPEED_INDEX_CONST);

        /* Adapt filer parameter */
        estimator.qKfilterEsdq = KFILTER_ESDQ_FW;

        /* Interpolation between two results from the Table */
        iTempInt1 = fdWeakParm.qInvKFiCurve[fdWeakParm.qIndex] -
                    fdWeakParm.qInvKFiCurve[fdWeakParm.qIndex + 1];

        qInvKFi = fdWeakParm.qInvKFiCurve[fdWeakParm.qIndex] -
                  (int16_t) (__builtin_mulss(iTempInt1, iTempInt2) >> SPEED_INDEX_CONST);


        /* Interpolation between two results from the Table */
        iTempInt1 = fdWeakParm.qLsCurve[fdWeakParm.qIndex] -
                    fdWeakParm.qLsCurve[fdWeakParm.qIndex + 1];

        iTempInt1 = fdWeakParm.qLsCurve[fdWeakParm.qIndex] -
                    (int16_t) (__builtin_mulss(iTempInt1, iTempInt2) >> SPEED_INDEX_CONST);

        /* Lsdt = Lsdt0*Ls/Ls0 */
        qLsDt = (int16_t) (__builtin_mulss(qLsDt, iTempInt1) >> 14);
    }

    motorParm.qInvKFi = qInvKFi;
    motorParm.qLsDt = qLsDt;
    
    return fdWeakParm.qIdRef;
}

