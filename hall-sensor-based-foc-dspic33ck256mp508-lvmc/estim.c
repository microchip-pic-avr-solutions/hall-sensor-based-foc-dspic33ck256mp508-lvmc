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

#include <libq.h>
#include "motor_control_noinline.h"
#include "userparms.h"
#include "estim.h"
#include "control.h"

#define DECIMATE_NOMINAL_SPEED    NOMINAL_SPEED_RPM*NOPOLESPAIRS/10
#define NOMINAL_ELECTRICAL_SPEED  NOMINAL_SPEED_RPM*NOPOLESPAIRS

/** Variables */
ESTIM_PARM_T estimator;
MOTOR_ESTIM_PARM_T motorParm;
MC_ALPHABETA_T bemfAlphaBeta;
MC_DQ_T bemfdq;
MC_SINCOS_T sincosThetaEstimator;

// *****************************************************************************

/* Function:
    Estim()

  Summary:
    Motor speed and angle estimator

  Description:
    Estimation of the speed of the motor and field angle based on inverter
    voltages and motor currents.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void Estim(void) 
{
    int32_t tempint;
    uint16_t index = (estimator.qDiCounter - 7)&0x0007;

    /* dIalpha = Ialpha-oldIalpha,  dIbeta  = Ibeta-oldIbeta
       For lower speed the granularity of difference is higher - the
       difference is made between 2 sampled values @ 8 ADC ISR cycles */
    if (_Q15abs(estimator.qVelEstim) < NOMINAL_ELECTRICAL_SPEED) 
    {

        estimator.qDIalpha = (ialphabeta.alpha -
                estimator.qLastIalphaHS[index]);
        /* The current difference can exceed the maximum value per 8 ADC ISR
           cycle .The following limitation assures a limitation per low speed -
           up to the nominal speed */
        if (estimator.qDIalpha > estimator.qDIlimitLS) 
        {
            estimator.qDIalpha = estimator.qDIlimitLS;
        }
        if (estimator.qDIalpha < -estimator.qDIlimitLS) 
        {
            estimator.qDIalpha = -estimator.qDIlimitLS;
        }
        estimator.qVIndalpha = (int16_t) (__builtin_mulss(motorParm.qLsDt,
                estimator.qDIalpha) >> 10);

        estimator.qDIbeta = (ialphabeta.beta - estimator.qLastIbetaHS[index]);
        /* The current difference can exceed the maximum value per 8 ADC ISR cycle
           the following limitation assures a limitation per low speed - up to
           the nominal speed */
        if (estimator.qDIbeta > estimator.qDIlimitLS) 
        {
            estimator.qDIbeta = estimator.qDIlimitLS;
        }
        if (estimator.qDIbeta < -estimator.qDIlimitLS) 
        {
            estimator.qDIbeta = -estimator.qDIlimitLS;
        }
        estimator.qVIndbeta = (int16_t) (__builtin_mulss(motorParm.qLsDt,
                estimator.qDIbeta) >> 10);

    } 
    else 
    {

        estimator.qDIalpha = (ialphabeta.alpha -
                estimator.qLastIalphaHS[(estimator.qDiCounter)]);
        /* The current difference can exceed the maximum value per 1 ADC ISR cycle
           the following limitation assures a limitation per high speed - up to
           the maximum speed */
        if (estimator.qDIalpha > estimator.qDIlimitHS) 
        {
            estimator.qDIalpha = estimator.qDIlimitHS;
        }
        if (estimator.qDIalpha < -estimator.qDIlimitHS) 
        {
            estimator.qDIalpha = -estimator.qDIlimitHS;
        }
        estimator.qVIndalpha = (int16_t) (__builtin_mulss(motorParm.qLsDt,
                estimator.qDIalpha) >> 7);

        estimator.qDIbeta = (ialphabeta.beta -
                estimator.qLastIbetaHS[(estimator.qDiCounter)]);

        /* The current difference can exceed the maximum value per 1 ADC ISR cycle
           the following limitation assures a limitation per high speed - up to
           the maximum speed */
        if (estimator.qDIbeta > estimator.qDIlimitHS) 
        {
            estimator.qDIbeta = estimator.qDIlimitHS;
        }
        if (estimator.qDIbeta < -estimator.qDIlimitHS) 
        {
            estimator.qDIbeta = -estimator.qDIlimitHS;
        }
        estimator.qVIndbeta = (int16_t) (__builtin_mulss(motorParm.qLsDt,
                estimator.qDIbeta) >> 7);
    }

    /* Update  LastIalpha and LastIbeta */
    estimator.qDiCounter = (estimator.qDiCounter + 1) & 0x0007;
    estimator.qLastIalphaHS[estimator.qDiCounter] = ialphabeta.alpha;
    estimator.qLastIbetaHS[estimator.qDiCounter] = ialphabeta.beta;

    /* Stator voltage equations
     Ualpha = Rs * Ialpha + Ls dIalpha/dt + BEMF
     BEMF = Ualpha - Rs Ialpha - Ls dIalpha/dt */

    bemfAlphaBeta.alpha =  valphabeta.alpha -
                        (int16_t) (__builtin_mulss(motorParm.qRs, 
                                  ialphabeta.alpha) >> 11) -
                        estimator.qVIndalpha;

    /* The multiplication between the Rs and Ialpha was shifted by 14 instead
       of 15 because the Rs value normalized exceeded Q15 range, so it was
       divided by 2 immediately after the normalization - in userparms.h */

    /* Ubeta = Rs * Ibeta + Ls dIbeta/dt + BEMF
       BEMF = Ubeta - Rs Ibeta - Ls dIbeta/dt */
    bemfAlphaBeta.beta =   valphabeta.beta -
                        (int16_t) (__builtin_mulss(motorParm.qRs,
                                 ialphabeta.beta) >> 11) -
                        estimator.qVIndbeta;

    /* The multiplication between the Rs and Ibeta was shifted by 14 instead of 15
     because the Rs value normalized exceeded Q15 range, so it was divided by 2
     immediately after the normalization - in userparms.h */
    MC_CalculateSineCosine_Assembly_Ram((estimator.qRho + estimator.qRhoOffset),
                                        &sincosThetaEstimator);

    /*  Park_BEMF.d =  Clark_BEMF.alpha*cos(Angle) + Clark_BEMF.beta*sin(Rho)
       Park_BEMF.q = -Clark_BEMF.alpha*sin(Angle) + Clark_BEMF.beta*cos(Rho)*/
    MC_TransformPark_Assembly(&bemfAlphaBeta, &sincosThetaEstimator, &bemfdq);

    /* Filter first order for Esd and Esq
       EsdFilter = 1/TFilterd * Integral{ (Esd-EsdFilter).dt } */
    tempint = (int16_t) (bemfdq.d - estimator.qEsdf);
    estimator.qEsdStateVar += __builtin_mulss(tempint, estimator.qKfilterEsdq);
    estimator.qEsdf = (int16_t) (estimator.qEsdStateVar >> 15);

    tempint = (int16_t) (bemfdq.q - estimator.qEsqf);
    estimator.qEsqStateVar += __builtin_mulss(tempint, estimator.qKfilterEsdq);
    estimator.qEsqf = (int16_t) (estimator.qEsqStateVar >> 15);

    /* OmegaMr= InvKfi * (Esqf -sgn(Esqf) * Esdf)
       For stability the condition for low speed */
    if (_Q15abs(estimator.qVelEstim) > DECIMATE_NOMINAL_SPEED) 
    {
        if (estimator.qEsqf > 0) 
        {
            tempint = (int16_t) (estimator.qEsqf - estimator.qEsdf);
            estimator.qOmegaMr =  (int16_t) (__builtin_mulss(motorParm.qInvKFi,
                                    tempint) >> 15);
        } 
        else 
        {
            tempint = (int16_t) (estimator.qEsqf + estimator.qEsdf);
            estimator.qOmegaMr = (int16_t) (__builtin_mulss(motorParm.qInvKFi,
                                    tempint) >> 15);
        }
    }        
    /* if estimator speed<10% => condition VelRef<>0 */
    else 
    {
        if (estimator.qVelEstim > 0) 
        {
            tempint = (int16_t) (estimator.qEsqf - estimator.qEsdf);
            estimator.qOmegaMr = (int16_t) (__builtin_mulss(motorParm.qInvKFi,
                                    tempint) >> 15);
        } 
        else 
        {
            tempint = (int16_t) (estimator.qEsqf + estimator.qEsdf);
            estimator.qOmegaMr = (int16_t) (__builtin_mulss(motorParm.qInvKFi,
                                    tempint) >> 15);
        }
    }
    /* The result of the calculation above is shifted left by one because
       initial value of InvKfi was shifted by 2 after normalizing -
       assuring that extended range of the variable is possible in the
       lookup table the initial value of InvKfi is defined in userparms.h */
    estimator.qOmegaMr = estimator.qOmegaMr << 1;
    
    /* the integral of the angle is the estimated angle */
    estimator.qRhoStateVar += __builtin_mulss(estimator.qOmegaMr,
                                estimator.qDeltaT);
    estimator.qRho = (int16_t) (estimator.qRhoStateVar >> 15);


    /* The estimated speed is a filter value of the above calculated OmegaMr.
       The filter implementation is the same as for BEMF d-q components
       filtering */
    tempint = (int16_t) (estimator.qOmegaMr - estimator.qVelEstim);
    estimator.qVelEstimStateVar += __builtin_mulss(tempint,
                                    estimator.qVelEstimFilterK);
    estimator.qVelEstim = (int16_t) (estimator.qVelEstimStateVar >> 15);

}
// *****************************************************************************

/* Function:
    InitEstimParm ()

  Summary:
    Initializes Motor speed and angle estimator parameters

  Description:
    Initialization of the parameters of the estimator.

  Precondition:
    None.

  Parameters:
    None

  Returns:
    None.

  Remarks:
    None.
 */
void InitEstimParm(void) 
{
    /* Constants are defined in usreparms.h */

    motorParm.qLsDtBase = NORM_LSDTBASE;
    motorParm.qLsDt = motorParm.qLsDtBase;
    motorParm.qRs = NORM_RS;

    motorParm.qInvKFiBase = NORM_INVKFIBASE;
    motorParm.qInvKFi = motorParm.qInvKFiBase;

    estimator.qRhoStateVar = 0;
    estimator.qOmegaMr = 0;
    estimator.qDiCounter = 0;
    estimator.qEsdStateVar = 0;
    estimator.qEsqStateVar = 0;

    estimator.qDIlimitHS = D_ILIMIT_HS;
    estimator.qDIlimitLS = D_ILIMIT_LS;

    estimator.qKfilterEsdq = KFILTER_ESDQ;
    estimator.qVelEstimFilterK = KFILTER_VELESTIM;

    estimator.qDeltaT = NORM_DELTAT;
    estimator.qRhoOffset = INITOFFSET_TRANS_OPEN_CLSD;

}
