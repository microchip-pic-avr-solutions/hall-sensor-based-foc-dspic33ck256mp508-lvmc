/*******************************************************************************
  Motor Control Library Types Header File

  File Name:
    motor_control_types.h

  Summary:
    This header file lists all the types used by the Motor Control library.

  Description:
    This header file lists the type defines for structures used by the Motor 
    Control library. 
*******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc. All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _MOTOR_CONTROL_TYPES_H_    // Guards against multiple inclusion
#define _MOTOR_CONTROL_TYPES_H_

#include <stdint.h>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif


// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Alpha-Beta reference frame data type

  Description:
    This structure will host parameters related to Alpha-Beta reference frame.
*/
typedef struct
{
    // Alpha component
    int16_t alpha;
    
    // Beta component
    int16_t beta;
} MC_ALPHABETA_T;

// *****************************************************************************
/* Sine-Cosine data type

  Description:
    This structure will host parameters related to Sine and Cosine components of the motor angle.
*/
typedef struct
{
    // Cosine component
    int16_t cos;
    
    // Sine component
    int16_t sin;
} MC_SINCOS_T;

// *****************************************************************************
/* D-Q reference frame data type

  Description:
    This structure will host parameters related to D-Q reference frame.
*/
typedef struct
{
    // D-axis component
    int16_t d;
    
    // Q-axis component
    int16_t q;
} MC_DQ_T;

// *****************************************************************************
/* Duty-cycle data type

  Description:
    This structure will host parameters related to PWM module Duty Cycle values.
*/
typedef struct
{
    // Duty cycle for phase #1
    uint16_t dutycycle1;
    
    // Duty cycle for phase #2
    uint16_t dutycycle2;
    
    // Duty cycle for phase #3
    uint16_t dutycycle3;
} MC_DUTYCYCLEOUT_T;

// *****************************************************************************
/* ABC reference frame data type

  Description:
    This structure will host parameters related to ABC reference frame.
*/
typedef struct
{
    // Phase A component 
    int16_t a;
    
    // Phase B component
    int16_t b;
    
    // Phase C component
    int16_t c;
} MC_ABC_T;

// *****************************************************************************
/* PI Controller State data type

  Description:
    This structure will host parameters related to the PI Controller state.
*/
typedef struct
{
    // Integrator sum
    int32_t integrator;
    
    // Proportional gain co-efficient term
    int16_t kp;
    
    // Integral gain co-efficient term
    int16_t ki;
    
    // Excess gain co-efficient term
    int16_t kc;

    // Maximum output limit
    int16_t outMax;
    
    // Minimum output limit
    int16_t outMin;
} MC_PISTATE_T;

// *****************************************************************************
/* PI Controller Input data type

  Summary:
    PI Controller input type define

  Description:
    This structure will host parameters related to the PI Controller input. PI
    controller state is a part of the PI Controller input.
*/
typedef struct
{
    // PI state as input parameter to the PI controller
    MC_PISTATE_T piState;
    
    // Input reference to the PI controller
    int16_t inReference;
    
    // Input measured value
    int16_t inMeasure;
} MC_PIPARMIN_T;

// *****************************************************************************
/* PI Controller Output data type

  Description:
    This structure will host parameters related to the PI Controller output.
*/
typedef struct
{
    int16_t out;        // Output of the PI controller
} MC_PIPARMOUT_T;

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif // _MOTOR_CONTROL_TYPES_H_


/* EOF */


