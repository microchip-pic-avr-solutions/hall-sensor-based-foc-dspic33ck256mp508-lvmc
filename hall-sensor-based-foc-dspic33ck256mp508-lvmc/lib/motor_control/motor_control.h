/*******************************************************************************
  Motor Control Library Interface Header File

  File Name:
    motor_control.h

  Summary:
    This header file lists all the interfaces used by the Motor Control library.

  Description:
    This header file lists the type defines for structures used by the Motor 
    Control library. Library function definitions are also listed along with
    information regarding the arguments of each library function. This header file
    also includes another header file that hosts inline definitions of certain
    library functions.
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
/*******************************************************************************
Note:
* Some parts of this header file are protected by #ifdef __XC16__. These protections
  are provided to accomodate non-XC16 compilers to work with this header file.
  Similarly, sections of the header file related to the MATLAB-based compiler are
  protected by #ifdef __MATLAB_MEX__ protections.
* Some of the function declarations have a MC_ATTRB prefix. This prefix has been
  provided as a placeholder for adding attributes for supporting future versions
  of the compiler.
*******************************************************************************/
// DOM-IGNORE-END

#ifndef _MOTOR_CONTROL_H_    // Guards against multiple inclusion
#define _MOTOR_CONTROL_H_

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#ifdef __XC16__  // See comments at the top of this header file
#include <xc.h>
#endif // __XC16__
#include "motor_control_mapping.h"

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

#ifdef __MATLAB_MEX__ // See comments at the top of this header file
#define inline
#endif // __MATLAB_MEX


#include "motor_control_declarations.h"
#include "motor_control_inline_declarations.h"


#ifdef __XC16__   // See comments at the top of this header file
#include "./motor_control_inline_dspic.h"
#endif // __XC16__

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif // _MOTOR_CONTROL_H


/* EOF */


