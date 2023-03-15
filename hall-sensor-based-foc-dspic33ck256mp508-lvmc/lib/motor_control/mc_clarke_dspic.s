;*********************************************************************************
; Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.
; Microchip licenses to you the right to use, modify, copy and distribute
; Software only when embedded on a Microchip microcontroller or digital signal
; controller that is integrated into your product or third party product
; (pursuant to the sublicense terms in the accompanying license agreement).
; 
; You should refer to the license agreement accompanying this Software for
; additional information regarding your rights and obligations.
; 
; SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
; EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
; MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
; IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
; CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
; OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
; INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
; CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
; SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
; (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
;*********************************************************************************

          .include "mc_interfaces_dspic.inc"
; Inputs:
;  ABC.a
;  ABC.b

; Outputs:
;  ALPHABETA.alpha
;  ALPHABETA.beta

; Register usage
          .equ workW0,  w0  ; working register
	      .equ workW1,  w1  ; working register
		  
          .equ Sq3W4,   w4  ; OneBySq3
          .equ CorconW4, w4 ; CORCON temp

          .equ aW6,    w6   ; a
          .equ bW7,    w7   ; b
          .equ betaW7, w7   ; beta replaces b
		  
; Constants
          .equ OneBySq3,0x49E7   ; 1/sqrt(3) in 1.15 format

; Note
;  This function requires CORCON register to be setup in a certain state
;  in order to operate correctly. Due to this requirement, this function
;  will save the CORCON register on the stack in the beginning of the
;  function and restore it before the function return.
;  After saving the CORCON register, this function writes to all bits of
;  the CORCON register. Thus, for the brief duration when this function is
;  executing, the state of CORCON register may be different from its state
;  as set by the function caller. This may change the CPU core behavior with
;  respect to exception processing latency, DO loop termination, CPU interrupt
;  priority level and DSP-engine behavior.

;=================== CODE =====================

          .section  .text
          .global   _MC_TransformClarke_Assembly
          .global   MC_TransformClarke_Assembly

_MC_TransformClarke_Assembly:
MC_TransformClarke_Assembly:
     ;; Save CORCON, then configure as needed
          push      CORCON
          mov.w     #0x00E2,CorconW4
          mov.w     CorconW4,CORCON
     ;; workW0 : ABC*
     ;; workW1 : ALPHABETA*

          mov.w     #OneBySq3,Sq3W4     ; 1/sqrt(3) in 1.15 format

     ;; alpha = a
          mov.w     [workW0+ABC_a],aW6
          mov.w     aW6,[workW1+ALPHABETA_alpha]		  

     ;; beta = a*OneBySq3 + 2*b*OneBySq3
          mpy       Sq3W4*aW6,A
          mov.w     [workW0+ABC_b],bW7
          mac       Sq3W4*bW7,A
          mac       Sq3W4*bW7,A
          sac       A,betaW7
          mov.w     betaW7,[workW1+ALPHABETA_beta]
		  
	 ;; Return 1	  
		  mov.w     #1, workW0		  
          pop       CORCON
          return
          .end
