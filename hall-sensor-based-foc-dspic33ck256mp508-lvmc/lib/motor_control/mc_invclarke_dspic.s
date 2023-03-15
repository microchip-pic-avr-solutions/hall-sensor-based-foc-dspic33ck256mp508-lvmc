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
;  ALPHABETA.alpha
;  ALPHABETA.beta

; Outputs:
;  ABC.a
;  ABC.b
;  ABC.c		  
		  
; Register usage
          .equ workW0,  w0  ; working register
		  .equ workW1,  w1  ; working register

          .equ workW2,  w2  ; working register
          .equ CorconW4,w4  ; CORCON temp
          .equ alphaW4, w4  ; alpha
          .equ betaW5,  w5  ; beta
          .equ Sq3Ov2W6,w6  ; sqrt(3)/2

; Constants
          .equ Sq3OV2,0x6ED9 ; sqrt(3)/2 in 1.15 format

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
          .global   _MC_TransformClarkeInverseSwappedInput_Assembly
          .global   MC_TransformClarkeInverseSwappedInput_Assembly

_MC_TransformClarkeInverseSwappedInput_Assembly:
MC_TransformClarkeInverseSwappedInput_Assembly:
     ;; Save CORCON, then configure as needed
          push      CORCON
          mov.w     #0x00E2,CorconW4
          mov.w     CorconW4,CORCON
     ;; workW0 : ALPHABETA*
     ;; workW1 : ABC*

     ;; Get alpha, beta from ALPHABETA structure
          mov.w     [workW0+ALPHABETA_alpha],alphaW4
          mov.w     [workW0+ALPHABETA_beta],betaW5

     ;; a = beta
          mov.w     betaW5,[workW1+ABC_a]

     ;; Load Sq(3)/2
          mov.w     #Sq3OV2,Sq3Ov2W6

     ;; AccA = -beta/2
          lac       betaW5,#1,A
          neg       A

     ;; b = -beta/2 + (sqrt(3)/2) * alpha
          mac       alphaW4*Sq3Ov2W6,A           ;add alpha*sqrt(3)/2 to A
          sac       A,workW2
          mov.w     workW2,[workW1+ABC_b]

     ;; AccA = -beta/2
          lac       betaW5,#1,A
          neg       A

     ;; c = -beta/2 - (sqrt(3)/2) * alpha
          msc       alphaW4*Sq3Ov2W6,A            ;sub alpha*sqrt(3)/2 from A
          sac       A,workW2
          mov.w     workW2,[workW1+ABC_c]
		  
	 ;; Return 1	  
		  mov.w     #1, workW0
          pop       CORCON
          return
          .end
