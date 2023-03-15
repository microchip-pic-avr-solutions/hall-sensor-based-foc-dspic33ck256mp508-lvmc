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
;  SINCOS.cos
;  SINCOS.sin
;  ALPHABETA.alpha
;  ALPHABETA.beta

; Outputs:
;  DQ.d
;  DQ.q

; Register usage
          .equ workW0,  w0    ; working register
	      .equ workW1,  w1    ; working register
	      .equ workW2,  w2    ; working register
          .equ CorconW4,w4    ; CORCON temp
          .equ sinW4,   w4    ; Sine
          .equ cosW5,   w5    ; Cosine
          .equ alphaW6, w6    ; alpha
          .equ betaW7,  w7    ; beta

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
          .global   _MC_TransformPark_Assembly
          .global   MC_TransformPark_Assembly

_MC_TransformPark_Assembly:
MC_TransformPark_Assembly:
     ;; Save CORCON, then configure as needed
          push      CORCON
          mov.w     #0x00E2,CorconW4
          mov.w     CorconW4,CORCON
     ;; workW0 : ALPHABETA*
	 ;; workW1 : SINCOS*
	 ;; workW2 : DQ*
          
     ;; Get sin, cos from SINCOS structure
          mov.w     [workW1+SINCOS_sin],sinW4
          mov.w     [workW1+SINCOS_cos],cosW5

     ;; Get alpha, beta from ALPHABETA structure
          mov.w     [workW0+ALPHABETA_alpha],alphaW6
          mov.w     [workW0+ALPHABETA_beta],betaW7		  
		  
     ;; d = alpha*cos(Angle) + beta*sin(Angle)
          mpy       sinW4*betaW7,A          ; beta*sin -> A
          mac       cosW5*alphaW6,A         ; add alpha*cos to A
          sac       A,workW0                ; store A into workW0
		  mov.w     workW0,[workW2+DQ_d]    ; Update d

     ;; q = -alpha*sin(Angle) + beta*cos(Angle)
          mpy       cosW5*betaW7,A          ; beta*cos -> A
          msc       sinW4*alphaW6,A         ; sub alpha*sin from A
          sac       A,workW0                ; store A into workW0
		  mov.w     workW0,[workW2+DQ_q]    ; Update q
		  
	 ;; Return 1	  
		  mov.w     #1, workW0		  
          pop       CORCON
          return

          .end
