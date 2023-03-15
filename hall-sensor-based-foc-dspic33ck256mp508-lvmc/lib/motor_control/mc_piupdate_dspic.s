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
;  PIPARMIN kp,ki,kc,out_Max,out_Min,in_Ref,in_Meas

; Outputs:
;  PIPARMOUT out

; Register usage

          .equ workW0,   w0  ; Working register
          .equ workW1,   w1  ; Working register (function I/O only, don't reuse)
          .equ workW2,   w2  ; Working register (function I/O only, don't reuse)
          .equ workW3,   w3  ; Working register (function I/O only, don't reuse)
		  
          .equ StateptrW1,   w1   ; PI state pointer
          .equ OutptrW2,     w2   ; Pointer to out
          .equ OutW3,        w3   ; Output
          .equ ErrW4,        w4   ; Error term: in_Ref-in_Meas
          .equ CorconW4,     w4   ; CORCON temp
          .equ workW5,       w5   ; Working register
          .equ UnlimitW6,    w6   ; U: unlimited output 
          .equ workW7,       w7   ; Working register

; Constants		  
		  .equ NKo,4 ; PI scaling coeff

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
          .global   _MC_ControllerPIUpdate_Assembly
          .global   MC_ControllerPIUpdate_Assembly

_MC_ControllerPIUpdate_Assembly:
MC_ControllerPIUpdate_Assembly:
     ;; Save CORCON, then configure as needed
          push      CORCON
          mov.w     #0x00E2,CorconW4
          mov.w     CorconW4,CORCON
     ;; workW0 : in_Ref
	 ;; workW1 : in_Meas
	 ;; workW2 : PISTATE*
	 ;; workW3 : PIPARMOUT* out

     ;; Err  = in_Ref - in_Meas
          lac       workW0,#0,A
          lac       workW1,#0,B
          sub       A
          sac       A,#0,ErrW4

          mov.w		workW2,StateptrW1
		  mov.w     workW3,OutptrW2   
          
     ;; U  = integrator + Kp * Err * 2^NKo
          mov.w     [StateptrW1+PISTATE_integratorH],workW0
          lac       workW0,B                 ; AccB = integrator
          mov.w     [StateptrW1+PISTATE_integratorL],workW0
          mov.w     workW0,ACCBL

          mov.w     [StateptrW1+PISTATE_kp],workW5
          mpy       ErrW4*workW5,A
          sftac     A,#-NKo                  ; AccA = Kp*Err*2^NKo     
          add       A                        ; Sum = Sum + Kp*Err*2^NKo
          sac       A,UnlimitW6              ; store U before tests

     ;; if( U > Outmax )
     ;;     Out = Outmax
     ;; else if( U < Outmin )
     ;;     Out = Outmin
     ;; else        
     ;;     Out = U 

          mov.w     [StateptrW1+PISTATE_out_Max],OutW3
          cp        UnlimitW6,OutW3
          bra       GT,jPI5             ; U > Outmax; OutW3 = Outmax

          mov.w     [StateptrW1+PISTATE_out_Min],OutW3
          cp        UnlimitW6,OutW3
          bra       LE,jPI5             ; U < Outmin; OutW3 = Outmin

          mov.w     UnlimitW6,OutW3     ; OutW3 = U
jPI5:
          mov.w     OutW3,[OutptrW2]

     ;; Ki * Err
          mov.w     [StateptrW1+PISTATE_ki],workW5
          mpy       ErrW4*workW5,A              ; A = Ki * ErrW4

     ;; Exc = U - Out
          sub.w     UnlimitW6,OutW3,UnlimitW6   ; UnlimitW6 = UnlimitW6 - OutW3

     ;; Ki * Err - Kc * Exc 
          mov.w     [StateptrW1+PISTATE_kc],workW5
          msc       workW5*UnlimitW6,A          ; A = A - (Kc * Exc)

     ;; Sum = Sum + Ki * Err - Kc * Exc 
          add       A

     ;; Store the integrator
          sac       A,workW0
          mov.w     workW0,[StateptrW1+PISTATE_integratorH] 
          mov.w     ACCAL,workW0
          mov.w     workW0,[StateptrW1+PISTATE_integratorL]

	 ;; Return 1	  
		  mov.w     #1, workW0
          pop       CORCON
          return

          .end
