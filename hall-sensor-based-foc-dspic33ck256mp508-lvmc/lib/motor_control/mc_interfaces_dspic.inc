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

.equ ALPHABETA_alpha,0	; alpha
.equ ALPHABETA_beta,2	; beta
.extern _ALPHABETA

.equ SINCOS_cos,0       ; cos
.equ SINCOS_sin,2       ; sin
.extern _SINCOS

.equ DQ_d,0				; d
.equ DQ_q,2				; q
.extern _DQ

.equ DUTYCYCLEOUT_dutycycle1,0      ; dutycycle1
.equ DUTYCYCLEOUT_dutycycle2,2      ; dutycycle2
.equ DUTYCYCLEOUT_dutycycle3,4      ; dutycycle3
.extern _DUTYCYCLEOUT

.equ ABC_a,0            ; a
.equ ABC_b,2            ; b
.equ ABC_c,4            ; c
.extern _ABC

.equ PISTATE_integratorL,0    
.equ PISTATE_integratorH,2    
.equ PISTATE_kp,4      
.equ PISTATE_ki,6      
.equ PISTATE_kc,8      
.equ PISTATE_out_Max,10 
.equ PISTATE_out_Min,12 
.extern _PISTATE

.equ PIPARMIN_sumL,0    
.equ PIPARMIN_sumH,2    
.equ PIPARMIN_kp,4      
.equ PIPARMIN_ki,6      
.equ PIPARMIN_kc,8      
.equ PIPARMIN_out_Max,10 
.equ PIPARMIN_out_Min,12 
.equ PIPARMIN_in_Ref,14  
.equ PIPARMIN_in_Meas,16 
.extern _PIPARMIN

.equ PIPARMOUT_out,0  
.extern _PIPARMOUT

; EOF


