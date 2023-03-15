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
;  ABC.c
;  iPwmPeriod

; Outputs:
;  DUTYCYCLEOUT.dutycycle1
;  DUTYCYCLEOUT.dutycycle2
;  DUTYCYCLEOUT.dutycycle3

; Register usage
          .equ workW0,      w0  ; Working register             
          .equ workW1,      w1  ; Working register
          .equ workW2,      w2  ; working register          
                        
          .equ t1W2,        w2
          .equ t2W3,        w3
                        
          .equ workDLoW4,   w4  ; double word (multiply results)
          .equ aW4,         w4
          .equ taW4,        w4
          .equ workDHiW5,   w5  ; double word (multiply results)
          .equ bW5,         w5
          .equ tbW5,        w5
          .equ cW6,         w6
          .equ tcW6,        w6
		  .equ pwmperW7,    w7
          
		  .equ BaseptrW10,  w10
		  
;=================== CODE =====================

          .section  .text
          .global   _MC_CalculateSpaceVectorPhaseShifted_Assembly
          .global   MC_CalculateSpaceVectorPhaseShifted_Assembly

_MC_CalculateSpaceVectorPhaseShifted_Assembly:
MC_CalculateSpaceVectorPhaseShifted_Assembly:
     ;; workW0 : ABC*
     ;; workW1 : iPwmPeriod
     ;; workW2 : DUTYCYCLEOUT*
          push       BaseptrW10
	      mov.w      workW0,BaseptrW10
          mov.w      workW1,pwmperW7
	 
     ;; Get a,b,c
          mov.w     [BaseptrW10+ABC_a],aW4
          mov.w     [BaseptrW10+ABC_b],bW5
          mov.w     [BaseptrW10+ABC_c],cW6
          mov.w      workW2,BaseptrW10

     ;; Test a
          cp0       aW4
          bra       LT,jCalcRef20       ; aW4 < 0

     ;; Test b
          cp0       bW5
          bra       LT,jCalcRef10       ; bW5 < 0

     ;; Must be Sector 3 since Sector 7 not allowed
     ;; Sector 3: (0,1,1)  0-60 degrees

     ;; T1 = a
     ;; T2 = b
          mov.w     bW5,t2W3
          mov.w     aW4,t1W2

     ;; Since T1 is in 1.15 and PWM in integer we do multiply by
     ;; PWM*T1 as integers and use upper word of results

     ;; Load PWMPeriod
          mov.w	    pwmperW7,workW0
		  sl.w      workW0,workW0
		  
     ;; T1 = PWM*T1		  
          mul.us    workW0,t1W2,workDLoW4
          mov.w     workDHiW5,t1W2

     ;; T2 = PWM*T2
          mul.us    workW0,t2W3,workDLoW4
          mov.w     workDHiW5,t2W3

     ;; Tc = (PWM-T1-T2)/2
          sub.w     pwmperW7,t1W2,workW1     ;PWM-T1
          sub.w     workW1,t2W3,workW1     ; -T2
          asr.w     workW1,workW1          ; /2
          mov.w     workW1,tcW6            ; store Tc

     ;; Tb = Tc + T1
          add.w     workW1,t1W2,workW1
          mov.w     workW1,tbW5

     ;; Ta = Tb + T2
          add.w     workW1,t2W3,workW1
          mov.w     workW1,taW4

     ;; dutycycle1 = Ta
     ;; dutycycle2 = Tb
     ;; dutycycle3 = Tc
          mov.w     taW4,[BaseptrW10+DUTYCYCLEOUT_dutycycle1]
          mov.w     tbW5,[BaseptrW10+DUTYCYCLEOUT_dutycycle2]
          mov.w     tcW6,[BaseptrW10+DUTYCYCLEOUT_dutycycle3]
          bra		exitSVM

jCalcRef10:
     ;; Test c
          cp0       cW6
          bra       LT,jCalcRef15       ; cW6 < 0

     ;; Sector 5: (1,0,1)  120-180 degrees
     ;; T1 = c
     ;; T2 = a
          mov.w     aW4,t2W3
          mov.w     cW6,t1W2

     ;; Since T1 is in 1.15 and PWM in integer we do multiply by
     ;; PWM*T1 as integers and use upper word of results

     ;; Load PWMPeriod
          mov.w	    pwmperW7,workW0
		  sl.w      workW0,workW0

     ;; T1 = PWM*T1
          mul.us    workW0,t1W2,workDLoW4
          mov.w     workDHiW5,t1W2

     ;; T2 = PWM*T2
          mul.us    workW0,t2W3,workDLoW4
          mov.w     workDHiW5,t2W3

     ;; Tc = (PWM-T1-T2)/2
          sub.w     pwmperW7,t1W2,workW1     ;PWM-T1
          sub.w     workW1,t2W3,workW1     ; -T2
          asr.w     workW1,workW1          ; /2
          mov.w     workW1,tcW6            ; store Tc

     ;; Tb = Tc + T1
          add.w     workW1,t1W2,workW1
          mov.w     workW1,tbW5

     ;; Ta = Tb + T2
          add.w     workW1,t2W3,workW1
          mov.w     workW1,taW4

     ;; dutycycle1 = Tc
     ;; dutycycle2 = Ta
     ;; dutycycle3 = Tb
          mov.w     tcW6,[BaseptrW10+DUTYCYCLEOUT_dutycycle1]
          mov.w     taW4,[BaseptrW10+DUTYCYCLEOUT_dutycycle2]
          mov.w     tbW5,[BaseptrW10+DUTYCYCLEOUT_dutycycle3]
          bra		exitSVM

jCalcRef15:

     ;; Sector 1: (0,0,1)  60-120 degrees
     ;; T1 = -c
     ;; T2 = -b
          neg.w     bW5,t2W3
          neg.w     cW6,t1W2

     ;; Since T1 is in 1.15 and PWM in integer we do multiply by
     ;; PWM*T1 as integers and use upper word of results

     ;; Load PWMPeriod
          mov.w	    pwmperW7,workW0
		  sl.w      workW0,workW0

     ;; T1 = PWM*T1
          mul.us    workW0,t1W2,workDLoW4
          mov.w     workDHiW5,t1W2

     ;; T2 = PWM*T2
          mul.us    workW0,t2W3,workDLoW4
          mov.w     workDHiW5,t2W3

     ;; Tc = (PWM-T1-T2)/2
          sub.w     pwmperW7,t1W2,workW1     ;PWM-T1
          sub.w     workW1,t2W3,workW1     ; -T2
          asr.w     workW1,workW1          ; /2
          mov.w     workW1,tcW6            ; store Tc

     ;; Tb = Tc + T1
          add.w     workW1,t1W2,workW1
          mov.w     workW1,tbW5

     ;; Ta = Tb + T2
          add.w     workW1,t2W3,workW1
          mov.w     workW1,taW4

     ;; dutycycle1 = Tb
     ;; dutycycle2 = Ta
     ;; dutycycle3 = Tc
          mov.w     tbW5,[BaseptrW10+DUTYCYCLEOUT_dutycycle1]
          mov.w     taW4,[BaseptrW10+DUTYCYCLEOUT_dutycycle2]
          mov.w     tcW6,[BaseptrW10+DUTYCYCLEOUT_dutycycle3]
          bra		exitSVM

jCalcRef20:

     ;; Test b
          cp0       bW5
          bra       LT,jCalcRef30       ; bW5 < 0

     ;; Test c
          cp0       cW6
          bra       LT,jCalcRef25       ; cW6 < 0

     ;; Sector 6: (1,1,0)  240-300 degrees
     ;; T1 = b
     ;; T2 = c
          mov.w     cW6,t2W3
          mov.w     bW5,t1W2

     ;; Since T1 is in 1.15 and PWM in integer we do multiply by
     ;; PWM*T1 as integers and use upper word of results

     ;; Load PWMPeriod
          mov.w	    pwmperW7,workW0
		  sl.w      workW0,workW0

     ;; T1 = PWM*T1
          mul.us    workW0,t1W2,workDLoW4
          mov.w     workDHiW5,t1W2

     ;; T2 = PWM*T2
          mul.us    workW0,t2W3,workDLoW4
          mov.w     workDHiW5,t2W3

     ;; Tc = (PWM-T1-T2)/2
          sub.w     pwmperW7,t1W2,workW1     ;PWM-T1
          sub.w     workW1,t2W3,workW1     ; -T2
          asr.w     workW1,workW1          ; /2
          mov.w     workW1,tcW6            ; store Tc

     ;; Tb = Tc + T1
          add.w     workW1,t1W2,workW1
          mov.w     workW1,tbW5

     ;; Ta = Tb + T2
          add.w     workW1,t2W3,workW1
          mov.w     workW1,taW4

     ;; dutycycle1 = Tb
     ;; dutycycle2 = Tc
     ;; dutycycle3 = Ta
          mov.w     tbW5,[BaseptrW10+DUTYCYCLEOUT_dutycycle1]
          mov.w     tcW6,[BaseptrW10+DUTYCYCLEOUT_dutycycle2]
          mov.w     taW4,[BaseptrW10+DUTYCYCLEOUT_dutycycle3]
          bra		exitSVM

jCalcRef25:
     ;; Sector 2: (0,1,0)  300-360 degrees
     ;; T1 = -a
     ;; T2 = -c
          neg.w     cW6,t2W3
          neg.w     aW4,t1W2

     ;; Since T1 is in 1.15 and PWM in integer we do multiply by
     ;; PWM*T1 as integers and use upper word of results

     ;; Load PWMPeriod
          mov.w	    pwmperW7,workW0
		  sl.w      workW0,workW0

     ;; T1 = PWM*T1
          mul.us    workW0,t1W2,workDLoW4
          mov.w     workDHiW5,t1W2

     ;; T2 = PWM*T2
          mul.us    workW0,t2W3,workDLoW4
          mov.w     workDHiW5,t2W3

     ;; Tc = (PWM-T1-T2)/2
          sub.w     pwmperW7,t1W2,workW1     ;PWM-T1
          sub.w     workW1,t2W3,workW1     ; -T2
          asr.w     workW1,workW1          ; /2
          mov.w     workW1,tcW6            ; store Tc

     ;; Tb = Tc + T1
          add.w     workW1,t1W2,workW1
          mov.w     workW1,tbW5

     ;; Ta = Tb + T2
          add.w     workW1,t2W3,workW1
          mov.w     workW1,taW4

     ;; dutycycle1 = Ta
     ;; dutycycle2 = Tc
     ;; dutycycle3 = Tb
          mov.w     taW4,[BaseptrW10+DUTYCYCLEOUT_dutycycle1]
          mov.w     tcW6,[BaseptrW10+DUTYCYCLEOUT_dutycycle2]
          mov.w     tbW5,[BaseptrW10+DUTYCYCLEOUT_dutycycle3]
          bra		exitSVM

jCalcRef30:
     ;; Must be Sector 4 since Sector 0 not allowed
     ;; Sector 4: (1,0,0)  180-240 degrees

     ;; T1 = -b
     ;; T2 = -a
          neg.w     aW4,t2W3
          neg.w     bW5,t1W2

     ;; Since T1 is in 1.15 and PWM in integer we do multiply by
     ;; PWM*T1 as integers and use upper word of results

     ;; Load PWMPeriod
          mov.w	    pwmperW7,workW0
		  sl.w      workW0,workW0

     ;; T1 = PWM*T1
          mul.us    workW0,t1W2,workDLoW4
          mov.w     workDHiW5,t1W2

     ;; T2 = PWM*T2
          mul.us    workW0,t2W3,workDLoW4
          mov.w     workDHiW5,t2W3

     ;; Tc = (PWM-T1-T2)/2
          sub.w     pwmperW7,t1W2,workW1     ;PWM-T1
          sub.w     workW1,t2W3,workW1     ; -T2
          asr.w     workW1,workW1          ; /2
          mov.w     workW1,tcW6            ; store Tc

     ;; Tb = Tc + T1
          add.w     workW1,t1W2,workW1
          mov.w     workW1,tbW5

     ;; Ta = Tb + T2
          add.w     workW1,t2W3,workW1
          mov.w     workW1,taW4

     ;; dutycycle1 = Tc
     ;; dutycycle2 = Tb
     ;; dutycycle3 = Ta
          mov.w     tcW6,[BaseptrW10+DUTYCYCLEOUT_dutycycle1]
          mov.w     tbW5,[BaseptrW10+DUTYCYCLEOUT_dutycycle2]
          mov.w     taW4,[BaseptrW10+DUTYCYCLEOUT_dutycycle3]

exitSVM:
	 ;; Return 1	  
		  mov.w     #1, workW0
          pop       BaseptrW10          
          return




