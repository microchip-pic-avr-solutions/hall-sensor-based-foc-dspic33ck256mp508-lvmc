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
;  Angle

; Outputs:
;  SINCOS.sin
;  SINCOS.cos

; Constants
          .equ TableSize,128

; Local register usage
          .equ workW0,      w0  ; Working register
          .equ workW1,      w1  ; Working register

          .equ remainderW2, w2  ; Fraction for interpolation: 0->0xFFFF
          .equ indexW3,     w3  ; Index into table

          .equ pTabPtrW4,   w4  ; Pointer into table 
          .equ pTabBaseW5,  w5  ; Pointer into table base

          .equ y0W6,        w6  ; Y0 = mcSineTableInRam[Index]
          .equ angleW7,     w7  ; Angle
		  
		  .equ BaseptrW10, w10

      ;; Note: remainderW2 and workW0 must be even registers


;=================== CODE =====================

          .section  .text
          .global   _MC_CalculateSineCosine_Assembly_Ram
          .global   MC_CalculateSineCosine_Assembly_Ram

_MC_CalculateSineCosine_Assembly_Ram:
MC_CalculateSineCosine_Assembly_Ram:
     ;; workW0 : Angle
	 ;; workW1 : SINCOS*
          mov.w     workW0,angleW7
          push      BaseptrW10          
	      mov.w     workW1,BaseptrW10

          mov.w     #mcSineTableInRam,pTabBaseW5     ; Pointer into table base		  

     ;; Calculate Index and Remainder for fetching and interpolating Sin
          mov.w     #TableSize,workW0
          mul.uu    workW0,angleW7,remainderW2   ; high word in indexW3

     ;; Double Index since offsets are in bytes not words
          add.w     indexW3,indexW3,indexW3

     ;; Note at this point the indexW3 register has a value 0x00nn where nn
     ;; is the offset in bytes from the TabBase.  If below we always
     ;; use BYTE operations on the indexW3 register it will automatically
     ;; wrap properly for a TableSize of 128.  

     ;; Check for zero remainder
          cp0.w     remainderW2
          bra       nz,jInterpolate

     ;; Zero remainder allows us to skip the interpolation and use the 
     ;; table value directly
          add.w     indexW3,pTabBaseW5,pTabPtrW4
          mov.w     [pTabPtrW4],workW0    
		  mov.w     workW0,[BaseptrW10+SINCOS_sin]  ; write to qSin

     ;; Add 0x40 to Sin index to get Cos index.  This may go off end of
     ;; table but if we use only BYTE operations the wrap is automatic.
          add.b     #0x40,indexW3
          add.w     indexW3,pTabBaseW5,pTabPtrW4
          mov.w     [pTabPtrW4],workW0
		  mov.w     workW0,[BaseptrW10+SINCOS_cos]  ; write to qCos
	  
	 ;; Return 1	  
		  mov.w     #1, workW0
          pop       BaseptrW10		  
          return

jInterpolate:

     ;; ================= SIN =========================

     ;; Get Y1-Y0 = mcSineTableInRam[Index+1] - mcSineTableInRam[Index]
          add.w     indexW3,pTabBaseW5,pTabPtrW4
          mov.w     [pTabPtrW4],y0W6              ; Y0
          inc2.b    indexW3,indexW3               ; (Index += 2)&0xFF
          add.w     indexW3,pTabBaseW5,pTabPtrW4
          subr.w    y0W6,[pTabPtrW4],workW0      ; Y1 - Y0
  
     ;; Calcuate Delta = (Remainder*(Y1-Y0)) >> 16
          mul.us    remainderW2,workW0,workW0

     ;; workW1 contains upper word of (Remainder*(Y1-Y0)) 
     ;; *pSin = Y0 + Delta
          add.w     workW1,y0W6,workW0
		  mov.w     workW0,[BaseptrW10+SINCOS_sin]   ; write to qSin

     ;; ================= COS =========================

     ;; Add 0x40 to Sin index to get Cos index.  This may go off end of
     ;; table but if we use only BYTE operations the wrap is automatic.
     ;; Actualy only add 0x3E since Index increment by two above
          add.b     #0x3E,indexW3

     ;; Get Y1-Y0 = mcSineTableInRam[Index+1] - mcSineTableInRam[Index]
          add.w     indexW3,pTabBaseW5,pTabPtrW4
          mov.w     [pTabPtrW4],y0W6              ; Y0
          inc2.b    indexW3,indexW3               ; (Index += 2)&0xFF
          add.w     indexW3,pTabBaseW5,pTabPtrW4
          subr.w    y0W6,[pTabPtrW4],workW0       ; Y1 - Y0
  
     ;; Calcuate Delta = (Remainder*(Y1-Y0)) >> 16
          mul.us    remainderW2,workW0,workW0

     ;; workW1 contains upper word of (Remainder*(Y1-Y0)) 
     ;; *pCos = Y0 + Delta
          add.w     workW1,y0W6,workW0     
		  mov.w     workW0,[BaseptrW10+SINCOS_cos]   ; write to qCos

	 ;; Return 2
		  mov.w     #2, workW0		  
          pop       BaseptrW10	  
          return

          .end
