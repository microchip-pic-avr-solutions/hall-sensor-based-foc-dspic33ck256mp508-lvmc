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

;=================== LOCAL DATA =====================

          .section .sinetbl,data,near
          .align	256
          .global mcSineTableInRam
          .global _mcSineTableInRam
_mcSineTableInRam:
mcSineTableInRam:  
  .word 0,1608,3212,4808,6393,7962,9512,11039
  .word 12540,14010,15446,16846,18205,19520,20787,22005
  .word 23170,24279,25330,26319,27245,28106,28898,29621
  .word 30273,30852,31357,31785,32138,32413,32610,32728
  .word 32767,32728,32610,32413,32138,31785,31357,30852
  .word 30273,29621,28898,28106,27245,26319,25330,24279
  .word 23170,22005,20787,19520,18205,16846,15446,14010
  .word 12540,11039,9512,7962,6393,4808,3212,1608
  .word 0,-1608,-3212,-4808,-6393,-7962,-9512,-11039
  .word -12540,-14010,-15446,-16846,-18205,-19520,-20787,-22005
  .word -23170,-24279,-25330,-26319,-27245,-28106,-28898,-29621
  .word -30273,-30852,-31357,-31785,-32138,-32413,-32610,-32728
  .word -32767,-32728,-32610,-32413,-32138,-31785,-31357,-30852
  .word -30273,-29621,-28898,-28106,-27245,-26319,-25330,-24279
  .word -23170,-22005,-20787,-19520,-18205,-16846,-15446,-14010
  .word -12540,-11039,-9512,-7962,-6393,-4808,-3212,-1608

          .end
