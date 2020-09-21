; ******************************************************************************
; * © 2008 Microchip Technology Inc.
; *
; SOFTWARE LICENSE AGREEMENT:
; Microchip Technology Incorporated ("Microchip") retains all ownership and 
; intellectual property rights in the code accompanying this message and in all 
; derivatives hereto.  You may use this code, and any derivatives created by 
; any person or entity by or on your behalf, exclusively with Microchip's
; proprietary products.  Your acceptance and/or use of this code constitutes 
; agreement to the terms and conditions of this notice.
;
; CODE ACCOMPANYING THIS MESSAGE IS SUPPLIED BY MICROCHIP "AS IS".  NO 
; WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED 
; TO, IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A 
; PARTICULAR PURPOSE APPLY TO THIS CODE, ITS INTERACTION WITH MICROCHIP'S 
; PRODUCTS, COMBINATION WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
;
; YOU ACKNOWLEDGE AND AGREE THAT, IN NO EVENT, SHALL MICROCHIP BE LIABLE, WHETHER 
; IN CONTRACT, WARRANTY, TORT (INCLUDING NEGLIGENCE OR BREACH OF STATUTORY DUTY), 
; STRICT LIABILITY, INDEMNITY, CONTRIBUTION, OR OTHERWISE, FOR ANY INDIRECT, SPECIAL, 
; PUNITIVE, EXEMPLARY, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, FOR COST OR EXPENSE OF 
; ANY KIND WHATSOEVER RELATED TO THE CODE, HOWSOEVER CAUSED, EVEN IF MICROCHIP HAS BEEN 
; ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
; ALLOWABLE BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY RELATED TO 
; THIS CODE, SHALL NOT EXCEED THE PRICE YOU PAID DIRECTLY TO MICROCHIP SPECIFICALLY TO 
; HAVE THIS CODE DEVELOPED.
;
; You agree that you are solely responsible for testing the code and 
; determining its suitability.  Microchip has no obligation to modify, test, 
; certify, or support the code.
;
; *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.equ __33FJ16GS502, 1
.include "p33FJ16GS502.inc"

; Buck 1 Maximum Duty cycle for voltage mode control
.equ Buck1MaxDC, 2400	    ; half of the period for the PHASE2

; Buck 1 Minimum Duty cycle for voltage mode control
.equ Buck1MinDC, 0

.equ    offsetabcCoefficients, 0
.equ    offsetcontrolHistory, 2
.equ    offsetcontrolOutput, 4
.equ    offsetmeasuredOutput, 6
.equ    offsetcontrolReference, 8

.equ    trigger_offset, 150

.data
.text


.global __ADCP0Interrupt


__ADCP0Interrupt:
    push w0
    push w1
    push w2

;    btg LATB, #10                       ;toggle RB10 (pin7) on every measurement
    mov _zero_transducer, w0            ; load zero value
    mov ADCBUF0, w1                     ; load the current measurement
    sub.w w1, w0, w1                        ; subtract the zero value to get the final current.
                                        
   ; neg.w w1, w1                        ; the transducer is oposite connected
    bra NN, bypass                      ; only positive result of the current measurements
    clr   w1
bypass:
    sl  w1, #5, w1						; Since only a 10-bit ADC shift left
								; by 5 leaving the 16-bit 0 for a positive #

    mov.w w1, w2                ; ! preserve w1
    mov.w _average, w0                    ; w0 is the average
    sub.w w1, w0, w1              ; form a delta =new measurement - old average
    asr  w1, #6, w1                      ; divide delta by 64
    add.w w1, w0, w1              ; new average =add delta to the old average
    mov.w w1, _average

    mov ADCBUF1, w1               ; load Out voltage
    sl  w1, #5, w1                ; Since only a 10-bit ADC shift left 5 steps
    mov.w _vout_average, w0       ; w0 is the average
    sub.w w1, w0, w1              ; form a delta =new measurement - old average
    asr  w1, #6, w1                      ; divide delta by 64
    add.w w1, w0, w1              ; new average =add delta to the old average
    mov.w w1, _vout_average

    mov.w _PID_enable, w1
    cp0	w1
    bra	Z, clear_adc_finish
    
    mov.w w2, w1                ; ! restore w1
    
    mov #_Buck1VoltagePID, w0
    mov w1, [w0+#offsetmeasuredOutput]
    call _PID  							; Call PID routine
    mov.w [w0+#offsetcontrolOutput], w1 ; Clamp PID output to allowed limits
    
    asr w1, #2, w0						; Scaling factor for voltage mode control

	mov.w #Buck1MinDC, w1				; Saturate to minimum duty cycle
	cpsgt w0, w1
	mov.w w1, w0
	mov.w #Buck1MaxDC, w1				; Saturate to maximum duty cycle
	cpslt w0, w1
	mov.w w1, w0

	mov.w w0, PHASE2						; Update new Duty Cycle
;    sub.w #trigger_offset, w0           ; subtract a small offset from the duty cycle to measure the current right before the switch is off
;    mov.w w0, TRIG1						; Update new trigger value to correspond to new duty cycle

clear_adc_finish:
    bclr	ADSTAT,	#0					; Clear Pair 0 conversion status bit
    bclr 	IFS6, #14					; Clear Pair 0 Interrupt Flag

    pop w2
    pop w1
    pop w0
	retfie								; Return from interrupt

	.end
