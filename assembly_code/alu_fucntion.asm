; main.asm
; Runs on LM4F120/TM4C123
; Test the GPIO initialization functions by setting the LED
; color according to the status of the switches.
; The Reflex Test (no longer supported; each LED turns others off):
; This program is functionally similar to SwitchTestMain.c
; in Switch_4F120asm.  When switch #1 is pressed, the blue
; LED comes on.  When switch #2 is pressed, the red LED
; comes on.  When both switches are pressed, the green LED
; comes on.  A short delay is inserted between
; polls of the buttons to compensate for your reflexes and
; the button bounce.  The following color combinations can
; be made:
; Color    LED(s) Illumination Method
; dark     ---    release both buttons
; red      R--    press right button (#2)
; blue     --B    press left button (#1)
; green    -G-    press both buttons exactly together
; yellow   RG-    press right button, then press left button
; sky blue -GB    press left button, then press right button
; white    RGB    press either button, then press the other
;                 button, then release the first button
; pink     R-B    press either button, then release the
;                 first button and immediately press the
;                 other button
; Daniel Valvano
; May 3, 2015

;  This example accompanies the book
;  "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
;  ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2015
;  Section 4.2    Program 4.1
;
;Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
;   You may use, edit, run or distribute this file
;   as long as the above copyright notice remains
;THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
;OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
;MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
;VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
;OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
;For more information about my classes, my research, and my books, see
;http://users.ece.utexas.edu/~valvano/

; negative logic switch #2 connected to PF0 on the Launchpad
; red LED connected to PF1 on the Launchpad
; blue LED connected to PF2 on the Launchpad
; green LED connected to PF3 on the Launchpad
; negative logic switch #1 connected to PF4 on the Launchpad
; NOTE: The NMI (non-maskable interrupt) is on PF0.  That means that
; the Alternate Function Select, Pull-Up Resistor, Pull-Down Resistor,
; and Digital Enable are all locked for PF0 until a value of 0x4C4F434B
; is written to the Port F GPIO Lock Register.  After Port F is
; unlocked, bit 0 of the Port F GPIO Commit Register must be set to
; allow access to PF0's control registers.  On the LM4F120, the other
; bits of the Port F GPIO Commit Register are hard-wired to 1, meaning
; that the rest of Port F can always be freely re-configured at any
; time.  Requiring this procedure makes it unlikely to accidentally
; re-configure the JTAG and NMI pins as GPIO, which can lock the
; debugger out of the processor and make it permanently unable to be
; debugged or re-programmed.
       .thumb
       .text
       .align  2
GPIO_PORTF_DATA_R  .field 0x400253FC,32		;pin 2:9 of GPIODATA
GPIO_PORTF_DIR_R   .field 0x40025400,32
GPIO_PORTF_AFSEL_R .field 0x40025420,32
GPIO_PORTF_PUR_R   .field 0x40025510,32
GPIO_PORTF_DEN_R   .field 0x4002551C,32
GPIO_PORTF_LOCK_R  .field 0x40025520,32
GPIO_PORTF_CR_R    .field 0x40025524,32
GPIO_PORTF_AMSEL_R .field 0x40025528,32
GPIO_PORTF_PCTL_R  .field 0x4002552C,32
GPIO_LOCK_KEY      .field 0x4C4F434B,32  ; Unlocks the GPIO_CR register
RED       .equ 0x02
BLUE      .equ 0x04
GREEN     .equ 0x08
SW1       .equ 0x10                 ; on the left side of the Launchpad board
SW2       .equ 0x01                 ; on the right side of the Launchpad board

;-----------------------------------------
;GPIO PORTA for switch
GPIO_PORTA_DATA_R  .field 0x400043FC,32		;pin 2:9 of GPIODATA
GPIO_PORTA_DIR_R   .field 0x40004400,32
GPIO_PORTA_AFSEL_R .field 0x40004420,32
GPIO_PORTA_PUR_R   .field 0x40004510,32
GPIO_PORTA_DEN_R   .field 0x4000451C,32
GPIO_PORTA_LOCK_R  .field 0x40004520,32
GPIO_PORTA_CR_R    .field 0x40004524,32
GPIO_PORTA_AMSEL_R .field 0x40004528,32
GPIO_PORTA_PCTL_R  .field 0x4000452C,32

;---------------------------------------------
;GPIO PORTB for LED
GPIO_PORTB_DATA_R  .field 0x400053FC,32		;pin 2:9 of GPIODATA
GPIO_PORTB_DIR_R   .field 0x40005400,32
GPIO_PORTB_AFSEL_R .field 0x40005420,32
GPIO_PORTB_PUR_R   .field 0x40005510,32
GPIO_PORTB_DEN_R   .field 0x4000551C,32
GPIO_PORTB_LOCK_R  .field 0x40005520,32
GPIO_PORTB_CR_R    .field 0x40005524,32
GPIO_PORTB_AMSEL_R .field 0x40005528,32
GPIO_PORTB_PCTL_R  .field 0x4000552C,32


SYSCTL_RCGCGPIO_R  .field 0x400FE608,32

      .global main

main:  .asmfunc
    BL  Ports_Init                  ; initialize input and output pins of Port F
loop
    MOV R3, #0x00
	MOV R4, #0x00
    LDR R0, ONESEC                ; R0 = FIFTHSEC (delay 0.2 second)
    BL  delay                       ; delay at least (3*R0) cycles
    ;MOV R2, #0x01
    ;BL  PortB_Output                ; turn LED-1 on
    ;LDR R0, FIFTHSEC                ; R0 = FIFTHSEC (delay 0.2 second)
    ;BL  delay
    ;MOV R2, #0x00
    ;BL  PortB_Output                ; read all of the switches on Port F
	;B loop

; step-1
	MOV R2, #0x08;					; green on
	BL PortF_Output

	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL  delay

swread
	MOV R2, #0x08;					; green on
	BL PortF_Output

	BL PortF_Input				; 	 sw2 status
	CMP R2, #0x01
	BEQ swread

; if switch pressed, following instruction will execute
	MOV R2, #0x00;					; green off
	BL PortF_Output
	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL  delay

	BL PortA_Input				; read switch status of PORT A
;
	LSR R2,#2
	ORRS R3,R3,R2
	LSL R3, #4

	ADD R4,#1
	CMP R4,#0x04			; read status of switch 4 times
	BNE swread
	LSR R3,#4				;
	NOP

	MOV R4,#0x00
	AND R4,R3,#0xC000		; store operand

	LSR R4,#14

	CMP R4,#0x00
	BEQ copy

	CMP R4,#0x01
	BEQ addition

	CMP R4,#0x02
	BEQ subtraction

	CMP R4,#0x03
	BEQ multiply

copy

	MOV R2,R3
	BL PortB_Output

	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL  delay
    LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL delay

    LSR R2,#0x08
	BL PortB_Output

	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL  delay
   	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL delay
	B copy

addition
	MOV R2,R3
	MOV R6,R3
	AND R6,#0x7F
	AND R2,#0x3F80
	LSR R2,#7
	ADD R2,R2,R6

	BL PortB_Output

	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL  delay
	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL delay

    LSR R2,#0x08
	BL PortB_Output

	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL  delay
 	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL delay

	B addition
subtraction

	MOV R2,R3
	MOV R6,R3
	AND R6,#0x7F
	AND R2,#0x3F80
	LSR R2,#7
	SUB R2,R2,R6

	BL PortB_Output

	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL  delay
	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL delay

    LSR R2,#0x08
	BL PortB_Output

	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL  delay
 	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL delay

	B subtraction
multiply
	MOV R2,R3
	MOV R6,R3
	AND R6,#0x7F
	AND R2,#0x3F80
	LSR R2,#7
	MUL R2,R2,R6
	BL PortB_Output

	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL  delay
	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL delay

    LSR R2,#0x08
	BL PortB_Output

	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL  delay
	LDR R0, ONESEC                ; R0 = ONESEC (delay 1 second)
    BL delay
 	B multiply

    .endasmfunc
;------------delay------------
; Delay function for testing, which delays about 3*count cycles.
; Input: R0  count
; Output: none
ONESEC             .field 5333333,32      ; approximately 1s delay at ~16 MHz clock
QUARTERSEC         .field 1333333,32      ; approximately 0.25s delay at ~16 MHz clock
FIFTHSEC           .field 1066666,32      ; approximately 0.2s delay at ~16 MHz clock
delay:  .asmfunc
    SUBS R0, R0, #1                 ; R0 = R0 - 1 (count = count - 1)
    BNE delay                       ; if count (R0) != 0, skip to 'delay'
    BX  LR                          ; return
    .endasmfunc

;------------PortF_Init------------
; Initialize GPIO Port F for negative logic switches on PF0 and
; PF4 as the Launchpad is wired.  Weak internal pull-up
; resistors are enabled, and the NMI functionality on PF0 is
; disabled.  Make the RGB LED's pins outputs.
; Input: none
; Output: none
; Modifies: R0, R1, R2
Ports_Init:  .asmfunc
    LDR R1, SYSCTL_RCGCGPIO_R       ; 1) activate clock for Port F , A, B
    LDR R0, [R1]
    ORR R0, R0, #0x23;               ; set bit 5,1,0 to turn on clock
    STR R0, [R1]
    NOP
    NOP                             ; allow time for clock to finish
    LDR R1, GPIO_PORTF_LOCK_R       ; 2) unlock the lock register
    LDR R0, GPIO_LOCK_KEY             ; unlock GPIO Port F Commit Register
    STR R0, [R1]

; CR_R
    LDR R1, GPIO_PORTF_CR_R         ; enable commit for Port F
    MOV R0, #0xFF                   ; 1 means allow access
    STR R0, [R1]
    LDR R1, GPIO_PORTA_CR_R         ; enable commit for Port F
    MOV R0, #0xFF                   ; 1 means allow access
    STR R0, [R1]
    LDR R1, GPIO_PORTB_CR_R         ; enable commit for Port F
    MOV R0, #0xFF                   ; 1 means allow access
    STR R0, [R1]

; AMSEL
    LDR R1, GPIO_PORTF_AMSEL_R      ; 3) disable analog functionality
    MOV R0, #0                      ; 0 means analog is off
    STR R0, [R1]
    LDR R1, GPIO_PORTA_AMSEL_R      ; 3) disable analog functionality
    MOV R0, #0                      ; 0 means analog is off
    STR R0, [R1]
    LDR R1, GPIO_PORTB_AMSEL_R      ; 3) disable analog functionality
    MOV R0, #0                      ; 0 means analog is off
    STR R0, [R1]

;PCTL
    LDR R1, GPIO_PORTF_PCTL_R       ; 4) configure as GPIO
    MOV R0, #0x00000000             ; 0 means configure Port F as GPIO
    STR R0, [R1]
    LDR R1, GPIO_PORTA_PCTL_R       ; 4) configure as GPIO
    MOV R0, #0x00000000             ; 0 means configure Port F as GPIO
    STR R0, [R1]
    LDR R1, GPIO_PORTB_PCTL_R       ; 4) configure as GPIO
    MOV R0, #0x00000000             ; 0 means configure Port F as GPIO
    STR R0, [R1]

; DIR
    LDR R1, GPIO_PORTF_DIR_R        ; 5) set direction register of PORT F
    MOV R0,#0x0E                    ; PF0 and PF4 input, PF3-1 output
    STR R0, [R1]
    LDR R1, GPIO_PORTA_DIR_R        ; 5) set direction register of PORT A
    MOV R0,#0x00;                   ; PA2 to PA5 for 4 switches
    STR R0, [R1]
    LDR R1, GPIO_PORTB_DIR_R        ; 5) set direction register of PORT B
    MOV R0,#0xFF                    ; PB0 to PB7 for 8 LEDs
    STR R0, [R1]

;AFSEL
    LDR R1, GPIO_PORTF_AFSEL_R      ; 6) regular port function
    MOV R0, #0                      ; 0 means disable alternate function
    STR R0, [R1]
    LDR R1, GPIO_PORTA_AFSEL_R      ; 6) regular port function
    MOV R0, #0                      ; 0 means disable alternate function
    STR R0, [R1]
    LDR R1, GPIO_PORTB_AFSEL_R      ; 6) regular port function
    MOV R0, #0                      ; 0 means disable alternate function
    STR R0, [R1]

;PUR
    LDR R1, GPIO_PORTF_PUR_R        ; pull-up resistors for PF4,PF0
    MOV R0, #0x11                   ; enable weak pull-up on PF0 and PF4
    STR R0, [R1]
    LDR R1, GPIO_PORTA_PUR_R        ; pull-up resistors for PA2 to PA5
    MOV R0, #0x3C                   ; enable weak pull-up on PA2 to PA5
    STR R0, [R1]

;DEN
    LDR R1, GPIO_PORTF_DEN_R        ; 7) enable Port F digital port
    MOV R0, #0xFF                   ; 1 means enable digital I/O
    STR R0, [R1]
    LDR R1, GPIO_PORTB_DEN_R        ; 7) enable Port B digital port
    MOV R0, #0xFF                   ; 1 means enable digital I/O
    STR R0, [R1]
    LDR R1, GPIO_PORTA_DEN_R        ; 7) enable Port A digital port
    MOV R0, #0xFF                   ; 1 means enable digital I/O
    STR R0, [R1]

    BX  LR
    .endasmfunc

;------------PortF_Input------------
; Read and return the status of the switches.
; Input: none
; Output: R0  0x01 if only Switch 1 is pressed
;         R0  0x10 if only Switch 2 is pressed
;         R0  0x00 if both switches are pressed
;         R0  0x11 if no switches are pressed
; Modifies: R1
PortF_Input:  .asmfunc
    LDR R1, GPIO_PORTF_DATA_R  ; pointer to Port F data
    LDR R2, [R1]               ; read all of Port F
    AND R2,R2,#0x01            ; just the input pins PF0 and PF4
    BX  LR                     ; return R0 with inputs
    .endasmfunc

;------------PortF_Output------------
; Set the output state of PF3-1.
; Input: R0  new state of PF
; Output: none
; Modifies: R1
PortF_Output:  .asmfunc
    LDR R1, GPIO_PORTF_DATA_R  ; pointer to Port F data
    STR R2, [R1]               ; write to PF3-1
    BX  LR
    .endasmfunc

;------------PortA_Input------------
; Read and return the status of the switches.
; Input: none
; Output: R0  0x01 if only Switch 1 is pressed
;         R0  0x10 if only Switch 2 is pressed
;         R0  0x00 if both switches are pressed
;         R0  0x11 if no switches are pressed
; Modifies: R1

PortA_Input:  .asmfunc
    LDR R1, GPIO_PORTA_DATA_R  ; pointer to Port F data
    LDR R2, [R1]               ; read all of Port F
    AND R2,R2,#0x3C            ; just the input pins PA2 to PA5
    BX  LR                     ; return R0 with inputs
    .endasmfunc

;------------PortB_Output------------
; Set the output state of PB0-8.
; Input: R0  new state of PF
; Output: none
; Modifies: R1
PortB_Output:  .asmfunc
    LDR R1, GPIO_PORTB_DATA_R  ; pointer to Port F data
    STR R2, [R1]               ; write to PF3-1
    BX  LR
    .endasmfunc

    .end                             ; end of file

