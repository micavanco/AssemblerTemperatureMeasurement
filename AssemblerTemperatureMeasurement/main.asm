;
; AssemblerTemperatureMeasurement.asm
;
; Created: 2018-07-05 23:56:52
; Author : MO
;
.nolist 
.include "m644Pdef.inc"
.list 
;-------------------------------------------
;    Declaring constants for LCD driver
.equ D_LCD	 = DDRA  ; register of direction LCD
.equ O_LCD   = PORTA ; output port register of LCD
.equ LCD_RS  = 0     ; number of RS  signal line
.equ LCD_EN  = 1     ; number of E   signal line
.equ LCD_DB4 = 4     ; number of DB4 signal line
.equ LCD_DB5 = 5     ; number of DB5 signal line
.equ LCD_DB6 = 6     ; number of DB6 signal line
.equ LCD_DB7 = 7     ; number of DB7 signal line

.cseg 
.org $0000 jmp start
.org OC0Aaddr jmp Tim0_compA  ;Overflow0 Interrupt Vector Address

start:
.DEF RED_LED=R18	;defining variables
.DEF GREEN_LED=R20
.DEF BLUE_LED=R22
.DEF counter=R17
.DEF DELAY_TIME=R25
.DEF DELAY_MUL=R24

cli					;disable interrupts 

ldi R16, high(RAMEND) ;initialization of stack pointer
ldi R17, low(RAMEND)
out SPH, R16
out SPL, R17

ldi counter, 200
ldi RED_LED, 2	
ldi DELAY_MUL, 2
ldi DELAY_TIME, 5	

lds R16, TCCR2A		;load address of TCCR2A to R16
ORI R16, (1<<COM2A1)|(1<<COM2A0)|(1<<WGM20)|(1<<WGM21) ; load data to R16 (logical OR operation), set OC2A on Compare Match, clear OC2A at BOTTOM (inverting mode) fastPWM
sts TCCR2A, R16		;store direct to data space I/O from R16

lds R16, TCCR2B		;load address of TCCR2B to R16
ORI R16, (1<<CS20)	;load to R16 data (logical OR operation), set (No prescaling)
sts TCCR2B, R16		;store direct to data space I/O from R16

lds R16, OCR2A		;load address of OCR2A to R16
mov R16, RED_LED	;load value to R16
sts OCR2A, R16		;store direct to data space from R16

ldi R16, (1<<COM0A1)|(1<<COM0A0)|(1<<WGM01) ;set compare output mode, clear OC0A at BOTTOM (inverting mode) CTC
out TCCR0A, R16
ldi R16, (1<<CS01)	;set /8 (From prescaler)
out TCCR0B, R16
ldi R16, 200
out OCR0A, R16
lds R16, TIMSK0		;load address of TIMSK0 to R16
ORI R16, (1<<OCIE0A);load to R16 data (logical OR operation), set interrupt on compare enable
sts TIMSK0, R16		;store direct to data space from R16

sbi DDRC, DDC2		;set Data Direct Register as a input
sbi DDRC, DDC1		;set Data Direct Register as a input
sbi DDRD, DDD7
sbi DDRD, (0<<DDD6) ;set Data Direct Register as a output (connected button)

ldi R16, (1<<PINC1)|(1<<PINC2) ;set Port as a output
out PORTC, R16 
ldi R16, (0<<PIND7)|(1<<PIND6) ;pull-up internal resistor in PIND6
out PORTD, R16 
sei					;enable interrupts

;*************************************************
main:				;main loop
sbic PIND, PIND6	;wait for pushed button (if PIND6 is clear, skip next instruction)
rjmp main
nop
inc RED_LED			;increment value
cpi	RED_LED, 252	;if R18 equals or higher 252 do next command
brsh toone
lds R19, OCR2A		;load address of OCR2A to R16
mov R19, RED_LED	;load value to R16
sts OCR2A, R19		;store direct to data space from R16
nop 
call delay_ms
rjmp main
toone: 
ldi RED_LED, 1
call delay_ms
rjmp main
;*************************************************		

;---------------------------------------------------------
;     Delay functions
delay_ms:
	push R19				;push values to stack
	push R20
	push R21
	push R16

	mov R19, DELAY_MUL		;copy value to register
	mov R20, DELAY_TIME
	ldi R21, 16

Wait_ms_0:				;_______________________
	mov R20, DELAY_TIME ;			loop 0		\
Wait_ms_1:				;__________________		|
	mov R19, DELAY_MUL  ;			loop 1	\	|
Wait_ms_2:				;_______________	|	|
	ldi R16, 249		;		loop 2	\	|	|
	nop					;				|	|	|
Wait_ms_3:				;______			|	|	|
	nop					;loop 3	\		|	|	|
	dec R16				;		/		|	|	|
	brne Wait_ms_3		;______/		|	|	|
    dec R19				;				/	|	|	
	brne Wait_ms_2		;______________/	|	|
	dec R20				;				   /	|
	brne Wait_ms_1		;_________________/		|
	dec R21				;						/
	brne Wait_ms_0		;______________________/

	pop R16
	pop R21					;get values from stack 
	pop R20
	pop R19
ret

;---------------------------------------------------------
;     Function to send byte to LCD


;---------------------------------------------------------
;     Interrupt functions

Tim0_compA:
push R16
dec counter			;decrement value
cpi counter, 1		
brlo zero			;if value counter lower than 1 go to zero
cp counter, R18
brsh off			;if value of register R18 higher or same go to off
ldi R16, (1<<PINC1)|(1<<PINC2) ;set Port as a input
out PORTC, R16
rjmp back
off:
ldi R16, (0<<PINC1)|(0<<PINC2) ;set Port as a output
out PORTC, R16 
rjmp back
zero:
ldi counter, 200
back:
pop R16
reti				;return from interrupt 

