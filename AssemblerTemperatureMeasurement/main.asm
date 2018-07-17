;
; AssemblerTemperatureMeasurement.asm
;
; Created: 2018-07-05 23:56:52
; Author : MO

.include "m644Pdef.inc"

 ;-------------------------------------------
;    Declaring constants for LCD driver
.EQU D_LCD	 = DDRA  ; register of direction LCD
.EQU O_LCD   = PORTA ; output port register of LCD
.EQU LCD_RS  = 0     ; number of RS  signal line
.EQU LCD_EN  = 1     ; number of E   signal line
.EQU LCD_DB4 = 4     ; number of DB4 signal line
.EQU LCD_DB5 = 5     ; number of DB5 signal line
.EQU LCD_DB6 = 6     ; number of DB6 signal line
.EQU LCD_DB7 = 7     ; number of DB7 signal line


.DEF DATA_TO_SEND=R16
.DEF DELAY_TIME=R17
.DEF DELAY_MUL=R18
.DEF RED_LED=R23	; defining variables
.DEF GREEN_LED=R20
.DEF BLUE_LED=R21
.DEF counter=R22

.CSEG 
.org 0 jmp start
.org OC0Aaddr jmp Tim0_compA  ; Overflow0 Interrupt Vector Address
.org 0x30			 ; jump to first section after interrupt vectors


Text:	.db "Temperature ", END_OF_STRING ; declaring const string in program memory

start:
cli					; disable interrupts 

ldi counter, 200
ldi RED_LED, 2	
ldi DELAY_MUL, 2
ldi DELAY_TIME, 5	

ldi R17, high(RAMEND) ; initialization of stack pointer
ldi R16, low(RAMEND)
out SPH, R17
out SPL, R16

rcall Ini_LCD		; initialization of LCD

ldi ZH, high(Text<<1)	; load address of first character to pointer
ldi ZL, low(Text<<1)
rcall display_string_LCD

clr R16
lds R16, TCCR2A		; load address of TCCR2A to R16
ORI R16, (1<<COM2A1)|(1<<COM2A0)|(1<<WGM20)|(1<<WGM21) ; load data to R16 (logical OR operation), set OC2A on Compare Match, clear OC2A at BOTTOM (inverting mode) fastPWM
sts TCCR2A, R16		; store direct to data space I/O from R16

lds R16, TCCR2B		; load address of TCCR2B to R16
ORI R16, (1<<CS20)	; load to R16 data (logical OR operation), set (No prescaling)
sts TCCR2B, R16		; store direct to data space I/O from R16

lds R16, OCR2A		; load address of OCR2A to R16
mov R16, RED_LED	; load value to R16
sts OCR2A, R16		; store direct to data space from R16

ldi R16, (1<<COM0A1)|(1<<COM0A0)|(1<<WGM01) ; set compare output mode, clear OC0A at BOTTOM (inverting mode) CTC
out TCCR0A, R16
ldi R16, (1<<CS01)	; set /8 (From prescaler)
out TCCR0B, R16
ldi R16, 200
out OCR0A, R16
lds R16, TIMSK0		; load address of TIMSK0 to R16
ORI R16, (1<<OCIE0A); load to R16 data (logical OR operation), set interrupt on compare enable
sts TIMSK0, R16		; store direct to data space from R16

;sbi DDRC, DDC2		; set Data Direct Register as a input
;sbi DDRC, DDC1		; set Data Direct Register as a input
;sbi DDRD, DDD7
sbi DDRD, (0<<DDD6) ; set Data Direct Register as a output (connected button)

ldi R16, (1<<PINC1)|(1<<PINC2) ; set Port as a output
out PORTC, R16 
ldi R16, (0<<PIND7)|(1<<PIND6) ; pull-up internal resistor in PIND6
out PORTD, R16 
sei					; enable interrupts 

;*************************************************
main:				; main loop
sbic PIND, PIND6	; wait for pushed button (if PIND6 is clear, skip next instruction)
rjmp main
nop
inc RED_LED			; increment value
cpi	RED_LED, 252	; if R18 equals or higher 252 do next command
brsh toone
lds R24, OCR2A		; load address of OCR2A to R16
mov R24, RED_LED	; load value to R16
sts OCR2A, R24		; store direct to data space from R16
nop 
ldi DELAY_TIME, 20
ldi DELAY_MUL, 1
call delay_ms
rjmp main
toone: 
ldi RED_LED, 1
ldi DELAY_TIME, 15
ldi DELAY_MUL, 1
call delay_ms
rjmp main
;*************************************************		

;---------------------------------------------------------
;     Interrupt functions

Tim0_compA:
push R18
dec counter			; decrement value
cpi counter, 1		
brlo zero			; if value counter lower than 1 go to zero
cp counter, RED_LED
brsh off			; if value of register R18 higher or same go to off
ldi R18, (1<<PINC1)|(1<<PINC2) ;set Port as a input
out PORTC, R18
rjmp back
off:
ldi R18, (0<<PINC1)|(0<<PINC2) ; set Port as a output
out PORTC, R18 
rjmp back
zero:
ldi counter, 200
back:
pop R18
reti				; return from interrupt 
;---------------------------------------------------------
;     Include external files
.INCLUDE "Delay.inc"	; Delay functions
.INCLUDE "LCD.inc"		; LCD functions
