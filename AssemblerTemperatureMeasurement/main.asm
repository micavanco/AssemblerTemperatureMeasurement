;
; AssemblerTemperatureMeasurement.asm
;
; Created: 2018-07-05 23:56:52
; Author : MO
;
.nolist 
.include "m644Pdef.inc"
.list 

.cseg 
.org 0 

start:
cli					;disable interrupts 

lds R16, TCCR2A		;load address of TCCR2A to R16
ORI R16, (1<<COM2A1)|(1<<COM2A0)|(1<<WGM20)|(1<<WGM21) ; load data to R16 (logical OR operation), set OC2A on Compare Match, clear OC2A at BOTTOM (inverting mode) fastPWM
sts TCCR2A, R16		;store direct to data space I/O from R16

lds R16, TCCR2B		;load address of TCCR2B to R16
ORI R16, (1<<CS20)	;load to R16 data (logical OR operation), set (No prescaling)
sts TCCR2B, R16		;store direct to data space I/O from R16

lds R16, OCR2A		;load address of OCR2A to R16
ori R16, 100		;load value to R16
sts OCR2A, R16		;store direct to data space from R16

sbi DDRC, DDC1		;set Data Direct Register as a input
sbi DDRD, DDD7

ldi R16, (0<<PINC1)|(0<<PINC2) ;set Port as a output
out PORTC, R16 
ldi R16, (0<<PIND7)
out PORTD, R16 
sei					;enable interrupts

main:				;main loop
nop 
rjmp main 