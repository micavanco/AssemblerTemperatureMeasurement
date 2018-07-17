/*
 * Delay.inc
 *
 *  Created: 2018-07-12 17:32:43
 *   Author: MO
 */ 
 ;---------------------------------------------------------
;     Delay functions
;	Delay in ms
delay_ms:
	push R19				;push values to stack
	push R20
	push R21
	push R22

	mov R19, DELAY_MUL		;copy value to register
	mov R20, DELAY_TIME
	ldi R21, 16

Wait_ms_0:				;_______________________
	mov DELAY_MUL, R19	;			loop 0		\
Wait_ms_1:				;__________________		|
	mov DELAY_TIME, R20 ;			loop 1	\	|
Wait_ms_2:				;_______________	|	|
	ldi R22, 249		;		loop 2	\	|	|
	nop					;				|	|	|
Wait_ms_3:				;______			|	|	|
	nop					;loop 3	\		|	|	|
	dec R22				;		/		|	|	|
	brne Wait_ms_3		;______/		|	|	|
    dec DELAY_TIME		;				/	|	|	
	brne Wait_ms_2		;______________/	|	|
	dec DELAY_MUL		;				   /	|
	brne Wait_ms_1		;_________________/		|
	dec R21				;						/
	brne Wait_ms_0		;______________________/

	pop R22
	pop R21					;get values from stack 
	pop R20
	pop R19
ret
; Delay in us
delay_us:
	push R20				;push values to stack
	push R21
		
	mov R20, DELAY_TIME		;copy value to register
	ldi R21, 16

Wait_us_0:				;_______________________
	mov DELAY_TIME, R20 ;			loop 0		\
Wait_us_1:				;__________________		|
	nop					;				   |	|
	nop					;				   |	|
	nop					;				   |	|
	nop					;				   |	|
	nop					;				   |	|
	nop					;				   |	|
	nop					;			loop 1 |	|	
	dec DELAY_TIME		;				   /	|
	brne Wait_us_1		;_________________/		|
	dec R21				;						/
	brne Wait_us_0		;______________________/
			
	pop R21			;get values from stack 
	pop R20
ret