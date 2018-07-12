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
; Delay in us
delay_us:
	push R20				;push values to stack
	push R21
		
	mov R20, DELAY_TIME		;copy value to register
	ldi R21, 16

Wait_us_0:				;_______________________
	mov R20, DELAY_TIME ;			loop 0		\
Wait_us_1:				;__________________		|
	nop					;				   |	|
	nop					;				   |	|
	nop					;				   |	|
	nop					;				   |	|
	nop					;				   |	|
	nop					;				   |	|
	nop					;			loop 1 |	|	
	dec R20				;				   /	|
	brne Wait_us_1		;_________________/		|
	dec R21				;						/
	brne Wait_us_0		;______________________/
			
	pop R21			;get values from stack 
	pop R20
ret