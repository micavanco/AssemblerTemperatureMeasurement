/*
 * LCD.inc
 *
 *  Created: 2018-07-12 17:31:48
 *   Author: MO
 */ 
 ;---------------------------------------------------------
 ;	Declaring labels for driver instructions
 .equ LCD_CLEAR		= 0 ; clear screen and cursor return
 .equ LCD_BACK		= 1 ; return cursor to first position

 .equ LCD_MODE		= 2 ; change memory save mode
 .equ LCD_T_INC		= 1 ; increment address after save
 .equ LCD_T_DEC		= 2 ; decrement address after save
 .equ LCD_T_MOVE	= 0 ; move screen after every saving

 .equ LCD_CTRL		= 3 ; control screen
 .equ LCD_SW_ON		= 2 ; switch on screen
 .equ LCD_SW_OFF	= 3 ; switch off screen
 .equ LCD_SK_ON		= 1 ; switch on cursor
 .equ LCD_SK_OFF	= 3 ; switch off cursor
 .equ LCD_SM_ON		= 0 ; set on cursor blinking
 .equ LCD_SM_OFF	= 3 ; set off cursor blinking

 .equ LCD_MOVE		= 4 ; moving
 .equ LCD_C_MOVE	= 4 ; move cursor
 .equ LCD_S_MOVE	= 3 ; move screen
 .equ LCD_LEFT		= 4 ; move left
 .equ LCD_RIGHT		= 2 ; move right

 .equ LCD_SET		= 5 ; set interface and screen
 .equ LCD_SET_8BIT	= 4 ; 8-Bit interface
 .equ LCD_SET_4BIT	= 5 ; 4-Bit interface
 .equ LCD_SET_2LINE	= 3 ; 2-line screen
 .equ LCD_SET_1LINE = 5 ; 1-line screen
 .equ LCD_SET_5x10  = 2 ; 5x10 marks size
 .equ LCD_SET_5x8   = 5 ; 5x8  marks size

 .equ LCD_CGADR		= 6 ; change CGRAM address
 .equ LCD_DDADR		= 7 ; change DDRAM address

;---------------------------------------------------------
;     Function to initialize LCD screen

Ini_LCD:
	push DATA_TO_SEND		; put value on stack
	push DELAY_MUL			; put value on stack
	push DELAY_TIME

	sbi D_LCD, LCD_RS		; set bit RS and so on
	sbi D_LCD, LCD_EN
	sbi D_LCD, LCD_DB4
	sbi D_LCD, LCD_DB5
	sbi D_LCD, LCD_DB6
	sbi D_LCD, LCD_DB7

	cbi O_LCD, LCD_RS		; clear bit RS and EN (send instructions)
	cbi O_LCD, LCD_EN

	ldi DELAY_TIME, 45
	ldi DELAY_MUL, 1
	rcall delay_ms			; wait 45 ms

		
	sbi O_LCD, LCD_EN    ; set bit E and so on
	sbi O_LCD, LCD_DB4
	sbi O_LCD, LCD_DB5
	cbi O_LCD, LCD_DB6
	cbi O_LCD, LCD_DB7
	nop
	nop
	cbi O_LCD, LCD_EN

	ldi DELAY_TIME, 5
	ldi DELAY_MUL, 1
	rcall delay_ms			; wait 5 ms

	ldi DELAY_MUL, 2
	Ini_LCD_1:
	sbi O_LCD, LCD_EN
	ldi DELAY_TIME, 1
	rcall delay_us			; wait 1 us
	cbi O_LCD, LCD_EN
	ldi DELAY_TIME, 20
	rcall delay_us			; wait 20 us
	dec DELAY_MUL
	brne Ini_LCD_1			; if not equal 0 move to Ini_LCD_1

	sbi O_LCD, LCD_EN
	cbi O_LCD, LCD_DB4
	ldi DELAY_TIME, 1
	rcall delay_us			; wait 1 us
	cbi O_LCD, LCD_EN
	ldi DELAY_TIME, 10
	rcall delay_us			; wait 10 us

	ldi DATA_TO_SEND, (1<<LCD_SET)|(1<<LCD_SET_4BIT)|(1<<LCD_SET_2LINE)|(1<<LCD_SET_5x8)
	rcall instruction_LCD

	ldi DATA_TO_SEND, (1<<LCD_MODE)|(1<<LCD_T_INC)
	rcall instruction_LCD

	ldi DATA_TO_SEND, (1<<LCD_CTRL)|(1<<LCD_SW_ON)|(1<<LCD_SK_OFF)|(1<<LCD_SM_OFF)
	rcall instruction_LCD

	sbi O_LCD, LCD_RS

	rcall clear_LCD

	pop DELAY_TIME
	pop DELAY_MUL			; take value from stack
	pop DATA_TO_SEND		; take value from stack
	ret

;---------------------------------------------------------
;     Function to send instruction to LCD
instruction_LCD:
	cbi O_LCD, LCD_RS	;clear RS bit/sending values are instructions
	rcall send_to_LCD
	sbi O_LCD, LCD_RS	;set RS bit/sending values are data
	ret

;---------------------------------------------------------
;     Function to send byte to LCD
send_to_LCD:
	sbi O_LCD, LCD_EN	;set bit EN
	cbi	O_LCD, LCD_DB4	;clear bit DB4
	cbi	O_LCD, LCD_DB5
	cbi	O_LCD, LCD_DB6
	cbi	O_LCD, LCD_DB7
	sbrc DATA_TO_SEND, 7
	sbi	O_LCD, LCD_DB7	;do this instruction if bit 7 is set
	sbrc DATA_TO_SEND, 6
	sbi	O_LCD, LCD_DB6	;do this instruction if bit 6 is set
	sbrc DATA_TO_SEND, 5
	sbi	O_LCD, LCD_DB5	;do this instruction if bit 5 is set
	sbrc DATA_TO_SEND, 4
	sbi	O_LCD, LCD_DB4	;do this instruction if bit 4 is set

	nop

	cbi O_LCD, LCD_EN	;set EN to low - send first half byte

	nop
	nop

	sbi O_LCD, LCD_EN	;set bit EN
	cbi	O_LCD, LCD_DB4	;clear bit DB4
	cbi	O_LCD, LCD_DB5
	cbi	O_LCD, LCD_DB6
	cbi	O_LCD, LCD_DB7
	sbrc DATA_TO_SEND, 3
	sbi	O_LCD, LCD_DB7	;do this instruction if bit 7 is set
	sbrc DATA_TO_SEND, 2
	sbi	O_LCD, LCD_DB6	;do this instruction if bit 6 is set
	sbrc DATA_TO_SEND, 1
	sbi	O_LCD, LCD_DB5	;do this instruction if bit 5 is set
	sbrc DATA_TO_SEND, 0
	sbi	O_LCD, LCD_DB4	;do this instruction if bit 4 is set

	nop

	cbi O_LCD, LCD_EN	;set EN to low - send second half byte

	ldi DELAY_TIME, 5	;set delay time to 5us
	rcall delay_us
ret


;---------------------------------------------------------
;     Function to clear LCD screen
clear_LCD:
	push DATA_TO_SEND				; put value on stack
	ldi DATA_TO_SEND, (1<<LCD_CLEAR)
	rcall instruction_LCD			; clear screen and return cursor to first position

	ldi DATA_TO_SEND, 180
	rcall delay_us					; delay 1,8 ms
	pop DATA_TO_SEND				; take value from stack
	ret

;---------------------------------------------------------
;     Function to return cursor on LCD screen
return_cursor:
	push DATA_TO_SEND				; put value on stack
	push DELAY_TIME
	ldi  DATA_TO_SEND, (1<<LCD_BACK); return cursor to first position
	rcall instruction_LCD
	ldi DELAY_TIME, 180
	rcall delay_us					; delay 1,8 ms
	pop DELAY_TIME
	pop DATA_TO_SEND				; take value from stack
	ret

;---------------------------------------------------------
;     Function to display char string
.equ END_OF_STRING = 255	; label of end of string

display_string_LCD:
	
	push DATA_TO_SEND		; put value on stack
	push R19				; put value on stack

	loop_dsp_1:
	lpm DATA_TO_SEND, Z+	; load data from address pointer Z and increment address

	cpi DATA_TO_SEND, END_OF_STRING
	breq end_dsp_loop			; if this mark is END_OF_STRING mark, end loop
	rcall send_to_LCD
	rjmp loop_dsp_1

	end_dsp_loop:
	pop R19					; take value from stack
	pop DATA_TO_SEND		; take value from stack
	ret

;---------------------------------------------------------
;     Function to display decimal number

show_dec_LCD:
	push R19				; put value on stack

	mov R19, DATA_TO_SEND
	clt						; clear bit register T 

	clr DATA_TO_SEND		; clear register
	loop_show_1:
	subi R19, 100			; subtract 100
	brlo loop_show_2		; branch if value is than zero
	inc DATA_TO_SEND
	rjmp loop_show_1

	loop_show_2:
	subi R19, -100			; add 100
	tst DATA_TO_SEND
	breq loop_show_3
	set						; if DATA_TO_SEND is higher than 0, set bit register T 
	ori DATA_TO_SEND, 0b00110000 ; convert to ASCII
	rcall send_to_LCD		; and display on LCD screen
	
	clr DATA_TO_SEND
	loop_show_3:
	subi R19, 10
	brlo loop_show_4
	inc DATA_TO_SEND
	rjmp loop_show_3

	loop_show_4:
	subi R19, -10
	tst DATA_TO_SEND
	brne loop_show_5		; if number of tens is equal to zero go to loop_show_5
	brne loop_show_6		; if bit register T was clear

	loop_show_5:
	set
	ori DATA_TO_SEND, 0b00110000 ; convert to ASCII
	rcall send_to_LCD		; and display on LCD screen

	loop_show_6:
	mov DATA_TO_SEND, R19
	ori DATA_TO_SEND, 0b00110000 ; convert to ASCII
	rcall send_to_LCD		; and display on LCD screen

	pop R19					; take value from stack
	ret	 

;---------------------------------------------------------
;     Function to add spaces to display 

add_space_LCD:
	push R19

	mov R19, DATA_TO_SEND	; copy number of spaces

	add_space_LCD_1:
	ldi R19, ' '			; load ASCII "space" symbol
	rcall send_to_LCD		; and display on LCD screen
	dec R19
	brne add_space_LCD_1	; if not equal to zero go to add_space_LCD_1

	pop R19
	ret

;---------------------------------------------------------
;     Function to copy constant values to CGRAM memory

;save_to_CGRAM:
	;push DATA_TO_SEND		; put value on stack
	;push R19				; put value on stack
														;*********in progress*************
	;ldi DATA_TO_SEND, 0|(1<<LCD_CGADR)
	;rcall instruction_LCD	; set CGADR address mode and set 0 address in that memory

	;ldi 

