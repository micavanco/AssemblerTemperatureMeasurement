/*
 * DS18B20.inc
 *
 *  Created: 2018-07-17 19:16:33
 *   Author: MO
 */ 
 ;---------------------------------------------------------
;     Constant definitions
	.EQU	SYS_FREQ	 = 16000000

	.EQU	READROM		 = 0x33		; read rom command
	.EQU	ROMCODE		 = 1
;---------------------------------------------------------
;     Initializing of sensor
;		D_TEMP			register of direction DS18B20
;		O_TEMP  = PORTD output port register of DS18B20
;		TEMP_DQ = 6     number of DQ data line
;		TEMP_PIN		PIN of used port
;
 DS18B20_Reset:
	push DELAY_TIME

	sbi	D_TEMP, TEMP_DQ
	cbi O_TEMP, TEMP_DQ

	ldi DELAY_TIME, 240
	rcall delay_us
	rcall delay_us			; wait 480 us

	cbi	D_TEMP, TEMP_DQ

	ldi DELAY_TIME, 70
	rcall delay_us			; wait 70 us
	set
	sbis TEMP_PIN, TEMP_DQ
	clt
	ldi DELAY_TIME, 240
	rcall delay_us			; wait 240 us

	pop DELAY_TIME
ret

;---------------------------------------------------------
;     Send bit to sensor
;		D_TEMP			register of direction DS18B20
;		O_TEMP			output port register of DS18B20
;		TEMP_DQ			number of DQ data line
;					
DS18B20_Send_Bit:
	push DELAY_TIME

	brcc Send_Bit_Zero
	ldi DELAY_TIME, 1
	rjmp Send_Bit_One
	Send_Bit_Zero:
	ldi DELAY_TIME, 120
	Send_Bit_One:
	sbi	D_TEMP, TEMP_DQ
	rcall delay_us
	cbi	D_TEMP, TEMP_DQ

	ldi DELAY_TIME, 60
	rcall delay_us			; wait 60 us

	pop DELAY_TIME
ret

;---------------------------------------------------------
;     Send byte to sensor
;		D_TEMP			register of direction DS18B20
;		O_TEMP			output port register of DS18B20
;		TEMP_DQ			number of DQ data line
;		DATA_TO_SEND	byte to write
;		R24				counter			
DS18B20_Send_Byte:
	push R24

	ldi R24, 0
	DS18B20_Send_Byte_loop:			;______
	ror DATA_TO_SEND				;	   \
	rcall DS18B20_Send_Bit			;		|
	inc R24							; loop	|
	cpi R24, 8						;		/
	brne DS18B20_Send_Byte_loop		;______/

	pop R24
ret


;---------------------------------------------------------
;     Read bit from sensor
;		D_TEMP			register of direction DS18B20
;		O_TEMP			output port register of DS18B20
;		TEMP_DQ			number of DQ data line
;					
DS18B20_Read_Bit:
	push DELAY_TIME
	
	ldi DELAY_TIME, 1
	sbi	D_TEMP, TEMP_DQ
	rcall delay_us			; wait 1 us
	cbi	D_TEMP, TEMP_DQ
	ldi DELAY_TIME, 5
	rcall delay_us			; wait 5 us
	clt
	sbic TEMP_PIN, TEMP_DQ
	set
	ldi DELAY_TIME, 50
	rcall delay_us			; wait 50 us
	sec
	brts DS18B20_Read_Bit_End
	clc
	DS18B20_Read_Bit_End:
	pop DELAY_TIME
ret

;---------------------------------------------------------
;     Read byte from sensor
;		D_TEMP			register of direction DS18B20
;		O_TEMP			output port register of DS18B20
;		TEMP_DQ			number of DQ data line
;		DATA_TO_SEND	byte to read
;		R24				counter			
DS18B20_Read_Byte:
	push R24

	ldi R24, 0
	DS18B20_Read_Byte_loop:			;______
	rcall DS18B20_Read_Bit			;	   \
	ror DATA_TO_SEND				;	    |		
	inc R24							; loop	|
	cpi R24, 8						;		/
	brne DS18B20_Read_Byte_loop		;______/

	pop R24
ret
;---------------------------------------------------------
;     Update CRC value
;			
CRC8Update:
	  push	R20
	  push	R21
	  push	R22
	  push	R20

	  ldi	R21, 8
	CRC8L:
	  lds	R22, _crc
	  eor	R20, r22
	  ror	R20
	  lds	R20, _crc
	  brcc	CRC8zero
	  ldi	R22, 0x18
	  eor	R20, R22
	CRC8zero:
	  ror	R20
	  sts	_crc, R20
	  pop	R20
	  lsr	R20
	  push	R20
	  dec	r21
	  brne	CRC8L
	  pop	r20
	  pop	r22
	  pop	r21
	  pop	r20
 ret

;---------------------------------------------------------
;     Clear CRC value
;	
CRC8Init:
		push R16
		ldi	R16, 0
		sts	_crc, R16
		pop	R16
ret

;---------------------------------------------------------
;     Get CRC value
;		DATA_TO_SEND	byte to read
;
GetCRC8:
  lds	DATA_TO_SEND, _crc 
 ret