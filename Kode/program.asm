.include "m32def.inc"


.equ Set=0x55
.equ Get=0xAA
.equ Reply=0xBB

.equ Start=0x10
.equ Stop=0x11
.equ Register=0x12

.org 0
	jmp	Reset

.org 0x14
	jmp	Speed_Measure

.ORG URXCaddr
	jmp	USART_Receive

.org 0x2A
Reset:
	ldi	R19, 0x00	;Clear speed mesure bits
	ldi	R20, 0x00	;Clear speed mesure bits
	ldi	R21, 0x00	;Clear speed mesure bits
	ldi	R22, 0x00	;Clear speed mesure bits


	ldi	R23, 0x00	;Målt hastighed.

	ldi	R24, 0x00	;LapCounter register.
	ldi	R25, 0x00	;Speed register .

	ldi	R28, 0x00	;Status register, 0-bit sættes høj når seriel data skal sendes.
				;1-bit sættes høj når banelængde skal sendes.
				;2-bit sættes høj når counter1 skal nulstilles.

	ldi	R29, 0x00	;Første byte seriel.
	ldi	R30, 0x00	;Anden byte seriel.
	ldi	R31, 0x00	;Tredje byte seriel.

;********************
;********************
;*  Initialisering  *
;********************
;********************
PORTA_Init:			;
	sbi	DDRA, 0		;Sættes til output. Gul LED
	cbi	DDRA, 1		;Sættes til input. CNY70
	cbi	DDRA, 2		;Sættes til input. Accelerometer, læses via ADC.

PORTB_Init:
	sbi	DDRB, 5		;MOSI
	cbi	DDRB, 6		;MISO
	sbi	DDRB, 7		;CLK

PORTC_Init:
	sbi	DDRC, 2		;Sættes til output. Enable H-Bro.
	sbi	DDRC, 3		;Sættes til output. Dir, H-Bro.

PORTD_Init:
	sbi	DDRD, 2
	cbi	PORTD, 2

ADC_Init:
	ldi	R16, 0b11100111
	out	ADCSR, R16

PWM_Init:
	sbi	DDRD, 7
	ldi	R16, 0x00
	out	OCR2, R16
	ldi	R16, 0b01101010
	out	TCCR2, R16

USART_Init:
	ldi	R16, 0b00000000	;UBRRH bruges til at sætte baudrate.
	out 	UBRRH, R16
	ldi	R16, 103	;UBRRL bruges til at sætte baudrate.
	out 	UBRRL, R16


	ldi 	R16, (1<<UDRE)	;
	out 	UCSRA, R16

	ldi 	R16, (1<<RXCIE) | (1<<RXEN) | (1<<TXEN)
	out 	UCSRB, R16

	ldi 	R16, (1<<UCSZ1) | (1<<UCSZ0) | (1<<URSEL)
	out 	UCSRC, R16

Stack_init:
	ldi	R16, HIGH(RAMEND)
	out	SPH, R16
	ldi	R16, LOW(RAMEND)
	out	SPL, R16

Timer_init0:			; CTC mode, 155 + 1 = 156 ticks !!! For præcis 10ms = 156.25 ticks
	ldi	R16, 0x9B	; 155 + 1 ticks
	out	OCR0, R16

	ldi	R16, 0b00001101	; 1024 Prescale Og CTC mode
	out	TCCR0, R16

Timer_Interrupt0:
	ldi	R16, 0b00000010
	out 	TIMSK, R16
	sei

Timer_init1:
	ldi	R16, 0b00000000
	out	TCCR1A, R16
	ldi	R16, 0b01000111
	out	TCCR1B, R16

	ldi	R16, 0x00
	out	TCNT1L, R16
	out	TCNT1H, R16


	sbi	PORTC, 2
	cbi	PORTC, 3
;********************
;********************
;*       Main       *
;********************
;********************
Main:				;Main loop.
	out	OCR2, R25	;Hastigheden sættes til værdien i R25.

	sbic	PINA, 1		;Tjek om den hvidelinje bliver detekteret.
	call	LapCounter

	sbrc	R28, 0		;Tjek om der er data som skal sendes over seriel.
	call	USART_Transmit

	sbrc	R28, 1		;Tjek om der er banedata som skal sendes over seriel.
	call	Transmit_Length

	sbrc	R28, 2		;Tjek om tælleren skal nulstilles.
	call	Clear_Counter

	;call	Read_Acc	;Læs accelerometeret og send værdien over seriel.

jmp	Main			;Slutningen af mainloop.

;********************
;*    LapCounter    *
;********************
LapCounter:
	sbi	PORTA, 0
	sbic	PINA, 1
	jmp 	LapCounter
	inc	R24

	ldi	R29, 0xBB
	ldi	R30, 0x12
	mov	R31, R24
	sbr	R28, 0b00000111

	cbi	PORTA, 0
	ret

;********************
;* Transmit Length  *
;********************
Transmit_Length:
	in	R16, TCNT1L
	in	R31, TCNT1H

	ldi	R29, 0xBB	;Transmit MSB af tælleregisteret.
	ldi	R30, 0x13
	call	USART_Transmit

	ldi	R29, 0xBB	;Transmit LSB af tælleregisteret.
	ldi	R30, 0x14
	mov	R31, R16
	call	USART_Transmit
	cbr	R28, 0b00000010
	ret

;********************
;*  Clear Counter   *
;********************
Clear_Counter:
	ldi	R16, 0x00
	out	TCNT1H, R16
	out	TCNT1L, R16
	cbr	R28, 0b00000100
	ret

;********************
;*  Speed Measure   *
;********************
Speed_Measure:
	cli
	in	R17, TCNT1L	;WheelSpeed LSB
	in	R18, TCNT1H	;WheelSpeed MSB


	mov	R22, R18	;Flyt læste værdier til at gemmes for næste udregning
	mov	R21, R17

	sub	R17, R19	;Substrat LSB
	sbc	R18, R20	;Substrat MSB med Carry

				;Hvis negativt flag er sat er udregningen forkert
	brmi	Error_Calculation

	mov	R20, R22	;Flyt de gemte værdier tilbage til korrekt register for næste udregning
	mov	R19, R21


	mov	R23, R17	;Flyt pulses/10 ms til Speed register.

	;ldi	R29, 0xBB	;Transmit LSB af tælleregisteret.
	;ldi	R30, 0x16
	;mov	R31, R23
	;call	USART_Transmit

	sei
	reti

Error_Calculation:
	ldi	R23, 0xBB
	sei
	reti

;********************
;*     Read Acc     *
;********************
Read_Acc:
	ldi	R16, 0b00100010	;ADC2 vælges
	out	ADMUX, R16

	sbis	ADCSR, ADIF	;Venter på at ADC'en er klar.
	rjmp	Read_Acc
	sbi	ADCSR, ADIF	;ADIF flag ryddes.
	in	R31, ADCL	;ADC læses, ADCL bruges ikke.
	in	R31, ADCH	;ADC læses, ADCH bruges.

	ldi	R29, 0xBB	;Seriel data gøres klar. Reply
	ldi	R30, 0x15	;Seriel data gøres klar. Acc

	call	USART_Transmit
	ret

;********************
;*  USART Receiver  *
;********************
USART_Receive:
				;De 3 bytes gemmes i R29, R30 og R31.
	cli
USART_Receive1:
	sbis	UCSRA, RXC
	rjmp	USART_Receive1
				;Get and return received data from buffer
	in	R29, UDR

USART_Receive2:
	sbis	UCSRA, RXC
	rjmp	USART_Receive2
				;Get and return received data from buffer
	in	R30, UDR

USART_Receive3:
	sbis	UCSRA, RXC
	rjmp 	USART_Receive3
				;Get and return received data from buffer
	in	R31, UDR


;********************
;*       Type	    *
;********************
	cpi	R29, Set
	breq	Type_Set

	cpi	R29, Get
	breq	Type_Get

	cpi	R29, Reply
	breq	Type_Reply

	jmp	Error

;********************
;*   Set Command    *
;********************
Type_Set:
	cpi	R30, Start
	breq	Type_Set_Start

	cpi	R30, Stop
	breq	Type_Set_Stop

	jmp	Error
;********************
;*   Get Command    *
;********************
Type_Get:
	cpi	R30, Register
	breq	Type_Get_Register

	jmp	Error

;********************
;*    Set Start     *
;********************
Type_Set_Start:

	call Upscale_255

	ldi	R29, 0xBB
	ldi	R30, 0x10
	sbr	R28, 0b00000001

	sei
	reti

;********************
;*  Upscale speed   *
;********************
Upscale_255:				; Skalerer hastighed fra 0-100 til interval 0-255
	ldi     R16, 255                ; Multiplicerer med 255
	mov     R17, R31                ; input værdi 0-100

	mul     R17, R16                ; Multiplikation
	movw    R16, R0                 ; Flyt resultat til R16:R17

	ldi     R18, 100                ; Indlæs denominator
	clr     R27                     ; Clear kvotienten
	clr     R26                     ; Clear Kvotienten
DIV16_100:
	adiw    R27:R26, 1              ; Increment kviotenten

	sub     R16, R18                ; Sub 100 fra Input LSB
	sbci    R17, 0                  ; Sub 0 med carry MSB
	brcc    DIV16_100               ; Break hvis NumMSB >=0

	subi    R26, 1                  ; En gang for mange subs
	sbci    R27, 0                  ; sub 1 fra kviotenten
	mov	R25, R26		; Flyt resultatet LSB til speed register

	ret

;********************
;*    Set Stop      *
;********************
Type_Set_Stop:
	ldi 	R25, 0x00
	ldi	R29, 0xBB
	ldi	R30, 0x11
	ldi	R31, 0x00
	sbr	R28, 0b00000001
	sei
	reti

;********************
;*Get Register Data *
;********************
Type_Get_Register:
	cpi	R31, 0x20
	breq	Reply_Register_20

	cpi	R31, 0x21
	breq	Reply_Register_21

	cpi	R31, 0x22
	breq	Reply_Register_22

	cpi	R31, 0x23
	breq	Reply_Register_23

	cpi	R31, 0x24
	breq	Reply_Register_24

	cpi	R31, 0x25
	breq	Reply_Register_25

	jmp	Error

;********************
;*  Reply Command   *
;********************
Type_Reply:

Error:
	ldi	R29, 0xBB
	ldi	R30, 0xBB
	ldi	R31, 0xBB
	sbr	R28, 0b00000001
	sei
	reti
;####################
Reply_Register_20:
	ldi	R29, 0xBB
	ldi	R30, 0x12
	mov	R31, R20
	sbr	R28, 0b00000001
	sei
	reti

Reply_Register_21:
	ldi	R29, 0xBB
	ldi	R30, 0x12
	mov	R31, R21
	sbr	R28, 0b00000001
	sei
	reti

Reply_Register_22:
	ldi	R29, 0xBB
	ldi	R30, 0x12
	mov	R31, R22
	sbr	R28, 0b00000001
	sei
	reti

Reply_Register_23:
	ldi	R29, 0xBB
	ldi	R30, 0x12
	mov	R31, R23
	sbr	R28, 0b00000001
	sei
	reti

Reply_Register_24:
	ldi	R29, 0xBB
	ldi	R30, 0x12
	mov	R31, R24
	sbr	R28, 0b00000001
	sei
	reti

Reply_Register_25:
	ldi	R29, 0xBB
	ldi	R30, 0x12
	mov	R31, R25
	sbr	R28, 0b00000001
	sei
	reti


;********************
;*  USART Transmit  *
;********************
USART_Transmit:
USART_Transmit1:
				;Wait for empty transmit buffer
	sbis 	UCSRA, UDRE
	rjmp 	USART_Transmit1
				;Put data (R29) into buffer, sends the data
	out	UDR, R29

USART_Transmit2:
	sbis 	UCSRA, UDRE
	rjmp 	USART_Transmit2
				;Put data (R30) into buffer, sends the data
	out	UDR, R30

USART_Transmit3:
	sbis 	UCSRA, UDRE
	rjmp 	USART_Transmit3

				;Put data (R31) into buffer, sends the data
	out	UDR, R31
	cbr	R28, 0b00000001
	ret
