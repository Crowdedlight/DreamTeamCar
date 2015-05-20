;Dette program sender længden af det sving bilen netop har kørt igennem et sving. Stopper når den har kørt en omgang ( over hvid linje)
.include "m32def.inc"


.equ Set=0x55
.equ Get=0xAA
.equ Reply=0xBB

.equ Start=0x10
.equ Stop=0x11
.equ Register=0x12

.equ LowCorner= 110         ; Bestemmer grensen til at et sving bliver godkendt
.equ HighCorner= 145        ; Bestemmer grensen til at et sving bliver godkendt

.equ Corner=0xc0

.org 0
	jmp	Reset
    
.org 0x02
    jmp Speed_Measure_V2

.org 0x16
    jmp Timer0_Clear

.ORG URXCaddr
	jmp	USART_Receive

.org 0x2A
Reset:
    ldi R18, 0x00   ; Register til at tælle op til 4 acc målinger
	ldi	R19, 0x00	;Clear speed mesure bits
	ldi	R20, 0x00	;Clear speed mesure bits
	ldi	R21, 0x00	;Clear speed mesure bits
	ldi	R22, 0x00	;Clear speed mesure bits


	ldi	R23, 0x00	;Målt hastighed.

	ldi	R24, 0x00	;LapCounter register.
	ldi	R25, 0x00	;Speed register .
    ldi R26, 0x00   ; Speed V2
    ldi R27, 0x00   ; acc reg

	ldi	R28, 0x00	;Status register, 0-bit sættes høj når seriel data skal sendes.
				;1-bit sættes høj når banelængde skal sendes.
				;2-bit sættes høj når counter1 skal nulstilles.
                
                ;5-bit sættes højt hvis den er inde i et sving
                ;
                ;6 og 7 bit bruges til at bestemme sving
                ;0b10xxxxxx = højre
                ;0b01xxxxxx = venstre
                ;0b00xxxxxx = ligeud

	ldi	R29, 0x00	;Første byte seriel.
	ldi	R30, 0x00	;Anden byte seriel.
	ldi	R31, 0x00	;Tredje byte seriel.

;********************
;********************
;*  Initialisering  *
;********************
;********************
PORTA_Init:			;
	sbi	DDRA, 0		;Sættes til output. grøn LED
	cbi	DDRA, 1		;Sættes til input. CNY70
	cbi	DDRA, 2		;Sættes til input. Accelerometer, læses via ADC.

PORTB_Init:
	sbi	DDRB, 5		;MOSI
	cbi	DDRB, 6		;MISO
	sbi	DDRB, 7		;CLK
    sbi DDRB, 4     ;Fron LED
    sbi DDRB, 3     ;Left LED
    sbi DDRB, 2     ;Right LED


PORTC_Init:
	sbi	DDRC, 2		;Sættes til output. Enable H-Bro.
	sbi	DDRC, 3		;Sættes til output. Dir, H-Bro.
    cbi PORTC,3

PORTD_Init:
	;sbi	DDRD, 2
	;cbi	PORTD, 2

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

Timer_init0:			
    
    ; CTC mode, 155 + 1 = 156 ticks !!! For præcis 10ms = 156.25 ticks
	;ldi	R16, 0x9B	; 155 + 1 ticks
	;out	OCR0, R16

	;ldi	R16, 0b00001101	; 1024 Prescale Og CTC mode
    ldi     R16, 0b00000100
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

Hardware_int_init:
    ldi R16, 1<<INT0
    out GICR,R16            ; Initialiser interupt på PD2
    ldi R16, (1<<ISC01|0<<ISC00)
    out MCUCR,R16           ; Sætter INT0 til at trigge på rising
    
;********************
;********************
;*       Main       *
;********************
;********************
Main:				;Main loop.
	out	OCR2, R25	;Hastigheden sættes til værdien i R25.
    sbi PORTB,4

	sbic	PINA, 1		;Tjek om den hvidelinje bliver detekteret.
	;call	LapCounter
    call    Break
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
;*     Read Acc     *
;********************
Read_Acc:
    


    call Main_Acc_Read
    ldi R16,LowCorner
    sub R16,R27 
    brpl Turn_Right              ;Hvis det er en negativ acc. Sving til højre
    ldi R16,HighCorner
    sub R16,R27 
    brmi Turn_Left              ; Hvis der er positiv acc. Sving til venstre
    sbrs    R28,4
    call    Ount_Corner
    
    
    cbr     R28,0b10000000               ; Hvis den ikker er i et sving, sattes bit 5,6 og 7 til 0.
    cbr     R28,0b01000000               ; Da dette betyder at den ikk er i et sving
    cbr     R28,0b00100000               ; Bit 5 er det bit der fortæller om den er i et sving eller ej

    cbi     PORTB,2
    cbi     PORTB,3

	ret                     ; Hvis ikke det er nogen af de testet senarier køre den lige ud
Ount_Corner:
    sbr     R28,0b00010000               ; Dette bit sættes når den lige er kommet ud af et sving
    call    Transmit_Length
    call    Clear_Counter
ret    

Turn_Right:
    
    
    cbr R28,6                   ; Her sættes bit 6 til 0, da dette betyder at den ikke er i et venstre sving.
    
    sbrs        R28,7
    jmp         Right_First               ; Hop hvis det er første gang den kommer her
    sbrs        R28,5
    jmp         Right_Corner
    

ret
    
Right_Corner:
    sbi         PORTB,3
    sbr     R28,0b00100000                  ; Her sættes det at den er inde i et sving
    cbr     R28,0b00010000                  ; Bruges til at sige at man er i et sving
    call Transmit_Length
    call Clear_Counter
ret
        
Right_First:
   
    sbr R28,0b10000000 ; Her sættes at der er tale om et højresving i R18 0b10xxxxxx
ret
    
Turn_Left:
    cbr     R28,0b10000000
    sbrs    R28,6
    jmp     Left_First
    sbrs    R28,5
    jmp     Left_Corner
ret
    
Left_Corner:
    sbi     PORTB,2
    sbr     R28,0b00100000
    cbr     R28,0b00010000                  ; Bruges til at sige at man er i et sving
    Call    Transmit_Length
    call    Clear_Counter
ret
    
    
Left_First:
    sbr     R28,0b01000000
ret

ret


    
Main_Acc_Read:
    
	ldi	R16, 0b00100010	;ADC2 vælges
	out	ADMUX, R16

	sbis	ADCSR, ADIF	;Venter på at ADC'en er klar.
	rjmp	Main_Acc_Read
	sbi	ADCSR, ADIF	;ADIF flag ryddes.
	in	R27, ADCL	;ADC læses, ADCL bruges ikke.
	in	R27, ADCH	;ADC læses, ADCH bruges.
    lsr R27
    
    
    Acc_Read_2:
    sbis	ADCSR, ADIF	;Venter på at ADC'en er klar.
	rjmp	Acc_Read_2
	sbi	ADCSR, ADIF	;ADIF flag ryddes.
	in	R16, ADCL	;ADC læses, ADCL bruges ikke.
	in	R16, ADCH	;ADC læses, ADCH bruges.
    lsr R16
    
    add R27, R16
    
    ;mov R31,R27

	;ldi	R29, 0xBB	;Seriel data gøres klar. Reply
	;ldi	R30, 0x15	;Seriel data gøres klar. Acc

	;call	USART_Transmit
    ret                  ; Slut på Main_acc_read
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
	mov 	R25, R31
	ldi	R29, 0xBB
	ldi	R30, 0x10
	sbr	R28, 0b00000001

	sei
	reti

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
    
    
;********************
;*  Speed_Measur_V2 *
;********************
Speed_Measure_V2:
    cli
    
    in      R26,TCNT0       ; læs counter register over i R26
    mov     R31,R26         ; flytter farten over i data send 
    ;ldi     R30,0x16        ; Sender speed
    ;ldi     R29,0xBB        ; Reply
    
    ;sbi     R28,0b00000001  ; Fortæller at der skal sendes data.
    ldi     R16,0x00
    out     TCNT0,R16       ; Nulstiller timer counter
    ;sbi     PORTB,0
    Call Read_Acc
    ;sbrs    R28,5
    ;call Sving
    
    sei
    
    reti

    
;********************
;*  Clear Timer0    *
;********************    
Timer0_Clear:
    cli
    sei
    reti
;********************
;*  Break           *
;********************    
Break:
    ldi     R25,0x00
    sbr     R28,0b00100000
ret



;********************
;*  Sving           *
;******************** 
Sving:
		in	R17, OCR2 				;Load nuværende hastighed
		cpi	R23, Corner				;Sammenligner reelle hastighed med teoretisk		
		brge 	Decrease				;Tjekker wheelspeed mod teoretisk maks
		inc 	R17					;Øger hastighedsregister
		out 	OCR2, R17				;Outer ændret hastighed til OCR2. 
		ret;jmp 	Sving					;Hopper tilbage til check.
		
		Decrease:
		dec	R17					;Sænker hastighedsregister
		out	OCR2, R17
		ret;jmp	Sving					;Hopper tilbage til check
ret
