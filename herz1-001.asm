.include "m32def.inc"

.equ delx=0x2
.equ length=48
.equ length2=8
.equ length3=64
.equ repeat1=30

.def temp1=r16
.def temp2=r17
.def temp3=r18
.def temp4=r19
.def temp5=r20
.def temp6=r21
.def temp7=r22
; Scanline: which 8-bit-scanline is actually showed?
.def scanline=r23	


.def scanbit1=r1
.def scanbit2=r2
.def leds8=r3
.def intense=r4

.def Count=r5
.def itemp1=r6
.def itemp2=r7

.def keysave=r8
.def keyrepeat=r9

.def SubDelay=r10

.def DelScan=r11

.equ maxrepeat=30

.equ Timer0Startwert=255-16

.equ ShortDelay=1
.equ ShortDelay2=10
.equ MidDelay=20
.equ LongDelay=40

; Speed Timer2
; stopped:		0
; no divider :	1<<CS20
; / 8 :			1<<CS21
; / 32 :		(1<<CS21) | (1<<CS20)
; / 64 :		1<CS22
; / 128 :		(1<<CS22) | (1<<CS20)
; / 256 :		(1<<CS22) | (1<<CS21)
; / 1024 :		(1<<CS22) | (1<<CS21) | (1<<CS20)

.equ timer2aus=0
.equ timer2an=(1<<CS21) ; divided by 8
.equ MinIntense=1
.equ MaxIntense=32
.equ StartIntense=MaxIntense

; Speed Timer1
; stopped :					0
; no division :				1<<CS10
; / 8 :						1<<CS11
; / 64 :					(1<<CS11) | (1<<CS10)
; / 256 :					1<CS12
; / 1024 :					(1<<CS12) | (1<<CS10)
; ExtClk falling edge :		(1<<CS12) | (1<<CS11)
; ExtClk rising edge :		(1<<CS12) | (1<<CS11) | (1<<C10)

.equ timer1aus=0
;~ .equ timer1an=(1<<CS11) | (1<<CS10) ; divided by 64
;~ .equ timer1an=1<<CS11 ; divided by 8
.equ timer1an=1<<CS12 ; divided by 256
	
	jmp RESET; Reset Handler
	jmp EXT_INT0; IRQ0 Handler
	jmp EXT_INT1; IRQ1 Handler
	jmp EXT_INT2; IRQ2 Handler
	jmp TIM2_COMP; Timer2 Compare Handler
	jmp TIM2_OVF; Timer2 Overflow Handler
	jmp TIM1_CAPT; Timer1 Capture Handler
	jmp TIM1_COMPA; Timer1 CompareA Handler
	jmp TIM1_COMPB; Timer1 CompareB Handler
	jmp TIM1_OVF; Timer1 Overflow Handler
	jmp TIM0_COMP; Timer0 Compare Handler
	jmp TIM0_OVF; Timer0 Overflow Handler
	jmp SPI_STC; SPI Transfer Complete Handler
	jmp USART_RXC; USART RX Complete Handler
	jmp USART_UDRE; UDR Empty Handler
	jmp USART_TXC; USART TX Complete Handler
	jmp LADC; ADC Conversion Complete Handler
	jmp EE_RDY; EEPROM Ready Handler	
	jmp ANA_COMP; Analog Comparator Handler
	jmp TWSI; Two-wire Serial Interface Handler
	jmp SPM_RDY; Store Program Memory Ready Handler

EXT_INT0: 	; IRQ0 Handler
EXT_INT1: 	; IRQ1 Handler
;TIM2_COMP: 	; Timer2 Compare Handler
;TIM2_OVF: 	; Timer2 Overflow Handler
TIM1_CAPT:	; Timer1 Capture Handler
TIM1_COMPA:	; Timer1 CompareA Handler
TIM1_COMPB:	; Timer1 CompareB Handler
;TIM1_OVF:	; Timer1 Overflow Handler
;TIM0_OVF:	; Timer0 Overflow Handler
SPI_STC:	; SPI Transfer Complete Handler
USART_RXC:	; USART RX Complete Handler
USART_UDRE:	; UDR Empty Handler
USART_TXC:	; USART TX Complete Handler
LADC:		; ADC Conversion Complete Handler
EE_RDY:		; EEPROM Ready Handler
ANA_COMP:	; Analog Comparator Handler
TWSI:		; Two-wire Serial Interface Handler
EXT_INT2:	; IRQ2 Handler
TIM0_COMP:	; Timer0 Compare Handler
SPM_RDY:	; Store Program Memory Ready Handler
	reti

RESET:
	; Set Stackpointer
	cli
	ldi temp1,high(RAMEND) ; End of RAM
	out SPH,temp1
	ldi temp1,low(RAMEND)
	out SPL,temp1
	; Statusregister, Enable Interrupts
	ldi temp1,0x80
	out SREG,temp1
; Init the ports, Port A Output, Port B output Bits 0-4, Port C Output Bits 0-5, Input Bits 6-7
	ldi temp1,0xFF
	out DDRA,temp1
	ldi temp1,0x1F
	out DDRB,temp1
	ldi temp1,0x3F
	out DDRC,temp1
	ldi temp1,0xC0
	out PORTC,temp1

; Timer for Scanline-Timer Timer0
; Geschwindigkeit Timer2
; stopped :					0
; no division:				1<<CS00
; / 8 :						1<<CS01
; / 64 :					(1<<CS01) | (1<<CS00)
; / 256 :					1<<CS02
; / 1024 :					(1<<CS02) | (1<<CS00)
; extClk FallingEdge :		(1<<CS02) | (1<<CS01)
; extClk RisingEdge :		(1<<CS02) | (1<<CS01) | (1<<CS00)
;
	ldi temp1, (1<<CS01) ; divided by 8
	out TCCR0,temp1
	ldi temp1,Timer0Startwert
	out TCNT0,temp1
	clr DelScan
	inc DelScan
	
; Timer für Helligkeit Timer2
	; Timer 2 8 Bit Compare-Match und Overflow, 255 Bit, Prescaler /64 -> 250HZ PWM-Frequenz
	; / 8
	;~ ldi temp1,1<<CS21 ;| 1<<CS20
	; /32
	; ldi temp1, 1<<CS21 | 1<<CS20
	; /1
	ldi temp1,timer2aus
	out TCCR2,temp1
; Timer für Tastanabfrage Timer1	
	ldi temp1,0
	out TCCR1A,temp1
	ldi temp1,timer1an
	out TCCR1B,temp1
	ldi temp1,0x00
	ldi temp2,0xFF
	out TCNT1H,temp2
	out TCNT1L,temp1
	; Activate the timer-interrupts
	ldi temp1,1<<TOIE0 | 1<<OCIE2 | 1<<TOIE2 | 1<< TOIE1
	out TIMSK,temp1
	; Scanline to 0
	ldi scanline,0x00
	; Set Y-reg to start of "video-RAM"
	ldi XL,0x60
	ldi XH,0x00
	ldi temp1,StartIntense
	mov intense,temp1
	out OCR2,intense
	ldi temp2,0xC0
	mov keysave,temp1
; Set Subdelay to maximum possible
	ldi temp1,0xFF
	mov SubDelay,temp1
; Enable Interrupts	
	sei
	nop
	nop
	nop
; Clear Screen
	ldi temp2,0x00
	call SetScreen
; Loop-label, only used if added runprog terminates
loop1:

	ldi ZL,low(2*pr_fortest01)
	ldi ZH,high(2*pr_fortest01)
	call runprog
	rjmp loop1

; Interrupt-Program for scanline-output
TIM0_OVF:	; Timer0 Overflow Handler
	cli						; Disable Interrupts
	push temp1				; Push some Varaibles and some bits (SREG)
	push temp2
	IN temp1,SREG
	push temp1
; Delscan can be used for debugging, can slow down scanline-process. Just put DelScan to a high value
; by using b.e.:
;		ldi temp1,0x80
;		mov DelScan,temp1
; and disable the clr Delscan and inc Delscan
	dec DelScan
	brne t0con3
	clr DelScan
	inc DelScan
; Output the acual scanline
; Delete output for LED-anodes (Port A)
	ldi temp1,0x00
	out PORTA,temp1
; Is scanline == 0 ? -> set cathode-scanbit1 (PortB=1) and scanbit2 (Port C=0) 
	cpi scanline,0x00
	brne t0con1
	ldi temp1,0x01
	mov scanbit1,temp1
	ldi temp1,0x00
	mov scanbit2,temp1
	rjmp t0con2
t0con1:
	; Scanline <>0
	; Is scanline == 5 ? -> change the cathode-output from Port B to Port C
	cpi scanline,0x05
	brne t0con2
	ldi temp1,0x00
	mov scanbit1,temp1
	ldi temp1,0x01
	mov scanbit2,temp1
t0con2:
	; Output the Scanbits to Port B and Port C
	mov temp1,scanbit1
	ldi temp2,0x1F
	eor temp1,temp2
	out PORTB,temp1
	mov temp1,scanbit2
	ldi temp2,0x3F
	eor temp1,temp2
	; Set Bits 6 and 7 in Port C for pushbutton-input
	ori temp1,0xC0
	out PORTC,temp1
	; Load the anode-output for this scanline
	ld leds8,X+
	; Output LEDS to PortA
	out PORTA,leds8

	; shift the scanbits by one bit left
	clc
	lsl scanbit1
	clc
	lsl scanbit2
	; Next scanline
	inc scanline
	; Last scanline?
	cpi scanline,0x0B
	brne TIM0_Con1
	; Reset to Scanline 0
	ldi scanline,0x00
	ldi XL,0x60
	ldi XH,0x00
TIM0_CON1:
; Set Timer 2 and start it (for LED-brightness)
	ldi temp1,0
	out TCNT2,temp1
	ldi temp1,Timer2An
	out TCCR2,temp1
	ldi temp1,Timer0Startwert
	out TCNT0,temp1
T0con3:
	; Pop the varibales back from stack
	pop temp1
	out SREG,temp1
	pop temp2
	pop temp1
	sei										; Enable Interrups
	reti


; Interrupt-Programs for LED-brightness
TIM2_OVF: 	; Timer2 Overflow Handler
	cli										; Disable Interrups
	push temp1								; Push some varaibles to Stack
	; stop Timer 2
	ldi temp1,Timer2Aus
	out TCCR2,temp1
	pop temp1								; Pop some variables from stack
	sei										; Enable Interrups
	reti

TIM2_COMP: 	; Timer2 Compare Handler
	cli										; Disable Interrupts
	push temp1								; Push some varaibles and bits (SREG) to stack
	IN temp1,SREG
	push temp1
	; Stop Timer 2
	ldi temp1,Timer2Aus
	out TCCR2,temp1
	; Reset Timer 2
	ldi temp1,0
	out TCNT2,temp1
	; delete LEDs when intense <>MaxIntense
	mov temp1,intense
	cpi temp1,MaxIntense
	breq tim2con
	ldi temp1,0
	out PORTA,temp1
tim2con:	
	pop temp1								; Pop some Variables and bits
	out SREG,temp1
	pop temp1
	sei										; Enable Interrupts
	reti

; Interrupt-Program Timer 1: Scan the Push-bittons
TIM1_OVF:
	cli										; Disable Interrupts
	push temp1								; Push some variable and bits (SREG) to stack
	push temp2
	IN temp1,SREG
	push temp1
	call tastenabfrage						; Call program to look for the buttons
	ldi temp1,0x00
	ldi temp2,0xFF
	out TCNT1H,temp2						; Set the timer
	out TCNT1L,temp1
	pop temp1								; Pop some varaibles and bits
	out SREG,temp1
	pop temp2
	pop temp1
	sei										; Enable interrupts
	reti

; Some programs to control LEDs

; temp2 is the look for each scanline, 0x00 is all out, 0xff is all on
SetScreen:
	push temp1
	push YL
	push YH
	ldi YL,0x60
	ldi YH,0x00
	ldi temp1,0x0B
SeScLoop:
	st y+,temp2
	dec temp1
	brne SeScLoop
	pop YH
	pop YL
	pop temp1
	ret
	
; temp2 is the led-adress (0 bis 83), temp3 == 0 -> LED off ; 1 -> LED on
PixelChange:
	cli
	push temp1
	push temp2
	push temp3
	IN temp1,SREG
	push temp1
	push YL
	push YH
	andi temp3,0x7F
	ldi YL,0x60
	ldi YH,0x00
	mov temp1,temp2
	lsr temp1
	lsr temp1
	lsr temp1
	add YL,temp1
	mov temp1,temp2
	andi temp1,0x07
	ldi temp2,0x01
PCLoop1:	
	cpi temp1,0x00
	breq PCCon1
	dec temp1
	lsl temp2
	rjmp PCLoop1
PCCon1:
	nop
	ld temp1,y
	nop
	cpi temp3,0x00
	breq PCPixelAus
;Pixel on
	or temp1,temp2
	rjmp PCCon2
PCPixelAus:
; Pixel off
	com temp2
	and temp1,temp2
PCCon2:
	cli
	nop
	st y,temp1
	sei
	nop
	pop YH
	pop YL
	pop temp1
	out SREG,temp1
	pop temp3
	pop temp2
	pop temp1
	sei
	ret

; temp2 is LED-Addres (0-83), result in temp1
PixelGet:
	cli
	push temp2
	push temp3
	push YL
	push YH
	ldi YL,0x60
	ldi YH,0x00
	mov temp1,temp2
	lsr temp1
	lsr temp1
	lsr temp1
	add YL,temp1
	mov temp1,temp2
	andi temp1,0x07
	ldi temp2,0x01
PGLoop1:	
	cpi temp1,0x00
	breq PGCon1
	dec temp1
	lsl temp2
	rjmp PGLoop1
PGCon1:
	nop
	ld temp1,y
	nop
	and temp1,temp2
	cpi temp1,0
	breq PGCon2
	ldi temp1,1
PGcon2:
	pop YH
	pop YL
	pop temp3
	pop temp2
	sei
	ret

; RPGetVar1
; get the variable, if value is 0xFx, X is Variable 0 to F
; Put them in to temp5, 6, 4 and 3

RPGetVar1:
	cli
	push temp1
	push temp2
	push YL
	push YH
	ldi YH,0
	mov temp1,temp5
	andi temp1,0xF0
	cpi temp1,0xF0
	brne rpgcon1

	mov temp1,temp5
	andi temp1,0x0F
	ldi YL,0xC0
	add yl,temp1
	ld temp5,y

rpgcon1:
	mov temp1,temp6
	andi temp1,0xF0
	cpi temp1,0xF0
	brne rpgcon2

	mov temp1,temp6
	ldi YL,0xC0
	andi temp1,0x0F
	add yl,temp1
	ld temp6,y

rpgcon2:
	mov temp1,temp4
	andi temp1,0xF0
	cpi temp1,0xF0
	brne rpgcon3

	mov temp1,temp4
	ldi YL,0xC0
	andi temp1,0x0F
	add yl,temp1
	ld temp4,y
	
rpgcon3:
	mov temp1,temp3
	andi temp1,0xF0
	cpi temp1,0xF0
	brne rpgcon4

	mov temp1,temp3
	ldi YL,0xC0
	andi temp1,0x0F
	add yl,temp1
	ld temp3,y

rpgcon4:
	pop YH
	pop YL
	pop temp2
	pop temp1
	sei
	ret
	
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------
; RUNPROG - Start one program
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------


; ZH,ZL -> Pointer to program in flash-memory	
RunProg:
	push temp1								; Push some variables to stack
	push temp2
	push temp3
	push temp4
	push temp5
	push temp6
	push ZL
	push ZH
; get first byte, needs to be Zero
	lpm temp1,z+
	cpi temp1,0x00
	breq rpcon1
rpcon2:
	; End this program
	pop ZH									; Pop some variables from stack
	pop ZL
	pop temp6
	pop temp5
	pop temp4
	pop temp3
	pop temp2
	pop temp1
	ret
rpcon1:
	; get next Byte
	lpm temp1,z+
	; Stop program if byte 0xFF
	cpi temp1,0xff
	breq rpcon2
	
; *********************************************************
; Opcode 1: Set some LEDS, by Count, Delay, on/off (L_SETLED1)
;**********************************************************

	cpi temp1,L_SETLED1
	brne rpcon3

; Lesen Anzahl, Delay, An/Aus
; Read Count, Delay, On/off
	lpm temp5,z+ ; temp5 Count
	lpm temp4,z+ ; temp4 Delay
	lpm temp3,z+ ; temp3 On/Off
rploop1:
	lpm temp2,z+ ; temp5 LED-number to change
	dec temp2
	call PixelChange
	call Delay
	dec temp5
	brne rploop1
	call Delay
	rjmp rpcon1

rpcon3:

; ************************************************************************************************
; befehl 2: Zwischen Start und Ende setzen oder Löschen, Richtung Bit 8 an/Aus, mit Delay (1-84) L_SETLED2
; ************************************************************************************************
; Start, Ende,Delay, An/Aus können durch Variablen ersetzt werden (0xFX)
	cpi temp1,L_SETLED2
	brne rpcon4

; Lesen Start, Ende, Delay, An/Aus
	lpm temp5,z+ ; temp5 Start
	lpm temp6,z+ ; Temp6 Ende
	lpm temp4,z+ ; temp4 Delay
	lpm temp3,z+ ; temp3 anaus
	call RpGetVar1
	mov temp1,temp3
	andi temp1,0x80
	dec temp6
	mov temp7,temp1
	mov temp2,temp5
	dec temp2
rploop2:
	call PixelChange
	call Delay
	cp temp2,temp6
	brne rpcon5
;	call Delay
	rjmp rpcon1
rpcon5:
	cpi temp7,0x00
	brne rpcon6
	inc temp2
	rjmp rploop2
rpcon6:
	dec temp2
	rjmp rploop2

rpcon4:

;*************************************
; Befehl 3: LED setzen L_SETLED
; ************************************
; Zu setzende LED und Wartezeit kann durch Variable ersetzt werden (0xFX)

	cpi temp1,L_SETLED
	brne rpcon7
	
	lpm temp5,z+
	lpm temp4,z+
	ldi temp6,0
	ldi temp3,0
	call RpGetVar1
	mov temp2,temp5
	dec temp2
	ldi temp3,0x01
	call PixelChange
	call Delay
	rjmp rpcon1
	

rpcon7:

; **********************************
; Befehl 4: LED löschen L_DELLED
;***********************************
; Zu löschende LED und Wartezeit kann durch Variable ersetzt werden (0xFX)

	cpi temp1,L_DELLED
	brne rpcon8

	lpm temp5,z+
	lpm temp4,z+
	ldi temp3,0
	ldi temp6,0
	call RpGetVar1
	mov temp2,temp5
	dec temp2
	ldi temp3,0x00
	call PixelChange
	call Delay
	rjmp rpcon1

rpcon8:

; **********************************
; Befehl 5: Delay L_WAIT
; **********************************

	cpi temp1,L_WAIT
	brne rpcon9

	lpm temp4,z+
	ldi temp5,0
	ldi temp6,0
	ldi temp3,0
	call RpGetVar1
	call Delay
	rjmp rpcon1

rpcon9:

; *******************************************************************************************************
; Befehl 7: Zwischen Start Ende kurzfristig setzen oder löschen, mit Delay, An/Aus plus 80 für rückwärts L_SETLED4
; *******************************************************************************************************
; Variablen möglich

	cpi temp1,L_SETLED4
	brne rpcon10

	lpm temp5,z+
	lpm temp6,z+
	lpm temp4,z+
	lpm temp3,z+
	call RpGetVar1
	mov temp1,temp3
	andi temp1,0x80
	mov temp7,temp1
	mov temp2,temp5
	dec temp2
rploop3:
	call PixelChange
	call Delay
	ldi temp1,0x01
	eor temp3,temp1
	call PixelChange
	eor temp3,temp1
	cpi temp7,0x00
	brne rpcon11
	inc temp2
	rjmp rpcon12
rpcon11:
	dec temp2
rpcon12:
	cp temp2,temp6
	brne rploop3
	rjmp rpcon1

rpcon10:

; ****************************************************
; Befehl 6: Anzahl LEDS kurzfristig löschen oder setzen L_SETLED3
; ****************************************************

	cpi temp1,L_SETLED3
	brne rpcon13

	lpm temp5,z+
	lpm temp4,z+
	lpm temp3,z+
	ldi temp7,0x01
rploop4:
	lpm temp2,z+
	dec temp2
	call PixelChange
	call Delay
	eor temp3,temp7
	call PixelChange
	eor temp3,temp7
	call Delay
	dec temp5
	brne rploop4
	rjmp rpcon1

rpcon13:

; ************************************************************
; Befehl 8: LEDS von/bis mehrfach an/Aus /Blinken L_BLINK1
; ************************************************************

	ldi temp2,0x01
	cpi temp1,L_BLINK1
	breq rpcon14_1

	ldi temp2,0x00
	cpi temp1,L_BLINK2
	brne rpcon14
	
rpcon14_1:
	lpm temp5,z+
	lpm temp6,z+
	lpm temp4,z+
	lpm temp3,z+
	call RpGetVar1
	mov temp1,temp3
	mov temp3,temp2
	ldi temp7,0x01	
rploop7:
	mov temp2,temp5
	dec temp2
rploop5:
	call PixelChange
	inc temp2
	cp temp2,temp6
	brne rploop5
	call Delay
	eor temp3,temp7
	mov temp2,temp5
	dec temp2
rploop6:
	call PixelChange
	inc temp2
	cp temp2,temp6
	brne rploop6
	call Delay
	eor temp3,temp7
	dec temp1
	brne rploop7
	rjmp rpcon1

rpcon14:
	
; *****************************************************
; Befehl 10: alle LEDS im Anhang invertieren L_L_TOGGLE1
; *****************************************************
	
	cpi temp1,L_TOGGLE1
	brne rpcon15
	
	lpm temp5,z+
	lpm temp4,z+
	ldi temp7,0x01
rploop8:
	lpm temp2,z+
	dec temp2
	call PixelGet
	eor temp1,temp7
	mov temp3,temp1
	call PixelChange
	call Delay
	dec temp5
	brne rploop8
	rjmp rpcon1
	
rpcon15:

; ****************************************************
; Befehl 11: Start bis End invertieren L_TOGGLE2
; ****************************************************

	cpi temp1,L_TOGGLE2
	brne rpcon16
	
	lpm temp5,z+
	lpm temp6,z+
	lpm temp4,z+
	ldi temp3,0x00
	mov temp2,temp5
	dec temp2
	ldi temp7,0x01
rploop9:
	Call PixelGet
	eor temp1,temp7
	mov temp3,temp1
	call PixelChange
	call Delay
	inc temp2
	cp temp2,temp6
	brne rploop9
	rjmp rpcon1

rpcon16:
; ****************************************************
; Befehl 12: Start bis End invertieren, rückwärts L_TOGGLE3
; ****************************************************

	cpi temp1,L_TOGGLE3
	brne rpcon17
	
	lpm temp5,z+
	lpm temp6,z+
	dec temp6
	dec temp6
	lpm temp4,z+
	ldi temp3,0x00
	mov temp2,temp5
	dec temp2
	ldi temp7,0x01
rploop10:
	Call PixelGet
	eor temp1,temp7
	mov temp3,temp1
	call PixelChange
	call Delay
	dec temp2
	cp temp2,temp6
	brne rploop10
	rjmp rpcon1

rpcon17:
; ****************************************************
; Befehl 13: Start bis End rollen, Start< End L_ROL
; ****************************************************

	cpi temp1,L_ROL
	brne rpcon18
	
	lpm temp5,z+
	lpm temp6,z+
	dec temp6
	dec temp6
	mov temp2,temp5
	dec temp2
; Sichern der ersten LED, kommt in letzte LED
	call PixelGet
	mov temp4,temp1 ; Gesichert in temp4
rploop11:
	inc temp2
	call PixelGet
	mov temp3,temp1
	dec temp2
	call PixelChange
	cp temp2,temp6
	breq rpcon19
	inc temp2
	rjmp rploop11
rpcon19:
	inc temp2
	mov temp3,temp4
	call PixelChange
	rjmp rpcon1
	
rpcon18:
; ****************************************************
; Befehl 14: Start bis Ende rollen, Start > Ende L_ROR
; ****************************************************

	cpi temp1,L_ROR
	brne rpcon20
	
	lpm temp5,z+
	lpm temp6,z+
;	dec temp6
;	dec temp6
	mov temp2,temp5
	dec temp2
; Sichern der letzten LED, kommt in erste LED
	call Pixelget
	mov temp4,temp1 ; Gesichert in temp4
rploop12:
	dec temp2
	call PixelGet
	mov temp3,temp1
	inc temp2
	call PixelChange
	cp temp2,temp6
	breq rpcon21
	dec temp2
	rjmp rploop12
rpcon21:
	dec temp2
	mov temp3,temp4
	call PixelChange
	rjmp rpcon1

; *******************************************************************************************************
; Befehl 15: 	Anzahl LEDS mit Wait, LEDs ohne 0x80 werden schnell gesetzt und zwischengespeichert L_SETLeD5
; 				kommt eine 0x80-LED, dann werden die gespeicherten gelöscht/gesetzt und Delay ausgeführt
; *******************************************************************************************************

rpcon20:
	cpi temp1,L_SETLED5
	brne rpcon22

; Die gesetzten /gelöschten LEDs werden hier gesichert
	ldi yl,0xB5
	ldi yh,0
; Anzahl der gesicherten LEDs
	clr Count
; Holen der Parameter
	lpm temp5,z+ ; Anzahl LEDs
	lpm temp6,z+ ; Waittime
	lpm temp3,z+ ; Setzen/löschen
rploop13:
	lpm temp2,z+ ; Zu setzende LED
	mov temp7,temp2
	andi temp2,0x7F
	dec temp2
; Sichern
	st y+,temp2
	inc Count
; Setzen/Löschen
	call PixelChange
; Prüfen ob letzte LED der Folge
	andi temp7,0x80
	breq rpcon23
; gesetzte LED war letzte der Folge
; Umstellen auf Löschen/Setzen
	ldi temp7,1
	eor temp3,temp7
; Die folge komplett löschen oder setzen
	mov temp4,temp6
	call Delay
rploop14:	
	ld temp2,-y
	call PixelChange
	dec Count
	brne rploop14
; Umstellen auf Setzen/Löschen
	ldi temp7,1
	eor temp3,temp7
rpcon23:
	dec temp5
	brne rploop13
	ldi temp7,0x00
	cp Count,temp7
	breq rpcon24
; Ein Delay ausführen, wen leztes Element kein Ende einer Folge war
	mov temp4,temp6
	call Delay
rpcon24:
	rjmp rpcon1
	
rpcon22:

; *******************************************************************************************************
; Befehl 17:	Es wird die Variable Var mit Wert gesetzt 	L_VARSET
; *******************************************************************************************************

	cpi temp1,L_VARSET
	brne rpcon25

; temp2 -> Variable; temp3 -> Wert
	lpm temp2,z+
	lpm temp3,z+
	andi temp2,0x0F
	ldi yl,0xC0
	ldi yh,0
	; Variable 0 bis F
	add yl,temp2
	st y,temp3
rpcon26:
	rjmp rpcon1

rpcon25:

; *******************************************************************************************************
; Befehl 18:	Der LED-Zustand wird in die Variable geschrieben L_LEDGET	
; *******************************************************************************************************

	cpi temp1,L_LEDGET
	brne rpcon27

; temp3 -> Variable; temp2 -> LED-Nummer
	lpm temp3,z+
	lpm temp2,z+
	ldi yl,0xC0
	ldi yh,0
	andi temp3,0x0F
	; Variable 0 bis 9
	add yl,temp3
	dec temp2 ; Variable 1-84 -> 0-83
	call PixelGet
	cpi temp1,0
	breq rpcon29
	ldi temp1,1
rpcon29:
	st y,temp1
rpcon28:
	rjmp rpcon1

rpcon27:

; *******************************************************************************************************
; Befehl 16:	Beliebige LEDS rollen, erste LED wird mit Variable gesetzt 	L_ROLX
; *******************************************************************************************************

	cpi temp1,L_ROLX
	brne rpcon30
	
	; temp5 -> Anzahl; temp2 -> Variable; temp4 -> Wait
	lpm temp5,z+		; Anzahl
	lpm temp2,z+		; Variable
	lpm temp4,z+		; Wartezeit
	; Variablenwert holen
	ldi yl,0xC0			; y Basis setzen
	ldi yh,0
	andi temp2,0x0F		; temp2 im korrekten bereich?
	; Variable 0 bis 9
	add yl,temp2		; y auf Variable setzen
	; temp3 zu setzender Wert aus Variable
	ld temp3,y			; temp3 enthält zu setzenden LED-Wert
	push temp3 			; Am ende zu setzende LED auf Stack zwischenspeichern
	;~ dec temp5			; Anzahl um 1 verringern
rploop15:
	; Was muss getan werden? LED-Nummer 1 holen, z++, LED-Nummer 2 holen, dann Pixel-Zustand holen (PixelGet), Speichern in LED 1	
	lpm temp2,z+		; LED1-Nummer holen in temp2
	dec temp2 			; Minus 1 (0-84)
	push temp2 			; LED1-Nummer auf Stack zwischenspeichern
	mov temp6,temp2 	; diese LED-Nummer in temp6 sichern
	lpm temp2,z 		; LED2-Nummer holen
	dec temp2			; Auch hier um 1 verringern
	call PixelGet 		; Pixel-Get LED2, temp1 enthält Zustand
	cpi temp1,0 		; temp1 auf 0 oder 1
	breq rpcon31
	ldi temp1,1
rpcon31:
	mov temp3,temp1 	; temp1 in temp3 ablegen
	pop temp2 			; LED1-Nummer von Stack holen
	call PixelChange	; LED-Nummer eins mit Zustand LED2 setzen
	dec temp5			; Anzahl um 1 verringern
	brne rploop15
	pop temp3 			; temp3 enthält den Zustand der letzten LED aus der Variablen
	mov temp2,temp6 	; die letzte LED in temp2 legen
	call PixelChange	; Die letzte LED wird mit dem Wert aus der Variable geladen
	call Delay
	rjmp rpcon1
	
rpcon30:

; *******************************************************************************************************
; Befehl 19:	Kopien der LEDS von Start bis Ende, an Position Neu, hier mit 0x80 geodert L_LEDCOPY1
;				für rückwärts 	
; *******************************************************************************************************
	
	cpi temp1,L_LEDCOPY1
	brne rpcon32
	
; Temp5 -> Start ; Temp6 -> Ende
	lpm temp5,z+
	lpm temp6,z+
; Temp4 -> Neu
	lpm temp4,z+
; Temp4 -> 1 rückwärts, 0 -> Vorwärts	
	mov temp7,temp5
	dec temp7
	mov temp5,temp4
	andi temp4,0x7F
	dec temp4
	andi temp5,0x80
	cpi temp5,0
	breq rpcon33
	ldi temp5,1
	;~ dec temp6
rpcon33:
	;~ dec temp6
; Temp 7 als Zähler
rploop16:
	mov temp2,temp7
	call PixelGet
rpcon34:
	mov temp3,temp1
	mov temp2,temp4
	call PixelChange
; richtung temp4?
	cpi temp5,0
	brne rpcon35
	inc temp4
	rjmp rpcon36
rpcon35:
	dec temp4
rpcon36:
	inc temp7
	cp temp7,temp6
	brne rploop16
	rjmp rpcon1
	
rpcon32:	

; *******************************************************************************************************
; Befehl 20:	Es werden Anzahl LED-Paare gelesen, die erste LED je paar in die zweite des Paares L_LEDCOPY2
; 				kopiert 
; *******************************************************************************************************

	cpi temp1,L_LEDCOPY2
	brne rpcon37

; temp5 : Anzahl
	lpm temp5,z+
rploop17:
; temp2: zu kopierende LED, temp3: zu überschreibende LED 	
	lpm temp2,z+
	lpm temp3,z+
	call PixelGet
	cpi temp1,0
	breq rpcon38
	ldi temp1,1
rpcon38:
	mov temp2,temp3
	mov temp3,temp1
	call PixelChange
	dec temp5
	brne rploop17
	rjmp rpcon1
	
rpcon37:

; *******************************************************************************************************
; Befehl 21:	Es werden LEDS von Start bie Ende gesetzt, mit einer Bitmaske von Anzahl Länge, L_SETLED6
;				laut der Bitmaske
; *******************************************************************************************************

	cpi temp1,L_SETLED6
	brne rpcon39
	
; temp5 : Start; temp6 : Ende; temp3 : Anzahl; temp4 : Bitmaske
	lpm temp5,z+
	lpm temp6,z+
	lpm temp3,z+
	lpm temp4,z+
	call RpGetVar1
; temp7 : Bit 1
	ldi temp7,1 
; itemp1 : Sichere Anzahl (temp3)
	mov itemp1,temp3
; temp2 auf Start minus 1 setzen	
	mov temp2,temp5
	dec temp2
rploop18:
; ANDe temp2 nach Sicherung mit temp4
	mov temp5,temp7
	and temp5,temp4
	cpi temp5,0
; Ist Bit 0?	
	breq rpcon40
; Temp3 zwischenspeichern auf Stack	
	push temp3
; Temp3 = 1
	ldi temp3,1
	rjmp rpcon41
rpcon40:
; Temp3 zwischenspeichern auf Stack	
	push temp3
; Temp3 = 0
	ldi temp3,0
rpcon41:
; Pixel setzen	
	call PixelChange
; Temp3 com Stack wiederherstellen	
	pop temp3
; Bit 7 ein Bit nach links schieben
	rol temp7
; Anzahl=Anzahll-1
	dec temp3
	brne rpcon42
; Anzahl komplett abgearbeitet	
; temp7 wieder auf Bit1	
	ldi temp7,1
; Temp3 zurücksetzen	
	mov temp3,itemp1
rpcon42:
; Nächste LED	
	inc temp2
	cp temp2,temp6
	brne rploop18
	rjmp rpcon1

rpcon39:

; *******************************************************************************************************
; Befehl 22:	Variable mit Wert setzen, die aktuelle Adresse z auf den Stack legen 	L_FOR
; *******************************************************************************************************

	cpi temp1,L_FOR
	brne rpcon44

; temp2 -> Variable; temp3 -> Wert
	lpm temp2,z+
	lpm temp3,z+
	ldi yl,0xC0
	ldi yh,0
	andi temp2,0x0F
	; Variable 0 bis F
	add yl,temp2
	st y,temp3
; Adresse z auf Stack legen
	push ZL
	push ZH
rpcon45:
	rjmp rpcon1
	
rpcon44:

; *******************************************************************************************************
; Befehl 23:	Variable dekrementieren (um 1 verringern) 	L_DECVAR
; *******************************************************************************************************

	cpi temp1,L_DECVAR
	brne rpcon46

; temp2 -> Variable
	lpm temp2,z+
	ldi yl,0xC0
	ldi yh,0
	andi temp2,0x0F
	; Variable 0 bis 9
	add yl,temp2
	ld temp3,y
	dec temp3
	st y,temp3
rpcon47:
	rjmp rpcon1

rpcon46:

; *******************************************************************************************************
; Befehl 24:	Rücksprung zu Adresse, wenn Variable nicht gleich 0 L_NEXT1
;				Sonst rücksprungadresse vom Stack entfernen 	
; *******************************************************************************************************

	cpi temp1,L_NEXT1
	brne rpcon48

; temp2 -> Variable
	lpm temp2,z+
	ldi yl,0xC0
	ldi yh,0
	andi temp2,0x0F
	; Variable 0 bis 9
	add yl,temp2
	ld temp3,y
	cpi temp3,0
	breq rpcon51
; Variable ist ungleich 0, Rücksprung
	pop ZH
	pop ZL
	push ZL
	push ZH
	rjmp rpcon1
rpcon51:
; Variable ist gleich 0, Rücksprungadresse ebtfernen
	pop temp1
	pop temp1
	rjmp rpcon1
	
rpcon50:
	rjmp rpcon1

rpcon48:

; *******************************************************************************************************
; Befehl 25:	Rücksprung zu Adresse, immer L_NEXT2
; *******************************************************************************************************

	cpi temp1,L_NEXT2
	brne rpcon52

	pop ZH
	pop ZL
	push ZL
	push ZH
	rjmp rpcon1

rpcon52:

; *******************************************************************************************************
; Befehl 27:	Variable inkrementieren (um 1 erhöhen) 	L_INCVAR
; *******************************************************************************************************

	cpi temp1,L_INCVAR
	brne rpcon53

; temp2 -> Variable
	lpm temp2,z+
	ldi yl,0xC0
	ldi yh,0
	andi temp2,0x0F
	; Variable 0 bis 9
	add yl,temp2
	ld temp3,y
	inc temp3
	st y,temp3
rpcon54:
	rjmp rpcon1

rpcon53:

; *******************************************************************************************************
; Befehl 26:	Zwei variablen mit "+-*/ao<>" versehen 	L_VARMATH
; *******************************************************************************************************

	cpi temp1,L_VARMATH
	brne rpcon55
	; Variable1
	lpm temp2,z+
	andi temp2,0x0F
	; variable2
	lpm temp3,z+
	andi temp3,0x0F
	; Rechnenart
	lpm temp4,z+
	ldi yl,0xC0
	ldi yh,0x00
	add yl,temp2
	; Inhalt Var1
	ld temp5,y
	ldi yl,0xC0
	add yl,temp3
	; Inhalt Var2
	ld temp6,y
	cpi temp4,'+'
	brne rpcon56
	add temp5,temp6
	rjmp rpcon62
rpcon56:
	cpi temp4,'-'
	brne rpcon57
	sub temp5,temp6
	rjmp rpcon62
rpcon57:
	cpi temp4,'*'
	brne rpcon58
	mul temp5,temp6
	rjmp rpcon62
rpcon58:
	cpi temp4,'a'
	brne rpcon59
	and temp5,temp6
	rjmp rpcon62
rpcon59:
	cpi temp4,'o'
	brne rpcon60
	or temp5,temp6
	rjmp rpcon62
rpcon60:
	cpi temp4,'<'
	brne rpcon61
	rol temp5
	rjmp rpcon62
rpcon61:
	cpi temp5,'>'
	brne rpcon62
	ror temp6
rpcon62:
	; Temp5 Ergebnis, temp2 Variable
	ldi yl,0xC0
	ldi yh,0x00
	add yl,temp2
	; Var1 speichern
	st y,temp5
	rjmp rpcon1

rpcon55:	

; *******************************************************************************************************
; Befehl 32:	Variable, Wert, Abfrage - Next-Schleifen beenden, wenn Abfrage stimmt
; *******************************************************************************************************

	cpi temp1,L_NEXT3
	brne rpcon63
	lpm temp5,z+ ; Variable
	lpm temp6,z+ ; Vergleichswert
	lpm temp4,z+ ; Vergleich ( < > s g = n )
	ldi yl,0xC0
	ldi yh,0x00
	andi temp5,0x0F
	add yl,temp5
	ld temp5,y	; temp1 ist Wert der Variablen
	push temp5
	push temp4
	call RPGetVar1
	pop temp4
	pop temp5
	; Welche Art von Vergleich?
	cpi temp4,'=' ; Gleich
	brne rpcon64
	cp temp5,temp6
	brne rpcon65
	rjmp rpcon66
rpcon64:
	cpi temp4,'n' ; Ungleich
	brne rpcon67
	cp temp5,temp6
	breq rpcon65
	rjmp rpcon66
rpcon67:
	cpi temp4,'<' ; Kleiner
	brne rpcon68
	cp temp5,temp6
	brge rpcon65
	rjmp rpcon66
rpcon68:
	cpi temp4,'s' ; Kleiner gleich
	brne rpcon69
	cp temp6,temp5
	brlt rpcon65
	rjmp rpcon66
rpcon69:
	cpi temp4,'g' ; größer gleich
	brne rpcon70
	cp temp5,temp6
	brge rpcon66
	rjmp rpcon65
rpcon70:
	cpi temp4,'>'
	brne rpcon66
	cp temp6,temp5
	brlt rpcon66
	rjmp rpcon65
rpcon65: ; Vergleich falsch, rücksprung
	pop ZH
	pop ZL
	push ZL
	push ZH
	rjmp rpcon1
rpcon66:
	pop temp1
	pop temp1
	jmp rpcon1

rpcon63:
; *******************************************************************************************************
; Befehl 33:	Subdelay für Wait-befehle anpassen
; *******************************************************************************************************

	cpi temp1,L_SUBDELAY
	brne rpcon71
	
	lpm temp5,z+
	ldi temp6,0
	ldi temp3,0
	ldi temp4,0
	call RpGetVar1
	mov SubDelay,temp5
	rjmp rpcon1
	
rpcon71:
	rjmp rpcon1


;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------



; temp4 : Verzögerungswert
Delay:
	push temp2
	push temp3
	push temp4
	cpi temp4,0x00
	breq dlcon1
dlloop1:
	mov temp3,SubDelay
dlloop2:
	ldi temp2,0xFF
dlloop3:
	dec temp2
	brne dlloop3
	dec temp3
	brne dlloop2
	dec temp4
	brne dlloop1
dlcon1:
	pop temp4
	pop temp3
	pop temp2
	ret

; Tastenabfrage, Entprellung durch keysave
tastenabfrage:
	push temp1
	push temp2
	in temp1,PINC
; TasteLinks (dunkler)
	mov temp2,temp1
	andi temp2,0x40
	brne tast1nichtgedrueckt
; Taste 1 ist gedrückt
; Prüfen, ob Taste vorher schon gedrückt
	mov temp2,keysave
	andi temp2,0x40
	breq tast1vorher
	dec intense
	mov temp2,intense
	cpi temp2,MinIntense
	brne tacon1
	ldi temp2,MinIntense+1
	mov intense,temp2
tacon1:
	out OCR2,intense
; Keysave auffrischen, taste gedrückt	
	mov temp2,keysave
	andi temp2,0x80
	mov keysave,temp2
	rjmp tacon2
tast1nichtgedrueckt:
; Keysave auffrischen, Taste 1 nicht gedrückt
	mov temp2,keysave
	ori temp2,0x40
	mov keysave,temp2
; TasteRechts (heller)	
	mov temp2,temp1
	andi temp2,0x80
	brne tast2nichtgedrueckt
; Taste 2 ist gedrückt
; Prüfen, ob taste vorher schon gedrückt
	mov temp2,keysave
	andi temp2,0x80
	breq tast2vorher
	inc intense
	mov temp2,intense
	cpi temp2,MaxIntense+1
	brne tacon3
	ldi temp2,MaxIntense
	mov intense,temp2
tacon3:
	out OCR2,intense
; Keysave auffrischen, Taste gedrückt
	mov temp2,keysave
	andi temp2,0x40
	mov keysave,temp2
	rjmp tacon2
tast2nichtgedrueckt:
; Keysave aufrischenm Taste 2 nicht gedrückt
	mov temp2,keysave
	ori temp2,0x80
	mov keysave,temp2
	ldi temp1,maxrepeat
	mov keyrepeat,temp1
	rjmp tacon2
tast1vorher:
tast2vorher:
	dec keyrepeat
	brne tacon2
	ldi temp1,maxrepeat
	mov keyrepeat,temp1
	ldi temp1,0xC0
	mov keysave,temp1
tacon2:
	pop temp2
	pop temp1
	ret

; Datenbeschreibung
; 0x00 -> Start oder Lückenfüller (um gerade Byteanzahl zu erreichen
;	befehl(e)
; 0xff -> Ende

;---------------------------------------------------------------------------------------------------------------- 
; 01 	| Anzahl	| Warten	| An/Aus	|        	|	-> Mehrere LEDS setzen oder löschen, mit Wartezeit,
;		|			|			|			|			|		Wait, Anaus 0 aus, 1 setzen, es folgen 
;		|			|			|			|			|		Anzahl LEDs (1-84)
;---------------------------------------------------------------------------------------------------------------- 
.equ L_SETLED1=1

;----------------------------------------------------------------------------------------------------------------
; 02 	| Start  	| Ende		| Warten  	| An/Aus 	| 	-> Zwischen Start und End setzen oder löschen,
;		|			|			|			|			|		Wartezeit Wait, Anaus 0 löschen, 1 setzen,
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;	Geplante Erweiterung: Start,Ende, Warten, Anaus = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9 *** erledigt ***
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_SETLED2=2

;----------------------------------------------------------------------------------------------------------------
; 03 	| LED    	| Warten	|       	|        	|   -> LED setzen
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;	Geplante Erweiterung: LED = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9 *** erledigt ***
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_SETLED=3

;----------------------------------------------------------------------------------------------------------------
; 04 	| LED    	| Warten	|       	|        	|   -> LED löschen
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;
;	Geplante Erweiterung: LED = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9 *** erledigt ***
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_DELLED=4

;----------------------------------------------------------------------------------------------------------------
; 05 	| Warten   	|			|       	|        	|   -> Verzögerung Wait
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;
;	Geplante Erweiterung: Warten = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9 *** erledigt ***
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_WAIT=5

;----------------------------------------------------------------------------------------------------------------
; 06 	| Anzahl 	| Warten	| An/Aus	|        	|   -> Wie 01, aber kurzfristig gesetzt, gelöscht 
;---------------------------------------------------------------------------------------------------------------- 
.equ L_SETLED3=6

;----------------------------------------------------------------------------------------------------------------
; 07 	| Start  	| Ende		| Warten  	| An/Aus 	|   -> Wie 02, aber kurzfristig an/aus bzw. aus/an
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;
;	Geplante Erweiterung: Start oder Ende = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_SETLED4=7

;----------------------------------------------------------------------------------------------------------------
; 08 	| Start  	| Ende		| Warten   	| Count  	|   -> Die LEDs von Start bis end mehrfach an- und
;		|			|			|			|			|		ausschalten (Blinken), und zwar count-mal
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;	Geplante Erweiterung: Start oder Ende = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9 *** erledigt ***
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_BLINK1=8

;----------------------------------------------------------------------------------------------------------------
; 09 	| Start  	| Ende		| Warten   	| Count  	|   -> Die LEDs von Start bis end mehrfach aus-
;		|			|			|			|			|		und anschalten (Blinken), und zwar count-mal
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;	Geplante Erweiterung: Start oder Ende = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9 *** erledigt ***
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_BLINK2=9

;----------------------------------------------------------------------------------------------------------------
; 10 	| Anzahl 	| Warten	|        	|        	|	-> Wie 1, die LEDs werden getoggelt,
;		|			|			|			|			|		Anzahl LEDS folgen
;---------------------------------------------------------------------------------------------------------------- 
.equ L_TOGGLE1=10

;----------------------------------------------------------------------------------------------------------------
; 11 	| Start  	| Ende		| Warten   	|        	|	-> Wie 2, aufwärts, die LEDs werden getoggelt,
;		|			|			|			|			|		Richtung nicht genutzt
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;	Geplante Erweiterung: Start oder Ende = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_TOGGLE2=11

;----------------------------------------------------------------------------------------------------------------
; 12 	| Start  	| Ende		| Warten   	|  			|	-> Wie 2, abwärts. doe LEDs werden getoggelt,
;		|			|			|			|			|		Richtung nicht genutzt
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;	Geplante Erweiterung: Start oder Ende = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_TOGGLE3=12

;----------------------------------------------------------------------------------------------------------------
; 13 	| Start  	| Ende		| 			|			|	-> Rollen von unten nach oben,
;		|			|			|			|			|		letztes wird in erstes gerollt 	
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;	Geplante Erweiterung: Start oder Ende = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_ROL=13

;----------------------------------------------------------------------------------------------------------------
; 14	| Start		| Ende		|			|			|	-> Rollen von oben nach unten,
;		|			|			|			|			|		erstes wird in letztes gerollt
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;	Geplante Erweiterung: Start oder Ende = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_ROR=14

;----------------------------------------------------------------------------------------------------------------
; 15	| Anzahl	| Warten	|			|			|	-> Setzen mehrerer LEDs, Folgen werden mit
;		|			|			|			|			|		LED+128 beendet, Folge bekommt Delay
;---------------------------------------------------------------------------------------------------------------- 
.equ L_SETLED5=15

;----------------------------------------------------------------------------------------------------------------
; 16	| Anzahl	| Variable	| Warten	|			|	-> Rollen beliebiger LEDs, erste LED bekommt
;		|			|			|			|			|		Wert in Var, Wartezeit
;---------------------------------------------------------------------------------------------------------------- 
.equ L_ROLX=16

;----------------------------------------------------------------------------------------------------------------
; 17	| Variable	| Wert		|			|			|	-> Variable mit Wert setzen 0->A, 1->B, 2->C, 3->D,
;		|			|			|			|			|		4->E, 5->F, 6->G, 7->H, 8->I, 9->J
;---------------------------------------------------------------------------------------------------------------- 
.equ L_VARSET=17

;----------------------------------------------------------------------------------------------------------------
; 18	| Variable	| Nr		|			|			|	-> Variable =LED[Nr]
;		|			|			|			|			|		(0 : gelöscht, 1: gesetzt)
;---------------------------------------------------------------------------------------------------------------- 
.equ L_LEDGET=18

;----------------------------------------------------------------------------------------------------------------
; 19	| Start		| Ende		| Neu		| 			|	-> Es werden kopien der LEDS zwischen Start und Ende
;		|			|			|			|			|		erstellt, beginnend bei Neu.
;		|			|			|			|			|		Neu |0x80 -> rückwärts
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;	Geplante Erweiterung: Start, Endeoder Neu = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_LEDCOPY1=19

;----------------------------------------------------------------------------------------------------------------
; 20	| Anzahl	| 			|			|			|	-> Es wird die erste LED je Paar in die zweite
;		|			|			|			|			|		kopiert, das nächste Paar eingelesen
;---------------------------------------------------------------------------------------------------------------- 
.equ L_LEDCOPY2=20

;----------------------------------------------------------------------------------------------------------------
; 21	| Start		| Ende		| Anzahl	| Bitmaske	|	-> VOn start bis Ende werden immer Anzahl LEDS
;		|			|			|			|			|		gesetzt, laut den passenden Bits der Bitmaske,
;		|			|			|			|			|		 maixmal 8 LEDS als Anzahl
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
;	Geplante Erweiterung: Start, Ende, Anzahl oder Bitmaske = 0xF0 bis 0xF9 -> Nimm wie Variable 0 bis 9  *** erledigt ***
;
;---------------------------------------------------------------------------------------------------------------- 
.equ L_SETLED6=21

;----------------------------------------------------------------------------------------------------------------
; 22	| Variable	| Wert		|			|			|	-> Setze Schleifen-Rücksprungadresse für
;		|			|			|			|			|		"for-Schleife", Variable mit Wert setzen
;---------------------------------------------------------------------------------------------------------------- 
.equ L_FOR=22

;----------------------------------------------------------------------------------------------------------------
; 23	| Variable	|			|			|			|	-> Decrement der Variable
;---------------------------------------------------------------------------------------------------------------- 
.equ L_DECVAR=23

;----------------------------------------------------------------------------------------------------------------
; 24	| Variable	|			|			|			|	-> Rücksprung zur letzten for-Schleife, wenn
;		|			|			|			|			|		Variable <>0, ansonsten etfernen
;		|			|			|			|			|		der Rücksprung-Adressse
;---------------------------------------------------------------------------------------------------------------- 
.equ L_NEXT1=24

;----------------------------------------------------------------------------------------------------------------
; 25	|			|			|			|			|	-> Rücksprung zur letzten for-Schleife, immer
;---------------------------------------------------------------------------------------------------------------- 
.equ L_NEXT2=25

;----------------------------------------------------------------------------------------------------------------
; 27	| Varaible	|			|			|			|	-> Increment der Variable
;---------------------------------------------------------------------------------------------------------------- 
.equ L_INCVAR=27

;----------------------------------------------------------------------------------------------------------------
; 26	| Variable1	| Variable1	| "+-*/ao<>"|			|	-> Variable1 = Variable1 "+-*/ao<>" Variable2
;		|			|			|			|			|		(Plus, Minus, Mal, Geteilt,
;		|			|			|			|			|		AND, OR, Shiftleft, Shiftright)
;---------------------------------------------------------------------------------------------------------------- 
.equ L_VARMATH=26

;----------------------------------------------------------------------------------------------------------------

;----------------------------------------------------------------------------------------------------------------
; 28	| Variable1	| Variable2	|			|			|	-> Variable1 = Variable2
;---------------------------------------------------------------------------------------------------------------- 
.equ L_MOVVAR=28

;----------------------------------------------------------------------------------------------------------------
; 29	| Variable1	| Variable2	|			|			|	-> LED[Variable1]=Variable2
;---------------------------------------------------------------------------------------------------------------- 
.equ L_LEDVAR=29

;----------------------------------------------------------------------------------------------------------------
; 30	| Variable1	| Variable2	|			|			|	-> Variable2 = LED[Variable1]
;---------------------------------------------------------------------------------------------------------------- 
.equ L_VARLED=30

;----------------------------------------------------------------------------------------------------------------
; 31	| Variable	| Wert		| "+-*/ao"	|			|	-> Variable wird "immediate" gerechnet mit Wert
;		|			|			|			|			|		(Plus, Minus, Mal, Geteilt,
;		|			|			|			|			|		AND OR)
;---------------------------------------------------------------------------------------------------------------- 
.equ L_VARIMATH=31
;----------------------------------------------------------------------------------------------------------------

;----------------------------------------------------------------------------------------------------------------
; 32	| Variable	| Wert		| "<>gs="	|			|	-> Next-befehl für FOR-Schleife, die Variable wird
;		|			|			|			|			|		geprüft, ob "<" Kleiner, ">" Größer,
;		|			|			|			|			|		"s" Kleiner-Gleich, "g" Größer-Gleich, "=" gleich
;		|			|			|			|			|		"n" ungleich
;---------------------------------------------------------------------------------------------------------------- 
.equ L_NEXT3=32
;----------------------------------------------------------------------------------------------------------------

;----------------------------------------------------------------------------------------------------------------
; 33	| Wert		|	 		|	 		|			|	-> SubDelay mit Wert setzen
;---------------------------------------------------------------------------------------------------------------- 
.equ L_SUBDELAY=33
;----------------------------------------------------------------------------------------------------------------



; Variablen
.equ L_A=0
.equ L_B=1
.equ L_C=2
.equ L_D=3
.equ L_E=4
.equ L_F=5
.equ L_G=6
.equ L_H=7
.equ L_I=8
.equ L_J=9


pr_clearscreen:
	.db 0,L_SETLED2,1,84,0,0,0xff,0

pr_setscreen:
	.db 0,L_SETLED2,1,84,0,1,0xff,0

pr_fortest02:
.db 0,L_FOR,9,1
	.db 0,L_SETLED2,1,84,0,0
	.db 0,L_FOR,0,84
		.db 0,L_SETLED,0xF0,0
		.db 0,L_WAIT,0x1,0
		.db 0,L_DECVAR,0,0
	.db 0,L_NEXT1,0,0
.db 0,L_NEXT2

pr_fortest01:


; Hauptschleife, ewig laufend
.db 0,L_SETLED2,1,84,0,0
.db 0,L_FOR,9,1				; Schleife C (Wert egal)
; 	
	.db 0,L_FOR,3,2	
		.db 0,L_SETLED2,1,42,0,0 ; Löschen Rand
		.db 0,L_SETLED,1,0
		.db 0,L_SETLED,42,0
		.db 0,L_WAIT,10,0
		.db 0,L_SUBDELAY,20,0
		.db 0,L_FOR,8,2
			.db 0,L_VARSET,1,41
			.db 0,L_FOR,0,21
				.db 0,L_SETLED,0xF0,0
				.db 0,L_SETLED,0xF1,0
				.db 0,L_WAIT,1,0
				.db 0,L_DELLED,0xF0,0
				.db 0,L_DELLED,0xF1,0
				.db 0,L_WAIT,1,0
				.db 0,L_DECVAR,1,0
				.db 0,L_DECVAR,0,0
			.db 0,L_NEXT3,0,0xF8,"<",0
			.db 0,L_INCVAR,0,0
			.db 0,L_INCVAR,1,0
			.db 0,L_SETLED,0xF0,0
			.db 0,L_SETLED,0xF1,0
			.db 0,L_INCVAR,8,0
		.db 0,L_NEXT3,8,21,">",0
		.db 0,L_SUBDELAY,0xE0,0
		.db 0,L_WAIT,0xEF,0
		.db 0,L_SUBDELAY,20,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_DECVAR,3,0
	.db 0,L_NEXT1,3,0

;
	.db 0,L_SUBDELAY,0xFF,0
	.db 0,L_WAIT,0xEF,0
	.db 0,L_FOR,3,2	
		.db 0,L_SETLED2,1,84,0,0 ; Löschen Rand
		.db 0,L_WAIT,10,0
		.db 0,L_SUBDELAY,2,0
		.db 0,L_FOR,8,1
			.db 0,L_FOR,0,84
				.db 0,L_SETLED,0xF0,0
				.db 0,L_WAIT,1,0
				.db 0,L_DELLED,0xF0,0
				.db 0,L_WAIT,1,0
				.db 0,L_DECVAR,1,0
				.db 0,L_DECVAR,0,0
			.db 0,L_NEXT3,0,0xF8,"<",0
			.db 0,L_INCVAR,0,0
			.db 0,L_SETLED,0xF0,0
			.db 0,L_INCVAR,8,0
		.db 0,L_NEXT3,8,84,">",0
		.db 0,L_SUBDELAY,40,0
		.db 0,L_WAIT,0xEF,0
		.db 0,L_SUBDELAY,20,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_TOGGLE2,43,84,1,0
		.db 0,L_WAIT,0xEF,0
		.db 0,L_DECVAR,3,0
	.db 0,L_NEXT1,3,0

;
	.db 0,L_WAIT,0xEF,0
	.db 0,L_SETLED2,1,84,1,0
	.db 0,L_SUBDELAY,0X4,0
	.db 0,L_WAIT,10,0
	.db 0,L_FOR,0,20
		.db 0,L_FOR,1,1
			.db 0,L_SETLED,0xF1,0
			.db 0,L_WAIT,10,0
			.db 0,L_DELLED,0xF1,0
			.db 0,L_INCVAR,1,0
		.db 0,L_NEXT3,1,21,">",0
		.db 0,L_FOR,1,42
			.db 0,L_SETLED,0xF1,0
			.db 0,L_WAIT,10,0
			.db 0,L_DELLED,0xF1,0
			.db 0,L_DECVAR,1,0
		.db 0,L_NEXT3,1,22,"<",0
		.db 0,L_DECVAR,0,0
	.db 0,L_NEXT1,0,0

;
	.db 0,L_SUBDELAY,0X4,0
	.db 0,L_WAIT,10,0
	.db 0,L_FOR,1,1
		.db 0,L_SETLED,0xF1,0
		.db 0,L_WAIT,10,0
		.db 0,L_INCVAR,1,0
	.db 0,L_NEXT3,1,21,">",0
	.db 0,L_FOR,1,42
		.db 0,L_SETLED,0xF1,0
		.db 0,L_WAIT,10,0
		.db 0,L_DECVAR,1,0
	.db 0,L_NEXT3,1,22,"<",0

;
	.db 0,L_SUBDELAY,0xEF,0
	.db 0,L_BLINK1,43,84,20,2
	.db 0,L_BLINK1,1,84,20,2

;
	.db 0,L_SUBDELAY,4,0
	.db 0,L_SETLED2,1,84,0,0
	.db 0,L_SETLED1,7,0,1,7,13,19,39,33,27,0
	.db 0,L_SETLED,79,0
	.db 0,L_FOR,2,1
		.db 0,L_FOR,1,0xEF
			.db 0,L_LEDGET,0,1,0,0
			.db 0,L_ROLX,42,0,20,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,42,41,40,39,38,37,36,35,34,33,32,31,30,29,28,27,26,25,24,23,22,0
			.db 0,L_VARSET,0,1
			.db 0,L_ROLX,44,0,20,79,80,81,72,71,70,69,68,67,51,52,53,54,55,56,57,58,50,49,48,47,46,45,44,43,59,60,61,62,63,64,65,66,78,77,76,75,74,73,82,83,84,0
			.db 0,L_DECVAR,1,0
		.db 0,L_NEXT1,1,0
		.db 0,L_WAIT,0xEF,0
		.db 0,L_FOR,1,0xEF
			.db 0,L_LEDGET,0,22,0,0
			.db 0,L_ROLX,42,0,20,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0
			.db 0,L_VARSET,0,0
			.db 0,L_ROLX,44,0,20,84,83,82,73,74,75,76,77,78,66,65,64,63,62,61,60,59,43,44,45,46,47,48,49,50,58,57,56,55,54,53,52,51,67,68,69,70,71,72,81,80,79,0
			.db 0,L_DECVAR,1,0
		.db 0,L_NEXT1,1,0
		.db 0,L_WAIT,0xEF,0
		
		.db 0,L_DECVAR,2,0
	.db 0,L_NEXT1,2,0
	
	.db 0,L_WAIT,0x20,0

;
	.db 0,L_SUBDELAY,0x15,0
	.db 0,L_FOR,0,12
		.db 0,L_SETLED2,1,84,0,0
		.db 0,L_WAIT,80,0
		.db 0,L_SETLED,42,0,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_DELLED,42,0,0,0
		.db 0,L_SETLED1,2,0,1,21,41,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,21,41,0
		.db 0,L_SETLED1,3,0,1,20,50,40,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,20,50,40,0,0
		.db 0,L_SETLED1,2,0,1,19,39,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,19,39,0
		.db 0,L_SETLED1,3,0,1,18,49,38,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,18,49,38,0,0
		.db 0,L_SETLED1,4,0,1,17,58,66,37,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,4,0,0,17,58,66,37,0
		.db 0,L_SETLED1,5,0,1,16,57,48,65,36,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,5,0,0,16,57,48,65,36,0,0
		.db 0,L_SETLED1,3,0,1,15,47,35,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,15,47,35,0,0
		.db 0,L_SETLED1,4,0,1,72,56,64,78,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,4,0,0,72,56,64,78,0
		.db 0,L_SETLED1,3,0,1,14,46,34,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,14,46,34,0,0
		.db 0,L_SETLED1,6,0,1,13,71,55,63,77,33,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,6,0,0,13,71,55,63,77,33,0
		.db 0,L_SETLED1,3,0,1,81,45,84,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,81,45,84,0,0
		.db 0,L_SETLED1,6,0,1,12,70,54,62,76,32,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,6,0,0,12,70,54,62,76,32,0
		.db 0,L_SETLED1,9,0,1,11,80,69,53,44,61,75,83,31,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,9,0,0,11,80,69,53,44,61,75,83,31,0,0
		.db 0,L_SETLED1,9,0,1,10,79,68,52,43,60,74,82,30,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,9,0,0,10,79,68,52,43,60,74,82,30,0,0
		.db 0,L_SETLED1,3,0,1,79,1,82,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,79,1,82,0,0
		.db 0,L_SETLED1,8,0,1,9,67,51,2,22,59,73,29,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,8,0,0,9,67,51,2,22,59,73,29,0
		.db 0,L_SETLED1,4,0,1,8,3,23,28,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,4,0,0,8,3,23,28,0
		.db 0,L_SETLED1,4,0,1,7,4,24,27,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,4,0,0,7,4,24,27,0
		.db 0,L_SETLED1,4,0,1,6,5,25,26,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,4,0,0,6,5,25,26,0
		.db 0,L_SETLED1,4,0,1,6,5,25,26,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,4,0,0,6,5,25,26,0
		.db 0,L_SETLED1,4,0,1,7,4,24,27,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,4,0,0,7,4,24,27,0
		.db 0,L_SETLED1,4,0,1,8,3,23,28,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,4,0,0,8,3,23,28,0
		.db 0,L_SETLED1,8,0,1,9,67,51,2,22,59,73,29,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,8,0,0,9,67,51,2,22,59,73,29,0
		.db 0,L_SETLED1,3,0,1,79,1,82,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,79,1,82,0,0
		.db 0,L_SETLED1,9,0,1,10,79,68,52,43,60,74,82,30,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,9,0,0,10,79,68,52,43,60,74,82,30,0,0
		.db 0,L_SETLED1,9,0,1,11,80,69,53,44,61,75,83,31,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,9,0,0,11,80,69,53,44,61,75,83,31,0,0
		.db 0,L_SETLED1,6,0,1,12,70,54,62,76,32,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,6,0,0,12,70,54,62,76,32,0
		.db 0,L_SETLED1,3,0,1,81,45,84,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,81,45,84,0,0
		.db 0,L_SETLED1,6,0,1,13,71,55,63,77,33,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,6,0,0,13,71,55,63,77,33,0
		.db 0,L_SETLED1,3,0,1,14,46,34,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,14,46,34,0,0
		.db 0,L_SETLED1,4,0,1,72,56,64,78,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,4,0,0,72,56,64,78,0
		.db 0,L_SETLED1,3,0,1,15,47,35,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,15,47,35,0,0
		.db 0,L_SETLED1,5,0,1,16,57,48,65,36,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,5,0,0,16,57,48,65,36,0,0
		.db 0,L_SETLED1,4,0,1,17,58,66,37,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,4,0,0,17,58,66,37,0
		.db 0,L_SETLED1,3,0,1,18,49,38,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,18,49,38,0,0
		.db 0,L_SETLED1,2,0,1,19,39,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,19,39,0
		.db 0,L_SETLED1,3,0,1,20,50,40,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,20,50,40,0,0
		.db 0,L_SETLED1,2,0,1,21,41,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,21,41,0
		.db 0,L_SETLED,42,0,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_DELLED,42,0,0,0
		.db 0,L_DECVAR,0,0
	.db 0,L_NEXT1,0,0
	.db 0,L_FOR,0,12
		.db 0,L_SETLED1,3,0,1,10,11,12,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,10,11,12,0,0
		.db 0,L_SETLED1,2,0,1,9,13,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,9,13,0
		.db 0,L_SETLED1,5,0,1,8,79,80,81,14,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,5,0,0,8,79,80,81,14,0,0
		.db 0,L_SETLED1,2,0,1,7,15,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,7,15,0
		.db 0,L_SETLED1,9,0,1,6,67,68,69,70,71,72,16,17,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,9,0,0,6,67,68,69,70,71,72,16,17,0,0
		.db 0,L_SETLED1,2,0,1,5,18,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,5,18,0
		.db 0,L_SETLED1,10,0,1,4,51,52,53,54,55,56,57,58,19,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,10,0,0,4,51,52,53,54,55,56,57,58,19,0
		.db 0,L_SETLED1,2,0,1,3,20,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,3,20,0
		.db 0,L_SETLED1,2,0,1,2,21,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,2,21,0
		.db 0,L_SETLED1,10,0,1,1,43,44,45,46,47,48,49,50,42,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,10,0,0,1,43,44,45,46,47,48,49,50,42,0
		.db 0,L_SETLED1,2,0,1,22,41,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,22,41,0
		.db 0,L_SETLED1,2,0,1,23,40,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,23,40,0
		.db 0,L_SETLED1,10,0,1,24,59,60,61,62,63,64,65,66,39,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,10,0,0,24,59,60,61,62,63,64,65,66,39,0
		.db 0,L_SETLED1,2,0,1,25,38,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,25,38,0
		.db 0,L_SETLED1,9,0,1,26,73,74,75,76,77,78,36,37,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,9,0,0,26,73,74,75,76,77,78,36,37,0,0
		.db 0,L_SETLED1,2,0,1,27,35,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,27,35,0
		.db 0,L_SETLED1,5,0,1,28,82,83,84,34,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,5,0,0,28,82,83,84,34,0,0
		.db 0,L_SETLED1,2,0,1,29,33,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,29,33,0
		.db 0,L_SETLED1,3,0,1,30,31,32,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,30,31,32,0,0
		.db 0,L_SETLED1,3,0,1,30,31,32,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,30,31,32,0,0
		.db 0,L_SETLED1,2,0,1,29,33,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,29,33,0
		.db 0,L_SETLED1,5,0,1,28,82,83,84,34,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,5,0,0,28,82,83,84,34,0,0
		.db 0,L_SETLED1,2,0,1,27,35,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,27,35,0
		.db 0,L_SETLED1,9,0,1,26,73,74,75,76,77,78,36,37,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,9,0,0,26,73,74,75,76,77,78,36,37,0,0
		.db 0,L_SETLED1,2,0,1,25,38,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,25,38,0
		.db 0,L_SETLED1,10,0,1,24,59,60,61,62,63,64,65,66,39,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,10,0,0,24,59,60,61,62,63,64,65,66,39,0
		.db 0,L_SETLED1,2,0,1,23,40,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,23,40,0
		.db 0,L_SETLED1,2,0,1,22,41,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,22,41,0
		.db 0,L_SETLED1,10,0,1,1,43,44,45,46,47,48,49,50,42,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,10,0,0,1,43,44,45,46,47,48,49,50,42,0
		.db 0,L_SETLED1,2,0,1,2,21,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,2,21,0
		.db 0,L_SETLED1,2,0,1,3,20,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,3,20,0
		.db 0,L_SETLED1,10,0,1,4,51,52,53,54,55,56,57,58,19,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,10,0,0,4,51,52,53,54,55,56,57,58,19,0
		.db 0,L_SETLED1,2,0,1,5,18,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,5,18,0
		.db 0,L_SETLED1,9,0,1,6,67,68,69,70,71,72,16,17,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,9,0,0,6,67,68,69,70,71,72,16,17,0,0
		.db 0,L_SETLED1,2,0,1,7,15,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,7,15,0
		.db 0,L_SETLED1,5,0,1,8,79,80,81,14,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,5,0,0,8,79,80,81,14,0,0
		.db 0,L_SETLED1,2,0,1,9,13,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,2,0,0,9,13,0
		.db 0,L_SETLED1,3,0,1,10,11,12,0,0
		.db 0,L_WAIT,4,0
		.db 0,L_SETLED1,3,0,0,10,11,12,0,0
		.db 0,L_DECVAR,0
	.db 0,L_NEXT1,0,0
	
;
.db 0,L_NEXT2						; Ewig-Schleifen-Ende
.db 0xff,0							; ENDE
