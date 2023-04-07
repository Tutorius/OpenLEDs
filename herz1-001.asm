.include "m32def.inc"

;~ .define mod1
;~ .define mod2
;~ .define mod3
;~ .define mod4
;~ .define mod5
;~ .define mod6
;~ .define NewRol
;~ .define NewCopy
.define ForTest
;~ .define SLOWTEST


.def temp1=r16
.def temp2=r17
.def temp3=r18
.def temp4=r19
.def temp5=r20
.def temp6=r21
.def temp7=r22
; Scanline: Die 8-Bit-Linie zwischen 0 und 7, die gerade angezeigt wird im Interrupt
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

.equ maxrepeat=4

.equ Timer0Startwert=255-16

.equ ShortDelay=1
.equ ShortDelay2=10
.equ MidDelay=20
.equ LongDelay=40

; Geschwindigkeit Timer2
; gestoppt:		0
; kein Teiler:	1<<CS20
; / 8 :			1<<CS21
; / 32 :		(1<<CS21) | (1<<CS20)
; / 64 :		1<CS22
; / 128 :		(1<<CS22) | (1<<CS20)
; / 256 :		(1<<CS22) | (1<<CS21)
; / 1024 :		(1<<CS22) | (1<<CS21) | (1<<CS20)

.equ timer2aus=0
.equ timer2an=(1<<CS21) ; geteilt durch 8
.equ MinIntense=1
.equ MaxIntense=128
.equ StartIntense=MaxIntense

; Geschwindigkeit Timer1
; gestoppt:					0
; kein Teiler:				1<<CS10
; / 8 :						1<<CS11
; / 64 :					(1<<CS11) | (1<<CS10)
; / 256 :					1<CS12
; / 1024 :					(1<<CS12) | (1<<CS10)
; ExtClk falling edge :		(1<<CS12) | (1<<CS11)
; ExtClk rising edge :		(1<<CS12) | (1<<CS11) | (1<<C10)

.equ timer1aus=0
.equ timer1an=(1<<CS11) | (1<<CS10) ; geteilt durch 64







	
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
	ldi temp1,high(RAMEND) ; Main program start
	out SPH,temp1
	ldi temp1,low(RAMEND)
	out SPL,temp1
	; Statusregister, Enable Interrupts
	ldi temp1,0x80
	out SREG,temp1
; Port-Initialisierung: Port A Ausggang, Port B Ausgang, Port C Ausgang
	ldi temp1,0xFF
	out DDRA,temp1
	ldi temp1,0x1F
	out DDRB,temp1
	ldi temp1,0x3F
	out DDRC,temp1
	ldi temp1,0xC0+0x3F
	out PORTC,temp1
; Timer-Initialisierung
	; Timer 0 8 Bit normal, bei 4MHz und Prescaler /1024 -> ca. 3,9KHz, bei 16 Zyklen -> 250 Hz pro Scanline -> 31,25 HZ pro Screen 

; Timer für Scannen	Timer0
; Geschwindigkeit Timer2
; gestoppt:					0
; kein Teiler:				1<<CS00
; / 8 :						1<<CS01
; / 64 :					(1<<CS01) | (1<<CS00)
; / 256 :					1<<CS02
; / 1024 :					(1<<CS02) | (1<<CS00)
; extClk FallingEdge :		(1<<CS02) | (1<<CS01)
; extClk RisingEdge :		(1<<CS02) | (1<<CS01) | (1<<CS00)
	ldi temp1, (1<<CS01) | (1<<CS00) ; Teiler 64
	out TCCR0,temp1
	ldi temp1,Timer0Startwert
	out TCNT0,temp1
	
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
	; Timer Interrupts aktivieren
	ldi temp1,1<<TOIE0 | 1<<OCIE2 | 1<<TOIE2 | 1<< TOIE1
	out TIMSK,temp1
	; Scanline auf 0 setzen
	ldi scanline,0x00
	; Y-Register auf Speicheranfang setzen
	ldi XL,0x60
	ldi XH,0x00
	; Bildschirm löschen
	ldi temp1,StartIntense
	mov intense,temp1
	out OCR2,intense
	ldi temp2,0xC0
	mov keysave,temp1
;	ldi temp2,0x00
;	call SetScreen
	sei
	nop
	nop
	nop
	
loop1:
	ldi temp4,ShortDelay
	ldi temp2,0
	
	ldi zh,high(pr_clearscreen*2)
	ldi zl,low(pr_clearscreen*2)
	call runprog

	ldi zh,high(pr_fortest01*2)
	ldi zl,low(pr_fortest01*2)
	call runprog

	rjmp loop1


; Interrupt-Routine für Scanline-LED-Ansteuerung
TIM0_OVF:	; Timer0 Overflow Handler
	cli
	push temp1
	push temp2
	IN temp1,SREG
	PUSH temp1
	; Scanline ausgeben
; LEDS vorher löschen
	ldi temp1,0x00
	out PORTA,temp1
	cpi scanline,0x00
	brne t0con1
	ldi temp1,0x01
	mov scanbit1,temp1
	ldi temp1,0x00
	mov scanbit2,temp1
	rjmp t0con2
t0con1:
	cpi scanline,0x05
	brne t0con2
	ldi temp1,0x00
	mov scanbit1,temp1
	ldi temp1,0x01
	mov scanbit2,temp1
t0con2:
	mov temp1,scanbit1
	ldi temp2,0x1F
	eor temp1,temp2
	out PORTB,temp1
	mov temp1,scanbit2
	ldi temp2,0x3F
	eor temp1,temp2
	ori temp1,0xC0
	out PORTC,temp1
	ld leds8,X+
; LEDS anzeigen
	out PORTA,leds8
; Timer 2 setzen und starten
	ldi temp1,0
	out TCNT2,temp1
	ldi temp1,Timer2An
	out TCCR2,temp1
;
	lsl scanbit1
	lsl scanbit2
	inc scanline
	cpi scanline,0x0B
	brne TIM0_Con1
	ldi scanline,0x00
	ldi XL,0x60
	ldi XH,0x00
TIM0_CON1:
	ldi temp1,Timer0Startwert
	out TCNT0,temp1
	pop temp1
	out SREG,temp1
	pop temp2
	pop temp1
	sei
	reti


; Interrupt-Routine für Helligkeits-Steuerung der LEDs
TIM2_OVF: 	; Timer2 Overflow Handler
	cli
	push temp1
	; Timer 2 stoppen
	ldi temp1,Timer2Aus
	out TCCR2,temp1
	pop temp1
	sei
	reti

TIM2_COMP: 	; Timer2 Compare Handler
	cli
	push temp1
	IN temp1,SREG
	push temp1
	; Timer 2 stoppen
	ldi temp1,Timer2Aus
	out TCCR2,temp1
	; Timer 2 zurücksetzen
	ldi temp1,0
	out TCNT2,temp1
	; LEDs löschen, wenn intense <>MaxIntense
	mov temp1,intense
	cpi temp1,MaxIntense
	breq tim2con
	ldi temp1,0
	out PORTA,temp1
tim2con:	
	pop temp1
	out SREG,temp1
	pop temp1
	sei
	reti

; Interrupt-Routine Timer 1: Tastenabfrage
TIM1_OVF:
	cli
	push temp1
	push temp2
	IN temp1,SREG
	push temp1
	call tastenabfrage
	ldi temp1,0x00
	ldi temp2,0xFF
	out TCNT1H,temp2
	out TCNT1L,temp1
	pop temp1
	out SREG,temp1
	pop temp2
	pop temp1
	sei
	reti

; temp2 enthält Bitfolge für alle LEDs
SetScreen:
	; cli
	push temp1
	push YL
	push YH
	ldi YL,0x60
	ldi YH,0x00
	ldi temp1,0x0B
SeScLoop:
	cli
	nop
	st y+,temp2
	sei
	nop
	dec temp1
	brne SeScLoop
	pop YH
	pop YL
	pop temp1
	;sei
	ret
	
; temp2 enthält Bit-Adresse (0 bis 86), temp3 0 -> aus 1 -> an
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
;	cli
	nop
	ld temp1,y
;	sei
	nop
	cpi temp3,0x00
	breq PCPixelAus
;Pixel an
	or temp1,temp2
	rjmp PCCon2
PCPixelAus:
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

; temp2 enthält LED-Adresse (0-83), ergebnis in temp1 0 oder >0
PixelGet:
	cli
	push temp2
	push temp3
	push YL
	push YH
	;~ IN temp2,SREG
	;~ push temp2
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
	;~ cli
	nop
	ld temp1,y
	;~ sei
	nop
	and temp1,temp2
	cpi temp1,0
	breq PGCon2
	ldi temp1,1
PGcon2:
	;~ pop temp2
	;~ out SREG,temp2
	pop YH
	pop YL
	pop temp3
	pop temp2
	sei
	ret


;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------
; RUNPROG - Ausführen der Befehle laut LED-programm
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------
;----------------------------------------------------------------------------------------------------------------------------------------------


; ZH,ZL -> Zeiger auf Programm	
RunProg:
	push temp1
	push temp2
	push temp3
	push temp4
	push temp5
	push temp6
	push ZL
	push ZH
; Erstes Byte lesen, muss 0 sein
	lpm temp1,z+
	cpi temp1,0x00
	breq rpcon1
rpcon2:
	pop ZH
	pop ZL
	pop temp6
	pop temp5
	pop temp4
	pop temp3
	pop temp2
	pop temp1
	ret
rpcon1:
	lpm temp1,z+
	; Ist 0xFF? Ende
	cpi temp1,0xff
	breq rpcon2
	
; *********************************************************
; Befehl 1: Mehrere LEDS setzen, Anzahl,Delay, An/Aus L_SETLED1
;**********************************************************

	cpi temp1,L_SETLED1
	brne rpcon3

; Lesen Anzahl, Delay, An/Aus
	lpm temp5,z+ ; temp5 Anzahl
	lpm temp4,z+ ; temp4 Delay
	lpm temp3,z+ ; temp3 An/Aus
rploop1:
	lpm temp2,z+ ; temp5 LED
	; dec temp2
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

	cpi temp1,L_SETLED2
	brne rpcon4

; Lesen Start, Ende, Delay, An/Aus
	lpm temp5,z+ ; temp5 Start
;	dec temp5
	lpm temp6,z+ ; Temp6 Ende
	dec temp6
	lpm temp4,z+ ; temp4 Delay
	lpm temp3,z+ ; temp3 anaus
	mov temp1,temp3
	andi temp1,0x80
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

	cpi temp1,L_SETLED
	brne rpcon7
	
	lpm temp2,z+
	dec temp2
	lpm temp4,z+
	ldi temp3,0x01
	call PixelChange
	call Delay
	rjmp rpcon1
	

rpcon7:

; **********************************
; Befehl 4: LED löschen L_DELLED
;***********************************

	cpi temp1,L_DELLED
	brne rpcon8

	lpm temp2,z+
	dec temp2
	lpm temp4,z+
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
	call Delay
	rjmp rpcon1

rpcon9:

; *******************************************************************************************************
; Befehl 7: Zwischen Start Ende kurzfristig setzen oder löschen, mit Delay, An/Aus plus 80 für rückwärts L_SETLED4
; *******************************************************************************************************

	cpi temp1,L_SETLED4
	brne rpcon10

	lpm temp5,z+
;	dec temp5
	lpm temp6,z+
;	dec temp6
	lpm temp4,z+
	lpm temp3,z+
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

	ldi temp3,0x01
	cpi temp1,L_BLINK1
	breq rpcon14_1
	ldi temp3,0
	cpi temp1,9
	brne rpcon14
	
rpcon14_1:
	lpm temp5,z+
	lpm temp6,z+
	lpm temp4,z+
	lpm temp1,z+
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
	ldi yl,0xC0
	ldi yh,0
	cpi temp2,10
	brge rpcon26
	; Variable 0 bis 9
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
	cpi temp3,10
	brge rpcon28
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
	cpi temp2,10		; temp2 im korrekten bereich?
	brge rpcon30
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
	cpi temp2,10
	brge rpcon45
	; Variable 0 bis 9
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
	cpi temp2,10
	brge rpcon47
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
	cpi temp2,10
	brge rpcon50
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
	ldi temp3,0x10
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
.equ L_SETLED1=1
;----------------------------------------------------------------------------------------------------------------
; 02 	| Start  	| Ende		| Warten  	| An/Aus 	| 	-> Zwischen Start und End setzen oder löschen,
;		|			|			|			|			|		Wartezeit Wait, Anaus 0 löschen, 1 setzen,
;		|			|			|			|			|		Anaus plus 0x80 für rückwärts
.equ L_SETLED2=2
;----------------------------------------------------------------------------------------------------------------
; 03 	| LED    	| Warten	|       	|        	|   -> LED setzen
.equ L_SETLED=3
;----------------------------------------------------------------------------------------------------------------
; 04 	| LED    	| Warten	|       	|        	|   -> LED löschen
.equ L_DELLED=4
;----------------------------------------------------------------------------------------------------------------
; 05 	| Warten   	|			|       	|        	|   -> Verzögerung Wait
.equ L_WAIT=5
;----------------------------------------------------------------------------------------------------------------
; 06 	| Anzahl 	| Warten	| An/Aus	|        	|   -> Wie 01, aber kurzfristig gesetzt, gelöscht 
.equ L_SETLED3=6
;----------------------------------------------------------------------------------------------------------------
; 07 	| Start  	| Ende		| Warten  	| An/Aus 	|   -> Wie 02, aber kurzfristig an/aus bzw. aus/an
.equ L_SETLED4=7
;----------------------------------------------------------------------------------------------------------------
; 08 	| Start  	| Ende		| Warten   	| Count  	|   -> Die LEDs von Start bis end mehrfach an- und
;		|			|			|			|			|		ausschalten (Blinken), und zwar count-mal
.equ L_BLINK1=8
;----------------------------------------------------------------------------------------------------------------
; 09 	| Start  	| Ende		| Warten   	| Count  	|   -> Die LEDs von Start bis end mehrfach aus-
;		|			|			|			|			|		und anschalten (Blinken), und zwar count-mal
.equ L_BLINK2=9
;----------------------------------------------------------------------------------------------------------------
; 10 	| Anzahl 	| Warten	|        	|        	|	-> Wie 1, die LEDs werden getoggelt,
;		|			|			|			|			|		Anzahl LEDS folgen
.equ L_TOGGLE1=10
;----------------------------------------------------------------------------------------------------------------
; 11 	| Start  	| Ende		| Warten   	|        	|	-> Wie 2, aufwärts, die LEDs werden getoggelt,
;		|			|			|			|			|		Richtung nicht genutzt
.equ L_TOGGLE2=11
;----------------------------------------------------------------------------------------------------------------
; 12 	| Start  	| Ende		| Warten   	|  			|	-> Wie 2, abwärts. doe LEDs werden getoggelt,
;		|			|			|			|			|		Richtung nicht genutzt
.equ L_TOGGLE3=12
;----------------------------------------------------------------------------------------------------------------
; 13 	| Start  	| Ende		| 			|			|	-> Rollen von unten nach oben,
;		|			|			|			|			|		letztes wird in erstes gerollt 	
.equ L_ROL=13
;----------------------------------------------------------------------------------------------------------------
; 14	| Start		| Ende		|			|			|	-> Rollen von oben nach unten,
;		|			|			|			|			|		erstes wird in letztes gerollt
.equ L_ROR=14
;----------------------------------------------------------------------------------------------------------------
; 15	| Anzahl	| Warten	|			|			|	-> Setzen mehrerer LEDs, Folgen werden mit
;		|			|			|			|			|		LED+128 beendet, Folge bekommt Delay
.equ L_SETLED5=15
;----------------------------------------------------------------------------------------------------------------
; 16	| Anzahl	| Variable	| Warten	|			|	-> Rollen beliebiger LEDs, erste LED bekommt
;		|			|			|			|			|		Wert in Var, Wartezeit
.equ L_ROLX=16
;----------------------------------------------------------------------------------------------------------------
; 17	| Variable	| Wert		|			|			|	-> Variable mit Wert setzen 0->A, 1->B, 2->C, 3->D,
;		|			|			|			|			|		4->E, 5->F, 6->G, 7->H, 8->I, 9->J
.equ L_VARSET=17
;----------------------------------------------------------------------------------------------------------------
; 18	| Variable	| 			|			|			|	-> Variable =LED[Nr]
;		|			|			|			|			|		(0 : gelöscht, 1: gesetzt)
.equ L_LEDGET=18
;----------------------------------------------------------------------------------------------------------------
; 19	| Start		| Ende		| Neu		| 			|	-> Es werden kopien der LEDS zwischen Start und Ende
;		|			|			|			|			|		erstellt, beginnend bei Neu.
;		|			|			|			|			|		Neu |0x80 -> rückwärts
.equ L_LEDCOPY1=19
;----------------------------------------------------------------------------------------------------------------
; 20	| Anzahl	| 			|			|			|	-> Es wird die erste LED je Paar in die zweite
;		|			|			|			|			|		kopiert, das nächste Paar eingelesen
.equ L_LEDCOPY2=20
;----------------------------------------------------------------------------------------------------------------
; 21	| Start		| Ende		| Anzahl	| Bitmaske	|	-> VOn start bis Ende werden immer Anzahl LEDS
;		|			|			|			|			|		gesetzt, laut den passenden Bits der Bitmaske,
;		|			|			|			|			|		 maixmal 8 LEDS als Anzahl
.equ L_SETLED6=21
;----------------------------------------------------------------------------------------------------------------
; 22	| Variable	| Wert		|			|			|	-> Setze Schleifen-Rücksprungadresse für
;		|			|			|			|			|		"for-Schleife", Variable mit Wert setzen
.equ L_FOR=22
;----------------------------------------------------------------------------------------------------------------
; 23	| Variable	|			|			|			|	-> Decrement der Variable
.equ L_DECVAR=23
;----------------------------------------------------------------------------------------------------------------
; 24	| Variable	|			|			|			|	-> Rücksprung zur letzten for-Schleife, wenn
;		|			|			|			|			|		Variable <>0, ansonsten etfernen
;		|			|			|			|			|		der Rücksprung-Adressse
.equ L_NEXT1=24
;----------------------------------------------------------------------------------------------------------------
; 25	|			|			|			|			|	-> Rücksprung zur letzten for-Schleife, immer
.equ L_NEXT2=25
;----------------------------------------------------------------------------------------------------------------


;----------------------------------------------------------------------------------------------------------------
; 26	| Variable1	| Variable1	| "+-*/ao<>"|			|	-> Variable1 = Variable1 "+-*/ao<>" Variable2
;		|			|			|			|			|		(Plus, Minus, Mal, Geteilt,
;		|			|			|			|			|		AND, OR, Shiftleft, Shiftright)
.equ L_VARMATH=26
;----------------------------------------------------------------------------------------------------------------
; 27	| Varaible	|			|			|			|	-> Increment der Variable
.equ L_INCVAR=27
;----------------------------------------------------------------------------------------------------------------
; 28	| Variable1	| Variable2	|			|			|	-> Variable1 = Variable2
.equ L_MOVVAR=28
;----------------------------------------------------------------------------------------------------------------
; 29	| Variable1	| Variable2	|			|			|	-> LED[Variable1]=Variable2
.equ L_LEDVAR=29
;----------------------------------------------------------------------------------------------------------------
; 30	| Variable1	| Variable2	|			|			|	-> Variable2 = LED[Variable1]
.equ L_VARLED=30
;----------------------------------------------------------------------------------------------------------------
; 31	| Variable	| Wert		| "+-*/ao"	|			|	-> Variable wird "immediate" gerechnet mit Wert
;		|			|			|			|			|		(Plus, Minus, Mal, Geteilt,
;		|			|			|			|			|		AND OR)
.equ L_VARIMATH=31
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

.equ delx=0x8
.equ length=48

pr_clearscreen:
	.db 0,2,1,84,0,0,0xff,0

pr_setscreen:
	.db 0,2,1,84,0,1,0xff,0

pr_fortest01:
	.db 0,L_FOR,2,1				; Schleife C (Wert egal)

	.db 0,L_SETLED2,1,84,0,0			; Löschen Bildschirm
	.db 0,L_SETLED6,1,24,6,0x01			; Setzen LEDs 1-24 mit 4 gesetzten, 2 gelöschten
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROL,1,24				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROL,1,24				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,48|128,0	; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROR,24,1				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROR,24,1				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,48|128,0	; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_SETLED2,1,84,0,0			; Löschen Bildschirm
	.db 0,L_SETLED6,1,24,6,0x03			; Setzen LEDs 1-24 mit 4 gesetzten, 2 gelöschten
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROL,1,24				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROL,1,24				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,48|128,0	; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROR,24,1				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROR,24,1				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,48|128,0	; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_SETLED2,1,84,0,0			; Löschen Bildschirm
	.db 0,L_SETLED6,1,24,6,0x07			; Setzen LEDs 1-24 mit 4 gesetzten, 2 gelöschten
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROL,1,24				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROL,1,24				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,48|128,0	; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROR,24,1				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROR,24,1				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,48|128,0	; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_SETLED2,1,84,0,0			; Löschen Bildschirm
	.db 0,L_SETLED6,1,24,6,0x0f			; Setzen LEDs 1-24 mit 4 gesetzten, 2 gelöschten
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROL,1,24				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROL,1,24				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,48|128,0	; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROR,24,1				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROR,24,1				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,48|128,0	; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_SETLED2,1,84,0,0			; Löschen Bildschirm
	.db 0,L_SETLED6,1,24,6,0x1f			; Setzen LEDs 1-24 mit 4 gesetzten, 2 gelöschten
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROL,1,24				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROL,1,24				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,48|128,0	; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROR,24,1				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,25,0		; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B

	.db 0,L_FOR,1,length				; For-Schleife B=50

	.db 0,L_WAIT,delx,0				; Wait 0x10
	.db 0,L_ROR,24,1				; Rollen 1-24
	.db 0,L_LEDCOPY1,1,24,48|128,0	; Kopieren der LEDs nach 25
	.db 0,L_DECVAR,1,0				; Decrement B

	.db 0,L_NEXT1,1,0				; BRNE For-B


	.db 0,L_NEXT2					; Ewig-Schleifen-Ende
	.db 0xff,0					; ENDE
