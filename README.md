# OpenLEDs
Open Assembler-Code for flashing light with AtMega 32

Uses an Atmel Atmega 32 chip. The amount of LEDS is 88 LEDS now as maximum, but can be more by simple changes in hardware and software.
The Atmel-chip has 8 more signals to drive more LEDs, so a maximum of 152 will be possible.
Port A is used to drive 8 LEDs as a row, 11 lines are used to light 11 columns.

Port C bits 6 and 7 are used for two switches. They are used to select intensity of LEDs in this version, but can be used for more functions my altering the code.
Port D is ot used an can be used to drive more LEDs, as described above.

The Colums are connected to Port B (bit 0 to 4) and Port C (Port 0 to 5)
The processor is used with an 8MHz internal clock-generator.
Caution: Disable the JTAG-Mode (by writing the fuse-flag) to be able to use all of Port-C.

The internal LED-programming-language is almost finished. It has many modes of flashing light, 16 variables
and ability to use for-loops for the creation of own lightng-gizmos.

The version here is created to drive a Heart-Gizmo with 84 LEDS. It has a big LED-program showing different lighting
programs on the heart.

The heart itself can be found at PCB-Way. You can order a PCB here.
One Atmel AtMega32 DIP-Case 40 Pin, 8 resistors 300 Ohm, a USB_B-plug and two 5-pin-connectors are needed. You can add two pushbuttons, they are for dimming at the time.

https://www.pcbway.com/project/shareproject/LED_Herz_LED_Heart_8a333d0c.html
