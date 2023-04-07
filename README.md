# OpenLEDs
Open Assembler-Code for flashing light with AtMega 32

Uses an Atmel Atmega 32 chip. The amount of LEDS is 88 LEDS now as maximum, but can be more by simple changes in hardware and software.
The Atmel-chip has 8 more signals to drive more LEDs, so a maximum of 152 will be possible.
Port A is used to drive 8 LEDs as a row, 11 lines are used to light 11 columns.

Port C bits 6 and 7 are used for two switches. They are used to select intensity of LEDs in this version, but can be used for more functions my altering the code.
Port D is ot used an can be used to drive more LEDs, as described above.

The Colums are connected to Port B (bit 0 to 4) and Port C (Port 0 to 5)
The processor is used with an 4MHz internal clock-generator.

There is a sort of programming-language for creating own flashing-programs.
The program is located in the program-flash, it may be moved to Data-RAM or more flexible things.
Programs from 01 to 25 are realized, programs 26 to 30 is work in progress.

Definition of programs is included in sourcetext.
