; ***************************************************************************
; **                                                                       **
; **          M e c h a n i z e d   A n d r o i d   F i g u r e            **
; **                                                                       **
; ***************************************************************************
; **           Original by Carsten Avenhaus, modified by d3kod             **
; ***************************************************************************
 
; +-------------------------------------------------------------------------+
; | This work is licensed under: Creative Commons                           |
; | Attribution-NonCommercial-ShareAlike 3.0 Unported License               |
; | http://creativecommons.org/licenses/by-nc-sa/3.0                        |
; +-------------------------------------------------------------------------+
 
; +-------------------------------------------------------------------------+
; |                                                                   
; |   History:                                                       
; |  ----------                                                       
; |                                                                   
; |  2010-10-24 Start (Carsten Avenhaus)                                           
; |  2011-02-12 Cleanup and better comments (Carsten Avenhaus)                                           
; |  2012-04-27 Added mute/unmute button support (d3kod)                                                                 
; +-------------------------------------------------------------------------+
 
.include "tn44Adef.inc"
       
 
; ***************************************************************************
; *
; * Define Constants 
; *
; ***************************************************************************
 
; AVR main clock: 8 MHz
.equ    CPU_CLOCK = (8000000)
 
; Timer0: 25.6 kHz reload value
.equ    TIMER0_RELOAD = 256-(CPU_CLOCK/8/25600)+1
 
 
;----------------------------------------------------------------------------
; I/O Port A pin configuration
;----------------------------------------------------------------------------
; Port A has a microphone, piezo and 3 body LEDs.
; The LEDs are low active

.equ PORT_A_OUTPUTS = 0x6E      ; Port A input / output PIN configuration
.equ PORT_A_DATA    = 0x6E      ; Port A output / pullup configuration
 
.equ MIC        = 0             ; Sound in
.equ LED1       = 1             ; red body LED
.equ LED2       = 2             ; green body LED
.equ LED3       = 3             ; yellow body LED
.equ SWITCH		= 5				; Switch that mutes/unmutes
.equ PIEZO      = 6             ; Sound out

.equ BODY_MASK  = (1<<LED1) | (1<<LED2) | (1<<LED3)
 
 
;----------------------------------------------------------------------------
; I/O Port B pin configuration
;----------------------------------------------------------------------------
; Port B has 2 eye LEDs. They are all low active
 
.equ PORT_B_OUTPUTS = 0x07          ; Port B input / output PIN configuration
.equ PORT_B_DATA    = 0xFF & ~0x04  ; Port B output / pullup configuration
 
.equ LEFT_EYE   = 0                 ; blue LED
.equ RIGHT_EYE  = 1                 ; blue LED
.equ SERVO      = 2                 ; Servo that moves the head
.equ EYE_MASK   = (1<<LEFT_EYE) | (1<<RIGHT_EYE)
 
;----------------------------------------------------------------------------
; Morse Code
;----------------------------------------------------------------------------
; http://en.wikipedia.org/wiki/Morse_code
 
; Morse Encoding
.equ MORSE_END   = 0b00000000   ; End of Morse Code Character
.equ MORSE_SHORT = 0b01000000   ; Signal Long
.equ MORSE_LONG  = 0b10000000   ; Signal Short
.equ MORSE_SPACE = 0b11000000   ; Long Pause
.equ MORSE_MASK  = 0b11000000   ; Two bit bit mask for Morse encoding
 
; Morse Timing in units
.equ MORSE_TIME_SHORT = 1       ; Short Signal
.equ MORSE_TIME_LONG = 3        ; Long Signal
.equ MORSE_TIME_GAP = 1         ; Pause between signals
.equ MORSE_TIME_LETTER_GAP = 3  ; Pause between letters
.equ MORSE_TIME_WORD_GAP = 7    ; Pause between words
 
; Morse Speed
.equ LED_TIME = 5               ; Morse speed with sound
.equ LED_TIME2 = 10             ; Morse speed without sound
 
 
;----------------------------------------------------------------------------
; Servo degree / PWM timing translation
;----------------------------------------------------------------------------
; Make sure these values match the servo that is actually used!
; If the PWM tries to move the servo past it's maximum positions, it may get
; damaged in the long run!
 
.equ SERVO_0   = 14     ;   0 deg = 0.5ms = 32.0
.equ SERVO_45  = 25     ;  45 deg
.equ SERVO_90  = 38     ;  90 deg = 1.50ms = 38.4
.equ SERVO_135 = 47     ; 135 deg
.equ SERVO_180 = 63     ; 180 deg = 2.5ms = 44.8
 
 
;----------------------------------------------------------------------------
; Sound detector
;----------------------------------------------------------------------------
; The sound detector checks if the sound level is above a certain threshold
; and then checks if has been over that threshold for a certain amount of
; time. This helps to filter out potential noise.
 
.equ SOUND_TRIGGER = 35 ; Sound level threshold
.equ SOUND_FILTER  = 10 ; Sound duration threshold
 
 
; ***************************************************************************
; *
; * Define CPU registers (R0 to R25 are free to use)
; *
; ***************************************************************************
; R0 to 15 can not be used with immediate data instructions like LDI, ANDI  
 
.def    TABLE_DATA  = R0    ; LPM loads data from ROM here
.def    PWM_COUNTER = R1    ; PWM Counter 
.def    PWM_DATA_0  = R2    ; PWM data for LED 1 (brightness) - PB0
.def    PWM_DATA_1  = R3    ; PWM data for LED 2 (brightness) - PB1
.def    PWM_DATA_2  = R4    ; PWM data for SERVO              - PB2
.def    PWM_DATA_3  = R5    ; PWM data for LED 3 (brightness) - PA1
.def    PWM_DATA_4  = R6    ; PWM data for LED 4 (brightness) - PA2
.def    PWM_DATA_5  = R7    ; PWM data for LED 5 (brightness) - PA3
 
.def    TALK        = R13   ; Indicates if the ADC interrupt detected noise
.def    SOUND_COUNT = R14   ; ADC noise filter
.def    SERVO_MASK  = R15   ; Enable/disable SERVO pulse
 
.def    RND1        = R16   ; Random Number Generator high
.def    RND2        = R17   ; Random Number Generator low
 
.def    TMP1        = R18   ; Temporary data
.def    TMP2        = R19   ; Temporary data
.def    TMP3        = R20   ; Temporary data
.def    LOOPS       = R21   ; Loop counter
 
.def    PWM_DIVIDER = R22   ; Divider counter for 50Hz servo PWM
.def    INT_SREG    = R23   ; interrupts save the status register here
.def    INT_TMP     = R24   ; Temporary data used in interrupts
.def    INT_COUNT   = R25   ; 100Hz counter

.equ	STATE		= GPIOR2	; register to hold the state (mute flag and button pressed flag)
.equ	S_MUTE		= GPIOR20	; mute flag
.equ	S_BUTTON	= GPIOR21	; button pressed flag
; ***************************************************************************
; *
; * Reset and interrupt vectors 
; *
; ***************************************************************************
 
.cseg                       ; Start code segment 
.org    $0000               ; The vector table starts at address 0
 
; ATtiny44A vector table
   RJMP START              ; Reset Handler
   RETI                    ; INT0 External Interrupt Request 0
   RJMP PROCESS_SWITCH     ; PCINT0 Pin Change Interrupt Request 0
   RETI                    ; PCINT1 Pin Change Interrupt Request 1
   RETI                    ; WDT Watchdog Time-out
   RETI                    ; TIM1_CAPT Timer/Counter1 Capture Event
   RETI                    ; TIM1_COMPA Timer/Counter1 Compare Match A
   RETI                    ; TIM1_COMPB Timer/Counter1 Compare Match B
   RETI                    ; TIM1_OVF Timer/Counter1 Overflow
   RETI                    ; TIM0_COMPA Timer/Counter0 Compare Match A
   RETI                    ; TIM0_COMPB Timer/Counter0 Compare Match B
   RJMP TIMER0_OVERFLOW    ; TIM0_OVF Timer/Counter0 Overflow
   RETI                    ; ANA_COMP Analog Comparator
   RJMP ADC_IRQ            ; ADC Conversion Complete
   RETI                    ; EE_RDY EEPROM Ready
   RETI                    ; USI_STR USI START
   RETI                    ; USI_OVF USI Overflow
 
 
; ***************************************************************************
; *                                                                         
; * Main Program                                                           
; *                                                                         
; ***************************************************************************
 
START:  
 
;----------------------------------------------------------------------------
; Setup stack pointer to end of RAM
;----------------------------------------------------------------------------
   LDI TMP1,high(RAMEND)   ; Load high byte of address 
   OUT SPH,TMP1            ; Set high byte of stack pointer
   LDI TMP1,low(RAMEND)    ; Load low byte of address
   OUT SPL,TMP1            ; Set low byte of stack pointer
 
;----------------------------------------------------------------------------
; Configure I/O Port-A
;----------------------------------------------------------------------------
   LDI     TMP1, PORT_A_DATA       ; Load pin data
   OUT     PORTA, TMP1             ; Set I/O pin logic levels 
   LDI     TMP1, PORT_A_OUTPUTS    ; Load I/O data
   OUT     DDRA, TMP1              ; Set pin I/O types
 
 
;----------------------------------------------------------------------------
; Configure I/O Port-B
;----------------------------------------------------------------------------
   LDI     TMP1, PORT_B_DATA       ; Load pin data
   OUT     PORTB, TMP1             ; Set I/O pin logic levels 
   LDI     TMP1, PORT_B_OUTPUTS    ; Load I/O data
   OUT     DDRB, TMP1              ; Set pin I/O types
 
 
;----------------------------------------------------------------------------
; Set System Clock Prescaler to 1
;----------------------------------------------------------------------------
 
   LDI TMP1, 1<<CLKPCE     ; Clock Prescaler Change Enable
   OUT CLKPR, TMP1         ; Enable prescaler change
   LDI TMP1, 0             ; CLKPS bits for prescale by 1
   OUT CLKPR, TMP1         ; Set Prescaler
 
 
;----------------------------------------------------------------------------
; Initialize Timer0 (8 bit)                                                         
;----------------------------------------------------------------------------
; Timer 0 will be our time base. We set the prescaler to 8
; so it will be clocked at 1MHz with a 8MHz CPU clock.
; 8 bit would then overflow at ca. 39 kHz but in the interrupt
; we reset the counter value every time to get interrupts at a
; 25.6 kHz rate
 
   LDI TMP1, 2             ; Timer presacaler: CLK/8
   OUT TCCR0B, TMP1        ; Start Timer 0
   LDI TMP1, 1<<TOIE0      ; Timer 0 overflow interrupt enable bit
   OUT TIMSK0, TMP1        ; Enable overflow timer interrupt
 
 
;----------------------------------------------------------------------------
; Initialize Timer1 (16 bit)                                                         
;----------------------------------------------------------------------------
; Timer1 generates the (500Hz) signal for the piezo
; It runs at the speed of the main clock (8Mhz).
; Whenever the counter reaches the compare value of 8000 the output pin is
; toggled, generating a 1kHz/2 = 500Hz signal.
 
   LDI TMP1, high(8000)    ; Load high byte of 8000
   OUT OCR1AH, TMP1        ; Output Compare Register 1 A High
   LDI TMP1, low(8000)     ; Load low byte of 8000
   OUT OCR1AL, TMP1        ; Output Compare Register 1 A Low
 
   LDI TMP1, (1<<COM1A0)   ; Toggle OC1A Pin on match
   OUT TCCR1A, TMP1        ; Timer/Counter Control Register A
   LDI TMP1, 0             ; Turn off sound
   OUT TCCR1B, TMP1        ; Timer/Counter Control Register B
 
 
 
;----------------------------------------------------------------------------
; Seed 16-bit Random Generator
;----------------------------------------------------------------------------
 
   LDI RND1, 23            ; Random Seed high byte
   LDI RND2, 42            ; Random Seed low byte
 
 
;----------------------------------------------------------------------------
; Initialize Analog to Digital Converter
;----------------------------------------------------------------------------
 
   LDI TMP1, (1<<REFS1)    ; ADC Reference: 1.1V / Channel 0
   OUT ADMUX, TMP1         ; ADC Multiplexer Selection Register
 
   LDI TMP1, (1<<ADLAR)    ; Left adjust ADC result
   OUT ADCSRB, TMP1        ; ADC Control and Status Register A
 
   ; Switch AD conversion on, start conversion, 
   ; auto trigger, divider rate = 16
   LDI TMP1, (1<<ADEN) | (1<<ADSC) | (1<<ADIE) | (1<<ADATE) | (1<<ADPS2)
   OUT ADCSRA, TMP1        ; ADC Control and Status Register A

   ; enable pin8 (PCINT5) interrupt
   LDI TMP1, 1<<PCIE0
   OUT GIMSK, TMP1
   LDI TMP1, 1<<PCINT5 
   OUT PCMSK0, TMP1

   ; clear button pressed flag
   CBI STATE, S_BUTTON 
   ; load initial mute flag value from eeeprom
   RCALL  EEPROM_read

;----------------------------------------------------------------------------
; Eye LEDs on, body LEDs off
;----------------------------------------------------------------------------
   SET                         ; Turn LED PWM off 
   IN  TMP1, PORTB             ; Read current servo / LED status
   ANDI    TMP1, ~EYE_MASK     ; Turn eye LEDs on
   OUT PORTB, TMP1             ; Turn eye LEDs on, keep servo PWM the same
   LDI TMP3, PORT_A_DATA       ; Turn all body LEDs off
   OUT PORTA, TMP3             ; Set I/O pin logic levels 
 
 
;----------------------------------------------------------------------------
; Enable all interrupts                                                         
;----------------------------------------------------------------------------
 
   SEI                     ; Global Interrupt Enable 
 
 
;----------------------------------------------------------------------------
; Clear Sound Detector
;----------------------------------------------------------------------------
   CLR SERVO_MASK      ; Turn servo PWM off
 
   LDI INT_COUNT, 100  ; Load 10ms wait counter
   RCALL WAIT          ; Wait for 1 second
 
   CLR TALK            ; Clear flag that indicates sound (set in ADC interrupt)
   CLR SOUND_COUNT     ; Clear noise filter counter
 
 
;----------------------------------------------------------------------------
; Main Loop                                                         
;----------------------------------------------------------------------------
MAIN:
   TST TALK                    ; Check if sound was detected
   BREQ NO_TALK                ; If not continue
   RCALL MOVE_HEAD             ; Else wake up and move the head
NO_TALK:
 
 
; ---------------------------------------------------------------------------
; * Fade all LEDs with each LED at a different brightness
; ---------------------------------------------------------------------------
 
   CLT                         ; Turn LED PWM on 
   LDI ZH, high(SQR_TABLE*2)   ; Load pointer to LED brightness table
   LDI LOOPS, 25               ; Number of loops for LED fading
   LDI TMP1, 0                 ; LED start brightness
LOOP:
   INC TMP1                    ; Change LED brightness value
   MOV ZL, TMP1                ; Load table pointer low
   LPM PWM_DATA_0, Z           ; Load Table Data into LED PWM register
;   SUBI ZL, -0x60              ; Add some brightness for the next LED
   LPM PWM_DATA_1, Z           ; Load Table Data into LED PWM register
   SUBI ZL, -0x60              ; Add some brightness for the next LED
   LPM PWM_DATA_3, Z           ; Load Table Data into LED PWM register
   SUBI ZL, -0x60              ; Add some brightness for the next LED
   LPM PWM_DATA_4, Z           ; Load Table Data into LED PWM register
   SUBI ZL, -0x60              ; Add some brightness for the next LED
   LPM PWM_DATA_5, Z           ; Load Table Data into LED PWM register
   LDI INT_COUNT, 1            ; Load 10ms wait counter
   RCALL   WAIT                ; Wait 10ms

LOOP_REST:
   TST TALK                    ; Check if there was sound
   BRNE MAIN                   ; If there was sound abort and wake up
   TST TMP1                    ; Check if a single fade cycle is complete
   BRNE LOOP                   ; If not continue fading
   DEC LOOPS                   ; Decrement loop counter
   BRNE LOOP                   ; Loop if fading not done
 
; ---------------------------------------------------------------------------
; * Blink all LEDs, randomly
; ---------------------------------------------------------------------------
 
   SET                         ; Turn LED PWM off 
   LDI LOOPS, 60               ; Show 60 random LED patterns
BLINK_LOOP:
   RCALL RANDOMIZE             ; Get new random number in TMP1
   ANDI TMP1, ~(1<<SERVO)      ; Do not change servo
   OUT PORTB, TMP1             ; Output random eye LED pattern
   RCALL RANDOMIZE             ; Get new random number in TMP1
   ORI TMP1, (1<<PIEZO) | (1<<SWITCH)	; Do not change PIEZO, do not trigger PCINT5 interrupt
   ANDI TMP1, PORT_A_DATA      ; Only change LED pins
   OUT PORTA, TMP1             ; Output random body LED pattern
   LDI INT_COUNT, 25           ; Load 10ms wait counter
   RCALL WAIT                  ; Wait 250ms
   TST TALK                    ; Check if there was sound
   BRNE MAIN                   ; If there was sound abort and wake up
   DEC LOOPS                   ; Decrement counter
   BRNE BLINK_LOOP             ; Loop if not done
 
   IN  TMP1, PORTB             ; Read current servo / LED status
   ANDI    TMP1, ~EYE_MASK     ; Turn eye LEDs on
   OUT PORTB, TMP1             ; Turn eye LEDs on, keep servo PWM the same
   LDI TMP3, PORT_A_DATA       ; Turn all body LEDs off
   OUT PORTA, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, 100          ; Load 10ms wait counter
   RCALL WAIT                  ; Wait 1 second
 
 
; ---------------------------------------------------------------------------
; * Fade all LEDs with all LEDs at the same brightness
; ---------------------------------------------------------------------------
 
   CLT                         ; Turn LED PWM on 
   LDI ZH, high(SQR_TABLE*2)   ; Load pointer to LED brightness table
   LDI LOOPS, 15               ; Number of loops for LED fading
   LDI TMP1, 0                 ; LED start brightness
LOOP1:
   INC TMP1                    ; Change LED brightness value
   MOV ZL, TMP1                ; Load table pointer low
   LPM PWM_DATA_0, Z           ; Load Table Data into LED PWM register
   LPM PWM_DATA_1, Z           ; Load Table Data into LED PWM register
   LPM PWM_DATA_3, Z           ; Load Table Data into LED PWM register
   LPM PWM_DATA_4, Z           ; Load Table Data into LED PWM register
   LPM PWM_DATA_5, Z           ; Load Table Data into LED PWM register
   LDI INT_COUNT, 2            ; Load 10ms wait counter
   RCALL WAIT                  ; Wait 20ms
   TST TALK                    ; Check if there was sound
   BRNE DONE                   ; If there was sound abort and wake up
   TST TMP1                    ; Check if a single fade cycle is complete
   BRNE LOOP1                  ; If not continue fading
   DEC LOOPS                   ; Decrement loop counter
   BRNE LOOP1                  ; Loop if fading not done
 
 
; ---------------------------------------------------------------------------
; * Morse all LEDs silently
; ---------------------------------------------------------------------------
 
   SET                         ; Turn LED PWM off
   LDI ZH, high(MORSE_TEXTS*2) ; Load address of message pointers table high
   LDI ZL, low(MORSE_TEXTS*2)  ; Load address of message pointers table low
   RCALL RANDOMIZE             ; Get new random number in TMP1
   ANDI TMP1, 0x0E             ; Bit mask for random number from (0 to 7)*2
   ADD ZL, TMP1                ; Add random number to table pointer
   LPM YL, Z+                  ; Load address of one of the messages high
   LPM YH, Z+                  ; Load address of one of the messages low
   MOV ZH, YH                  ; Move message pointer to Z-register high
   MOV ZL, YL                  ; Move message pointer to Z-register low
MORSE_LOOP1:
   LPM TMP1, Z+                ; Load Morse character
   TST TMP1                    ; Check if character is 0
   BREQ MORSE_DONE1            ; If character is 0, Morse message is done
   RCALL GET_MORSE             ; Get Morse Code for this character
   RCALL MORSE_ALL_LED         ; Send the Morse Code by blinking all LEDs
   TST TALK                    ; Check if there was sound
   BRNE DONE                   ; If there was sound abort and wake up
   RJMP MORSE_LOOP1            ; Loop to send next Morse character
MORSE_DONE1:
   IN  TMP1, PORTB             ; Read current servo / LED status
   ORI TMP1, EYE_MASK          ; Make sure eye LEDs are off
   OUT PORTB, TMP1             ; Turn eye LEDs off, keep servo PWM the same
   LDI TMP1, PORT_A_DATA       ; Load PORT-A idle data
   OUT PORTA, TMP1             ; Turn body LEDs off
   LDI INT_COUNT, 100          ; Load 10ms wait counter
   RCALL WAIT                  ; Wait 1 second
 
DONE:
   RJMP MAIN                   ; Loop back to the beginning
 
; ***************************************************************************
; * Move head and morse eye LEDs with beep sound
; ***************************************************************************
; Wake up Android, move the head and send a random Morse Code message.
; One eye shows long, the other shows short.
 
MOVE_HEAD:
   SET                         ; Turn LED PWM off
   IN  TMP1, PORTB             ; Read current servo / LED status
   ANDI TMP1, ~EYE_MASK        ; Turn eye LEDs on
   OUT PORTB, TMP1             ; Turn eye LEDs on, keep servo PWM the same
   LDI TMP3, PORT_A_DATA       ; Turn all body LEDs off
   OUT PORTA, TMP3             ; Set I/O pin logic levels 
 
   LDI TMP1, (1<<SERVO)        ; Load servo bit mask
   MOV SERVO_MASK, TMP1        ; Enable servo PWM
 
   LDI TMP1, SERVO_0           ; Load servo 0 degree head right PWM value
   MOV PWM_DATA_2, TMP1        ; Move the value into the servo PWM register
   LDI INT_COUNT, 100          ; Load 10ms wait counter
   RCALL WAIT                  ; Wait for the head to move
   LDI TMP1, SERVO_180         ; Load servo 180 degree head left PWM value
   MOV PWM_DATA_2, TMP1        ; Move the value into the servo PWM register
   LDI INT_COUNT, 100          ; Load 10ms wait counter
   RCALL WAIT                  ; Wait for the head to move
   LDI TMP1, SERVO_90          ; Load servo 90 degree head center PWM value
   MOV PWM_DATA_2, TMP1        ; Move the value into the servo PWM register
   LDI INT_COUNT, 50           ; Load 10ms wait counter
   RCALL WAIT                  ; Wait for the head to move
 
   IN  TMP1, PORTB             ; Read current servo / LED status
   ORI TMP1, EYE_MASK          ; Make sure eye LEDs are off
   OUT PORTB, TMP1             ; Turn eye LEDs off, keep servo PWM the same
 
   LDI ZH, high(MORSE_TEXTS*2) ; Load address of message pointers table high
   LDI ZL, low(MORSE_TEXTS*2)  ; Load address of message pointers table low
   RCALL RANDOMIZE             ; Get new random number in TMP1
   ANDI TMP1, 0x0E             ; Bit mask for random number from (0 to 7)*2
   ADD ZL, TMP1                ; Add random number to table pointer
   LPM YL, Z+                  ; Load address of one of the messages high
   LPM YH, Z+                  ; Load address of one of the messages low
   MOV ZH, YH                  ; Move message pointer to Z-register high
   MOV ZL, YL                  ; Move message pointer to Z-register low
   
   SBIS STATE, S_BUTTON        ; toggle mute flag if button has been pressed
   RJMP MOVE_HEAD_REST		   ; leave mute flag unchanged otherwise

TOGGLE_MUTE:
   SBIS STATE, S_MUTE          ; check if mute flag is set
   RJMP TOGGLE_ON			   ; if not, set it

   CBI STATE, S_MUTE		   ; if mute flag is set, clear it
   RJMP TOGGLE_FINISH

TOGGLE_ON:
   SBI STATE, S_MUTE           ; set the mute flag

TOGGLE_FINISH:
   RCALL EEPROM_write          ; write the new value the mute flag to the persistent memory
   CBI STATE, S_BUTTON         ; clear the button state as we had processed the press

MOVE_HEAD_REST:
   SBIC STATE, S_MUTE          ; if mute is cleared, morse loop normally
   RJMP MORSE_MUTE_LOOP        ; otherwise, morse loop muted

MORSE_LOOP:
   LPM TMP1, Z+                ; Load Morse character
   TST TMP1                    ; Check if character is 0
   BREQ MORSE_DONE             ; If character is 0, Morse message is done
   RCALL GET_MORSE             ; Get Morse Code for this character
   RCALL MORSE_EYE_LEDS        ; Send the Morse Code by blinking eye LEDs
   RJMP MORSE_LOOP             ; Loop to send next Morse character

MORSE_MUTE_LOOP:
   LPM TMP1, Z+                ; Load Morse character
   TST TMP1                    ; Check if character is 0
   BREQ MORSE_DONE             ; If character is 0, Morse message is done
   RCALL GET_MORSE             ; Get Morse Code for this character
   RCALL MORSE_MUTE_EYE_LEDS   ; Send the Morse Code by blinking eye LEDs (muted)
   RJMP MORSE_MUTE_LOOP        ; Loop (muted) to send next Morse character

MORSE_DONE:
 
   LDI INT_COUNT, 200          ; Load 10ms wait counter
   RCALL WAIT                  ; Wait 2 seconds
   
   IN TMP1, PORTB              ; Read current servo / LED status
   ANDI TMP1, ~EYE_MASK        ; Make sure eye LEDs are on
   OUT PORTB, TMP1             ; Turn eye LEDs on, keep servo PWM the same
 
   LDI TMP1, SERVO_135         ; Load servo 135 degree head left PWM value
   MOV PWM_DATA_2, TMP1        ; Move the value into the servo PWM register
   LDI INT_COUNT, 50           ; Load 10ms wait counter
   RCALL WAIT                  ; Wait for the head to move
   LDI TMP1, SERVO_45          ; Load servo 45 degree head right PWM value
   MOV PWM_DATA_2, TMP1        ; Move the value into the servo PWM register
   LDI INT_COUNT, 100          ; Load 10ms wait counter
   RCALL WAIT                  ; Wait for the head to move
   LDI TMP1, SERVO_90          ; Load servo 90 degree head center PWM value
   MOV PWM_DATA_2, TMP1        ; Move the value into the servo PWM register
   LDI INT_COUNT, 150          ; Load 10ms wait counter
   RCALL WAIT                  ; Wait for the head to move
   IN  TMP1, PORTB             ; Read current servo / LED status
   ORI TMP1, EYE_MASK          ; Make sure eye LEDs are off
   OUT PORTB, TMP1             ; Turn eye LEDs off, keep servo PWM the same
 
WAIT_SERVO:
   SBIC PORTB, SERVO           ; Check if servo PIN is inactive
   RJMP WAIT_SERVO             ; Wait until servo PIN is inactive
   CLR SERVO_MASK              ; Disable servo PWM
 
   LDI INT_COUNT, 150          ; Load 10ms wait counter
   RCALL WAIT                  ; Wait 1.5 seconds
 
   CLR SOUND_COUNT             ; Clear sound detector filter
   CLR TALK                    ; Clear sound detector flag
   RET                         ; Back to where we came from
 
 
; ***************************************************************************
; * Get Morse Code for a Symbol
; ***************************************************************************
; Input: TMP1 ASCII character
; Output: TMP1, TMP2 Morse Code (2-bit encoding)
; Modified: X-pointer
 
GET_MORSE:
   MOV XH, ZH                  ; Backup Z-pointer to X-pointer high
   MOV XL, ZL                  ; Backup Z-pointer to X-pointer low
   MOV ZL, TMP1                ; Copy ASCII character to Z-pointer
   SUBI ZL, ' '                ; Calculate Morse Code symbol number
   LSL ZL                      ; Multiply by 2 (2 bytes per Morse symbol)
   LDI ZH, high(MORSE_TABLE*2) ; Add start of table high
   ORI ZL, low(MORSE_TABLE*2)  ; Add start of table low
   LPM TMP1, Z+                ; Load first Morse byte
   LPM TMP2, Z+                ; Load second Morse byte
   LDI TMP2,4                  ; 4 signals per byte
   MOV ZH, XH                  ; Restore Z-pointer from X-pointer high
   MOV ZL, XL                  ; Restore Z-pointer from X-pointer low
   RET                         ; Return to where we came from
 
 
; ***************************************************************************
; * Morse two eyes with beep
; ***************************************************************************
; Input: TMP1, TMP2 Morse Code (2-bit encoding)
 
MORSE_EYE_LEDS:
   MOV TMP3, TMP1              ; Copy Morse Code high byte to TMP3
   ANDI TMP3, MORSE_MASK       ; Mask out the next two bit Morse signal
   BRNE MORSE_LED_DO           ; Continue if this is not the letter end
   LDI INT_COUNT, (MORSE_TIME_LETTER_GAP - MORSE_TIME_GAP) * LED_TIME
   RCALL WAIT                  ; Pause for inter letter gap
   RET                         ; Morse character is done. Return.
 
MORSE_LED_DO:
   ; Shift the 2-byte Morse character 2 bit to the left to get the
   ; next Morse signal into place
   LSL TMP2                    ; 16 bit shift low
   ROL TMP1                    ; 16 bit shift high
   LSL TMP2                    ; 16 bit shift low
   ROL TMP1                    ; 16 bit shift high
 
   CPI TMP3, MORSE_SHORT       ; Check if this is a short Morse signal
   BRNE MORSE_LED_NOT_SHORT    ; Continue if it is not
   LDI TMP3, (1<<WGM12) | 1    ; Start Timer 1 / Clear it on compare match
   OUT TCCR1B, TMP3            ; Enable piezo sound
   IN  TMP3, PORTB             ; Read current servo / LED status
   ORI TMP3, EYE_MASK          ; Turn both eye LEDs off
   ANDI TMP3, ~(1<<LEFT_EYE)   ; Turn left eye LED on
   OUT PORTB, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, MORSE_TIME_SHORT * LED_TIME
   RCALL WAIT                  ; Wait for short signal time
   LDI TMP3, 0                 ; Stop Timer 1
   OUT TCCR1B, TMP3            ; Disable piezo sound
   IN  TMP3, PORTB             ; Read current servo / LED status
   ORI TMP3, EYE_MASK          ; Turn both eye LEDs off
   OUT PORTB, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, MORSE_TIME_GAP * LED_TIME
   RCALL   WAIT                ; Pause for inter signal gap    
   RJMP MORSE_EYE_LEDS         ; Loop for next Morse signal
MORSE_LED_NOT_SHORT:
 
   CPI TMP3, MORSE_LONG        ; Check if this is a long Morse signal
   BRNE MORSE_LED_NOT_LONG     ; Continue if it is not
   LDI TMP3, (1<<WGM12) | 1    ; Start Timer 1 / Clear it on compare match
   OUT TCCR1B, TMP3            ; Enable piezo sound
   IN  TMP3, PORTB             ; Read current servo / LED status
   ORI TMP3, EYE_MASK          ; Turn both eye LEDs off
   ANDI TMP3, ~(1<<RIGHT_EYE)  ; Turn right eye LED on
   OUT PORTB, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, MORSE_TIME_LONG * LED_TIME
   RCALL WAIT                  ; Wait for long signal time
   LDI TMP3, 0                 ; Stop Timer 1
   OUT TCCR1B, TMP3            ; Disable piezo sound
   IN  TMP3, PORTB             ; Read current servo / LED status
   ORI TMP3, EYE_MASK          ; Turn both eye LEDs off
   OUT PORTB, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, MORSE_TIME_GAP * LED_TIME
   RCALL WAIT                  ; Pause for inter signal gap
   RJMP MORSE_EYE_LEDS         ; Loop for next Morse signal
MORSE_LED_NOT_LONG:
   
   ; Inter word gap
   LDI INT_COUNT, (MORSE_TIME_WORD_GAP - MORSE_TIME_LETTER_GAP - MORSE_TIME_GAP) * LED_TIME
   RCALL WAIT                  ; Pause for inter word gap
   RJMP MORSE_EYE_LEDS         ; Loop for next Morse signal
 
; ***************************************************************************
; * Morse two eyes no beep (d3h hacking)
; ***************************************************************************
; Input: TMP1, TMP2 Morse Code (2-bit encoding)
 
MORSE_MUTE_EYE_LEDS:           ; Same as MORSE_EYE_LEDS but with no sound
   MOV TMP3, TMP1              ; Copy Morse Code high byte to TMP3
   ANDI TMP3, MORSE_MASK       ; Mask out the next two bit Morse signal
   BRNE MORSE_MUTE_LED_DO      ; Continue if this is not the letter end
   LDI INT_COUNT, (MORSE_TIME_LETTER_GAP - MORSE_TIME_GAP) * LED_TIME
   RCALL WAIT                  ; Pause for inter letter gap
   RET                         ; Morse character is done. Return.
 
MORSE_MUTE_LED_DO:		       ; Same as MORSE_LED_DO but with disabled piezo sound
   ; Shift the 2-byte Morse character 2 bit to the left to get the
   ; next Morse signal into place
   LSL TMP2                    ; 16 bit shift low
   ROL TMP1                    ; 16 bit shift high
   LSL TMP2                    ; 16 bit shift low
   ROL TMP1                    ; 16 bit shift high
 
   CPI TMP3, MORSE_SHORT       ; Check if this is a short Morse signal
   BRNE MORSE_MUTE_LED_NOT_SHORT    ; Continue if it is not
   LDI TMP3, (1<<WGM12) | 1    ; Start Timer 1 / Clear it on compare match
   IN  TMP3, PORTB             ; Read current servo / LED status
   ORI TMP3, EYE_MASK          ; Turn both eye LEDs off
   ANDI TMP3, ~(1<<LEFT_EYE)   ; Turn left eye LED on
   OUT PORTB, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, MORSE_TIME_SHORT * LED_TIME
   RCALL WAIT                  ; Wait for short signal time
   LDI TMP3, 0                 ; Stop Timer 1
   IN  TMP3, PORTB             ; Read current servo / LED status
   ORI TMP3, EYE_MASK          ; Turn both eye LEDs off
   OUT PORTB, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, MORSE_TIME_GAP * LED_TIME
   RCALL   WAIT                ; Pause for inter signal gap    
   RJMP MORSE_MUTE_EYE_LEDS    ; Loop for next Morse signal
MORSE_MUTE_LED_NOT_SHORT:

   CPI TMP3, MORSE_LONG        ; Check if this is a long Morse signal
   BRNE MORSE_MUTE_LED_NOT_LONG; Continue if it is not
   LDI TMP3, (1<<WGM12) | 1    ; Start Timer 1 / Clear it on compare match
   IN  TMP3, PORTB             ; Read current servo / LED status
   ORI TMP3, EYE_MASK          ; Turn both eye LEDs off
   ANDI TMP3, ~(1<<RIGHT_EYE)  ; Turn right eye LED on
   OUT PORTB, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, MORSE_TIME_LONG * LED_TIME
   RCALL WAIT                  ; Wait for long signal time
   LDI TMP3, 0                 ; Stop Timer 1
   IN  TMP3, PORTB             ; Read current servo / LED status
   ORI TMP3, EYE_MASK          ; Turn both eye LEDs off
   OUT PORTB, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, MORSE_TIME_GAP * LED_TIME
   RCALL WAIT                  ; Pause for inter signal gap
   RJMP MORSE_MUTE_EYE_LEDS    ; Loop for next Morse signal
MORSE_MUTE_LED_NOT_LONG:
   
   ; Inter word gap
   LDI INT_COUNT, (MORSE_TIME_WORD_GAP - MORSE_TIME_LETTER_GAP - MORSE_TIME_GAP) * LED_TIME
   RCALL WAIT                  ; Pause for inter word gap
   RJMP MORSE_EYE_LEDS         ; Loop for next Morse signal

 
; ***************************************************************************
; * Morse all LEDs, no beep
; ***************************************************************************
; Input: TMP1, TMP2 Morse Code (2-bit encoding)
 
MORSE_ALL_LED:
   MOV TMP3, TMP1              ; Copy Morse Code high byte to TMP3
   ANDI TMP3, MORSE_MASK       ; Mask out the next two bit Morse signal
   BRNE MORSE_ALL_LED_DO       ; Continue if this is not the letter end
   LDI INT_COUNT, (MORSE_TIME_LETTER_GAP - MORSE_TIME_GAP) * LED_TIME2
   RCALL WAIT                  ; Pause for inter letter gap
   RET                         ; Morse character is done. Return.
 
MORSE_ALL_LED_DO:
   ; Shift the 2-byte Morse symbol 2 bit to the left to get the
   ; next Morse signal into place
   LSL TMP2                    ; 16 bit shift low
   ROL TMP1                    ; 16 bit shift high
   LSL TMP2                    ; 16 bit shift low
   ROL TMP1                    ; 16 bit shift high
 
   CPI TMP3, MORSE_SHORT       ; Check if this is a short Morse signal
   BRNE MORSE_ALL_LED_NOT_SHORT; Continue if it is not
   IN  TMP3, PORTB             ; Read current servo / LED status
   ANDI TMP3, ~EYE_MASK        ; Turn both eye LEDs on
   OUT PORTB, TMP3             ; Set I/O pin logic levels 
   LDI TMP3, PORT_A_DATA & ~BODY_MASK ; All body LEDs on
   OUT PORTA, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, MORSE_TIME_SHORT * LED_TIME2
   RCALL WAIT                  ; Wait for short signal time
   IN  TMP3, PORTB             ; Read current servo / LED status
   ORI TMP3, EYE_MASK          ; Turn both eye LEDs off
   OUT PORTB, TMP3             ; Set I/O pin logic levels 
   LDI TMP3, PORT_A_DATA       ; All body LEDs off
   OUT PORTA, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, MORSE_TIME_GAP * LED_TIME2
   RCALL WAIT                  ; Pause for inter signal gap
   RJMP MORSE_ALL_LED          ; Loop for next Morse signal
MORSE_ALL_LED_NOT_SHORT:
 
   CPI TMP3, MORSE_LONG        ; Check if this is a long Morse signal
   BRNE MORSE_ALL_LED_NOT_LONG ; Continue if it is not
   IN  TMP3, PORTB             ; Read current servo / LED status
   ANDI TMP3, ~EYE_MASK        ; Turn both eye LEDs on
   OUT PORTB, TMP3             ; Set I/O pin logic levels 
   LDI TMP3, PORT_A_DATA & ~BODY_MASK ; All body LEDs on
   OUT PORTA, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, MORSE_TIME_LONG * LED_TIME2
   RCALL WAIT                  ; Wait for long signal time
   IN  TMP3, PORTB             ; Read current servo / LED status
   ORI TMP3, EYE_MASK          ; Turn both eye LEDs off
   OUT PORTB, TMP3             ; Set I/O pin logic levels 
   LDI TMP3, PORT_A_DATA       ; All body LEDs off
   OUT PORTA, TMP3             ; Set I/O pin logic levels 
   LDI INT_COUNT, MORSE_TIME_GAP * LED_TIME2
   RCALL WAIT                  ; Pause for inter signal gap
   RJMP MORSE_ALL_LED          ; Loop for next Morse signal
MORSE_ALL_LED_NOT_LONG:
   
   ; Inter word gap
   LDI INT_COUNT, (MORSE_TIME_WORD_GAP - MORSE_TIME_LETTER_GAP - MORSE_TIME_GAP) * LED_TIME2
   RCALL WAIT                  ; Pause for inter word gap
   RJMP MORSE_ALL_LED          ; Loop for next Morse signal
 
 
; ***************************************************************************
; * Pseudo Random Number Generator
; ***************************************************************************
 
RANDOMIZE:
   MOV TMP1, RND1              ; Copy old random data to TMP1
   SWAP TMP1                   ; Swap nibbles
   EOR TMP1, RND1              ; XOR Bit7 and Bit 3
   LSL TMP1                    ; Swap nibbles
   EOR TMP1, RND1              ; XOR Bit7 and Bit 3
   EOR TMP1, RND2              ; XOR Bit7 and Bit 3
   SWAP TMP1                   ; Swap nibbles
   ROL TMP1                    ; Rotate bit 7 into carry
   ROL RND1                    ; Rotate carry into new number
   ROL RND2                    ; Rotate carry into new number
   MOV TMP1, RND2              ; Return new random number in TMP1
   RET                         ; Back to where we came from
 
 
; ***************************************************************************
; * Wait for the 10ms down counter INT_COUNT to reach 0                                                           
; ***************************************************************************
; INT_COUNT is decremented at 100Hz from the Timer 0 interrupt.
 
WAIT:
   SLEEP                       ; Sleep to conserve power
   TST INT_COUNT               ; Has the counter reached 0 yet?
   BRNE WAIT                   ; If not, keep waiting
   RET                         ; Return to where we came from
 
 
; ***************************************************************************
; * Interrupt: Timer 0 Overflow                                                           
; ***************************************************************************
; This interrupt implements the software PWMs for the LEDs and the servo
; and also provides the time base for the WAIT loop.
 
TIMER0_OVERFLOW:
   IN INT_SREG, SREG           ; Save status register
   LDI INT_TMP, TIMER0_RELOAD  ; Load timer 25.6 kHz value
   OUT TCNT0, INT_TMP          ; Reset timer
 
   INC PWM_COUNTER             ; Increment the PWM counter
   BRNE PWM_OK                 ; If it is not zero continue
   DEC INT_COUNT               ; Decrement 100 Hz counter
   INC PWM_DIVIDER             ; Increment servo PWM divider
PWM_OK:
 
   IN INT_TMP, PORTB           ; Read current servo / LED status
   ANDI INT_TMP, ~(1<<SERVO)   ; Turn off servo bit
   
   BRTS NO_DIMM                ; Skip LED PWM stuff if PWM is disabled
 
   LDI INT_TMP, PORT_A_DATA    ; Reset all bits to 1 (LEDs off)
   CP PWM_COUNTER, PWM_DATA_3  ; Compare PWM value with PWM counter 
   BRCC LED3_OFF               ; Keep LED off if lower
   ANDI INT_TMP, ~(1<<LED1)    ; Else turn LED on
LED3_OFF:
   CP PWM_COUNTER, PWM_DATA_4  ; Compare PWM value with PWM counter 
   BRCC LED4_OFF               ; Keep LED off if lower
   ANDI INT_TMP, ~(1<<LED2)    ; Else turn LED on
LED4_OFF:
   CP PWM_COUNTER, PWM_DATA_5  ; Compare PWM value with PWM counter 
   BRCC LED5_OFF               ; Keep LED off if lower
   ANDI INT_TMP, ~(1<<LED3)    ; Else turn LED on
LED5_OFF:
   OUT PORTA, INT_TMP          ; Output LED result to the PORT
 
   LDI INT_TMP, PORT_B_DATA    ; Reset all bits to 1 (LEDs off)
   CP PWM_COUNTER, PWM_DATA_0  ; Compare PWM value with PWM counter 
   BRCC LED0_OFF               ; Keep LED off if lower
   ANDI INT_TMP, ~(1<<LEFT_EYE); Else turn LED on
LED0_OFF:
   CP PWM_COUNTER, PWM_DATA_1  ; Compare PWM value with PWM counter 
   BRCC LED1_OFF               ; Keep LED off if lower
   ANDI INT_TMP, ~(1<<RIGHT_EYE); Else turn LED on
LED1_OFF:
NO_DIMM:
 
   CP PWM_COUNTER, PWM_DATA_2  ; Compare PWM value with PWM counter 
   BRCC PWM2_OFF               ; Keep servo off if lower
   SBRC PWM_DIVIDER, 0         ; Servo PWM is only 50Hz instead of 100Hz
   OR INT_TMP, SERVO_MASK      ; Else activate servo pulse
PWM2_OFF:
 
   OUT PORTB, INT_TMP          ; Output result to the LEDs
 
   OUT SREG, INT_SREG          ; Restore status register
   RETI                        ; Return from interrupt
 
 
; ***************************************************************************
; * Analog to Digital Converter Interrupt Handler
; ***************************************************************************
; This handler may introduce jitter to the PWM interrupt, causing the servo
; to activate and cause noise. For this reason the SERVO PWM is only when 
; the head actually moves. 
; The sound detector checks if the sound level is above a certain threshold
; and then checks if has been over that threshold for a certain amount of
; time.
 
ADC_IRQ:
   IN  INT_SREG, SREG          ; Save status register

   IN INT_TMP, ADCH            ; Read MSB of the AD conversion result
   CPI INT_TMP, SOUND_TRIGGER  ; Check if sound level is above threshold
   BRLO ADC_CLR                ; If not, clear noise filter
 
   INC SOUND_COUNT             ; Increment sound duration counter
   MOV INT_TMP, SOUND_COUNT    ; Load sound duration counter to register
   CPI INT_TMP, SOUND_FILTER   ; Check if sound duration is above threshold
   BRLT ADC_DONE               ; If not, jump to end of interrupt handler
   INC TALK                    ; There was sound. Set sound detector flag
 
ADC_CLR:
   CLR SOUND_COUNT             ; Clear sound duration counter (noise filter)
ADC_DONE:
   OUT SREG, INT_SREG          ; Restore status register
   RETI                        ; Return from interrupt
 
;----------------------------------------------------------------------------
; Some code added for the mute/unmute functionality
;---------------------------------------------------------------------------- 

EEPROM_write:                  
	; Write the value of the mute flag to persistent memory
	; Code modified from the ATtiny44A Manual, page 18
	; Wait for completion of previous write
	SBIC EECR, EEPE
	RJMP EEPROM_write 
	; Set Programming mode
	LDI TMP1, (0<<EEPM1)|(0<<EEPM0)
	OUT EECR, TMP1
	; Set up address 0 in address registers
	LDI TMP1, 0
	OUT EEARL, TMP1

	SBIS STATE, S_MUTE          ; Check if mute flag is set
	LDI TMP1, 0                 ; If not, make the TMP1 register to 0
	SBIC STATE, S_MUTE          ; Check if mute flag is clear
	LDI TMP1, 1                 ; If not, make the TMP1 register to 1
	; Write data (TMP1) to data register
	OUT EEDR, TMP1
	; Write logical one to EEMPE
	SBI EECR, EEMPE
	; Start eeprom write by setting EEPE
	SBI EECR, EEPE
	RET

EEPROM_read:
    ; Read the value of the mute flag from persistent memory
	; Code modified from the ATtiny44A Manual, page 19
	; Wait for completion of previous write
	SBIC EECR, EEPE
	RJMP EEPROM_read
	; Set up address 0 in address registers
	LDI TMP1, 0
	OUT EEARL, TMP1
	; Start eeprom read by writing EERE
	SBI EECR, EERE
	; Read data from data register
	IN TMP1, EEDR                ; Read the EEDR register in TMP1
	TST TMP1                     ; Check if TMP1 is equal to 0
	BREQ CLEAR_MUTE_RET          ; If it is, clear the mute flag
	SBI STATE, S_MUTE            ; If it is not, set the mute flag
	RET

CLEAR_MUTE_RET:
	CBI STATE, S_MUTE            ; Clear the mute flag
	RET

PROCESS_SWITCH:
	; Executed when voltage change interrupt occured on pin8 (PCINT5)
	; This has happened because the mute/unmute switch has been pressed
	SBI STATE, S_BUTTON          ; Set the button pressed flag
	RETI

; ***************************************************************************
; * Text Data
; ***************************************************************************
 
MORSE_TEXT_1:
.db "GOOGLE ANDROID ROCKS!", 0
   
MORSE_TEXT_2:
.db "ICE CREAM SANDWITCH TASTES GOOD. ", 0

MORSE_TEXT_3:
.db "I AM COMPLETELY OPERATIONAL AND ALL MY CIRCUITS ARE FUNCTIONING PERFECTLY. ", 0
 
MORSE_TEXT_4:
.db "WHAT IS ALL THIS NOISE? I AM TRYING TO CONCENTRATE!", 0

MORSE_TEXT_5:
.db "HONEYCOMB IS SO SWEET! ", 0

MORSE_TEXT_6:
.db "DID I HEAR SOMETHING?", 0

MORSE_TEXT_7:
.db "I WAS MADE BY CARSTEN AVENHAUS.", 0

MORSE_TEXT_8:
.db "I WAS UPDATED BY D3KOD.", 0
 
; ***************************************************************************
; * Table of pointers to Morse Code messages
; ***************************************************************************
 
.org    0x0730
MORSE_TEXTS:
.dw MORSE_TEXT_1*2, MORSE_TEXT_2*2, MORSE_TEXT_3*2, MORSE_TEXT_4*2
.dw MORSE_TEXT_4*2, MORSE_TEXT_5*2, MORSE_TEXT_6*2, MORSE_TEXT_7*2
 
 
; ***************************************************************************
; Morse Code Data
; This table maps ASCII code to Morse signals. Each symbol is two bytes long.
; Each tone is defined by 2 bits. The sequence starts with the two most 
; signifficant bits and and is ended by a 00 end marker.
;
; 00: end of sequence
; 01: short tone
; 10: long tone
; 11: word gap
; ***************************************************************************
 
.org    0x0740
MORSE_TABLE:
.db 0b11000000, 0b00000000  ; 20 [ ]  ?
.db 0b10011001, 0b10100000  ; 21 [!]  -.-.--    
.db 0b01100101, 0b10010000  ; 22 ["]  .-..-.
.db 0b00000000, 0b00000000  ; 23 [#]  ?
.db 0b01010110, 0b01011000  ; 24 [$]  ...-..-
.db 0b00000000, 0b00000000  ; 25 [%]  ?
.db 0b01100101, 0b01000000  ; 26 [&]  .-... 
.db 0b01101010, 0b10010000  ; 27 [']  .----.    
.db 0b10011010, 0b01000000  ; 28 [(]  -.--. 
.db 0b10011010, 0b01100000  ; 29 [)]  -.--.-    
.db 0b00000000, 0b00000000  ; 2A [*]  ?
.db 0b01100110, 0b01000000  ; 2B [+]  .-.-.
.db 0b10100101, 0b10100000  ; 2C [,]  --..--    
.db 0b10010101, 0b01100000  ; 2D [-]  -....-
.db 0b01100110, 0b01100000  ; 2E [.]  .-.-.-    
.db 0b10010110, 0b01000000  ; 2F [/]  -..-. 
 
.db 0b10101010, 0b10000000  ; 30 [0]  ----- 
.db 0b01101010, 0b10000000  ; 31 [1]  .---- 
.db 0b01011010, 0b10000000  ; 32 [2]  ..--- 
.db 0b01010110, 0b10000000  ; 33 [3]  ...-- 
.db 0b01010101, 0b10000000  ; 34 [4]  ....- 
.db 0b01010101, 0b01000000  ; 35 [5]  ..... 
.db 0b10010101, 0b01000000  ; 36 [6]  -.... 
.db 0b10100101, 0b01000000  ; 37 [7]  --... 
.db 0b10101001, 0b01000000  ; 38 [8]  ---.. 
.db 0b10101010, 0b01000000  ; 39 [9]  ----.
.db 0b10101001, 0b01010000  ; 3A [:]  ---...
.db 0b10011001, 0b10010000  ; 3B [;]  -.-.-.
.db 0b00000000, 0b00000000  ; 3C [<]  ?
.db 0b10010101, 0b10000000  ; 3D [=]  -...-
.db 0b00000000, 0b00000000  ; 3E [>]  ?
.db 0b01011010, 0b01010000  ; 3F [?]  ..--..    
 
.db 0b01101001, 0b10010000  ; 40 [@]  .--.-.
.db 0b01100000, 0b00000000  ; 41 [A]  .-    
.db 0b10010101, 0b00000000  ; 42 [B]  -...  
.db 0b10011001, 0b00000000  ; 43 [C]  -.-.  
.db 0b10010100, 0b00000000  ; 44 [D]  -..   
.db 0b01000000, 0b00000000  ; 45 [E]  . 
.db 0b01011001, 0b00000000  ; 46 [F]  ..-.  
.db 0b10100100, 0b00000000  ; 47 [G]  --.   
.db 0b01010101, 0b00000000  ; 48 [H]  ....  
.db 0b01010000, 0b00000000  ; 49 [I]  ..    
.db 0b01101010, 0b00000000  ; 4A [J]  .---  
.db 0b10011000, 0b00000000  ; 4B [K]  -.-   
.db 0b01100101, 0b00000000  ; 4C [L]  .-..  
.db 0b10100000, 0b00000000  ; 4D [M]  --    
.db 0b10010000, 0b00000000  ; 4E [N]  -.    
.db 0b10101000, 0b00000000  ; 4F [O]  ---   
 
.db 0b01101001, 0b00000000  ; 50 [P]  .--.  
.db 0b10100110, 0b00000000  ; 51 [Q]  --.-  
.db 0b01100100, 0b00000000  ; 52 [R]  .-.   
.db 0b01010100, 0b00000000  ; 53 [S]  ...   
.db 0b10000000, 0b00000000  ; 54 [T]  - 
.db 0b01011000, 0b00000000  ; 55 [U]  ..-   
.db 0b01010110, 0b00000000  ; 56 [V]  ...-  
.db 0b01101000, 0b00000000  ; 57 [W]  .--   
.db 0b10010110, 0b00000000  ; 58 [X]  -..-  
.db 0b10011010, 0b00000000  ; 59 [Y]  -.--  
.db 0b10100101, 0b00000000  ; 5A [Z]  --..
.db 0b00000000, 0b00000000  ; 5B [[]  ?
.db 0b00000000, 0b00000000  ; 5C [\]  ?
.db 0b00000000, 0b00000000  ; 5D []]  ?
.db 0b00000000, 0b00000000  ; 5E [^]  ?
.db 0b01011010, 0b01100000  ; 5F [_]  ..--.-
 
 
; ***************************************************************************
; * The brightness of an LED dimmed with PWM does not appear linear.
; * To compensate for this, we use a table to translate a brightness
; * value from 0 to 127 to the actual PWM value. The table is mirrored,
; * so that it's easy to dim in/out by just counting from 0 to 255.
; * The formula for the table is: y = (x/127)^2 * 255, so we use the
; * square between 0 and 1 and map it to byte values (0 to 255). 
; ***************************************************************************
 
.org    0x0780
SQR_TABLE:
.db 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x02, 0x02, 0x02, 0x03, 0x03, 0x04, 0x04
.db 0x04, 0x05, 0x06, 0x06, 0x07, 0x07, 0x08, 0x09, 0x0A, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10
.db 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x1A, 0x1B, 0x1C, 0x1E, 0x1F, 0x20, 0x22, 0x23
.db 0x25, 0x26, 0x28, 0x2A, 0x2B, 0x2D, 0x2F, 0x30, 0x32, 0x34, 0x36, 0x37, 0x39, 0x3B, 0x3D, 0x3F
.db 0x41, 0x43, 0x45, 0x47, 0x4A, 0x4C, 0x4E, 0x50, 0x52, 0x55, 0x57, 0x59, 0x5C, 0x5E, 0x61, 0x63
.db 0x66, 0x68, 0x6B, 0x6D, 0x70, 0x73, 0x75, 0x78, 0x7B, 0x7E, 0x81, 0x83, 0x86, 0x89, 0x8C, 0x8F
.db 0x92, 0x95, 0x98, 0x9B, 0x9F, 0xA2, 0xA5, 0xA8, 0xAB, 0xAF, 0xB2, 0xB5, 0xB9, 0xBC, 0xC0, 0xC3
.db 0xC7, 0xCA, 0xCE, 0xD2, 0xD5, 0xD9, 0xDD, 0xE0, 0xE4, 0xE8, 0xEC, 0xF0, 0xF4, 0xF7, 0xFB, 0xFF
.db 0xFF, 0xFB, 0xF7, 0xF4, 0xF0, 0xEC, 0xE8, 0xE4, 0xE0, 0xDD, 0xD9, 0xD5, 0xD2, 0xCE, 0xCA, 0xC7
.db 0xC3, 0xC0, 0xBC, 0xB9, 0xB5, 0xB2, 0xAF, 0xAB, 0xA8, 0xA5, 0xA2, 0x9F, 0x9B, 0x98, 0x95, 0x92
.db 0x8F, 0x8C, 0x89, 0x86, 0x83, 0x81, 0x7E, 0x7B, 0x78, 0x75, 0x73, 0x70, 0x6D, 0x6B, 0x68, 0x66
.db 0x63, 0x61, 0x5E, 0x5C, 0x59, 0x57, 0x55, 0x52, 0x50, 0x4E, 0x4C, 0x4A, 0x47, 0x45, 0x43, 0x41
.db 0x3F, 0x3D, 0x3B, 0x39, 0x37, 0x36, 0x34, 0x32, 0x30, 0x2F, 0x2D, 0x2B, 0x2A, 0x28, 0x26, 0x25
.db 0x23, 0x22, 0x20, 0x1F, 0x1E, 0x1C, 0x1B, 0x1A, 0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11
.db 0x10, 0x0F, 0x0E, 0x0D, 0x0C, 0x0B, 0x0A, 0x0A, 0x09, 0x08, 0x07, 0x07, 0x06, 0x06, 0x05, 0x04
.db 0x04, 0x04, 0x03, 0x03, 0x02, 0x02, 0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00
 
.exit                   ; End of code
