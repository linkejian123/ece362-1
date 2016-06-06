 ;***********************************************************************
; ECE 362 - Experiment 6 - Fall 2014
;***********************************************************************
;
; Completed by: < Junyan Shi >
;               < 5224-S >
;               < 6 >
;
;
; Academic Honesty Statement:  In signing this statement, I hereby certify
; that I am the individual who created this HC(S)12 source file and that I 
; have not copied the work of any other student (past or present) while 
; completing it. I understand that if I fail to honor this agreement, I will 
; receive a grade ofZERO and be subject to possible disciplinary action.
;
;
; Signature: _____Junyan Shi________________________________   Date: __10/6/2015__________
;
;       
;***********************************************************************
;
; The objective of this experiment is to implement a Big More-Than-Ten
; 19-second play clock on the 9S12C32 development kit.  A waveform
; generator will be used to provide a 100 Hz periodic interrupt. A
; GAL22V10 will be used to provide the requisite interfacing logic,
; which includes an interrupt device flag and a BCD-to-display-code
; decoder for driving a pair of common-anode 7-segment LEDs (see
; lab description document for details). Note that this application
; is intended to run in a "turn-key" fashion. 
;
; The following docking board resources will be used:
;
; - left pushbutton (PAD7): shot clock reset
; - right pushbutton (PAD6): shot clock start/stop
; - left LED (PT1): shot clock run/stop state
; - right LED (PT0): shot clock expired (00) state
;
; CAUTION: Be sure to set the DSO waveform generator output to produce a 
;          5 V peak-to-peak square wave with +2.5 V offset
;
;========================================================================
;
; MC68HC9S12C32 REGISTER MAP

INITRM	EQU	$0010	; INITRM - INTERNAL RAM POSITION REGISTER
INITRG	EQU	$0011	; INITRG - INTERNAL REGISTER POSITION REGISTER

; ==== CRG - Clock and Reset Generator

SYNR	  EQU	$0034	; CRG synthesizer register
REFDV 	EQU	$0035	; CRG reference divider register
CRGFLG	EQU	$0037	; CRG flags register
CRGINT	EQU	$0038
CLKSEL	EQU	$0039	; CRG clock select register
PLLCTL	EQU	$003A	; CRG PLL control register
RTICTL	EQU	$003B
COPCTL	EQU	$003C

; ==== PORT M

PTM	    EQU	$0250	; Port M data register (bit 3 will be used to clear device flag)
DDRM  	EQU	$0252	; Port M data direction register

; ==== Port AD - Digital Input (Pushbuttons)

PTAD    EQU     $0270   ; Port AD data register
DDRAD   EQU     $0272   ; Port AD data direction register
ATDDIEN	EQU	$008D	; Port AD digital input enable
			; (programs Port AD bit positions as digital inputs)

; ==== Port T - docking module LEDs and external PLD interface

PTT   	EQU	$0240	; Port T data register
DDRT	  EQU	$0242	; Port T data direction register

; ==== IRQ control register

IRQCR	EQU	$001E	; (will use default state out of reset) 

; ==== SCI Register Definitions

SCIBDH	EQU	$00C8	; SCI0BDH - SCI BAUD RATE CONTROL REGISTER
SCIBDL	EQU	$00C9	; SCI0BDL - SCI BAUD RATE CONTROL REGISTER
SCICR1	EQU	$00CA	; SCI0CR1 - SCI CONTROL REGISTER
SCICR2	EQU	$00CB	; SCI0CR2 - SCI CONTROL REGISTER
SCISR1	EQU	$00CC	; SCI0SR1 - SCI STATUS REGISTER
SCISR2	EQU	$00CD	; SCI0SR2 - SCI STATUS REGISTER
SCIDRH	EQU	$00CE	; SCI0DRH - SCI DATA REGISTER
SCIDRL	EQU	$00CF	; SCI0DRL - SCI DATA REGISTER

;***********************************************************************
;
; ASCII character definitions
;

CR	 equ	$0d	; RETURN
LF   equ  $0a ; LINE FEED

; ======================================================================
;
;  Variable declarations (SRAM)
;  Others may be added if deemed necessary


intcnt	equ	$3800	; number of IRQ interrupts accumulated
			          	; (set tenths flag when count reaches 10)
tenths	equ	$3801	; one-tenth second flag
leftpb	equ	$3802	; left pushbutton flag
rghtpb	equ	$3803	; right pushbutton flag
runstp	equ	$3804	; run/stop flag
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;???????????????????

time    equ $3805
prevpb  equ $3807      

;***********************************************************************
;  BOOT-UP ENTRY POINT
;***********************************************************************

	org	$8000
startup	sei			; Disable interrupts
	movb	#$00,INITRG	; set registers to $0000
	movb	#$39,INITRM	; map RAM ($3800 - $3FFF)
	lds	#$3FCE		; initialize stack pointer

;
; Set the PLL speed (bus clock = 24 MHz)
;

	bclr	CLKSEL,$80	; disengage PLL from system
	bset	PLLCTL,$40	; turn on PLL
	movb	#$2,SYNR	; set PLL multiplier
	movb	#$0,REFDV	; set PLL divider
	nop
	nop
plp	brclr	CRGFLG,$08,plp	; while (!(crg.crgflg.bit.lock==1))
	bset	CLKSEL,$80	; engage PLL 

;
; Disable watchdog timer (COPCTL register)
;
	movb	#$40,COPCTL	; COP off; RTI and COP stopped in BDM-mode

;
; Initialize asynchronous serial port (SCI) for 9600 baud, no interrupts
; Note - for debugging use only
;
	movb	#$00,SCIBDH	; set baud rate to 9600
	movb	#$9C,SCIBDL	; 24,000,000 / 16 / 156 = 9600 (approx)
	movb	#$00,SCICR1	; $9C = 156
	movb	#$0C,SCICR2	; initialize SCI for program-driven operation


;***********************************************************************
;  START OF CODE FOR EXPERIMENT 6
;***********************************************************************
;
;  Flag and variable initialization (others may be added)
;
	clr	intcnt
	clr	tenths
	clr	leftpb
	clr	rghtpb
	clr	runstp
 
;
;  Initialize Port AD for use as a digital input port (for pushbuttons)
;
	clr	DDRAD		; program port AD for input mode
	movb	#$C0,ATDDIEN	; program PTAD7 and PTAD6 pins as digital inputs

;
;  Initialize Port M to provide asynchronous reset to external device flag on bit 3
;
	bset	DDRM,$08		; configure pin 3 of Port M (PM3) as output signal
	bset	PTM,$08	; send active high clear pulse to external device flag
	bclr	PTM,$08	; return asynchronous clear signal to low state

;
;  < add any additional initialization code here >
;
  movb #$FF,DDRT
  movb #$94,PTT
  movb #$00,DDRAD
  movb #$FF,ATDDIEN
  movw #$190, time
  movw #$C0, prevpb
  
	cli		; enable IRQ interrupts - last step before starting main loop
;
;  Start of main program-driven polling loop
;
main

;  If the "tenth second" flag is set, then
;    - clear the "then second" flag
;    - If the "run/stop" flag is set, then
;      + decrement the play clock value (stored as packed 4-digit BCD number)
;      + update the 7-segment display
;        o if the time remaining is greater than or equal to 10 seconds,
;          display the tens and ones digits
;        o if the time remaining is less than 10 seconds but
;          greater than or equal to 1 second, blank the tens digit
;          and only display the ones digit
;        o if the time remaining is less than 1.0 second,
;          turn on the middle decimal point and display
;          the tenths digit instead of the ones digit
;          (starting with .9 and ending with .0)
;    - Endif
;  Endif

main1

  brclr tenths,$01,main2
  clr tenths
  brclr runstp,$01,main2
  ;decrement time by 1
  ldaa time+1
  adda #$99
  daa
  staa time+1
  
  ldaa time
  adca #$99
  daa
  staa time
;update display

;compare time with $0100  
  ldd time
  cpd #$0100
  blo main11
  
  stab PTT
  bset PTT,%0110
  bclr PTT,%1001
  bra main2
  
   
    
;compare time with $0010    
main11
  cpd #$0010
  blo main12
  stab PTT
  bset PTT,%0010
  bclr PTT,%1101
  bra main2
  
;time less than $0010  
main12
  lslb
  lslb
  lslb
  lslb
  stab PTT
  bset PTT,%1010
  bclr PTT,%0101
  
  
 
    
  
  
  
  
  
;  If the left pushbutton ("reset play clock") flag is set, then:
;    - clear the left pushbutton flag
;    - clear the "run/stop" flag
;    - turn off both the "run/stop" and "time expired" LEDs
;    - reset the display to "19" 
;  Endif

main2
  brclr leftpb,$01,left_done
  clr leftpb
  clr runstp
  movb #%10010100,PTT
  movw #$190,time   
left_done
  
  

;  If the right pushbutton ("start/stop") flag is set, then
;    - clear the right pushbutton flag
;    - toggle the "run/stop" flag
;    - toggle the "run/stop" LED
;  Endif

main3
  brclr rghtpb,$01,right_done
  clr rghtpb
  
  ldaa runstp
  eora #$01
  staa runstp
  
  ldaa PTT
  eora #$02
  staa PTT

right_done
  

;  If the play clock has reached ".0", then:
;    - clear the "run/stop" flag
;    - turn on the "time expired" LED
;    - turn off the "run/stop" LED
;  Endif
main4
  ldd time
  cpd #$0000
  bne not_zero
  clr runstp
  bset PTT,$01
  bclr PTT,$02
not_zero  

	jmp	main	;  main is a while( ) loop with no terminating condition


;***********************************************************************
;
; IRQ interrupt service routine
;
;  External waveform generator produces 100 Hz IRQ interrupt rate
;
;  Use INTCNT to determine when 10 IRQ interrupts have accumulated 
;    -> set tenths flag and clear INTCNT to 0
;
;  Sample state of pushbuttons (PAD7 = left, PAD6 = right)
;  If change in state from "high" to "low" is detected, set pushbutton flag
;     leftpb (for PAD7 H -> L), rghtpb (for PAD6 H -> L)
;     (pushbuttons are momentary contact closures to ground)
;

irq_isr


; < place your code for the IRQ interrupt service routine here >
                             
; < be sure to clear external device flag by asserting bit 3 of Port M - see above >
  ; bset PTM,$08
   ;bclr PTM,$08
       
  ; inc intcnt
 ;  ldaa intcnt
 ;  cmpa #$10
   
 ;  beq ten_iqr
 ;  bra else_if
   
;ten_iqr
;  bset tenths,$01
;  clr intcnt
  
;else_if


 ; bset PTM,$08
  
  
  ;;;lecture note mod3
  
  bset PTM,$08
  bclr PTM,$08
  brclr runstp,$01,chkpb
  inc intcnt
  ldaa intcnt
  cmpa #10 
  blt chkpb
  clr intcnt
  movb #$01,tenths

chkpb
  ldaa PTAD ; read current state of pushbuttons
  anda #$C0 ; mask off PTAD7 and PTAD6
  cmpa prevpb ; compare with previous PB state
  bne didchg ; if current state != previous state
  rti ; figure out which pushbutton involved
  ; else, exit with no change
  
didchg
  psha ; save current state of PB

chklft
  anda #$80 ; isolate current left PB
  eora #$80 ; complement current left PB
  psha
  ldab prevpb
  andb #$80 ; isolate previous left PB
  andb 1,sp+
  beq chkrgt ; if AND=0, left PB did not change from H->L so exit
  ; else, set LEFTPB flag
  movb #$01,leftpb
  bra rdone

chkrgt
  ldaa 0,sp ; reload current PB state
  anda #$40 ; isolate current right PB
  eora #$40 ; complement current right PB
  psha
  ldab prevpb
  andb #$40 ; isolate previous right PB
  andb 1,sp+
  beq rdone ; if AND=0, right PB did not change from H->L so exit
  ; else, set RGHTPB flag
  movb #$01,rghtpb
rdone
  pula
  staa prevpb ; update PB state

        rti


;***********************************************************************
;
; Add any additional subroutines needed here 
;
;  < place additional routines here >







;***********************************************************************
; Character I/O Library Routines for 9S12C32
;
; (these maay prove useful for testing and debugging but are otherwise
; not used for this application)
;***********************************************************************

rxdrf    equ   $20    ; receive buffer full mask pattern
txdre    equ   $80    ; transmit buffer empty mask pattern

;***********************************************************************
; Name:         inchar
; Description:  inputs ASCII character from SCI serial port
;                  and returns it in the A register
; Returns:      ASCII character in A register
; Modifies:     A register
;***********************************************************************

inchar  brclr  SCISR1,rxdrf,inchar
        ldaa   SCIDRL ; return ASCII character in A register
        rts

;***********************************************************************
; Name:         outchar
; Description:  outputs ASCII character passed in the A register
;                  to the SCI serial port
;***********************************************************************

outchar brclr  SCISR1,txdre,outchar
        staa   SCIDRL ; output ASCII character to SCI
        rts

;***********************************************************************
;
; Define 'where you want to go today' (reset and interrupt vectors) 
;
; If get a "bad" interrupt, just return from it

BadInt	rti

;
; ------------------ VECTOR TABLE --------------------
;

	org	$FF8A
	fdb	BadInt	;$FF8A: VREG LVI
	fdb	BadInt	;$FF8C: PWM emergency shutdown
	fdb	BadInt	;$FF8E: PortP
	fdb	BadInt	;$FF90: Reserved
	fdb	BadInt	;$FF92: Reserved
	fdb	BadInt	;$FF94: Reserved
	fdb	BadInt	;$FF96: Reserved
	fdb	BadInt	;$FF98: Reserved
	fdb	BadInt	;$FF9A: Reserved
	fdb	BadInt	;$FF9C: Reserved
	fdb	BadInt	;$FF9E: Reserved
	fdb	BadInt	;$FFA0: Reserved
	fdb	BadInt	;$FFA2: Reserved
	fdb	BadInt	;$FFA4: Reserved
	fdb	BadInt	;$FFA6: Reserved
	fdb	BadInt	;$FFA8: Reserved
	fdb	BadInt	;$FFAA: Reserved
	fdb	BadInt	;$FFAC: Reserved
	fdb	BadInt	;$FFAE: Reserved
	fdb	BadInt	;$FFB0: CAN transmit
	fdb	BadInt	;$FFB2: CAN receive
	fdb	BadInt	;$FFB4: CAN errors
	fdb	BadInt	;$FFB6: CAN wake-up
	fdb	BadInt	;$FFB8: FLASH
	fdb	BadInt	;$FFBA: Reserved
	fdb	BadInt	;$FFBC: Reserved
	fdb	BadInt	;$FFBE: Reserved
	fdb	BadInt	;$FFC0: Reserved
	fdb	BadInt	;$FFC2: Reserved
	fdb	BadInt	;$FFC4: CRG self-clock-mode
	fdb	BadInt	;$FFC6: CRG PLL Lock
	fdb	BadInt	;$FFC8: Reserved
	fdb	BadInt	;$FFCA: Reserved
	fdb	BadInt	;$FFCC: Reserved
	fdb	BadInt	;$FFCE: PORTJ
	fdb	BadInt	;$FFD0: Reserved
	fdb	BadInt	;$FFD2: ATD
	fdb	BadInt	;$FFD4: Reserved
	fdb	BadInt	;$FFD6: SCI Serial System
	fdb	BadInt	;$FFD8: SPI Serial Transfer Complete
	fdb	BadInt	;$FFDA: Pulse Accumulator Input Edge
	fdb	BadInt	;$FFDC: Pulse Accumulator Overflow
	fdb	BadInt	;$FFDE: Timer Overflow
	fdb	BadInt	;$FFE0: Standard Timer Channel 7
	fdb	BadInt	;$FFE2: Standard Timer Channel 6
	fdb	BadInt	;$FFE4: Standard Timer Channel 5
	fdb	BadInt	;$FFE6: Standard Timer Channel 4
	fdb	BadInt	;$FFE8: Standard Timer Channel 3
	fdb	BadInt	;$FFEA: Standard Timer Channel 2
	fdb	BadInt	;$FFEC: Standard Timer Channel 1
	fdb	BadInt	;$FFEE: Standard Timer Channel 0
	fdb	BadInt	;$FFF0: Real Time Interrupt (RTI)
	fdb	irq_isr	;$FFF2: IRQ (External Pin or Parallel I/O) (IRQ)
	fdb	BadInt	;$FFF4: XIRQ (Pseudo Non-Maskable Interrupt) (XIRQ)
	fdb	BadInt	;$FFF6: Software Interrupt (SWI)
	fdb	BadInt	;$FFF8: Illegal Opcode Trap ()
	fdb	startup	;$FFFA: COP Failure (Reset) ()
	fdb	BadInt	;$FFFC: Clock Monitor Fail (Reset) ()
	fdb	startup	;$FFFE: /RESET
	end

