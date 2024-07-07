     INCLUDE "p18f2550.inc"
     INCLUDE "pickit2.inc"
     
     CODE

;/******************************************************************************
; * Function:        JTAG2W4PH primitive
; *
; * Overview:        assembly primitive for PIC32 2-Wire 4-Phase JTAG
; *
; * PreCondition:    tris_ICSPCLK = 0 (output)
; *                  ICSPCLK_out = 0 (low)
; *
; * Input:           ASM_TEMP1_RAM - TDI (output) value in bit 0
; *                  ASM_TEMP2_RAM - TMS (output) value in bit 0
; *
; * Output:          ASM_TEMP3_RAM - TDO (input) value in bit 0
; *
; * Side Effects:    tris_ICSPDAT set to output, then to input
; *
; * Note:            Use with an assembly CALL instruction.
; *                  Min PGCx high or low time = 83ns
; *                  25 instruction cycles (including CALL & RETURN) = 2.1us
; *****************************************************************************/
 JTAG2W4PH:
      bsf     LATA, 3, 0      ;// CLK high phase 1
      bcf     LATA, 2, 0      ;// TDI = 0?
      btfsc   ASM_TEMP1_RAM, 0, 0 ;// Test for TDI value
      bsf     LATA, 2, 0      ;// TDI = 1
      bcf     TRISA, 2, 0     ;// Output TDI (PGD = output)  
      nop 
      bcf     LATA, 3, 0      ;// CLK low Phase 1
      btfss   ASM_TEMP2_RAM, 0, 0 ;// Test for TMS value low
      bcf     LATA, 2, 0      ;// TMS = 0
      bsf     LATA, 3, 0      ;// CLK high phase 2
      btfsc   ASM_TEMP2_RAM, 0, 0 ;// Test for TMS value high
      bsf     LATA, 2, 0      ;// TMS = 1
      bcf     ASM_TEMP3_RAM, 7, 0 ;// TDO = 0?
      bcf     LATA, 3, 0      ;// CLK low Phase 2
      bsf     LATA, 3, 0      ;// CLK high phase 3
      bsf     TRISA, 2, 0     ;// Input TDO (PGD = input)
      bcf     LATA, 3, 0      ;// CLK low Phase 3
      bsf     LATA, 3, 0      ;// CLK high phase 4
      btfsc   PORTA, 2, 0     ;// Test for TDO value
      bsf     ASM_TEMP3_RAM, 7, 0 ;// TDO = 1
      bcf     LATA, 3, 0      ;// CLK low Phase 4
      return 

     GLOBAL JTAG2W4PH

;/******************************************************************************
; * Function:        LOGICPREP 
; *
; * Overview:        assembly function for Logic Analyzer.
; *                  Clears RAM 0x600 to 0x7FF and creates
; *                  TrigMaskSwap(asm_temp2) and TrigStatesSwap(asm_temp4)
; *                  Also, for pre-run sampling, saves off PostTrigCount in
; *                  asm_temp9/10 and sets the count to 1023 to fill up the buffer
; *                  with pre-trigger data.
; *
; * PreCondition:    FSR0 saved
; *
; * Input:           asm_temp1 = TrigMask
; *                  asm_temp3 = TrigStates
; *                  asm_temp7 = PostTrigCountL
; *                  asm_temp8 = PostTrigCountH + 1
; *
; * Output:          asm_temp2 = SWAPF TrigMask
; *                  asm_temp4 = SWAPF TrigStates
; *                  asm_temp7 = 0xFE                  ; take 1023 presamples
; *                  asm_temp8 = 0x04
; *                  asm_temp9 = PostTrigCountL
; *                  asm_temp10 = PostTrigCountH + 1
; *
; * Side Effects:    Clears RAM addresses 0x600 to 0x7FF, turns on BUSY_LED
; *
; * Note:            
; *****************************************************************************/
LOGICPREP:
     swapf     ASM_TEMP1_RAM, w, 0
     movwf     ASM_TEMP2_RAM, 0
     swapf     ASM_TEMP3_RAM, w, 0
     movwf     ASM_TEMP4_RAM, 0  
     ; prep for pre-trigger sampling
     movff     ASM_TEMP7_RAM, ASM_TEMP9_RAM
     movff     ASM_TEMP8_RAM, ASM_TEMP10_RAM
     movlw     0xFE
     movwf     ASM_TEMP7_RAM, 0
     movlw     0x04
     movwf     ASM_TEMP8_RAM, 0
     lfsr      0, 0x600       ; FSR0 = 0x600
LOGICPREP_loop:
     clrf      POSTINC0, 0
     btfss     FSR0H, 3, 0    ; look for 0x800
     bra       LOGICPREP_loop
     return

     GLOBAL LOGICPREP

;/******************************************************************************
; * Function:        ANALYZER_RISING_1MHZ
; *
; * Overview:        Logic analyzer with rising edge detection, sample rate 1MHz
; *
; * PreCondition:    FSR0 saved
; *
; * Input:           asm_temp1-8
; *
; * Output:          asm_temp7 = TrigSaveL
; *                  asm_temp8 = TrigSaveH
; *                  RAM 0x600-0x7FF : samples
; *
; * Side Effects:    Writes RAM addresses 0x600 to 0x7FF
; *
; * Note:            
; *****************************************************************************/
ANALYZER_RISING_1MHZ:
     lfsr      0, 0x7FF
     rcall     ANALYZER_presample
     movff     ASM_TEMP9_RAM, ASM_TEMP7_RAM  ; restore PostTrigCount
     movff     ASM_TEMP10_RAM, ASM_TEMP8_RAM
     swapf     PORTA, 0, 0                   ; last pre-sample
     iorwf     POSTDEC0, 1, 0
     movwf     ASM_TEMP10_RAM, 0             ; store as last sample
     bsf       FSR0H, 1, 0
     bra       $+2
     bra       $+2
     bra       $+2
     bra       ANALYZER_RISING_1MHZ_entry

ANALYZER_RISING_1MHZ_notrig4:
     btfss     PORTB, 5, 0         ; check for abort
     bra       ANALYZER_trigabort
ANALYZER_RISING_1MHZ_loop:
     swapf     PORTA, 0, 0
     iorwf     POSTDEC0, 1, 0
     bsf       FSR0H, 1, 0
     movwf     ASM_TEMP10_RAM, 0
     andwf     ASM_TEMP2_RAM, 0 ,0
     xorwf     ASM_TEMP4_RAM, 0, 0
     bnz       ANALYZER_RISING_1MHZ_notrig1
     movf      ASM_TEMP9_RAM, 0
     andwf     ASM_TEMP5_RAM, 0, 0
     bnz       ANALYZER_RISING_1MHZ_notrig2
     dcfsnz    ASM_TEMP6_RAM, 1, 0
     bra       ANALYZER_triggerswap
ANALYZER_RISING_1MHZ_entry: 
     movf      PORTA, 0, 0
     movwf     INDF0, 0
     movwf     ASM_TEMP9_RAM, 0
     andwf     ASM_TEMP1_RAM, 0 ,0
     xorwf     ASM_TEMP3_RAM, 0 ,0
     bnz       ANALYZER_RISING_1MHZ_notrig3
     swapf     ASM_TEMP10_RAM, 0, 0
     andwf     ASM_TEMP5_RAM, 0, 0
     bnz       ANALYZER_RISING_1MHZ_notrig4
     decfsz    ASM_TEMP6_RAM, 1, 0
     bra       ANALYZER_RISING_1MHZ_loop
     bra       ANALYZER_trigger

ANALYZER_RISING_1MHZ_notrig2:
     nop
     movf      PORTA, 0, 0
     movwf     INDF0, 0
     movwf     ASM_TEMP9_RAM, 0
     andwf     ASM_TEMP1_RAM, 0 ,0
     xorwf     ASM_TEMP3_RAM, 0 ,0
     bnz       ANALYZER_RISING_1MHZ_notrig3
     swapf     ASM_TEMP10_RAM, 0, 0
     andwf     ASM_TEMP5_RAM, 0, 0
     bnz       ANALYZER_RISING_1MHZ_notrig4
     decfsz    ASM_TEMP6_RAM, 1, 0
     bra       ANALYZER_RISING_1MHZ_loop
     bra       ANALYZER_trigger

ANALYZER_RISING_1MHZ_notrig1:
     btfss     PORTB, 5, 0         ; check for abort
     bra       ANALYZER_trigabort
     bra       ANALYZER_RISING_1MHZ_entry

ANALYZER_RISING_1MHZ_notrig3:
     nop
     bra       ANALYZER_RISING_1MHZ_notrig4

     GLOBAL ANALYZER_RISING_1MHZ

;/******************************************************************************
; * Function:        ANALYZER_FALLING_1MHZ
; *
; * Overview:        Logic analyzer with falling edge detection, sample rate 1MHz
; *
; * PreCondition:    FSR0 saved
; *
; * Input:           asm_temp1-8
; *
; * Output:          asm_temp7 = TrigSaveL
; *                  asm_temp8 = TrigSaveH
; *                  RAM 0x600-0x7FF : samples
; *
; * Side Effects:    Writes RAM addresses 0x600 to 0x7FF
; *
; * Note:            
; *****************************************************************************/
ANALYZER_FALLING_1MHZ:
     lfsr      0, 0x7FF
     rcall     ANALYZER_presample
     movff     ASM_TEMP9_RAM, ASM_TEMP7_RAM  ; restore PostTrigCount
     movff     ASM_TEMP10_RAM, ASM_TEMP8_RAM
     swapf     PORTA, 0, 0                   ; last pre-sample
     iorwf     POSTDEC0, 1, 0
     movwf     ASM_TEMP10_RAM, 0             ; store as last sample
     bsf       FSR0H, 1, 0
     bra       $+2
     bra       $+2
     bra       $+2
     bra       ANALYZER_FALLING_1MHZ_entry

ANALYZER_FALLING_1MHZ_notrig4:
     btfss     PORTB, 5, 0         ; check for abort
     bra       ANALYZER_trigabort
ANALYZER_FALLING_1MHZ_loop:
     swapf     PORTA, 0, 0
     iorwf     POSTDEC0, 1, 0
     bsf       FSR0H, 1, 0
     movwf     ASM_TEMP10_RAM, 0
     andwf     ASM_TEMP2_RAM, 0 ,0
     xorwf     ASM_TEMP4_RAM, 0, 0
     bnz       ANALYZER_FALLING_1MHZ_notrig1
     movf      ASM_TEMP9_RAM, 0
     andwf     ASM_TEMP5_RAM, 0, 0
     bz        ANALYZER_FALLING_1MHZ_notrig2
     dcfsnz    ASM_TEMP6_RAM, 1, 0
     bra       ANALYZER_triggerswap
ANALYZER_FALLING_1MHZ_entry: 
     movf      PORTA, 0, 0
     movwf     INDF0, 0
     movwf     ASM_TEMP9_RAM, 0
     andwf     ASM_TEMP1_RAM, 0 ,0
     xorwf     ASM_TEMP3_RAM, 0 ,0
     bnz       ANALYZER_FALLING_1MHZ_notrig3
     swapf     ASM_TEMP10_RAM, 0, 0
     andwf     ASM_TEMP5_RAM, 0, 0
     bz        ANALYZER_FALLING_1MHZ_notrig4
     decfsz    ASM_TEMP6_RAM, 1, 0
     bra       ANALYZER_FALLING_1MHZ_loop
     bra       ANALYZER_trigger

ANALYZER_FALLING_1MHZ_notrig2:
     nop
     movf      PORTA, 0, 0
     movwf     INDF0, 0
     movwf     ASM_TEMP9_RAM, 0
     andwf     ASM_TEMP1_RAM, 0 ,0
     xorwf     ASM_TEMP3_RAM, 0 ,0
     bnz       ANALYZER_FALLING_1MHZ_notrig3
     swapf     ASM_TEMP10_RAM, 0, 0
     andwf     ASM_TEMP5_RAM, 0, 0
     bz        ANALYZER_FALLING_1MHZ_notrig4
     decfsz    ASM_TEMP6_RAM, 1, 0
     bra       ANALYZER_FALLING_1MHZ_loop
     bra       ANALYZER_trigger

ANALYZER_FALLING_1MHZ_notrig1:
     btfss     PORTB, 5, 0         ; check for abort
     bra       ANALYZER_trigabort
     bra       ANALYZER_FALLING_1MHZ_entry

ANALYZER_FALLING_1MHZ_notrig3:
     nop
     bra       ANALYZER_FALLING_1MHZ_notrig4

     GLOBAL ANALYZER_FALLING_1MHZ


;/******************************************************************************
; * Function:        ANALYZER_SLOWRATE
; *
; * Overview:        Logic analyzer with rising/falling edge detection, 
; *                  sample rate < 1MHz
; *
; * PreCondition:    FSR0 saved
; *
; * Input:           asm_temp1-8
; *
; * Output:          asm_temp7 = TrigSaveL
; *                  asm_temp8 = TrigSaveH
; *                  RAM 0x600-0x7FF : samples
; *
; * Side Effects:    Writes RAM addresses 0x600 to 0x7FF
; *
; * Note:            
; *****************************************************************************/
ANALYZER_SLOWRATE:
     lfsr      0, 0x7FF
     rcall     ANALYZER_presample
     movff     ASM_TEMP9_RAM, ASM_TEMP7_RAM  ; restore PostTrigCount
     movff     ASM_TEMP10_RAM, ASM_TEMP8_RAM
     rcall     ANALYZER_Delay       ; 11 cycles for 1
     nop
     swapf     PORTA, 0, 0                   ; last pre-sample
     iorwf     POSTDEC0, 1, 0
     movwf     ASM_TEMP10_RAM, 0             ; store as last sample
     bsf       FSR0H, 1, 0
     bra       $+2
     bra       $+2
     bra       $+2
     nop
     bra       ANALYZER_SLOWRATE_entry

ANALYZER_SLOWRATE_notrig4:
     btfss     PORTB, 5, 0         ; check for abort
     bra       ANALYZER_trigabort
ANALYZER_SLOWRATE_loop:
     rcall     ANALYZER_Delay       ; 11 cycles for 1
     swapf     PORTA, 0, 0
     iorwf     POSTDEC0, 1, 0
     bsf       FSR0H, 1, 0
     movwf     ASM_TEMP10_RAM, 0
     andwf     ASM_TEMP2_RAM, 0 ,0
     xorwf     ASM_TEMP4_RAM, 0, 0
     bnz       ANALYZER_SLOWRATE_notrig1
     movf      ASM_TEMP9_RAM, 0
     andwf     ASM_TEMP5_RAM, 0, 0
     xorwf     ASM_TEMP12_RAM, 0 , 0
     bnz       ANALYZER_SLOWRATE_notrig2
     dcfsnz    ASM_TEMP6_RAM, 1, 0
     bra       ANALYZER_triggerswap
ANALYZER_SLOWRATE_entry: 
     rcall     ANALYZER_Delay       ; 11 cycles for 1
     movf      PORTA, 0, 0
     movwf     INDF0, 0
     movwf     ASM_TEMP9_RAM, 0
     andwf     ASM_TEMP1_RAM, 0 ,0
     xorwf     ASM_TEMP3_RAM, 0 ,0
     bnz       ANALYZER_SLOWRATE_notrig3
     swapf     ASM_TEMP10_RAM, 0, 0
     andwf     ASM_TEMP5_RAM, 0, 0
     xorwf     ASM_TEMP12_RAM, 0 , 0
     bnz       ANALYZER_SLOWRATE_notrig4
     decfsz    ASM_TEMP6_RAM, 1, 0
     bra       ANALYZER_SLOWRATE_loop
     bra       ANALYZER_trigger

ANALYZER_SLOWRATE_notrig2:
     nop
     rcall     ANALYZER_Delay       ; 11 cycles for 1
     movf      PORTA, 0, 0
     movwf     INDF0, 0
     movwf     ASM_TEMP9_RAM, 0
     andwf     ASM_TEMP1_RAM, 0 ,0
     xorwf     ASM_TEMP3_RAM, 0 ,0
     bnz       ANALYZER_SLOWRATE_notrig3
     swapf     ASM_TEMP10_RAM, 0, 0
     andwf     ASM_TEMP5_RAM, 0, 0
     xorwf     ASM_TEMP12_RAM, 0 , 0
     bnz       ANALYZER_SLOWRATE_notrig4
     decfsz    ASM_TEMP6_RAM, 1, 0
     bra       ANALYZER_SLOWRATE_loop
     bra       ANALYZER_trigger

ANALYZER_SLOWRATE_notrig1:
     btfss     PORTB, 5, 0         ; check for abort
     bra       ANALYZER_trigabort
     bra       ANALYZER_SLOWRATE_entry

ANALYZER_SLOWRATE_notrig3:
     bra       $+2
     bra       ANALYZER_SLOWRATE_notrig4

     GLOBAL ANALYZER_SLOWRATE

; ////////////////////////////// TRIGGER ROUTINEs //////////////////////////////////
ANALYZER_trigabort:
     movlw     0x40
     movwf     ASM_TEMP2_RAM
     return    0
ANALYZER_presample:
     movf      PORTA, 0, 0 
     movwf     INDF0, 0
     bra       $+2
     bra       $+2
     bra       $+2
     nop
     movf      ASM_TEMP11_RAM, 1, 0     
     bz        ANALYZER_trigger_postswap
     rcall     ANALYZER_Delay
     bra       ANALYZER_trigger_postswap
ANALYZER_triggerswap:
     movf      PORTA, 0, 0    ; trigger sample will be off 1/12 of 1us
     movwf     INDF0, 0
     movff     FSR0L, ASM_TEMP1_RAM
     movf      FSR0H, 0, 0
     iorlw     0x80      ; set trigger type
     movwf     ASM_TEMP2_RAM, 0
     bra       $+2
     movf      ASM_TEMP11_RAM, 1, 0     
     bz        ANALYZER_trigger_postswap
     rcall     ANALYZER_Delay
     bra       ANALYZER_trigger_postswap
ANALYZER_trigger:
     swapf     PORTA, 0, 0    ; trigger sample will be off 1/12 of 1us
     iorwf     POSTDEC0, 1, 0
     bsf       FSR0H, 1, 0
     movff     FSR0L, ASM_TEMP1_RAM
     movff     FSR0H, ASM_TEMP2_RAM
     bra       $+2
     movf      ASM_TEMP11_RAM, 1, 0     
     bz        ANALYZER_trigger_post
     rcall     ANALYZER_Delay
     bra       ANALYZER_trigger_post

ANALYZER_trigger_post:
     movf      PORTA, 0, 0
     movwf     INDF0, 0   
     nop  
     decfsz    ASM_TEMP7_RAM, 1, 0      ; PostTrigCount
     bra       ANALYZER_trigger_redir
     dcfsnz    ASM_TEMP8_RAM, 1, 0
     return    0
     nop       
ANALYZER_trig_cont:
     nop
     movf      ASM_TEMP11_RAM, 1, 0     
     bz        ANALYZER_trigger_postswap
     rcall     ANALYZER_Delay
     bra       ANALYZER_trigger_postswap

ANALYZER_trigger_postswap:
     swapf     PORTA, 0, 0 
     iorwf     POSTDEC0, 1, 0
     bsf       FSR0H, 1, 0
     decfsz    ASM_TEMP7_RAM, 1, 0      ; PostTrigCount
     bra       ANALYZER_trigger_redir2
     dcfsnz    ASM_TEMP8_RAM, 1, 0
     return    0
     nop       
ANALYZER_trig_cont2:
     nop
     movf      ASM_TEMP11_RAM, 1, 0     
     bz        ANALYZER_trigger_post
     rcall     ANALYZER_Delay
     bra       ANALYZER_trigger_post

ANALYZER_trigger_redir:
     bra       ANALYZER_trig_cont
ANALYZER_trigger_redir2:
     bra       ANALYZER_trig_cont2

ANALYZER_Delay:
     ; NOTE 11 cycles including call/return for delay of 1
     movf      ASM_TEMP11_RAM, 0, 0
ANALYZER_Delay_loop:
     bra       $+2
     bra       $+2
     nop
     dcfsnz    WREG, 1, 0
     return    0
     bra       $+2
     nop
     bra       ANALYZER_Delay_loop



     END