/*********************************************************************
 *
 *                  Microchip PICkit 2 v2 Flash Starter Kit
 *
 *********************************************************************
 * FileName:        pk_isr.c
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 3.00
 * Company:         Microchip Technology, Inc.
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 * Author               Date        Comment
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Walter Kicinski   2006-May-15    Initial Write
 ********************************************************************/

/** I N C L U D E S **********************************************************/

#include <p18cxxx.h>
#include "io_cfg.h"
#include "system\typedefs.h"
#include "user\pk_isr.h"
#include "pickit.h"


/** V A R I A B L E S ********************************************************/



/** P R I V A T E  P R O T O T Y P E S ***************************************/


/** D E C L A R A T I O N S **************************************************/

#pragma code

/******************************************************************************
 * Function:        
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        
 *
 * Note:            Refer to PIC18F2455/2550/4455/4550 Errata concerning
 *                  high-priority interrupt problem.
 *   
 *****************************************************************************/

#pragma interruptlow InterruptHandler

void InterruptHandler (void)
{

    if (Pk2Status.UARTMode == 1)
    {
        // UART RX Start Bit Detect
        if ((PIR2bits.CMIF) && (PIE2bits.CMIE))
        {
            if (CMCONbits.C2OUT == 0)
            { // possible start bit - Approx 3uS here from bit edge.
                    PIE2bits.CMIE = 0;                  // shut off comparator int
                    T1CONbits.T1CKPS0 = 0;              // 1:1 prescale runs 2x as fast.
                    TMR1H = UARTStatus.TimerBaudLoadH;  // gives 1/2 bit to middle of start
                    if (((int)UARTStatus.TimerBaudLoadL + (int)UARTStartBitAdjust) > 0x00FF)
                    { // adjust about 6us to account for ISR O/H
                        TMR1H++;
                    }
                    TMR1L = (UARTStatus.TimerBaudLoadL + UARTStartBitAdjust);
                    PIR1bits.TMR1IF = 0;
                    PIE1bits.TMR1IE = 1;
                    
            }
            PIR2bits.CMIF = 0;
        }
        // UART Byte RX
        if ((PIR1bits.TMR1IF) && (PIE1bits.TMR1IE))
        {
            TMR1H = UARTStatus.TimerBaudLoadH;  // Full bit time at 1:2 prescale
            TMR1L = UARTStatus.TimerBaudLoadL;  
            PIR1bits.TMR1IF = 0;  
            if (UARTStatus.RXbits == 0)
            { // start bit
                T1CONbits.T1CKPS0 = 1;              // 1:2 prescale
                if (CMCONbits.C2OUT != 0)
                { // bad start bit.
                    PIE1bits.TMR1IE = 0;        // abort
                    PIR2bits.CMIF = 0;
                    PIE2bits.CMIE = 1;          // & look for another start.
                }
                else
                {
                    BUSY_LED = 1;
                    UARTStatus.RXbits++;
                }
            }
            else if (UARTStatus.RXbits > 8)
            { // stop bit
                if (CMCONbits.C2OUT == 1)
                {  // stop is good
                    if (UARTStatus.NewRX & 0x01)
                    {
                        UARTStatus.LastRXByte2 = UARTStatus.RXbyte;
                    }
                    else if (UARTStatus.NewRX & 0x02)
                    {
                        UARTStatus.LastRXByte3 = UARTStatus.RXbyte;
                    }
                    else
                    {
                        UARTStatus.LastRXByte = UARTStatus.RXbyte;
                    }
                    UARTStatus.NewRX++;
                }
                BUSY_LED = 0;
                UARTStatus.RXbits = 0;      // reset
                PIE1bits.TMR1IE = 0;        // Done
                PIR2bits.CMIF = 0;
                PIE2bits.CMIE = 1;          // & look for another start.
            }
            else
            { // data bit
                UARTStatus.RXbyte >>= 1;    // shift
                if (CMCONbits.C2OUT == 1)
                {
                    UARTStatus.RXbyte |= 0x80;    // set MSb
                }
                UARTStatus.RXbits++; 
            }
        } 

        // UART Byte TX
        if ((PIR2bits.TMR3IF) && (PIE2bits.TMR3IE))
        {
            TMR3H = UARTStatus.TimerBaudLoadH;  // Full bit time at 1:2 prescale
            TMR3L = UARTStatus.TimerBaudLoadL;  
            PIR2bits.TMR3IF = 0;
            if (UARTStatus.TXbits == 0)
            { // start bit
                ICSPCLK_out = 0;
                UARTStatus.TXbits++;
                BUSY_LED = 1;
            }
            else if (UARTStatus.TXbits == 9)
            { // stop bit
                ICSPCLK_out = 1;                
                UARTStatus.TXbits++;
            }
            else if (UARTStatus.TXbits > 9)
            { // done!
                UARTStatus.TXbits = 0;      // reset all to idle state
                PIE2bits.TMR3IE = 0;  
                BUSY_LED = 0;    
            }
            else
            { // data bit
                ICSPCLK_out = UARTStatus.TXByte & 0x01; // send LSb
                UARTStatus.TXbits++;
                UARTStatus.TXByte >>= 1;
            }
            // NOTE: when the value of ICSPCLK_out, it will generate an interrupt
            //       on the C1 comparator!
        }
    }
    else
    {
        if (PIR1bits.TMR1IF)        // Timer 1 ADC voltage monitor
        {
            PIR1bits.TMR1IF = 0;    // clear flag
            TMR1H = Timer1_Timeout_High;  // reset timer
            TMR1L = Timer1_Timeout_Low;
    
            // CHECK VDD VOLTAGE
            //if (ADCON0 == ADC_VDD_Tgt_Ch) // last conversion was Vdd (should always be)
            //{
    			if ((!Vdd_TGT_P_pin) && (ADRESH < VddVppLevels.VddThreshold))  // Don't check for short if not on.
    				{
    				// Conversion for VDD less than threshold.  Assume Short/heavy load.
    				// Must get MaxVError such conversions to minimize shutting off power due to glitches, ramp-up
    				if (VddVppLevels.VddErrCount == MaxErrorVDD)
    					{
    						Vpp_ON = 0;		// shut off VPP 
    	                    Vdd_TGT_P = 1;	// shut off VDD as well
    	                    //#!# BusyLED_BlinkOn();
    	                    Pk2Status.VddError = 1;		// indicate error in status
    					}
    				else
    					{
    					VddVppLevels.VddErrCount++;			// Increment on error.
    					} // end if (VddVppLevels.VddErrCount == n)
    				}
    			else
    				{
    				VddVppLevels.VddErrCount = 0;				// Reset on good voltage
    				} // end if ((!Vdd_TGT_P_pin) && (ADRESH < VddVppLevels.VddThreshold))
            //} 
            if (Pk2Status.StatusLow & 0x30) // if Pk2Status.VddError or Pk2Status.VppError is set
            {
                // blink Busy at 12 Herz on error.
                if (VddVppLevels.BlinkClount++ > BusyBlinkRate)
                {
                    BUSY_LED = !BUSY_LED;
                    VddVppLevels.BlinkClount = 0;
                }
            }
    
            PIR1bits.ADIF = 0; // clear flag
            ADCON0 = ADC_VPP_FB_Ch;			    // Check VPP next.
            PIE1bits.ADIE = 1;                  // interrupt when done
            ADCON0bits.GO = 1;          		// start conversion
        } // end if (PIR1bits.TMR1IF)
    
    
        if (PIR1bits.ADIF)  // VPP conversion interrupt
        {
            PIR1bits.ADIF = 0; // clear flag
    
            // CHECK VPP VOLTAGE
            //if (ADCON0 == ADC_VPP_FB_Ch) // last conversion was VPP (presently only ADIF interrupt)
            //{
    			// Maintain VPP voltage by adjusting PWM on/off
    			if (CCP2CON == VppPWM_Enable)	// skip if PWM not on
    				{
    				if (ADRESH > Vpp_PWM.UppperLimit)
    					{
    					CCPR2L = VppDutyCycleOff;
    					}
    				if (ADRESH < Vpp_PWM.LowerLimit)
    					{
    					CCPR2L = Vpp_PWM.CCPRSetPoint;
    					}	
        			// Check for short or heavy load
        			if ((Vpp_ON_pin == 1) && (ADRESH < VddVppLevels.VppThreshold))  // Don't check for short if not on
        				{
        				// Conversion for VPP less than threshold.  Assume Short/heavy load.
        				// NOTE: Q6 will go into active mode when VPP is shorted, so VPP_FEEDBACK will drop
        				// by less percentage-wise the lower VPP gets.
        				// Must get MaxVError such conversions to minimize shutting off power due to glitches, ramp-up
        				if (VddVppLevels.VppErrCount == MaxErrorVPP)
        					{
        						Vpp_ON = 0;		// shut off VPP 
        	                    Vdd_TGT_P = 1;	// shut off VDD as well
        	                    // #!# BusyLED_BlinkOn();
        	                    Pk2Status.VppError = 1;		// indicate error in status
        					}
        				else
        					{
        					VddVppLevels.VppErrCount++;			// Increment on error.
        					} // end if (VddVppErrCounts.VPP)
        				}
        			else
        				{
        				VddVppLevels.VppErrCount = 0;				// Reset on good voltage
        				} // end if ((Vpp_ON_pin == 1) && (ADRESH < VddVppLevels.VppThreshold))
                    } //end if  (CCP2CON = VppPWM_On)
            //} // end if (ADCON0 == ADC_VDD_Tgt_Ch) else if (ADCON0 == ADC_VPP_FB_Ch)
            ADCON0 = ADC_VDD_Tgt_Ch;		// Check VDD next.
            PIE1bits.ADIE = 0;              // don't interrupt - wait for Timer1 int
            ADCON0bits.GO = 1;          	// start conversion
        } // end if (PIR1bits.ADIF)
    }
    
    if (PIR2bits.HLVDIF)
    {
        Reset();
    }

} // end void InterruptHandler(void)

#pragma code

/******************************************************************************
 * Function: void VppVddADCTmr1_Start(void)       
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Starts Timer1 driving ADC conversions of VPP/VDD
 *
 * Side Effects:    Uses Timer1 and ADC
 *
 * Overview:        
 *
 * Note:            On a Timer1 interrupt, the results of the last VDD conversion are read
 *                  and a VPP conversion is started also enabling the ADC interrupt.
 *                  When the VPP conversion is finished, it interrupts, the results are
 *                  handled, and a VDD conversion is started also disabling the ADC interrupt.
 *                  The VDD result will not be read until the next Timer1 interrupt.
 *   
 *****************************************************************************/
void VppVddADCTmr1_Start(void)
{

	// Init Error Counts
	VddVppLevels.VddErrCount = 0;
	VddVppLevels.VppErrCount = 0;

    // Set up Timer 1
    T1CON = ADCMonitorTmr1_Setup;
    TMR1H = Timer1_Timeout_High;
    TMR1L = Timer1_Timeout_Low;
    PIR1bits.TMR1IF = 0;            // clear int flag
    PIE1bits.TMR1IE = 1;            // enable int
    T1CONbits.TMR1ON = 1;           // start timer

	// Assumes RA0/AN0 and RA1/AN1 set to analog by PICkitInit() and chip VDD/VSS selected as references.
    PIR1bits.ADIF = 0;              // clear A/D Converter Interrupt Flag
    PIE1bits.ADIE = 0;              // ensure A/D Converter Interrupt disabled
	ADCON2 = ADC_LftJust_8TAD_FoscDiv64;	// 8 TAD at Fosc/64.
    ADCON0 = ADC_VDD_Tgt_Ch;
    ADCON0bits.GO = 1;                      // start conversion.

}

/******************************************************************************
 * Function: void VppVddADCTMR1_Stop(void)       
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Shuts off Timer1, disables interrupt, and stops ADC.
 *
 * Side Effects:    None
 *
 * Overview:        
 *
 * Note:            
 *   
 *****************************************************************************/
void VppVddADCTMR1_Stop(void)
{
    // disable timer1
    PIE1bits.TMR1IE = 0;            // disable int
    T1CONbits.TMR1ON = 0;           // stop timer

	// Shut off ADC conversion in progress.
	ADCON0 = 0;

    // disable ADC
    PIR1bits.ADIF = 0;              // clear A/D Converter Interrupt Flag
    PIE1bits.ADIE = 0;              // ensure A/D Converter Interrupt disabled
}


/** EOF pk_isr.c ************************************************************/
