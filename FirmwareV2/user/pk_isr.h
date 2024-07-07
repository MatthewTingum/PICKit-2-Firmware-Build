/*********************************************************************
 *
 *                  Microchip PICkit 2 Flash Starter Kit
 *
 *********************************************************************
 * FileName:        vpp.h
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

#ifndef VPP_H
#define VPP_H

/** D E F I N I T I O N S ****************************************************/
                                    // (Fosc = 48 MHz, Postscale = 1:1, T2 Prescale = 1:1)
                                    // (at PR2 = 79 for 100% duty cycle CCPRxL:CCPxCON<5:4> = 320)
#define PWM_150kHz  79              // PWM period (PR2 register value)

                                    // PWM duty cycle for (CCPR1L register value)
#define Vdd_5V0     (192/4)         //  5.0V Vdd
#define Vdd_3V3     (116/4)         //  3.3V Vdd
#define Vdd_2V5     ( 88/4)         //  2.5V Vdd
#define Vdd_1V8     ( 64/4)         //  1.8V Vdd

#define VppPWM_Enable				0x0C	// turn on PWM (CCP2CON register value)
#define VppPWM_Disable				0x00    // turn off PWM (CCP2CON register value)
#define VppDutyCycleOff				0x00    // set DC = 0 (CCPR2L register value)

#define ADC_VDD_Tgt_Ch				0x05	// Set ADC channel for VDD_TGT_FB and turn on ADC (ADCON0)
#define ADC_VPP_FB_Ch				0x01	// Set ADC channel for VPP_FEEDBACK and turn on ADC (ADCON0)

#define ADC_LftJust_8TAD_FoscDiv64	0x26	// ADCON2 setting for voltage monitoring

#define ADCMonitorTmr1_Setup        0x80    // Rd16 1:1 prescale off Fosc/4
#define Timer1_Timeout_High         0xFA    // 65536-1480 = about 125us
#define Timer1_Timeout_Low          0x38
#define MaxErrorVDD                 12       // # conversions < error threshold to be seen as error.
#define MaxErrorVPP                 12
#define BusyBlinkRate               667     // 12Hz, so 0.083 / 125us

#define UARTStartBitAdjust          72      // about 6us

/** T Y P E S ****************************************************************/


/** P U B L I C  P R O T O T Y P E S *****************************************/

void InterruptHandler(void);
void VppVddADCTmr1_Start(void);
void VppVddADCTMR1_Stop(void);

/** E X T E R N S ***************************************************/


#endif // VPP_H
