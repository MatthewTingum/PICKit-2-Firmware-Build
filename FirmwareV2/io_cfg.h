/*********************************************************************
 *
 *                Microchip USB C18 Firmware - PICkit 2
 *
 *********************************************************************
 * FileName:        io_cfg.h
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
 * Steven Bible      2004-11-09     Initial Write
 * Steven Bible      2005-04-20     Version 1 Released
 ********************************************************************/

#ifndef IO_CFG_H
#define IO_CFG_H


/** I N C L U D E S *************************************************/
#include "autofiles\usbcfg.h"


/** T R I S *********************************************************/
#define INPUT_PIN           1
#define OUTPUT_PIN          0


/** U S B ***********************************************************/
#define usb_bus_sense       1       // device is always plugged in
#define self_power          0       // device is powered from USB


/** PICkit 2 ********************************************************/

    // ------
    // Port A
    // ------
                            // AN0 is analog input Vpp_FEEDBACK

                            // AN1 is analog input Vdd_TGT_FB

#define tris_ICSPDAT        TRISAbits.TRISA2    // AN2 Input/Output
#define ICSPDAT_in          PORTAbits.RA2
#define ICSPDAT_out         LATAbits.LATA2

#define tris_ICSPCLK        TRISAbits.TRISA3    // AN3 Input/Output
#define ICSPCLK_in          PORTAbits.RA3
#define ICSPCLK_out         LATAbits.LATA3

#define tris_AUX            TRISAbits.TRISA4    // AN4 Input/Output
#define AUX_in              PORTAbits.RA4
#define AUX                 LATAbits.LATA4

#define tris_MCLR_TGT       TRISAbits.TRISA5    // AN5 Output
#define MCLR_TGT            LATAbits.LATA5
#define MCLR_TGT_pin		PORTAbits.RA5		// WEK

    // ------
    // Port B
    // ------
                            // RB0 is I2C data (SDA)

                            // RB1 is I2C clock (SCL)

#define tris_Vpp_ON         TRISBbits.TRISB2    // RB2 Output
#define Vpp_ON              LATBbits.LATB2
#define Vpp_ON_pin			PORTBbits.RB2		// WEK

#define tris_Vdd_TGT_N      TRISBbits.TRISB3    // RB3 Output
#define Vdd_TGT_N           LATBbits.LATB3
#define Vdd_TGT_N_pin		PORTBbits.RB3		// WEK

#define tris_Vdd_TGT_P      TRISBbits.TRISB4    // RB4 Output
#define Vdd_TGT_P           LATBbits.LATB4
#define Vdd_TGT_P_pin		PORTBbits.RB4		// WEK

#define tris_PROG_SWITCH    TRISBbits.TRISB5    // RB5 Input (Weak Pull-up)
#define PROG_SWITCH_pin     PORTBbits.RB5

                            // RB6 is ICSPCLK (PGC)

                            // RB7 is ICSPDAT (PGD)
    // ------
    // Port C
    // ------

#define tris_BUSY_LED       TRISCbits.TRISC0    // RC0 Output
#define BUSY_LED            LATCbits.LATC0

#define tris_Vpp_PUMP       TRISCbits.TRISC1    // RC1 Output (CCP2)
#define Vpp_PUMP            LATCbits.LATC1

#define tris_Vdd_TGT_ADJ    TRISCbits.TRISC2    // RC2 Output (CCP1)
#define Vdd_TGT_ADJ         LATCbits.LATC2

                            // RC3 is Vusb

                            // RC4 is D-

                            // RC5 is D+

#define tris_WP             TRISCbits.TRISC6    // RC6 Output
#define WP                  LATCbits.LATC6

                            // RC7 is no connection


#endif //IO_CFG_H
