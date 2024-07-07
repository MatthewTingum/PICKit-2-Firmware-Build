/*********************************************************************
 *
 *                Microchip USB PICkit 2 Bootloader
 *
 *********************************************************************
 * FileName:        main.c
 * Dependencies:    See INCLUDES section below
 * Processor:       PIC18
 * Compiler:        C18 2.30.01+
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
 * Rawin Rojvanit     11/19/04      Original.
 * Steven Bible      2005-01-18     Adapted to PICkit 2
 * Steven Bible      2005-04-20     Version 1 Released
 ********************************************************************/

/** C O N F I G U R A T I O N   B I T S **************************************/

#pragma config PLLDIV = 5, CPUDIV = OSC1_PLL2, USBDIV = 2                   // CONFIG1L
#pragma config FOSC = HSPLL_HS, FCMEM = OFF, IESO = OFF                     // CONFIG1H
#pragma config PWRT = ON, BOR = OFF, BORV = 21, VREGEN = ON                 // CONFIG2L
#pragma config WDT = OFF, WDTPS = 32768                                     // CONFIG2H
#pragma config MCLRE = OFF, LPT1OSC = OFF, PBADEN = OFF, CCP2MX = ON        // CONFIG3H
#pragma config STVREN = ON, LVP = OFF, ICPRT = OFF, XINST = OFF, DEBUG = OFF// CONFIG4L
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF                   // CONFIG5L
#pragma config CPB = OFF, CPD = OFF                                         // CONFIG5H
#pragma config WRT0 = ON, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF                // CONFIG6L
#pragma config WRTB = ON, WRTC = OFF, WRTD = OFF                            // CONFIG6H
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF           // CONFIG7L
#pragma config EBTRB = OFF                                                  // CONFIG7H


/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include "system\typedefs.h"                        // Required
#include "system\usb\usb.h"                         // Required
#include "io_cfg.h"                                 // Required
#include "user\boot.h"
#include <delays.h>

#include "system\usb\usb_compile_time_validation.h" // Optional


/** V A R I A B L E S ********************************************************/

#pragma udata


/** P R I V A T E  P R O T O T Y P E S ***************************************/

static void InitializeSystem(void);
void USBTasks(void);


/** V E C T O R  R E M A P P I N G *******************************************/

#pragma code _HIGH_INTERRUPT_VECTOR = 0x000008
void _high_ISR (void)
{
    _asm goto RM_HIGH_INTERRUPT_VECTOR _endasm
}

#pragma code _LOW_INTERRUPT_VECTOR = 0x000018
void _low_ISR (void)
{
    _asm goto RM_LOW_INTERRUPT_VECTOR _endasm
}


/** D E C L A R A T I O N S **************************************************/

#pragma code


/******************************************************************************
 * Function:        void main (void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *****************************************************************************/
void main (void) {

    byte Temp;
    word Counter;

    //----------------------------------------
    // Check Bootload Mode Entry Condition
    //----------------------------------------

    tris_PROG_SWITCH = 1;           // RB4 Input
    PROG_SWITCH = 1;                // initialize port to 1
    INTCON2bits.NOT_RBPU = 0;       // initialize PORTB pull-ups on

    Delay10KTCYx(600);              // delay ~500 ms

    if (PROG_SWITCH) {         // is the push button pressed?

        // if no, does program memory location 0x7FFE contain 0x55?

        Temp = * ((rom far char *) 0x7FFE);

        if(Temp == 0x55) {

            _asm goto RM_RESET_VECTOR _endasm

        }

    } // end if (PROG_SWITCH)

    //----------------------------------------
    // Bootloader Mode
    //----------------------------------------

    InitializeSystem();

    while(1) {

        USBTasks();                 // USB Tasks
        BootService();              // See boot.c

        if (Counter == 0)           // blink Busy LED
            BUSY_LED ^= 1;

        Counter--;

    }

} // end main


/******************************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization routine.
 *                  All required USB initialization routines are called from
 *                  here.
 *
 *                  User application initialization routine should also be
 *                  called from here.                  
 *
 * Note:            None
 *****************************************************************************/
static void InitializeSystem(void)
{
    
    #if defined(USE_USB_BUS_SENSE_IO)       // see usbcfg.h
        tris_usb_bus_sense = INPUT_PIN;     // See io_cfg.h
    #endif
    
    #if defined(USE_SELF_POWER_SENSE_IO)    // see usbcfg.h
        tris_self_power = INPUT_PIN;
    #endif
    
    mInitializeUSBDriver();                 // See usbdrv.h

    BUSY_LED = 0;                           // Busy LED Off
    tris_BUSY_LED = 0;                      // Output

} // end InitializeSystem


/******************************************************************************
 * Function:        void USBTasks(void)
 *
 * PreCondition:    InitializeSystem has been called.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Service loop for USB tasks.
 *
 * Note:            None
 *****************************************************************************/
void USBTasks(void)
{
    /*
     * Servicing Hardware
     */

    USBCheckBusStatus();                    // Must use polling method

    if(UCFGbits.UTEYE!=1)
        USBDriverService();                 // Interrupt or polling method

} // end USBTasks


#pragma code user = RM_RESET_VECTOR

/** EOF main.c ***************************************************************/
