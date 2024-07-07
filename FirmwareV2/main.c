/*********************************************************************
 *
 *                Microchip USB C18 Firmware
 *
 *********************************************************************
 * FileName:        main.c
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
 * Rawin Rojvanit       7/21/04     Original
 * Steven Bible      2005-04-20     Version 1 Released
 ********************************************************************/

/** I N C L U D E S **********************************************************/

#include <p18cxxx.h>
#include "system\typedefs.h"                        // Required
#include "system\usb\usb.h"                         // Required
#include "io_cfg.h"                                 // Required

#include "system\usb\usb_compile_time_validation.h" // Optional
#include "user\pickit.h"
#include "user\pk_isr.h"


/** V A R I A B L E S ********************************************************/

#pragma udata


/** P R I V A T E  P R O T O T Y P E S ***************************************/

static void InitializeSystem(void);
void USBTasks(void);


/** V E C T O R  R E M A P P I N G *******************************************/

extern void _startup (void);        // See c018i.c in your C18 compiler dir
#pragma code _RESET_INTERRUPT_VECTOR = 0x002000
void _reset (void)
{
    _asm goto _startup _endasm
}

#pragma code _HIGH_INTERRUPT_VECTOR = 0x002008
void _high_ISR (void)
{
    _asm
        goto InterruptHandler       // jump to interrupt routine
    _endasm
}

#pragma code InterruptVectorLow = 0x002018
void InterruptVectorLow (void)
{
    _asm
        goto InterruptHandler       // jump to interrupt routine
    _endasm
}


/** D E C L A R A T I O N S **************************************************/

#pragma code


/******************************************************************************
 * Function:        void main(void)
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
void main(void)
{
    InitializeSystem();

    while(1) {

        //USBTasks();         // USB Tasks
        USBDriverService();                 // polling method
        ProcessIO();        // See user\pickit.c & .h

    } // end while

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
    
    PICkitInit();                           // See pickit.c and pickit.h

    USBCheckBusStatus(); 

} // end InitializeSystem



/*****************************************************************************
 * The following is a value stored in upper program memory to let the boot-
 * loader know if an application program is loaded.  If the bootloader does
 * not read this code, it jumps to bootloader mode.
 ****************************************************************************/

//#pragma romdata _bootcode = 0x7FFE
//rom word bootcode = 0x5555;

/** EOF main.c ***************************************************************/
