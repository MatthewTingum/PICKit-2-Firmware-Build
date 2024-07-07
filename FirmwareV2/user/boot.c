/*********************************************************************
 *
 *      Microchip USB C18 Firmware -  USB Bootloader Version 1.00
 *
 *********************************************************************
 * FileName:        boot.c
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
 * Rawin Rojvanit     11/19/04      Original. USB Bootloader
 * Steven Bible      2005-01-18     Adapted to PICkit 2
 * Steven Bible      2005-04-20     Version 1 Released
 ********************************************************************/

/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include "system\typedefs.h"
#include "system\usb\usb.h"
#include "io_cfg.h"
#include "user\boot.h"

/** V A R I A B L E S ********************************************************/
#pragma udata

byte counter;
byte byteTemp;
byte trf_state;

word big_counter;

volatile far BOOT_DATA_PACKET dataPacket;


/** P R I V A T E  P R O T O T Y P E S ***************************************/


/** D E C L A R A T I O N S **************************************************/
#pragma code

/** C L A S S  S P E C I F I C  R E Q ****************************************/

/** U S E R  A P I ***********************************************************/

void StartWrite(void) {

    /*
     * A write command can be prematurely terminated by MCLR or WDT reset
     */

    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1_WR = 1;

} // end StartWrite

//----------------------------------------

void ReadVersion(void)                      // TESTED: Passed
{
    dataPacket.data[0] = 'B';
    dataPacket.data[1] = MAJOR_VERSION;
    dataPacket.data[2] = MINOR_VERSION;

}//end ReadVersion

//----------------------------------------

void ReadProgMem(void)                      // TESTED: Passed
{
    for (counter = 0; counter < dataPacket.len; counter++)
    {
        // 2 separate inst prevents compiler from using RAM stack

        byteTemp = *((dataPacket.ADR.pAdr)+counter);
        dataPacket.data[counter] = byteTemp;

    }//end for
    
    TBLPTRU = 0x00;         // forces upper byte back to 0x00
                            // optional fix is to set large code model
}//end ReadProgMem

//----------------------------------------

void WriteProgMem(void)                     // TESTED: Passed
{
    /*
     * The write buffer for the 18F4550 family is 32-byte.
     */

//    dataPacket.ADR.low &= 0b11100000;  // Force 32-byte boundary

    EECON1 = 0b10000100;        // Setup writes: EEPGD=1, WREN=1

    // LEN = # of byte to write

    for (counter = 0; counter < (dataPacket.len); counter++) {

        *((dataPacket.ADR.pAdr)+counter) = dataPacket.data[counter];

        if (((counter & 0b00011111) == 0b00011111) || 
           (counter == ((dataPacket.len)-1))) {

            StartWrite();

        }

    }

} // end WriteProgMem

//----------------------------------------

void EraseProgMem(void)                     // TESTED: Passed
{
    // The most significant 16 bits of the address pointer points to the block
    // being erased. Bits 5:0 are ignored. (In hardware).

    // LEN = # of 64-byte block to erase

//    EECON1 = 0b10010100;     // Setup writes: EEPGD=1, FREE=1, WREN=1

    for(counter=0; counter < dataPacket.len; counter++)
    {
        EECON1 = 0b10010100;     // Setup writes: EEPGD=1, FREE=1, WREN=1
        *(dataPacket.ADR.pAdr+(((int)counter) << 6));  // Load TBLPTR
        StartWrite();

    }//end for

    TBLPTRU = 0;            // forces upper byte back to 0x00
                            // optional fix is to set large code model
                            // (for USER ID 0x20 0x00 0x00)
}//end EraseProgMem

//----------------------------------------

void ReadEE(void)                           // TESTED: Passed
{
    EECON1 = 0x00;
    for(counter=0; counter < dataPacket.len; counter++)
    {
        EEADR = (byte)dataPacket.ADR.pAdr + counter;
        EECON1_RD = 1;
        dataPacket.data[counter] = EEDATA;

    }//end for

}//end ReadEE

//----------------------------------------

void WriteEE(void)                          // TESTED: Passed
{
    for(counter=0; counter < dataPacket.len; counter++)
    {
        EEADR = (byte)dataPacket.ADR.pAdr + counter;
        EEDATA = dataPacket.data[counter];
        EECON1 = 0b00000100;    //Setup writes: EEPGD=0,WREN=1
        StartWrite();
        while(EECON1_WR);       //Wait till WR bit is clear

    }//end for

}//end WriteEE

//----------------------------------------

//WriteConfig is different from WriteProgMem because it can write a byte
void WriteConfig(void)                      // TESTED: Passed
{
    EECON1 = 0b11000100;        //Setup writes: EEPGD=1,CFGS=1,WREN=1
    for (counter = 0; counter < dataPacket.len; counter++)
    {
        *((dataPacket.ADR.pAdr)+counter) = dataPacket.data[counter];
        StartWrite();
    }//end for
    
    TBLPTRU = 0x00;         // forces upper byte back to 0x00
                            // optional fix is to set large code model
}//end WriteConfig


/******************************************************************************
 * Function:        void BootService (void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    counter is a global variable
 *
 * Overview:        
 *
 * Note:            None
 *****************************************************************************/
void BootService (void) {

    if (HIDRxReport(dataPacket._byte, EP1_BUF_SIZE) > 0) // USB receive buffer has data
    {
        counter = 0;
        switch(dataPacket.CMD)
        {
            case READ_VERSION:
                ReadVersion();
                counter=0x04;
                while(mHIDTxIsBusy()){}                 // blocking
                HIDTxReport(dataPacket._byte, EP1_BUF_SIZE);   // transmit packet

                break;

            case READ_FLASH:
            case READ_CONFIG:
                ReadProgMem();
                counter+=0x05;
                while(mHIDTxIsBusy()){}                 // blocking
                HIDTxReport(dataPacket._byte, EP1_BUF_SIZE);   // transmit packet

                break;

            case WRITE_FLASH:
                WriteProgMem();
                counter=0x01;
                break;

            case ERASE_FLASH:
                EraseProgMem();
                counter=0x01;
                break;

            case READ_EEDATA:
                ReadEE();
                counter+=0x05;
                while(mHIDTxIsBusy()){}                 // blocking
                HIDTxReport(dataPacket._byte, EP1_BUF_SIZE);   // transmit packet
                break;

            case WRITE_EEDATA:
                WriteEE();
                counter=0x01;
                break;

//            case WRITE_CONFIG:
//                WriteConfig();
//                counter=0x01;
//                break;
            
            case RESET:
                USBSoftDetach();
                big_counter = 0;
                while(--big_counter) {
                    Nop();
                    Nop();
                }
                Reset();
                break;

            default:                // unknown command - ignore
                break;

        }//end switch()

        // write commands are echoed
        // read commands are echoed with data read

//        while(mHIDTxIsBusy()){}                 // blocking
//        HIDTxReport(dataPacket._byte, EP1_BUF_SIZE);   // transmit packet

    }//end if

}//end BootService


/** EOF boot.c ***************************************************************/
