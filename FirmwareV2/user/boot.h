/*********************************************************************
 *
 *      Microchip USB C18 Firmware -  USB Bootloader Version 1.00
 *
 *********************************************************************
 * FileName:        boot.h
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
#ifndef BOOT_H
#define BOOT_H

/** I N C L U D E S **********************************************************/
#include "system\typedefs.h"


/** D E F I N I T I O N S ****************************************************/
/****** Compiler Specific Definitions *******************************/

#define EECON1_RD       EECON1bits.RD
#define EECON1_WR       EECON1bits.WR


/****** Processor Specific Definitions ******************************/

#if defined(__18F2455) || defined(__18F2550) || \
    defined(__18F4455) || defined(__18F4550)

   /****** Remapped Vectors ********************
    *   _____________________
    *   |       RESET       |   0x000000
    *   |      LOW_INT      |   0x000008
    *   |      HIGH_INT     |   0x000018
    *   |       TRAP        |   0x000028
    *   |     Bootloader    |   0x00002E
    *   .                   .
    *   .                   .
    *   |     USER_RESET    |   0x002000
    *   |    USER_LOW_INT   |   0x002008
    *   |    USER_HIGH_INT  |   0x002018
    *   |      USER_TRAP    |   0x002028
    *   |                   |
    *   |   Program Memory  |
    *   .                   .
    *   |___________________|   0x0005FFF / 0x0007FFF
    */
    #define RM_RESET_VECTOR             0x002000
    #define RM_HIGH_INTERRUPT_VECTOR    0x002008
    #define RM_LOW_INTERRUPT_VECTOR     0x002018
#else
    "Processor not supported."
#endif

/* Bootloader Version */
#define MINOR_VERSION   0x01    // Bootloader Version
#define MAJOR_VERSION   0x02

/* State Machine */
#define WAIT_FOR_CMD    0x00    // Wait for Command packet
#define SENDING_RESP    0x01    // Sending Response


/** S T R U C T U R E S ******************************************************/

/*********************************************************************
 * General Data Packet Structure:
 * ACCESS RAM: Buffer Starting Address is assigned by compiler.
 * __________________           DATA_PACKET.FIELD
 * |    COMMAND     |   0       [CMD]
 * |      LEN       |   1       [LEN]
 * |     ADDRL      |   2       [        ]  [ADR.LOW]
 * |     ADDRH      |   3       [ADR.pAdr]: [ADR.HIGH]
 * |     ADDRU      |   4       [        ]  [ADR.UPPER]
 * |                |   5       [DATA]
 * |                |
 * .      DATA      .
 * .                .
 * |                |   62
 * |________________|   63
 *
 ********************************************************************/
#define EP1_BUF_SIZE 64

#define OVER_HEAD   5           // Overhead: <CMD_CODE><LEN><ADDR:3>
#define DATA_SIZE   (EP1_BUF_SIZE - OVER_HEAD)

typedef union _BOOT_DATA_PACKET
{
    char _byte[EP1_BUF_SIZE];
    struct
    {
        enum
        {
            READ_VERSION    = 'v',
            READ_FLASH      = 0x01,
            WRITE_FLASH     = 0x02,
            ERASE_FLASH     = 0x03,
            READ_EEDATA     = 0x04,
            WRITE_EEDATA    = 0x05,
            READ_CONFIG     = 0x06,
            WRITE_CONFIG    = 0x07,
            RESET           = 0xFF
        }CMD;                           // <COMMAND>
        char len;                       // <LEN>
        union
        {
            rom far char *pAdr;         // Address Pointer
            struct
            {
                char low;               // <ADDRL>  (little-endian)
                char high;              // <ADDRH>
                char upper;             // <ADDRU>
            };
        }ADR;
        char data[DATA_SIZE];
    };
} BOOT_DATA_PACKET;


/** B O O T L O A D E R  C O M M A N D  B U F F E R **************************/

extern volatile far BOOT_DATA_PACKET dataPacket;


/** E X T E R N S ************************************************************/



/** P U B L I C  P R O T O T Y P E S *****************************************/

void BootService(void);


#endif //BOOT_H
