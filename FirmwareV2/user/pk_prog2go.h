/*********************************************************************
 *
 *                  Microchip PICkit 2 v2 Flash Starter Kit
 *
 *********************************************************************
 * FileName:        pk_prog2go.h
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
 * Walter Kicinski   2008-Feb-22    Initial Write
 ********************************************************************/

#ifndef PK2GO_H
#define PK2GO_H

/** D E F I N I T I O N S ****************************************************/


/** E X T E R N S ***************************************************/
#define checksum_l UARTStatus.RXbits
#define checksum_h UARTStatus.RXbyte
#define pk2go_error_code UARTStatus.TXByte
    // 0 = no errors
    // 1 = device ID error
#define pk2go_memsize UARTStatus.NewRX
    // 0 = 128K
    // 1 = 256K


/** P U B L I C  P R O T O T Y P E S *****************************************/
void PK2GoInit(void);
void EnterLearnMode(unsigned char usbindex);
char CheckKeySequence(unsigned char usbindex);
void PK2GoLearn(void);
void PK2GoExecute(void);
void Pk2GoErrorBlink(unsigned char blinksX2);


#endif // PICKIT_H