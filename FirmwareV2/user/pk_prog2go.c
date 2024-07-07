/*********************************************************************
 *
 *                  Microchip PICkit 2 v2 Flash Starter Kit
 *
 *********************************************************************
 * FileName:        pk_prog2go.c
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
 ********************************************************************


/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include "delays.h"
#include "system\typedefs.h"
#include "system\usb\usb.h"

#include "io_cfg.h"                 // I/O pin mapping
#include "user\pk_prog2go.h"
#include "user\pickit.h"
#include "user\pk_isr.h"




/** V A R I A B L E S ********************************************************/
#pragma udata   PK2GO=0x4B0
unsigned char eebuffer_idx;   
unsigned int ext_ee_addr;           // in multiples of 64 bytes.

#define osccal_save_h UARTStatus.TimerBaudLoadH
#define osccal_save_l UARTStatus.TimerBaudLoadL
#define checksum_bl_mr UARTStatus.TXbits
#define bandgap_save_h UARTStatus.LastRXByte

#pragma udata	ExtEE_Buffer=0x4C0   // end of USB RAM
unsigned char eebuffer[64];                      // buffer for 64-byte page writes/reads of External EE


/** P R I V A T E  P R O T O T Y P E S ***************************************/
void Wr64ToExtEE(void);
void Rd64FromExtEE(void);
unsigned char AddrExtEE(void);
void WrByteExtEE(unsigned char byteval);
unsigned char RdByteExtEE(void);
void ExitLearnMode(void);
void PK2GoStoreScriptInBuffer(void);
void CopyScriptToInbuffer(unsigned char len);
void PK2GoWriteDownloadDataBuffer(void);
void ReadOSCCAL(unsigned char addr_l, unsigned char addr_h);
void WriteOSCCAL(unsigned char addr_l, unsigned char addr_h);
void CalcUploadChecksum(void);
char CheckDeviceID(void);
void ReadBandGap(void);
void WriteCfgBandGap(void);

/** D E C L A R A T I O N S **************************************************/

#pragma code


/******************************************************************************
 * Function:        void PK2GoInit(void)
 *
 * Overview:        Hardware and Software initializations necessary for 
 *                  Programmer-To-Go functionality
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          eebuffer_idx = 0
 *                  ext_ee_addr = 0
 *                  pk2go_error_code = 0
 *
 * Side Effects:    Sets up MSSP module for Master I2C       
 *
 * Note:            None
 *****************************************************************************/
void PK2GoInit(void)
{
    char i;

    eebuffer_idx = 0;
    ext_ee_addr = 0;
    pk2go_error_code = 0; // no errors

    // Init MSSP module for Master I2C
    SSPCON2 = 0;
    SSPSTAT = 0;                    // 400kHz slew rate, I2C inputs
    SSPCON1 = 0x28;                 // SSP enabled, Master mode.
    SSPADD = 31;                    // about 390kHz  

    SSPCON2bits.PEN = 1; //send stop  
    while (SSPCON2bits.PEN); //wait for stop

}

/******************************************************************************
 * Function:        void Wr64ToExtEE(void)
 *
 * Overview:        Writes the 64 bytes in eebuffer to the external EEPROMs at
 *                  address ext_ee_addr*64
 *
 * PreCondition:    None
 *
 * Input:           ext_ee_addr
 *                  eebuffer = data to write
 *
 * Output:          eebuffer_idx = 0
 *                  ext_ee_addr++
 *
 * Side Effects:    Leaves EEPROMS write protected.       
 *
 * Note:            None
 *****************************************************************************/
void Wr64ToExtEE(void)
{
    unsigned char i2c_temp;

    WP = 0; // deassert WP

    AddrExtEE();

    // Send Data
    eebuffer_idx = 0;
    do
    {
        PIR1bits.SSPIF = 0;
        SSPBUF = eebuffer[eebuffer_idx++];
        // wait for it to transmit
        while (!PIR1bits.SSPIF);
        if (SSPCON2bits.ACKSTAT)
        {  // no acknowledge - abort
            SSPCON2bits.PEN = 1; //send stop
        } 
    } while (eebuffer_idx < 64);

    SSPCON2bits.PEN = 1; //send stop  
    while (SSPCON2bits.PEN); //wait for stop

    Delay10KTCYx(6);    // wait 5ms for write.

    eebuffer_idx = 0;       // reset buffer pointer
    ext_ee_addr++;          // inc address.

    WP = 1; // assert WP
}

/******************************************************************************
 * Function:        void Rd64FromExtEE(void)
 *
 * Overview:        Reads 64 bytes from the external EEPROMs at address 
 *                  ext_ee_addr*64 into the eebuffer buffer.
 *
 * PreCondition:    None
 *
 * Input:           ext_ee_addr
 *
 * Output:          eebuffer_idx = 0
 *                  eebuffer = 64 bytes from EEPROM
 *                  ext_ee_addr++
 *
 * Side Effects:    Leaves EEPROMS write protected.       
 *
 * Note:            None
 *****************************************************************************/
void Rd64FromExtEE(void)
{
    unsigned char i2c_read;

    i2c_read = AddrExtEE() | 0x01;

    // Send Restart
    SSPCON2bits.RSEN = 1;
    while (SSPCON2bits.RSEN); //wait for restart 

    // Send I2C Address Read
    PIR1bits.SSPIF = 0;
    SSPBUF = i2c_read;
    // wait for it to transmit
    while (!PIR1bits.SSPIF);
    if (SSPCON2bits.ACKSTAT)
    {  // no acknowledge - abort
        SSPCON2bits.PEN = 1; //send stop
    }   

    // Receive Data
    eebuffer_idx = 0;
    do
    {
        PIR1bits.SSPIF = 0;
        SSPCON2bits.RCEN = 1;           // start byte reception
        while (!PIR1bits.SSPIF);        // wait for byte to come in.
        eebuffer[eebuffer_idx++] = SSPBUF;        //read byte into USB buffer
        if (eebuffer_idx == 64)                  // last byte
            SSPCON2bits.ACKDT = 1;      // NACK last byte
        else      
            SSPCON2bits.ACKDT = 0;      // ACK
        SSPCON2bits.ACKEN = 1;          // acknowledge sequence
        while(SSPCON2bits.ACKEN);           // wait for end ack sequence
    } while (eebuffer_idx < 64);

    SSPCON2bits.PEN = 1; //send stop
    while (SSPCON2bits.PEN); //wait for stop
    eebuffer_idx = 0;
    ext_ee_addr++;          // inc address.
}

/******************************************************************************
 * Function:        unsigned char AddrExtEE(void)
 *
 * Overview:        Completes the I2C address byte and 2 EE address bytes
 *
 * PreCondition:    None
 *
 * Input:           -
 *
 * Output:          returns the I2C address byte value
 *
 * Side Effects:    None       
 *
 * Note:            None
 *****************************************************************************/
unsigned char AddrExtEE(void)
{
    unsigned char i2c_temp;  
    unsigned char i2c_addr = 0xA0;     // init for I2C control code byte

    if (pk2go_memsize == 0)
    { // 2 x 24LC512
            // Check for which EEPROM:
        if (ext_ee_addr & 0x400)
        { // location in upper 64K
            i2c_addr |= 0x2;        // Set A0 bit
        }
    }
    else
    { // 2 x 24LC1025
        // Check for which EEPROM:
        if (ext_ee_addr & 0x800)
        { // location in upper 128K
            i2c_addr |= 0x2;        // Set A0 bit
        }
        // Check for which Bank:
        if (ext_ee_addr & 0x400)
        { // location in upper bank
            i2c_addr |= 0x8;        // Set B0 bit
        }
    }

    // Send start bit
    SSPCON2bits.SEN = 1;
    while (SSPCON2bits.SEN); //wait for start 
    
    // Send I2C Control Code
    PIR1bits.SSPIF = 0;
    SSPBUF = i2c_addr;
    // wait for it to transmit
        // generate high EE address while waiting
        i2c_temp = (ext_ee_addr >> 2) & 0xFF;
        while (!PIR1bits.SSPIF);
    if (SSPCON2bits.ACKSTAT)
    {  // no acknowledge - abort
        SSPCON2bits.PEN = 1; //send stop
    }  

    // Send High EE address
    PIR1bits.SSPIF = 0;
    SSPBUF = i2c_temp;
    // wait for it to transmit
        // generate low EE address while waiting
        i2c_temp = (ext_ee_addr << 6) & 0xFF;
        while (!PIR1bits.SSPIF);
    if (SSPCON2bits.ACKSTAT)
    {  // no acknowledge - abort
        SSPCON2bits.PEN = 1; //send stop

    }  

    // Send Low EE address
    PIR1bits.SSPIF = 0;
    SSPBUF = i2c_temp;
    // wait for it to transmit
    while (!PIR1bits.SSPIF);
    if (SSPCON2bits.ACKSTAT)
    {  // no acknowledge - abort
        SSPCON2bits.PEN = 1; //send stop
    }

    return i2c_addr;
}

/******************************************************************************
 * Function:        void WrByteExtEE(unsigned char byteval)
 *
 * Overview:        Writes a byte to the eebuffer.  When 64 bytes are written,
 *                  Wr64ToExtEE() is called to write them to the external EE
 *
 * PreCondition:    None
 *
 * Input:           byteval = byte to write
 *                  eebuffer_idx
 *
 * Output:          eebuffer[eebuffer_idx] = byteval
 *                  eebuffer_idx++
 *
 * Side Effects:    None       
 *
 * Note:            None
 *****************************************************************************/
void WrByteExtEE(unsigned char byteval)
{
    eebuffer[eebuffer_idx++] = byteval;
    if (eebuffer_idx > 63)
        Wr64ToExtEE();
}

/******************************************************************************
 * Function:        unsigned char RdByteExtEE(void)
 *
 * Overview:        Reads a byte from eebuffer.  If eebuffer is empty,
 *                  Rd64FromExtEE() is called to read from the external EE
 *
 * PreCondition:    None
 *
 * Input:           byteval = byte to write
 *                  eebuffer_idx
 *
 * Output:          returns eebuffer[eebuffer_idx]
 *                  eebuffer_idx++
 *
 * Side Effects:    None       
 *
 * Note:            None
 *****************************************************************************/
unsigned char RdByteExtEE(void)
{
    if (eebuffer_idx > 63)
    {
        Rd64FromExtEE();    
        eebuffer_idx = 0;
    }
    return eebuffer[eebuffer_idx++];
}

/******************************************************************************
 * Function:        void EnterLearnMode(unsigned char *usbindex)
 *
 * Overview:        Checks for key sequence and enters PK2GO Learn mode
 *
 * PreCondition:    None
 *
 * Input:           *usbindex - index to start address in USB buffer
 *
 * Output:          pk2go_memsize
 *                  PK2Go_Mode
 *
 * Side Effects:    Stops normal execution of USB commands     
 *
 * Note:            None
 *****************************************************************************/
void EnterLearnMode(unsigned char usbindex)
{
    PK2Go_Mode = PK2GO_MODE_OFF;

    if (CheckKeySequence(usbindex))
    {
        // set pk2go_memsize
        pk2go_memsize = inbuffer[usbindex + 3];

        PK2Go_Mode = PK2GO_MODE_LEARN;
        PK2GoInit();
        BUSY_LED = 1;
    }
}

/******************************************************************************
 * Function:        char CheckKeySequence(unsigned char usbindex)
 *
 * Overview:        Checks for key sequence 
 *
 * PreCondition:    None
 *
 * Input:           usbindex - index to start address in USB buffer
 *
 * Output:          return 1 for correct sequence, 0 otherwise
 *
 * Side Effects:       
 *
 * Note:            None
 *****************************************************************************/
char CheckKeySequence(unsigned char usbindex)
{
    // check for key sequence
    if (inbuffer[usbindex++] != 0x50)
        return 0;

    if (inbuffer[usbindex++] != 0x4B)
        return 0;

    if (inbuffer[usbindex++] != 0x32)
        return 0;

    return 1;
}

/******************************************************************************
 * Function:        void ExitLearnMode(void)
 *
 * Overview:        Completes last EEPROM write and exits mode
 *
 * PreCondition:    None
 *
 * Input:           eebuffer_idx
 *
 * Output:          PK2Go_Mode = PK2GO_MODE_OFF
 *
 * Side Effects:    Stops normal execution of USB commands     
 *
 * Note:            None
 *****************************************************************************/
void ExitLearnMode(void)
{
    do // complete last write of eebuffer
    {
        WrByteExtEE(END_OF_BUFFER);
    } while (eebuffer_idx > 0);

    PK2Go_Mode = PK2GO_MODE_OFF;
    BUSY_LED = 0;
}

/******************************************************************************
 * Function:        void PK2GoLearn(void)
 *
 * Overview:        Process a USB packet, storing into external EEPROM
 *
 * PreCondition:    PK2GoInit() executed
 *
 * Input:           inbuffer
 *
 * Output:          None
 *
 * Side Effects:    None      
 *
 * Note:            None
 *****************************************************************************/
 void PK2GoLearn(void)
{
    unsigned char usb_idx = 0;		// index of USB buffer
    unsigned char temp = 0;

	do
		{
			switch(inbuffer[usb_idx])        // parse buffer for commands			
			{
                case END_OF_BUFFER:
                    usb_idx = 64;           // ignore the rest of the USB packet.
                    break; 

                case EXIT_LEARN_MODE:      // Exit PK2GO Learn Mode
                    // format:      0xB6
					// response:	-
                    usb_idx = 64;           // ignore the rest of the USB packet.
                    ExitLearnMode();
                    break; 

                case WR_INTERNAL_EE:        // ignore
                    // format:      0xB1 <address><datalength><data1><data2>....<dataN>
                    usb_idx+=2;
                    usb_idx+= inbuffer[usb_idx] + 1;
                    break; 

                case RD_INTERNAL_EE:        // ignore
                    // format:      0xB2 <address><datalength>
                    usb_idx+=3;
                    break; 

                case SETVDD:
                    // format:      0xA0 <CCPL><CCPH><VDDLim>
                    // response:    -            
                case SETVPP:
                    // format:      0xA1 <CCPR2L><VPPADC><VPPLim>
                    // response:    -
                    WrByteExtEE(inbuffer[usb_idx++]);
                    WrByteExtEE(inbuffer[usb_idx++]);
                    WrByteExtEE(inbuffer[usb_idx++]);
                    WrByteExtEE(inbuffer[usb_idx++]);
                    break;  

                case SET_VOLTAGE_CALS:      // ignore
                    // format:      0xB0 <adc_calfactorL><adc_calfactorH><vdd_offset><calfactor>
                    // response:    -
                    usb_idx+=5;
                    break;  

				case DOWNLOAD_SCRIPT:		// Store a script in the Script Buffer
					// format:		0xA4 <Script#><ScriptLengthN><Script1><Script2>....<ScriptN>
					// response:	-
					WrByteExtEE(inbuffer[usb_idx++]); // A4
					WrByteExtEE(inbuffer[usb_idx++]); // Script#
                    temp = inbuffer[usb_idx++]; // script length
                    WrByteExtEE(temp);
                    do
                        {
                            WrByteExtEE(inbuffer[usb_idx++]);
                            temp--;
                        } while (temp > 0);
					break;	

                case EXECUTE_SCRIPT:        // immediately executes the included script
                    // format:      0xA6 <ScriptLengthN><Script1><Script2>....<ScriptN>
					// response:	-
					WrByteExtEE(inbuffer[usb_idx++]); // A6
                    temp = inbuffer[usb_idx++]; // ScriptLengthN
                    WrByteExtEE(temp);
                    do
                        {
                            WrByteExtEE(inbuffer[usb_idx++]);
                            temp--;
                        } while (temp > 0);
					break; 




                case DOWNLOAD_DATA:         // add data to download buffer
                    // format:      0xA8 <datalength><data1><data2>....<dataN>
					// response:	-
					WrByteExtEE(inbuffer[usb_idx++]); // A8
                    temp = inbuffer[usb_idx++]; // datalength
                    WrByteExtEE(temp);
                    do
                        {
                            WrByteExtEE(inbuffer[usb_idx++]);
                            temp--;
                        } while (temp > 0);
					break;  

                case CLR_DOWNLOAD_BUFFER:   // empties the download buffer
                    // format:      0xA7
					// response:	-
                case CLR_UPLOAD_BUFFER:   // empties the upload buffer
                    // format:      0xA9
					// response:	-
                case UPLOAD_DATA:       // reads data from upload buffer   
                    // format:      0xAA
                    // response:    <DataLengthN><data1><data2>....<dataN>
                case CLR_SCRIPT_BUFFER:
                    // format:      0xAB
					// response:	-
                case UPLOAD_DATA_NOLEN:   // reads data from upload buffer   
                    // format:      0xAC
                    // response:    <data1><data2>....<dataN>
                // META-COMMANDS--------------------------------
                case READ_BANDGAP:      //
                case WRITE_CFG_BANDGAP:      //
                    WrByteExtEE(inbuffer[usb_idx++]);
                    break;   

                case READ_STATUS:
                case READ_VOLTAGES:         //ignore
                case RESET:                 // ignore
                    // format:      0xAE
					// response:	-
                case SCRIPT_BUFFER_CHKSM:   // ignore
                    usb_idx++;
                    break; 

                case RUN_SCRIPT:            // run a script from the script buffer
                    // format:      0xA5 <Script#><iterations>
					// response:	-
                // META-COMMANDS--------------------------------
                case START_CHECKSUM:    // clear checksum
                case VERIFY_CHECKSUM:
                case CHANGE_CHKSM_FRMT:
                case READ_OSCCAL:       // read OSCCAL value
                case WRITE_OSCCAL:      // write OSCCAL value
                    WrByteExtEE(inbuffer[usb_idx++]);
                    WrByteExtEE(inbuffer[usb_idx++]);
                    WrByteExtEE(inbuffer[usb_idx++]);
                    break; 
	            // META-COMMAND---------------------------------
                case CHECK_DEVICE_ID: 
                    WrByteExtEE(inbuffer[usb_idx++]);
                    WrByteExtEE(inbuffer[usb_idx++]);
                    WrByteExtEE(inbuffer[usb_idx++]);
                    WrByteExtEE(inbuffer[usb_idx++]);
                    WrByteExtEE(inbuffer[usb_idx++]);
                    break; 

				default:					// extraneous command
					usb_idx++;			    // ignore
			} // end switch
		} while (usb_idx < 64); // end DO
}

/******************************************************************************
 * Function:        void PK2GoExecute(void)
 *
 * Overview:        Processes commands and data from external EEPROM
 *
 * PreCondition:    
 *
 * Input:           
 *
 * Output:          None
 *
 * Side Effects:    None      
 *
 * Note:            None
 *****************************************************************************/
void PK2GoExecute(void)
{
    unsigned char NextCmd;
    unsigned char temp;

    PK2GoInit();
    eebuffer_idx = 64; // force a read on first access

    do
    {
        NextCmd = RdByteExtEE();

		switch(NextCmd) 		
		{

            // META-COMMANDS--------------------------------
            case READ_OSCCAL:       // read OSCCAL value
                temp = RdByteExtEE();
                ReadOSCCAL(temp, RdByteExtEE());
                break; 

            case WRITE_OSCCAL:       // write OSCCAL value
                temp = RdByteExtEE();
                WriteOSCCAL(temp, RdByteExtEE());
                break; 

            case START_CHECKSUM: 
                checksum_l = 0;
                checksum_h = 0; 
            case CHANGE_CHKSM_FRMT: 
                checksum_bl_mr = RdByteExtEE();
                RdByteExtEE(); // 2nd arg is not used
                break;

            case VERIFY_CHECKSUM:       // sets checksum to 0 if match.
                checksum_l = ~checksum_l;
                checksum_h = ~checksum_h;
                checksum_l ^= RdByteExtEE();
                checksum_h ^= RdByteExtEE();
                break; 

            case CHECK_DEVICE_ID:
                asm_temp5 = RdByteExtEE(); // mask_l
                asm_temp6 = RdByteExtEE(); // mask_h
                asm_temp7 = RdByteExtEE(); // val_l
                asm_temp8 = RdByteExtEE(); // valu_h
                if (!CheckDeviceID())
                {
                    pk2go_error_code = 1; // device ID error.
                    NextCmd = END_OF_BUFFER;			// Stop processing.
                }
                break;

            case READ_BANDGAP:
                ReadBandGap();
                break; 

            case WRITE_CFG_BANDGAP:
                WriteCfgBandGap();
                break; 

            // NORMAL COMMANDS--------------------------------
            case NO_OPERATION:          // Do nothing
                // format:      0x5A
                break;

            case SETVDD:
                // format:      0xA0 <CCPL><CCPH><VDDLim>
                //      CCPH:CCPL = ((Vdd * 32) + 10.5) << 6     where Vdd is desired voltage
                //      VDDLim = (Vfault / 5) * 255              where Vdd < VFault is error
                // response: -
                temp = RdByteExtEE();
                CalAndSetCCP1(RdByteExtEE(), temp);
                VddVppLevels.VddThreshold = CalThresholdByte(RdByteExtEE());          // Set error threshold
                break;                 

            case SETVPP:
                // format:      0xA1 <CCPR2L><VPPADC><VPPLim>
                //      CCPR2L = duty cycle.  Generally = 0x40
                //      VPPADC = Vpp * 18.61        where Vpp is desired voltage.
                //      VPPlim = Vfault * 18.61              where Vdd < VFault is error
                // response:    -
				Vpp_PWM.CCPRSetPoint = RdByteExtEE();
				Vpp_PWM.UppperLimit = CalThresholdByte(RdByteExtEE())+1;
				Vpp_PWM.LowerLimit = Vpp_PWM.UppperLimit - 2;   
                VddVppLevels.VppThreshold = CalThresholdByte(RdByteExtEE());
                break;    

			case DOWNLOAD_SCRIPT:		// Store a script in the Script Buffer
				// format:		0xA4 <Script#><ScriptLengthN><Script1><Script2>....<ScriptN>
				// response:	-
				PK2GoStoreScriptInBuffer();
				break;	

            case RUN_SCRIPT:            // run a script from the script buffer
                // format:      0xA5 <Script#><iterations>
				// response:	-
                temp = RdByteExtEE();
                RunScript(temp, RdByteExtEE());
                break;  

            case EXECUTE_SCRIPT:        // immediately executes the included script
                // format:      0xA6 <ScriptLengthN><Script1><Script2>....<ScriptN>
				// response:	-
                temp = RdByteExtEE(); // length
                CopyScriptToInbuffer(temp);
                ScriptEngine(&inbuffer[0], temp);
                break;  

            case CLR_DOWNLOAD_BUFFER:   // empties the download buffer
                // format:      0xA7
				// response:	-
                ClearDownloadBuffer();
                break;  

            case DOWNLOAD_DATA:         // add data to download buffer
                // format:      0xA8 <datalength><data1><data2>....<dataN>
				// response:	-
                PK2GoWriteDownloadDataBuffer();
                break;  

            case CLR_UPLOAD_BUFFER:   // empties the upload buffer
                // format:      0xA9
				// response:	-
                ClearUploadBuffer(); 
                break;  

            case UPLOAD_DATA_NOLEN:   // reads data from upload buffer 
            case UPLOAD_DATA:       // reads data from upload buffer   
                // format:      0xAA
                // response:    <DataLengthN><data1><data2>....<dataN>
                CalcUploadChecksum();
                break;  

            case CLR_SCRIPT_BUFFER:
                // format:      0xAB
				// response:	-
                ClearScriptTable();
                break; 

			default:					// End of Buffer or unrecognized command
				NextCmd = END_OF_BUFFER;			// Stop processing.
	    } // end switch
    } while(NextCmd != END_OF_BUFFER);

}

/******************************************************************************
 * Function:        char CheckDeviceID(void)
 *
 * Overview:        Reads and checks the target Device ID
 *
 * PreCondition:    None
 *
 * Input:           asm_temp2/1 = deviceID mask
 *                  asm_temp4/3 = deviceID value
 *
 * Output:          returns 1 if device ID matches, 0 if not
 *
 * Side Effects:    
 *
 * Note:            None
 *****************************************************************************/
char CheckDeviceID(void)
{
    // clear upload buffer
    ClearUploadBuffer(); 
    RunScript(0, 1); // program entry
    RunScript(2, 1); // Read Device ID
    RunScript(1, 1); // program exit
    // apply mask
    asm_temp5 &= uc_upload_buffer[uploadbuf_mgmt.read_index++];
    asm_temp6 &= uc_upload_buffer[uploadbuf_mgmt.read_index++];
    // reclear upload buffer
    ClearUploadBuffer(); 
    // check value
    if ((asm_temp5 == asm_temp7) && (asm_temp6 == asm_temp8))
    {
        return 1; // pass
    }
    return 0; //fail
}

/******************************************************************************
 * Function:        void CalcUploadChecksum(void)
 *
 * Overview:        Adds the bytes in the Upload buffer to checksum_h\l
 *
 * PreCondition:    None
 *
 * Input:           checksum_h\l
 *
 * Output:          checksum_h\l
 *
 * Side Effects:    Empties Upload buffer
 *
 * Note:            None
 *****************************************************************************/
void CalcUploadChecksum(void)
{
    unsigned int *checksum = &checksum_l;
    unsigned char i, temp;

    for (i = 0; i < uploadbuf_mgmt.used_bytes; i++)
    {
        temp = uc_upload_buffer[uploadbuf_mgmt.read_index++];
        if (checksum_bl_mr)
        { // clear start/stop bits.
            if (i & 0x1)
            {
                if (checksum_bl_mr & 0x01)
                    temp &= 0x7F; // flash
                else
                    temp &= 0x01; // eeprom
            }
            else
                temp &= 0xFE;
        }
	    *checksum += temp;
    }

    ClearUploadBuffer();

}

/******************************************************************************
 * Function:        void ReadBandGap(void)
 *
 * Overview:        Reads Badgap from target device config word
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          bandgap_save_h (masked with 0x30 unshifted)
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void ReadBandGap(void)
{
    // clear upload buffer
    ClearUploadBuffer(); 
    RunScript(0, 1); // program entry
    RunScript(13, 1); // CONFIG_RD Read
    RunScript(1, 1); // program exit
    bandgap_save_h = uc_upload_buffer[1] & 0x60;
    // reclear upload buffer
    ClearUploadBuffer(); 
}

/******************************************************************************
 * Function:        void WriteCfgBandGap(void)
 *
 * Overview:        Writes config word (expected in DL buffer)
 *                  with bandgap_save_h OR'd in. Expects 
 *                  that prog entry / exit are in DL stream.
 *
 * PreCondition:    config word in buffer must have all BG bits set to 0
 *
 * Input:           bandgap_save_h
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void WriteCfgBandGap(void)
{
    uc_download_buffer[1] |= bandgap_save_h;
    RunScript(15, 1); // CONFIG_WR 
}

/******************************************************************************
 * Function:        void ReadOSCCAL(unsigned char addr_l, unsigned char addr_h)
 *
 * Overview:        Reads OSCCAL from target device
 *
 * PreCondition:    None
 *
 * Input:           addr_l, addr_h = OSCCAL address
 *
 * Output:          osccal_save_x
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void ReadOSCCAL(unsigned char addr_l, unsigned char addr_h)
{
    // clear download buffer
    ClearDownloadBuffer();
    // clear upload buffer
    ClearUploadBuffer(); 
    // Put address in DL buffer
    uc_download_buffer[downloadbuf_mgmt.write_index++] = addr_l;
    uc_download_buffer[downloadbuf_mgmt.write_index++] = addr_h;
    uc_download_buffer[downloadbuf_mgmt.write_index++] = 0;
    downloadbuf_mgmt.used_bytes += 3;
    RunScript(0, 1); // program entry
    RunScript(20, 1); // OSCCAL Read
    RunScript(1, 1); // program exit
    osccal_save_l = uc_upload_buffer[uploadbuf_mgmt.read_index++];
    osccal_save_h = uc_upload_buffer[uploadbuf_mgmt.read_index++];
    // reclear upload buffer
    ClearUploadBuffer(); 
}

/******************************************************************************
 * Function:        void WriteOSCCAL(unsigned char addr_l, unsigned char addr_h)
 *
 * Overview:        Writes saved OSCCAL from target device
 *
 * PreCondition:    None
 *
 * Input:           addr_l, addr_h = OSCCAL address
 *
 * Output:          
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void WriteOSCCAL(unsigned char addr_l, unsigned char addr_h)
{
    // clear download buffer
    ClearDownloadBuffer();
    // clear upload buffer
    ClearUploadBuffer(); 
    // Put address & osccal in DL buffer
    uc_download_buffer[downloadbuf_mgmt.write_index++] = addr_l;
    uc_download_buffer[downloadbuf_mgmt.write_index++] = addr_h;
    uc_download_buffer[downloadbuf_mgmt.write_index++] = 0;
    uc_download_buffer[downloadbuf_mgmt.write_index++] = osccal_save_l;
    uc_download_buffer[downloadbuf_mgmt.write_index++] = osccal_save_h;
    downloadbuf_mgmt.used_bytes += 5;
    RunScript(0, 1); // program entry
    RunScript(21, 1); // OSCCAL write
    RunScript(1, 1); // program exit
}

/******************************************************************************
 * Function:        void PK2GoStoreScriptInBuffer(void)
 *
 * Overview:        Stores the script from eebuffer into Script Buffer & updates
 *                  the Script Table.
 *                  Prior script at the given script # is deleted and all following
 *                  scripts are moved up.  New script is appended at end.
 *
 * PreCondition:    None
 *
 * Input:           
 *
 * Output:          uc_script_buffer[] - updated
 *                  ScriptTable[] - updated
 *                  Pk2Status.ScriptBufOvrFlow - set if script length > remaining buffer
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void PK2GoStoreScriptInBuffer(void)
{
	int i;
	int LengthOfAllScripts;
	int Temp_1, Temp_2, Length;

    Temp_2 = RdByteExtEE();		// Script# of new script 
	Temp_1 = RdByteExtEE();  	// Length of new script
    Length = Temp_1;

	// First, make sure script length is valid
	if (Temp_1 > SCRIPT_MAXLEN)
	{
		Pk2Status.ScriptBufOvrFlow = 1;		// set error - script longer than max allowed
		return;
	}

	// calculate length of all scripts.
	LengthOfAllScripts = 0;
	for (i=0; i < SCRIPT_ENTRIES; i++)
	{
		LengthOfAllScripts += ScriptTable[i].Length;
	}
	LengthOfAllScripts -= ScriptTable[Temp_2].Length;	// don't count length of script being replaced
	if (Temp_1 > (SCRIPTBUFSPACE-LengthOfAllScripts)) // if there isn't enough room
	{
		Pk2Status.ScriptBufOvrFlow = 1;		// set error - not enough room in script buffer
		return;
	}	

	// Next, make sure script# is valid
	if (Temp_2 > (SCRIPT_ENTRIES-1))    // 0-31 valid
	{
		Pk2Status.ScriptBufOvrFlow = 1;		// set error - script# invalid
		return;
	}


	if (ScriptTable[Temp_2].Length != 0)  // If a script exists in that location
	{
		// Move space created by deleting existing script to end of buffer.
		Temp_1 = (SCRIPTBUFSPACE - ScriptTable[Temp_2].Length) - 1;  // last copy location.
		for (i=ScriptTable[Temp_2].StartIndex; i < Temp_1; i++)
		{
			*(uc_ScriptBuf_ptr + i) = *(uc_ScriptBuf_ptr + ScriptTable[Temp_2].Length + i);
		}
		// update script table entries
		for (i=0; i < SCRIPT_ENTRIES; i++)
		{
			if (ScriptTable[i].StartIndex > ScriptTable[Temp_2].StartIndex) // if script is in moved section
			{
				ScriptTable[i].StartIndex -= ScriptTable[Temp_2].Length;  // adjust by amount moved
			} 
		}
	}

	// Store new script at end of buffer
	ScriptTable[Temp_2].Length = Length;	// update Script Table Entry with new length.
	ScriptTable[Temp_2].StartIndex = LengthOfAllScripts;    // update entry with new index at end of buffer.
	for (i = 0; i < ScriptTable[Temp_2].Length; i++)
	{
		*(uc_ScriptBuf_ptr + LengthOfAllScripts + i) = 	RdByteExtEE();	
	}  

} // end void StoreScriptInBuffer

/******************************************************************************
 * Function:        void CopyScriptToInbuffer(unsigned char len)
 *
 * Overview:        Stores the script from eebuffer into the inbuffer
 *
 * PreCondition:    None
 *
 * Input:           
 *
 * Output:          inbuffer = script
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void CopyScriptToInbuffer(unsigned char len)
{
    unsigned char i;

    for (i = 0; i < len; i++)
    {
        inbuffer[i]= RdByteExtEE();
    }
}

/******************************************************************************
 * Function:        void PK2GoWriteDownloadDataBuffer(void)
 *
 * Overview:        Writes a given # of bytes into the data download buffer.
 *
 * PreCondition:    None
 *
 * Input:           
 *
 * Output:          uc_download_buffer[] - updated with new data
 *                  downloadbuf_mgmt.write_index - incremented by length of data stored.
 *                  downloadbuf_mgmt.used_bytes - incremented by length of data stored. 
 *                  Pk2Status.DownloadOvrFlow - set if data length > remaining buffer
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void PK2GoWriteDownloadDataBuffer(void)
{
    unsigned int i, numbytes;

    numbytes = RdByteExtEE() & 0xFF;   // i= # bytes data (length)

    if ((numbytes + downloadbuf_mgmt.used_bytes)  > DOWNLOAD_SIZE)     // not enough room for data
    {
        Pk2Status.DownloadOvrFlow = 1;
        return;
    }

    for (i = 0; i < numbytes; i++)
    {
        uc_download_buffer[downloadbuf_mgmt.write_index++] = RdByteExtEE();
        if (downloadbuf_mgmt.write_index >= DOWNLOAD_SIZE) // handle index wrap
        {
            downloadbuf_mgmt.write_index = 0;
        }
        downloadbuf_mgmt.used_bytes++;  // used another byte.
    }
} // end void PK2GoWriteDownloadDataBuffer(void)

/******************************************************************************
 * Function:        void Pk2GoErrorBlink(unsigned char blinksX2)
 *
 * Overview:        Blinks the BUSY LED for the given # of blinks until
 *                  the pushbutton is pressed
 *
 * PreCondition:    None
 *
 * Input:           blinksX2 = twice the # of blinks in sequence
 *
 * Output:          
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void Pk2GoErrorBlink(unsigned char blinksX2)
{
    char i = 0;

    while(PROG_SWITCH_pin)
    {
        if (i <= blinksX2)
            BUSY_LED = i++ & 0x1;
        else
            i = 0;
        Delay10KTCYx(240);      // 200ms
    }
}
