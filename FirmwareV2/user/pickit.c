/*********************************************************************
 *
 *                  Microchip PICkit 2 v2 Flash Starter Kit
 *
 *********************************************************************
 * FileName:        pickit.c
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
 * Steven Bible      2004-11-04     Initial Write
 * Steven Bible      2005-04-20     Version 1 Released
 * Walter Kicinski   2006-May-15    Rewrite for v2
 ********************************************************************
 * Change log
 * v2.01 - WK - 
 *       initial v2 release
 * v2.02 - WK - May 2007
 *       Ensures BOR bits in CONFIG2L are disabled at reset
 * v2.10 - WK - Jun 2007
 *       Added ShiftBitsOutICSPHold(), and new script commands
 *          WRITE_BITS_LIT_HLD, WRITE_BITS_BUF_HLD
 *       Added new script commands to support Serial EEPROM devices:
 *          SET_AUX, AUX_STATE_BUFFER, I2C_START, I2C_STOP,
 *          I2C_WR_BYTE_LIT, I2C_WR_BYTE_BUF, I2C_RD_BYTE_ACK, 
 *          I2C_RD_BYTE_NACK, SPI_WR_BYTE_LIT, SPI_WR_BYTE_BUF,
 *          SPI_RD_BYTE_BUF
 *       Added new commands for calibration and internal EEPROM
 *          SET_VOLTAGE_CALS, WR_INTERNAL_EE, RD_INTERNAL_EE
 *       Added Serial Mode support and commands
 *          ENTER_UART_MODE, EXIT_UART_MODE
 * v2.20 - WK - Jul 2007
 *      Updated USB stack to v1.3
 *      Added ICDSlaveBL_transmit & ICDSlaveBL_Receive to handle new
 *      handshaking for baseline debug to get around 16F50x-ICD ICD pins
 *      drive issues.
 * v2.30 - WK - Apr 2008
 *      Removed USB serial number to allow multiple units on one PC
 *      No longer calls EnsureBOROff() at reset (may have been causing
 *                a dead PICkit 2 issue with USB VREG off)
 *      Added MEASURE_PULSE script command
 *      Added UNIO_TX and UNIO_TX_RX script commands
 *      UARTModeService() now monitors VDD, and shuts off if below voltage.
 *                However, no error / indication is set.
 *      Modified USB stack to free up space from unused/unneeded portions
 *      Added Programmer-To-Go support
 *      Added Logic Analyzer
 * v2.30.01 - WK - Apr 2008
 *      Fixed issue in I2C_Read() with duplicate lables in different
 *      ASM sections.
 * v2.31.00 - WK - Jun 2008
 *      Fixed issue with Programmer-To-Go.  ENTER_LEARN_MODE needs to set
 *      pk2go_memsize.
 *      Added code to jump tables to handle PCLATH and prevent 256-byte
 *      boundary concern.
 * v2.32.00 - WK - July 2008
 *      Now reports the UnitID string as the USB Serial Number string
 *      descriptor.  This was necessary to identify devices via Unit ID
 *      without HID transactions.  The latter can mess up existing
 *      communication links when searching for devices.
 *      Freed up program space by adding ScriptIdxIncJumpEnd.
 *      
 ********************************************************************/

/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include "delays.h"
#include "system\typedefs.h"
#include "system\usb\usb.h"

#include "io_cfg.h"                 // I/O pin mapping
#include "user\pickit.h"
#include "user\pk_isr.h"
#include "user\pk_prog2go.h"

/** V A R I A B L E S ********************************************************/
#pragma udata	ScriptBuffer					// in banks 5,6, & 7.
// NOTE: must use SECTION command in linker script.
unsigned char uc_script_buffer[SCRIPTBUF_SIZE];		// Script Buffer

#pragma udata	USB_Buffers
char inbuffer[BUF_SIZE];            				// input to USB device buffer
char outbuffer[BUF_SIZE];            				// output to USB device buffer

#pragma udata	DownloadBuffer
unsigned char 	uc_download_buffer[DOWNLOAD_SIZE];	// Download Data Buffer

#pragma udata	UploadBuffer
unsigned char 	uc_upload_buffer[UPLOAD_SIZE];		// Upload Data Buffer

#include    "pickit2.inc"
#pragma udata access	AssemblyAccess = INLINE_ASM_RAM
near unsigned char asm_temp1;
near unsigned char asm_temp2;
near unsigned char asm_temp3;
near unsigned char asm_temp4;
near unsigned char asm_temp5;
near unsigned char asm_temp6;
near unsigned char asm_temp7;
near unsigned char asm_temp8;
near unsigned char asm_temp9;
near unsigned char asm_temp10;
near unsigned char asm_temp11;
near unsigned char asm_temp12;

#pragma udata	Misc
unsigned char 	*uc_ScriptBuf_ptr = &uc_script_buffer[0];

struct {						// Script table - keeps track of scripts in the Script Buffer.
	unsigned char	Length;
	int	StartIndex;	// offset from uc_script_buffer[0] of beginning of script.
} ScriptTable[SCRIPT_ENTRIES];

unsigned char icsp_pins;
unsigned char icsp_baud;
unsigned char aux_pin;

struct {
    unsigned char	write_index;        // buffer write index
    unsigned char	read_index;			// buffer read index
    unsigned int    used_bytes;         // # bytes in buffer
} downloadbuf_mgmt;

#pragma udata	Misc2
union {		// Status bits
	struct {
		unsigned char	StatusLow;
		unsigned char	StatusHigh;
	};
	struct{
		// StatusLow
		unsigned VddGNDOn:1;	// bit 0
		unsigned VddOn:1;
		unsigned VppGNDOn:1;
		unsigned VppOn:1;
		unsigned VddError:1;
		unsigned VppError:1;
        unsigned ButtonPressed:1;
		unsigned :1;
		//StatusHigh
        unsigned Reset:1;       // bit 0
		unsigned UARTMode:1;			
        unsigned ICDTimeOut:1;
		unsigned UpLoadFull:1;
		unsigned DownloadEmpty:1;
		unsigned EmptyScript:1;
		unsigned ScriptBufOvrFlow:1;
		unsigned DownloadOvrFlow:1;
	};
} Pk2Status;

struct {
    unsigned char	write_index;		// buffer write index
    unsigned char	read_index;			// buffer read index
    unsigned int    used_bytes;         // # bytes in buffer
} uploadbuf_mgmt; 

#pragma udata	Misc3

struct {
	unsigned char   VddThreshold;	// error detect threshold
	unsigned char   VppThreshold;	// error detect threshold
    unsigned char   VddErrCount;
    unsigned char   VppErrCount;
    unsigned int    BlinkClount;    // counter for blinking Busy on VDD/VPP error
} VddVppLevels;

#pragma udata	Misc4
struct {
	unsigned char CCPRSetPoint;
	unsigned char UppperLimit;
	unsigned char LowerLimit;	
} Vpp_PWM;

struct {
    unsigned int    adc_calfactor;      // CalibratedResult = (ADRES * adc_calfactor) >> 8
    signed char	    vdd_offset;
    unsigned char	vdd_calfactor;		// Calibrated CCP value = (((CCP >> 6) + vdd_offset) * vdd_calfactor) >> 7
} VoltageCalibration; 

#pragma udata access	MiscAccess
near struct {
    unsigned char   TimerBaudLoadL;      // timer value to load for particular baud rate
    unsigned char   TimerBaudLoadH;
    unsigned char   RXbits;             // # bits received during byte reception
    unsigned char   RXbyte;             // Byte value being received
    unsigned char   TXbits;             // # bits send during byte transmission
    unsigned char   TXByte;             // Byte value being sent
    unsigned char   LastRXByte;         // Received byte to be saved
    unsigned char   LastRXByte2;         // Received byte to be saved
    unsigned char   LastRXByte3;         // Received byte to be saved
    unsigned char   NewRX;              // flag for new data in LastRXByte
} UARTStatus;

near unsigned char PK2Go_Mode;

/** P R I V A T E  P R O T O T Y P E S ***************************************/
void EnterBootloader(void);
void SendFWVersionUSB(void);
void StoreScriptInBuffer(unsigned char *usbindex);
void SetICSP_PinStates(unsigned char icsp_byte);
unsigned char GetICSP_PinStates(void);
void SetAUX_PinState(unsigned char aux_byte);
unsigned char GetAUX_PinState(void);
void ShiftBitsOutICSP(unsigned char outputbyte, char numbits);
void ShiftBitsOutICSPHold(unsigned char outputbyte, char numbits);
unsigned char ReadDownloadBuffer(void);
void WriteDownloadDataBuffer(unsigned char *usbindex);
void WriteUploadBuffer(unsigned char byte2write);
unsigned char ShiftBitsInICSP(unsigned char numbits);
void ShortDelay(unsigned char count);
void LongDelay(unsigned char count);
void SendStatusUSB(void);
void SendVddVppUSB(void);
void ADCConvert(unsigned char channel);
unsigned char ICDSlave_Receive (void);
void ICDSlave_transmit (unsigned char TransmitByte);
void SendScriptChecksumsUSB(void);
unsigned char ShiftBitsInPIC24(unsigned char numbits);
void WriteByteDownloadBuffer(unsigned char DataByte);
//void EnsureBOROff(void);
void USBHIDTxBlocking(void);

void I2C_Start(void);
void I2C_Stop(void);
void I2C_Write(unsigned char outputbyte);
unsigned char I2C_Read(unsigned char giveack);
unsigned char SPI_ReadWrite(unsigned char outputbyte);

void ICDSlaveBL_transmit (unsigned char TransmitByte);
unsigned char ICDSlaveBL_Receive (void);

void SaveCalFactorsToEE(void);
void ReadCalFactorsFromEE(void);
unsigned int CalADCWord(unsigned int rawValue);
void WriteInternalEEPROM(unsigned char *usbindex);
void ReadInternalEEPROM(unsigned char *usbindex);

void UARTModeService(void);
void EnterUARTMode(unsigned char *usbindex);
void ExitUARTMode(void);

int MeasurePulse(void);

void UNIO (unsigned char device_addr, unsigned char txbytes, unsigned char rxbytes);

extern void JTAG2W4PH(void);
void P32SetMode (unsigned char numbits, unsigned char value);
unsigned char P32DataIO (unsigned char numbits, unsigned char tdi, unsigned char tms);
void P32SendCommand (unsigned char command);
void P32XferData8 (unsigned char byte0);
unsigned char P32XferData32 (unsigned char byte3, unsigned char byte2, unsigned char byte1, unsigned char byte0, char rxdata);
void P32XferFastData32(unsigned char byte3, unsigned char byte2, unsigned char byte1, unsigned char byte0);
void P32XferInstruction(void);
void P32GetPEResponse(unsigned char savedata, unsigned char execute);

void PK2GoModeService(void);
void EnablePK2GoMode(unsigned char usbindex);

void LogicAnalyzer(void);
extern void LOGICPREP(void);
extern void ANALYZER_RISING_1MHZ(void);
extern void ANALYZER_FALLING_1MHZ(void);
extern void ANALYZER_SLOWRATE(void);
void CopyRamUpload(unsigned char addrl, unsigned char addrh);

/** D E C L A R A T I O N S **************************************************/

#pragma code pickitc = 0x202A


/******************************************************************************
 * Function:        void PickitInit(void)
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
 * Note:            None
 *****************************************************************************/
void PICkitInit(void)
{
    byte i;                         // index variable

    // set up LVD
    HLVDCON = 0x03;
    HLVDCONbits.HLVDEN;
    PIR2bits.HLVDIF = 0;
    PIE2bits.HLVDIE = 1;

    //-------------------------
    // initialize I/O (see io_cfg.h)
    //-------------------------

    // ------
    // Port A
    // ------

    TRISAbits.TRISA0 = 1;           // RA0/AN0 Analog Input (Vpp_FEEDBACK)

    TRISAbits.TRISA1 = 1;           // RA1/AN1 Analog Input (Vdd_TGT_FB)

    ADCON1 = 0x0D;                  // RA0/AN0 and RA1/AN1 are analog input
    ADCON2 = 0x03;                  // left justified, 0 Tad, Frc

    tris_ICSPDAT = 1;               // RA2 Input (tristate)
    ICSPDAT_in = 0;                 // initialize output port to 0 (low)
    ICSPDAT_out = 0;                // initialize input latch to 0 (low)

    tris_ICSPCLK = 1;               // RA3 Input (tristate)
    ICSPCLK_in = 0;                 // initialize output port to 0 (low)
    ICSPCLK_out = 0;                // initialize input latch to 0 (low)

    tris_AUX = 1;                   // RA4 Input (tristate)
    AUX = 0;                        // initialize output latch to 0 (low)

    MCLR_TGT = 0;                   // initialize MCLR_TGT off (no MCLR_TGT)
    tris_MCLR_TGT = 0;              // RA5 Output

    // ------
    // Port B
    // ------

    // (future) need to initialize I2C for RB0 (SDA) and RB1 (SCL)

    // *************************************************************
    // * WARNING * WARNING * WARNING * WARNING * WARNING * WARNING * 
    // * Do not turn on both half-bridge gates at the same time.   *
    // * It will cause a direct short of +V_TGT to GROUND.         *
    // *************************************************************

    Vdd_TGT_N = 0;                  // initialize half-bridge N-gate off
    tris_Vdd_TGT_N = 0;             // Output

    Vdd_TGT_P = 1;                  // initialize half-bridge P-gate off
    tris_Vdd_TGT_P = 0;             // Output

    tris_PROG_SWITCH = 1;           // Input
    PROG_SWITCH_pin = 1;            // initialize port to 1
    INTCON2bits.NOT_RBPU = 0;       // initialize PORTB pull-ups on

    Vpp_ON = 0;                     // initialize Vpp_ON to off
    tris_Vpp_ON = 0;                // Output

    // ------
    // Port C
    // ------

    BUSY_LED = 0;                   // initialize Busy LED to off
    tris_BUSY_LED = 0;              // Output

    Vpp_PUMP = 0;                   // initialize latch to 0 (low)
    tris_Vpp_PUMP = 0;              // RC1 Output (CCP2)

    Vdd_TGT_ADJ = 0;                // initialize latch to 0 (low)
    tris_Vdd_TGT_ADJ = 0;           // RC2 Output (CCP1)

    tris_WP = 0;                    // Output
    WP = 1;                         // initialize WP to protect (1)

    //-------------------------
    // initialize variables
    //-------------------------

    // Init UnitID buffer for USB
    // no Unit ID default
    i=2; // 4 bytes
    unit_id_usb_buf[1] = DSC_STR;   // USB descriptor string
    unit_id_usb_buf[2] = 0x09;
    unit_id_usb_buf[3] = 0x04;

    if (EE_ReadByte(UNIT_ID) == '#')
    {
        for (; i < (2 * UNITID_MAX_LEN); i+=2)
        {
            unit_id_usb_buf[i] = EE_ReadByte(UNIT_ID + (i/2));
            unit_id_usb_buf[i + 1] = 0;
            if (unit_id_usb_buf[i] == 0)
                break;
        }
    }
    unit_id_usb_buf[0] = i + 2;

    PK2Go_Mode = PK2GO_MODE_OFF;

	Pk2Status.StatusLow = 0;		// init status.
	Pk2Status.StatusHigh = 1;       // set reset bit.

	icsp_pins = 0x03;		// default inputs
	icsp_baud = 0x00;		// default fastest
    aux_pin = 0x01;         // default input

    /*for (i=0; i<BUF_SIZE; i++) {    // initialize input and output buffer to 0
        inbuffer[i]=0;
        outbuffer[i]=0;
    }*/

	ClearScriptTable();     // init script table to empty.

    ClearDownloadBuffer();

    ClearUploadBuffer(); 

    ReadCalFactorsFromEE();
    if ((VoltageCalibration.adc_calfactor > 0x140) || (VoltageCalibration.adc_calfactor < 0xC0)
          || (VoltageCalibration.vdd_calfactor > 0xAF) || (VoltageCalibration.vdd_calfactor < 0x50))
    { // bad cals
        VoltageCalibration.adc_calfactor = 0x0100;  // Set default values. 
        VoltageCalibration.vdd_offset = 0x00;
        VoltageCalibration.vdd_calfactor = 0x80;
    }

    // configure Timer0 (used by SCRIPT_ENGINE)

    T0CON = 0x08;                   // Timer0 off, 16-bit timer, 
                                    // internal clock, prescaler not assigned

    // Timer1 is used by VddVpp voltage monitoring interrupt or UART RX.

    // configure Timer2 (used by CCP1 and CCP2)

    T2CON = 0x04;                   // 1:1 Postscale, 1:1 Prescale, Timer2 on
    PR2 = PWM_150kHz;               // Timer2 Period Register (sets PWM Period for CCP1 and 2)

    // Timer3 is used for UART TX.

    // initialize CCP1 for Vdd_TGT_ADJ
    // (Fosc = 48 MHz, Postscale = 1:1, Timer2 Prescale = 1:1)

    CCP1CON = 0x0C;                 // PWM mode
    CCPR1L = Vdd_3V3;               // PWM duty cycle- start at 3.3v so we don't blow PIC 18J & 24 WEK

	// initialize CCP2 for VPP WEK
	Vpp_PWM.CCPRSetPoint = 64;
	Vpp_PWM.UppperLimit = 62;		// default to 3.3v so we don't blow PIC 18J & 24
	Vpp_PWM.LowerLimit = 60;
	VddVppLevels.VddThreshold = 126;	// about 2.5 v
	VddVppLevels.VppThreshold = 45;		// about 2.5 v

    // initialize Comparator and Voltage Reference Modules

    CMCON = 0x07;                   // Comparators off
    CVRCON = 0x00;                  // VR off


    // Check BOR bits in CONFIG1L
    // EnsureBOROff();

	// Set up Interrupts
    INTCONbits.PEIE = 1;            // enable Peripheral Interrupt Enable
    INTCONbits.GIE = 1;             // enable global interrupts

	// Start voltage monitoring
    VppVddADCTmr1_Start();

    // Check for PK2GO mode
    if ((EE_ReadByte(PK2GO_KEY1) == 0x50) && (EE_ReadByte(PK2GO_KEY2) == 0x4B) && (EE_ReadByte(PK2GO_KEY3) == 0x32))
        {
            // get EEPROM size
            pk2go_memsize = EE_ReadByte(PK2GO_MEM);

            PK2Go_Mode = PK2GO_MODE_GO;
            asm_temp1 = 0;          // for Power LED blink
                                // response:    -
            CalAndSetCCP1(0x11, 0x00); // about 1.8V
            VddVppLevels.VddThreshold = CalThresholdByte(64);          // Set error threshold
            inbuffer[0] = ~GETVERSION;
        }

}

/******************************************************************************
 * Function:        void USBHIDTxBlocking(void)
 *
 * Overview:        blocking USB transmission
 *
 * PreCondition:    None
 *
 * Input:           outbuffer
 *
 * Output:          Transmits HID Tx report with data.
 * 
 * Side Effects:    None
 *
 * Note:            
 *****************************************************************************/
void USBHIDTxBlocking(void)
{
    while(mHIDTxIsBusy()){}         // blocking
    HIDTxReport(outbuffer, 64);
    USBDriverService();
}

/******************************************************************************
 * Function:        void ProcessIO(void)
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
 * Note:            None
 *****************************************************************************/
void ProcessIO(void)
{
	unsigned char usb_idx = 0;		// index of USB buffer
    unsigned char usb_data = 0;

    // poll programming switch
    if (!PROG_SWITCH_pin)   // active low
        Pk2Status.ButtonPressed = 1;

    if (HIDRxReport(inbuffer, 64) > 0)
    {
        usb_data = 1;
    }

    // PK2GO mode service
    if (PK2Go_Mode == PK2GO_MODE_LEARN)
    {
        if (usb_data)
            PK2GoLearn();
        return;
    }
    if (PK2Go_Mode == PK2GO_MODE_GO)
    {
        if ((!usb_data) || (inbuffer[0] != GETVERSION))
        { 
            PK2GoModeService();
            return;
        }
    }

    // UART Mode service
    if (Pk2Status.UARTMode)
    {
        UARTModeService();
    }

    if (usb_data)  // USB receive buffer has data
	{

		do
		{
            asm_temp1 = inbuffer[usb_idx];

			switch(asm_temp1)        // parse buffer for commands			
			{
                case NO_OPERATION:          // Do nothing
                    // format:      0x5A
                    usb_idx++;
                    break;

                case GETVERSION:			// Get firmware version
					// format: 		0x76
					// response:	<major><minor><dot>
					SendFWVersionUSB();
                    if (PK2Go_Mode == PK2GO_MODE_GO)
                    {
                        T0CON = 0x08; // T0 off
                        Vdd_TGT_P = 1; // VDD off
                        PK2Go_Mode = PK2GO_MODE_OFF;
                        EE_WriteByte(PK2GO_KEY1, 0xFF); // clear EEPROM keys
                        EE_WriteByte(PK2GO_KEY2, 0xFF);
                        EE_WriteByte(PK2GO_KEY3, 0xFF);    
                    }
                    usb_idx++;
                    break;

                case BOOTMODE:				// Enter Bootloader mode
					// format: 		0x42
					// response:	-
					EnterBootloader();
                    break;

				default:					// Linear command range
                    if (asm_temp1 < 0xA0)  // min command
                    {
                        usb_idx = 64;
                        break;
                    }
                    if (asm_temp1 > 0xB9)  // max command
                    {
                        usb_idx = 64;
                        break;
                    }
                    asm_temp1 -= 0xA0;  // A0 now zero
                    asm_temp1 *= 2;     // address is 2x value
                    asm_temp1 += 8;     // adjust for address difference between PCL instr
                    
                    _asm
                        CommandJumpTable:
                             movf      PCL, 0, 0             // load PCLATH and PCLATU by reading PCL
                             //movf      ASM_TEMP1_RAM, 0, 0     // in W
                             //addwf     PCL, 1, 0
                             addwf     ASM_TEMP1_RAM, 0, 0
                             btfsc     STATUS, 0, 0          // If a carry occurred 
                             incf      PCLATH, 1, 0
                             movwf     PCL, 0
                             bra       caseSETVDD     // 0xA0  
                             bra       caseSETVPP      // 0xA1
                             bra       caseREAD_STATUS        // 0xA2
                             bra       caseREAD_VOLTAGES        // 0xA3
                             bra       caseDOWNLOAD_SCRIPT       // 0xA4
                             bra       caseRUN_SCRIPT        // 0xA5
                             bra       caseEXECUTE_SCRIPT          // 0xA6
                             bra       caseCLR_DOWNLOAD_BUFFER       // 0xA7
                             bra       caseDOWNLOAD_DATA        // 0xA8
                             bra       caseCLR_UPLOAD_BUFFER          // 0xA9
                             bra       caseUPLOAD_DATA       // 0xAA
                             bra       caseCLR_SCRIPT_BUFFER        // 0xAB
                             bra       caseUPLOAD_DATA_NOLEN          // 0xAC
                             bra       caseEND_OF_BUFFER       // 0xAD
                             bra       caseRESET        // 0xAE
                             bra       caseSCRIPT_BUFFER_CHKSM          // 0xAF
                             bra       caseSET_VOLTAGE_CALS       // 0xB0
                             bra       caseWR_INTERNAL_EE        // 0xB1
                             bra       caseRD_INTERNAL_EE          // 0xB2
                             bra       caseENTER_UART_MODE       // 0xB3
                             bra       caseEXIT_UART_MODE        // 0xB4
                             bra       caseENTER_LEARN_MODE          // 0xB5
                             bra       caseEXIT_LEARN_MODE       // 0xB6
                             bra       caseENABLE_PK2GO_MODE        // 0xB7
                             bra       caseLOGIC_ANALYZER_GO        // 0xB8
                             bra       caseCOPY_RAM_UPLOAD          // 0xB9
                    _endasm
                JumpTableEnd:
                    break;

                caseSETVDD:
                    // format:      0xA0 <CCPL><CCPH><VDDLim>
                    //      CCPH:CCPL = ((Vdd * 32) + 10.5) << 6     where Vdd is desired voltage
                    //      VDDLim = (Vfault / 5) * 255              where Vdd < VFault is error
                    // response:    -
                    CalAndSetCCP1(inbuffer[usb_idx+2], inbuffer[usb_idx+1]);
                    VddVppLevels.VddThreshold = CalThresholdByte(inbuffer[usb_idx+3]);          // Set error threshold
                    usb_idx += 4;
                    _asm
                        bra     JumpTableEnd
                    _endasm                

                caseSETVPP:
                    // format:      0xA1 <CCPR2L><VPPADC><VPPLim>
                    //      CCPR2L = duty cycle.  Generally = 0x40
                    //      VPPADC = Vpp * 18.61        where Vpp is desired voltage.
                    //      VPPlim = Vfault * 18.61              where Vdd < VFault is error
                    // response:    -
					Vpp_PWM.CCPRSetPoint = inbuffer[usb_idx+1];
					Vpp_PWM.UppperLimit = CalThresholdByte(inbuffer[usb_idx+2])+1;
					Vpp_PWM.LowerLimit = Vpp_PWM.UppperLimit - 2;   
                    VddVppLevels.VppThreshold = CalThresholdByte(inbuffer[usb_idx+3]);
                    usb_idx += 4;
                    _asm
                        bra     JumpTableEnd
                    _endasm     

                caseREAD_STATUS: //0xA2
                    SendStatusUSB();
                    usb_idx++;
                    _asm
                        bra     JumpTableEnd
                    _endasm      

                caseREAD_VOLTAGES: // 0xA3
                    SendVddVppUSB();
                    usb_idx++;
                    _asm
                        bra     JumpTableEnd
                    _endasm        

				caseDOWNLOAD_SCRIPT:		// Store a script in the Script Buffer
					// format:		0xA4 <Script#><ScriptLengthN><Script1><Script2>....<ScriptN>
					// response:	-
					usb_idx++; 				// point to Script#
					StoreScriptInBuffer(&usb_idx);
                    _asm
                        bra     JumpTableEnd
                    _endasm     	

                caseRUN_SCRIPT:            // run a script from the script buffer
                    // format:      0xA5 <Script#><iterations>
					// response:	-
                    usb_idx++;
                    RunScript(inbuffer[usb_idx], inbuffer[usb_idx + 1]);
                    usb_idx+=2;
                    _asm
                        bra     JumpTableEnd
                    _endasm      

                caseEXECUTE_SCRIPT:        // immediately executes the included script
                    // format:      0xA6 <ScriptLengthN><Script1><Script2>....<ScriptN>
					// response:	-
                    usb_idx+=1; // points to length byte.
                    ScriptEngine(&inbuffer[usb_idx + 1], inbuffer[usb_idx]);
                    usb_idx += (inbuffer[usb_idx] + 1);
                    _asm
                        bra     JumpTableEnd
                    _endasm   

                caseCLR_DOWNLOAD_BUFFER:   // empties the download buffer
                    // format:      0xA7
					// response:	-
                    ClearDownloadBuffer();
                    usb_idx++;
                    _asm
                        bra     JumpTableEnd
                    _endasm       

                caseDOWNLOAD_DATA:         // add data to download buffer
                    // format:      0xA8 <datalength><data1><data2>....<dataN>
					// response:	-
                    usb_idx++;
                    WriteDownloadDataBuffer(&usb_idx);
                    _asm
                        bra     JumpTableEnd
                    _endasm       

                caseCLR_UPLOAD_BUFFER:   // empties the upload buffer
                    // format:      0xA9
					// response:	-
                    ClearUploadBuffer(); 
                    usb_idx++;
                    _asm
                        bra     JumpTableEnd
                    _endasm      

                caseUPLOAD_DATA:       // reads data from upload buffer   
                    // format:      0xAA
                    // response:    <DataLengthN><data1><data2>....<dataN>
                    ReadUploadDataBuffer();
                    usb_idx++;
                    _asm
                        bra     JumpTableEnd
                    _endasm      

                caseCLR_SCRIPT_BUFFER:
                    // format:      0xAB
					// response:	-
                    ClearScriptTable();
                    usb_idx++;
                    _asm
                        bra     JumpTableEnd
                    _endasm      

                caseUPLOAD_DATA_NOLEN:   // reads data from upload buffer   
                    // format:      0xAC
                    // response:    <data1><data2>....<dataN>
                    ReadUploadDataBufferNoLength();
                    usb_idx++;
                    _asm
                        bra     JumpTableEnd
                    _endasm       

                caseEND_OF_BUFFER: //0xAD
                    usb_idx = 64;
                    _asm
                        bra     JumpTableEnd
                    _endasm      

                caseRESET:
                    // format:      0xAE
					// response:	-
                    Reset();
                    usb_idx++;
                    _asm
                        bra     JumpTableEnd
                    _endasm      

                caseSCRIPT_BUFFER_CHKSM: //0xAF
                    SendScriptChecksumsUSB();
                    usb_idx++;
                    _asm
                        bra     JumpTableEnd
                    _endasm      

                caseSET_VOLTAGE_CALS:
                    // format:      0xB0 <adc_calfactorL><adc_calfactorH><vdd_offset><calfactor>
                    //      CCPH:CCPL = (((CCP >> 6) + vdd_offset) * vdd_calfactor) >> 7
                    //      CalibratedResult = (ADRES * adc_calfactor) >> 8
                    // response:    -
					VoltageCalibration.adc_calfactor = (inbuffer[usb_idx+1] & 0xFF);
					VoltageCalibration.adc_calfactor += (inbuffer[usb_idx+2] * 0x100);
					VoltageCalibration.vdd_offset = inbuffer[usb_idx+3];   
                    VoltageCalibration.vdd_calfactor = inbuffer[usb_idx+4];
                    SaveCalFactorsToEE();
                    usb_idx += 5;
                    _asm
                        bra     JumpTableEnd
                    _endasm      

                caseWR_INTERNAL_EE:        // write bytes to PIC18F2550 EEPROM
                    // format:      0xB1 <address><datalength><data1><data2>....<dataN>
                    //                   N = 32 Max
					// response:	-
                    usb_idx++;
                    WriteInternalEEPROM(&usb_idx);
                    _asm
                        bra     JumpTableEnd
                    _endasm     

                caseRD_INTERNAL_EE:        // read bytes from PIC18F2550 EEPROM
                    // format:      0xB2 <address><datalength>
                    //                   N = 32 Max
					// response:	<data1><data2>....<dataN>
                    usb_idx++;
                    ReadInternalEEPROM(&usb_idx);
                    _asm
                        bra     JumpTableEnd
                    _endasm      

                caseENTER_UART_MODE:      
                    // format:      0xB3 
					// response:	-
                    usb_idx++;
                    EnterUARTMode(&usb_idx);
                    _asm
                        bra     JumpTableEnd
                    _endasm  

                caseEXIT_UART_MODE:        // Exits the firmware from UART Mode
                    // format:      0xB4
					// response:	-
                    usb_idx++;
                    ExitUARTMode();
                    _asm
                        bra     JumpTableEnd
                    _endasm      

                caseENTER_LEARN_MODE:      // Puts the firmware in PK2GO Learn Mode
                    // format:      0xB5 <0x50><0x4B><0x32><EEsize>
					// response:	-
                    EnterLearnMode(++usb_idx);
                    usb_idx += 4;
                    _asm
                        bra     JumpTableEnd
                    _endasm   

                caseEXIT_LEARN_MODE:      // Ignore here
                    // format:      0xB6
					// response:	-
                    usb_idx++;
                    _asm
                        bra     JumpTableEnd
                    _endasm        

                caseENABLE_PK2GO_MODE:       // Puts the firmware in PK2GO Mode
                    // format:      0xB7 <0x50><0x4B><0x32><EEsize>
					// response:	-
                    usb_idx++;
                    EnablePK2GoMode(usb_idx);
                    usb_idx = 64;			// Stop processing.
                    _asm
                        bra     JumpTableEnd
                    _endasm    

                caseLOGIC_ANALYZER_GO:
                    // format:      0xB8<EdgeRising><TrigMask><TrigStates><EdgeMask>
                    //                   <TrigCount><PostTrigCountL><PostTrigCountH><SampleRateFactor>
                    // response:    <TrigLocL><TrigLocH>
                    usb_idx++;
                    asm_temp12 = inbuffer[usb_idx++];
                    asm_temp1 = inbuffer[usb_idx++];
                    asm_temp3 = inbuffer[usb_idx++];
                    asm_temp5 = inbuffer[usb_idx++];
                    asm_temp6 = inbuffer[usb_idx++];
                    asm_temp7 = inbuffer[usb_idx++];
                    asm_temp8 = inbuffer[usb_idx++] + 1;
                    asm_temp11 = inbuffer[usb_idx++];
                    LogicAnalyzer();
                    _asm
                        bra     JumpTableEnd
                    _endasm  

                caseCOPY_RAM_UPLOAD:   
                    // format:      0xB9<StartAddrL><StartAddrH>
					// response:	-
                    usb_idx++;
                    CopyRamUpload(inbuffer[usb_idx], inbuffer[usb_idx+1]);
                    usb_idx += 2;
                    _asm
                        bra     JumpTableEnd
                    _endasm 
	
			} // end switch
		} while (usb_idx < 64); // end DO

	} // end if (HIDRxReport(inbuffer, 64) > 0)
    
} // end void ProcessIO(void)


/******************************************************************************
 * Function:        void PK2GoModeService(void)
 *
 * Overview:        Handles PK2Go events
 *
 * PreCondition:    None
 *
 * Input:           
 *
 * Output:          
 * 
 * Side Effects:    None
 *
 * Note:            
 *****************************************************************************/
void PK2GoModeService(void)
{
    INTCONbits.T0IE = 0; // ensure Timer0 interrupt diabled.
    T0CON = 0x87;       // 16-bit timer, 1:256 prescale.
    if (INTCONbits.T0IF)
    {
        TMR0H = 0xD2; // about 250ms
        TMR0L = 0x3A; //
        INTCONbits.T0IF = 0;
        asm_temp1++;
        if (asm_temp1 > 5)
            asm_temp1 = 0;
        if (asm_temp1 < 4)
        {
           Vdd_TGT_P = (asm_temp1 & 0x1);
        }
        if (Pk2Status.StatusLow & 0x30)
        { // VDD/VPP errors
            Vdd_TGT_P = 1; // VDD off
            // wait for another button press to clear them
            while(PROG_SWITCH_pin){}; // wait for button to be pressed.
            Pk2Status.StatusLow &= 0x8F;
            Delay10KTCYx(240);      // 200ms
            while(!PROG_SWITCH_pin){}; // wait for button to be released.
            Delay10KTCYx(240);      // 200ms
        }
    }
    if (Pk2Status.ButtonPressed)
    { // button pressed
        Vdd_TGT_P = 1; // VDD off
        PK2GoExecute();
        Vdd_TGT_P = 1; // VDD off
        Vdd_TGT_N = 1; // VDD GND on
        while(!PROG_SWITCH_pin){}; // wait for button to be released.
        asm_temp1 = 0;          // for Power LED blink
        CalAndSetCCP1(0x11, 0x00); // about 1.8V
        VddVppLevels.VddThreshold = CalThresholdByte(64);          // Set error threshold
        // look for VDD/VPP error(s)
        if (Pk2Status.StatusLow & 0x30)
        { // VDD/VPP errors
            // wait for another button press to clear them
            while(PROG_SWITCH_pin){}; // wait for button to be pressed.
        }
        if (Pk2Status.StatusHigh & 0xFC)
        { // unrecoverable errors
            // wait for another button press to clear them
            Pk2GoErrorBlink(2*4); // 4 blinks
        }
        // look for Device ID error
        else if (pk2go_error_code == 1)
        {
            Pk2GoErrorBlink(2*2); // 2 blinks
        }
        // Look for Verify errors
        else if ((checksum_l | checksum_h) != 0)
        {
            Pk2GoErrorBlink(2*3); // 3 blinks
        }
        Delay10KTCYx(240);      // 200ms
        while(!PROG_SWITCH_pin){}; // wait for button to be released.
        Delay10KTCYx(240);      // 200ms
        // clear errors
        Pk2Status.StatusLow &= 0x8F;
        Pk2Status.StatusHigh &= 0x00; 
        Pk2Status.ButtonPressed = 0;      
    }
    BUSY_LED = 0;                   // ensure it stops blinking at off. 
}

/******************************************************************************
 * Function:        void UARTModeService(void)
 *
 * Overview:        Checks for received bytes, to put into the upload buffer
 *                  & checks for data in the download buffer to be sent.
 *
 * PreCondition:    None
 *
 * Input:           UARTStatus struct
 *
 * Output:          UploadBuffer & Download Buffers affected, TMR3 int affected.
 * 
 * Side Effects:    None
 *
 * Note:            
 *****************************************************************************/
void UARTModeService(void)
{
    unsigned char newValue;

    if (UARTStatus.NewRX)
    {
        do {
        newValue = UARTStatus.NewRX;
        UARTStatus.NewRX = 0;
        WriteUploadBuffer(UARTStatus.LastRXByte);
        if (newValue > 1)
        {
            WriteUploadBuffer(UARTStatus.LastRXByte2);
        }
        if (newValue > 2)
        {
            WriteUploadBuffer(UARTStatus.LastRXByte3);
        }
        } while (UARTStatus.NewRX);
    }
    if ((downloadbuf_mgmt.used_bytes) && (PIE2bits.TMR3IE == 0))
    { // start transmission if there is data and a TX is not in progress.
        UARTStatus.TXByte = ReadDownloadBuffer();
        PIR2bits.TMR3IF = 1;
        PIE2bits.TMR3IE = 1;
    }
    if ((!Vdd_TGT_P_pin) && (ADCON0bits.GO == 0)) // don't check if VDD not on
    { // check conversion results.
		if (ADRESH < VddVppLevels.VddThreshold)
			{
			// Conversion for VDD less than threshold.  Assume Short/heavy load.
			// Must get MaxVError such conversions to minimize shutting off power due to glitches, ramp-up
			if (VddVppLevels.VddErrCount == MaxErrorVDD)
				{
					Vpp_ON = 0;		// shut off VPP 
                    Vdd_TGT_P = 1;	// shut off VDD as well
                    // Pk2Status.VddError = 1;		// indicate error in status
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

        ADCON0bits.GO = 1; // next conversion.
    }

}

/******************************************************************************
 * Function:        void EnablePK2GoMode(unsigned char usbindex)
 *
 * Overview:        Puts the PICkit 2 into PK2Go mode
 *
 *
 * PreCondition:    None
 *
 * Input:           usbindex - index to start address in USB buffer
 *
 * Output:          PK2Go_Mode = PK2GO_MODE_GO;
 * 
 * Side Effects:    
 *
 * Note:            
 *****************************************************************************/
void EnablePK2GoMode(unsigned char usbindex)
{
    // check for key sequence
    if (CheckKeySequence(usbindex))
    {
        PK2Go_Mode = PK2GO_MODE_GO;
        asm_temp1 = 0;          // for Power LED blink
                            // response:    -
        CalAndSetCCP1(0x11, 0x00); // about 1.8V
        VddVppLevels.VddThreshold = CalThresholdByte(64);          // Set error threshold
    
        // save key to EEPROM so unit powers up in PK2GO
        EE_WriteByte(PK2GO_KEY1, 0x50);
        EE_WriteByte(PK2GO_KEY2, 0x4B);
        EE_WriteByte(PK2GO_KEY3, 0x32);
        // save pk2go_memsize
        pk2go_memsize = inbuffer[usbindex + 3];
        EE_WriteByte(PK2GO_MEM, pk2go_memsize);
        inbuffer[0] = ~GETVERSION;
    }
}

/******************************************************************************
 * Function:        void EnterUARTMode(unsigned char *usbindex)
 *
 * Overview:        Puts the PICkit 2 into UART Mode, where it functions as a
 *                  logic level UART.  ICSPCLK = TX, ICSPDAT = RX
 *
 *                  UART Mode is determined by Pk2Status.UARTMode value.
 *
 * PreCondition:    None
 *
 * Input:           *usbindex - index to start address in USB buffer
 *
 * Output:          Pk2Status.UARTMode = 1
 * 
 * Side Effects:    VppVddADC interrupts are disabled - no V error detection
 *                  The Script Engine will not execute scripts
 *                  ICSPCLK, ICSPDAT, AUX, and VPP pin settings are affected
 *                  Timer1 is reconfigured and used
 *                  Timer3 is used.
 *
 * Note:            The Comparator and Voltage Reference Modules are both used.
 *****************************************************************************/
void EnterUARTMode(unsigned char *usbindex)
{
    INTCONbits.GIE = 0;     // interrupts off

    // stop ADC interrupts
    VppVddADCTMR1_Stop();
    ADCON0 = ADC_VDD_Tgt_Ch;    // but re-enable ADC module.
    ADCON0bits.GO = 1;          // & start a conversion.

    // init UARTStatus values
    UARTStatus.TimerBaudLoadL = inbuffer[(*usbindex)++];// overhead about 3us per bit.
    UARTStatus.TimerBaudLoadH = inbuffer[(*usbindex)++];
    UARTStatus.RXbits = 0;
    UARTStatus.RXbyte = 0;
    UARTStatus.TXbits = 0;
    UARTStatus.TXByte = 0;
    UARTStatus.LastRXByte = 0;
    UARTStatus.NewRX = 0;

    // Set UART mode bit
    Pk2Status.UARTMode = 1;

    // Setup IO.  ICSPCLK = TX, ICSPDAT = RX
    tris_ICSPCLK = 0;   // output
    ICSPCLK_out = 1;    // idle
    tris_ICSPDAT = 1;   // input

    tris_AUX = 1;       // tri-state
    Vpp_ON = 0;         // VPP off
    MCLR_TGT = 0;       // release

    // Setup Comparator & Voltage Reference
    CMCON = 0x2E;       // C2 inverted, C2 Vin- = RA2 (C1 Vin- = RA3) Mode = 110
    CVRCON = 0xA5;      // CVref = 2.0V @ Vdd = 5.0V
    Delay10TCYx(12);    // delay 10us for comparator mode change / CVref settling time

    // Setup Timers
    PIE1bits.TMR1IE = 0;    // RX timer
    PIE2bits.TMR3IE = 0;    // TX timer
    T1CON = 0x91;           // 1:2 prescale, running
    T3CON = 0x91;           // 1:2 prescale, running

    // enable interrupts
    PIR2bits.CMIF = 0;  // clear flag
    PIE2bits.CMIE = 1;  // enable comparator ints.
    

    INTCONbits.GIE = 1;     // interrupts on
}

/******************************************************************************
 * Function:        void ExitUARTMode(void)
 *
 * Overview:        Exits the PICkit 2 from UART Mode, back into normal script
 *                  execution mode for programming operations.
 *
 *                  UART Mode is determined by Pk2Status.UARTMode value.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Pk2Status.UARTMode = 0
 * 
 * Side Effects:    VppVddADC interrupts re-enabled
 *                  ICSPCLK, ICSPDAT, and AUX pins are set to inputs
 *                  VPP pin settings are not altered
 *                  Timer1 is reconfigured and used
 *                  Timer3 is turned off.
 *
 * Note:            The Comparator and Voltage Reference Modules are both
 *                  turned off.
 *****************************************************************************/
void ExitUARTMode(void)
{
    INTCONbits.GIE = 0;     // interrupts off

    // clear mode bit
    Pk2Status.UARTMode = 0;
    
    // disabled used interrupts
    PIE2bits.CMIE = 0;
    PIE1bits.TMR1IE = 0;
    PIE2bits.TMR3IE = 0;

    // shut off timers
    T1CON = 0x00;
    T3CON = 0x00;

    // shut off comparators and voltage referece
    CMCON = 0x07;                   // Comparators off
    CVRCON = 0x00;                  // VR off

    // ICSPCLK to input
    tris_ICSPCLK = 1;

    // Restart ADC monitoring (since PIE1bits.TMR1IE = 0)
    VppVddADCTmr1_Start();

    INTCONbits.GIE = 1;     // interrupts on
}

/******************************************************************************
 * Function:        void CopyRamUpload(unsigned char addrl, unsigned char addrh)
 *
 * Overview:        Writes 128 bytes from the given RAM address to the UploadBuffer
 *
 * PreCondition:    None
 *
 * Input:           addrh:addrl = RAM address
 *
 * Output:          Data in upload
 * 
 * Side Effects:    If SFRs are read, all kinds!
 *
 * Note:            
 *****************************************************************************/
void CopyRamUpload(unsigned char addrl, unsigned char addrh)
{
    unsigned char i;
    unsigned char *ram_ptr;

    ram_ptr = (unsigned char *)(addrl + (addrh * 0x100));
    for (i = 0; i < 128; i++)
    {
        WriteUploadBuffer(*(ram_ptr++));
    }

}


/******************************************************************************
 * Function:        void ReadInternalEEPROM(unsigned char *usbindex)
 *
 * Overview:        Writes a given # of bytes into the internal MCU EEPROM.
 *
 * PreCondition:    None
 *
 * Input:           *usbindex - index to start address in USB buffer
 *
 * Output:          Transmits HID Tx report with data.
 * 
 * Side Effects:    None
 *
 * Note:            If the length byte is > 32, only the first 32 bytes are 
 *                  Read.
 *****************************************************************************/
void ReadInternalEEPROM(unsigned char *usbindex)
{
    unsigned int i, numbytes;
    unsigned char ee_address;

    ee_address = inbuffer[(*usbindex)++];   // starting address.
    numbytes = inbuffer[(*usbindex)++] & 0xFF;   // i= # bytes data (length)

    if (numbytes  > 32)     // more than allowed # bytes
    {
        numbytes = 32;
    }
    if (numbytes == 0)
    {
        return;
    }

    for (i = 0; i < numbytes; i++)
    {
	    outbuffer[i] = EE_ReadByte(ee_address++);
    }

    // transmit data
    USBHIDTxBlocking();
} 

/******************************************************************************
 * Function:        void WriteInternalEEPROM(unsigned char *usbindex)
 *
 * Overview:        Writes a given # of bytes into the internal MCU EEPROM.
 *
 * PreCondition:    None
 *
 * Input:           *usbindex - index to start address of data in USB buffer
 *
 * Output:          Internal EEPROM - updated with new data
 *
 * Side Effects:    None
 *
 * Note:            If the length byte is > 32, only the first 32 bytes are 
 *                  written.
 *****************************************************************************/
void WriteInternalEEPROM(unsigned char *usbindex)
{
    unsigned int i, numbytes;
    unsigned char ee_address;

    ee_address = inbuffer[(*usbindex)++];   // starting address.
    numbytes = inbuffer[(*usbindex)++] & 0xFF;   // i= # bytes data (length)

    if (numbytes  > 32)     // more than allowed # bytes
    {
        numbytes = 32;
    }
    if (numbytes == 0)
    {
        return;
    }

    for (i = 0; i < numbytes; i++)
    {
        EE_WriteByte(ee_address++, inbuffer[(*usbindex)++]);
    }
}

/******************************************************************************
 * Function:        void CalAndSetCCP1(unsigned char ccp1_upper, 
 *                                                     unsigned char ccp1_lower)
 *
 * Overview:        Calibrates the 10-bit left-justified CCP1 value in the two
 *                  argument bytes, and loads it into CCPR1L & CCP1CON
 *                  CalResult = (((CCP1 >> 6) + VoltageCalibration.vdd_offset)
 *                                 * VoltageCalibration.vdd_calfactor) >> 7
 *
 * PreCondition:    None
 *
 * Input:           ccp1_upper - CCPR1L value, ccp1_lower 2 LSbs in bits 7:6
 *
 * Output:          sets CCPR1L and CCP1CON to calibrated 10-bit value.
 *
 * Side Effects:    None
 *
 * Note:            VoltageCalibration.vdd_offset is signed
 *****************************************************************************/
void CalAndSetCCP1(unsigned char ccp1_upper, unsigned char ccp1_lower)
{

    signed long short cal_value = (ccp1_upper * 0x100) + ccp1_lower;
    cal_value >>= 6;
    cal_value += VoltageCalibration.vdd_offset;
    cal_value *= VoltageCalibration.vdd_calfactor;
    cal_value >>= 1;
    CCPR1L = (unsigned char) (cal_value >> 8);
    cal_value &= 0xFF;
    CCP1CON = (CCP1CON & 0xCF) | (cal_value >> 2);

}

/******************************************************************************
 * Function:        unsigned int CalADCWord(unsigned int rawValue)
 *
 * Overview:        Calibrates the given word with cal factors in
 *                  VoltageCalibration.
 *                  CalResult = 
 *                      (rawValue * VoltageCalibration.adc_calfactor) >> 8
 *
 * PreCondition:    None
 *
 * Input:           VoltageCalibration struct, rawValue byte
 *
 * Output:          returns calibrated value.
 *
 * Side Effects:    None
 *
 * Note:            Performs "Reverse Calibration" to threshold value
 *                  matches actual ADC value.
 *****************************************************************************/
unsigned int CalADCWord(unsigned int rawValue)
{

    unsigned long cal_value = rawValue;
    cal_value *= VoltageCalibration.adc_calfactor;
    cal_value >>= 8;
    if (cal_value > 0xFFFF)
        cal_value = 0xFFFF;

    return (unsigned int) cal_value;
}


/******************************************************************************
 * Function:        unsigned char CalThresholdByte(unsigned char rawValue)
 *
 * Overview:        Calibrates the given byte with cal factors in
 *                  VoltageCalibration.
 *                  CalResult = (rawValue/VoltageCalibration.adc_calfactor) * 256
 *
 * PreCondition:    None
 *
 * Input:           VoltageCalibration struct, rawValue byte
 *
 * Output:          returns calibrated value.
 *
 * Side Effects:    None
 *
 * Note:            Performs "Reverse Calibration" to threshold value
 *                  matches actual ADC value.
 *****************************************************************************/
unsigned char CalThresholdByte(unsigned char rawValue)
{
    unsigned int inverse_cal = 0x0200 - VoltageCalibration.adc_calfactor;
    inverse_cal *= rawValue;
    inverse_cal >>= 8;

    return (unsigned char) inverse_cal;
}

/******************************************************************************
 * Function:        void SaveCalFactorsToEE(void)
 *
 * Overview:        Saves voltage calibration factors to EEPROM.
 *
 * PreCondition:    None
 *
 * Input:           VoltageCalibration struct.
 *
 * Output:          Stores data in 4 bytes of EE Memory
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void SaveCalFactorsToEE(void)
{
    EE_WriteByte(ADC_CAL_L, (unsigned char)VoltageCalibration.adc_calfactor);
    EE_WriteByte(ADC_CAL_H, (unsigned char)(VoltageCalibration.adc_calfactor >> 8));
    EE_WriteByte(CPP_OFFSET, VoltageCalibration.vdd_offset);
    EE_WriteByte(CPP_CAL, VoltageCalibration.vdd_calfactor);
}

/******************************************************************************
 * Function:        void ReadCalFactorsFromEE(void)
 *
 * Overview:        Reads voltage calibration factors from EEPROM.
 *
 * PreCondition:    None
 *
 * Input:           VoltageCalibration struct.
 *
 * Output:          Reads data in 4 bytes of EE Memory into VoltageCalibration
 *                  Struct.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void ReadCalFactorsFromEE(void)
{
    VoltageCalibration.adc_calfactor = (EE_ReadByte(ADC_CAL_L) & 0xFF);
    VoltageCalibration.adc_calfactor += (EE_ReadByte(ADC_CAL_H) * 0x100);
    VoltageCalibration.vdd_offset = EE_ReadByte(CPP_OFFSET);
    VoltageCalibration.vdd_calfactor = EE_ReadByte(CPP_CAL);
}

/******************************************************************************
 * Function:        void ClearUploadBuffer(void)
 *
 * Overview:        Clears the Upload Buffer
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          
 *
 * Side Effects:    
 *
 * Note:            None
 *****************************************************************************/
void ClearUploadBuffer(void)
{
    uploadbuf_mgmt.write_index = 0;     // init buffer to empty
    uploadbuf_mgmt.read_index = 0;
    uploadbuf_mgmt.used_bytes = 0; 
}

/******************************************************************************
 * Function:        void ClearDownloadBuffer(void)
 *
 * Overview:        Clears the Download Buffer
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          
 *
 * Side Effects:    
 *
 * Note:            None
 *****************************************************************************/
void ClearDownloadBuffer(void)
{
    downloadbuf_mgmt.write_index = 0;   // init buffer to empty
    downloadbuf_mgmt.read_index = 0;
    downloadbuf_mgmt.used_bytes = 0;
}

/******************************************************************************
 * Function:        void SendScriptChecksumsUSB(void)
 *
 * Overview:        Calculates and responds with checksums of the Script Buffer.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Transmits HID Tx report with 4 bytes.
 *                      First 2 bytes are a 16-bit sum of the lengths in the 
 *                           ScriptTable
 *                      Second 2 are 16-bit sum of all used script bytes.
 *
 * Side Effects:    Stops and restarts Timer1/ADC voltage monitor.
 *
 * Note:            None
 *****************************************************************************/
void SendScriptChecksumsUSB(void)
{
    int length_checksum = 0;
    int buffer_checksum = 0;
    int i = 0;
    
    for (i = 0; i < SCRIPT_ENTRIES; i++)
    {
        length_checksum += ScriptTable[i].Length;
    }

    for (i = 0; i < length_checksum; i++)
    {
        buffer_checksum += *(uc_ScriptBuf_ptr + i);
    }

    outbuffer[0] = (length_checksum & 0xFF);
    outbuffer[1] = (length_checksum >> 8);
	outbuffer[2] = (buffer_checksum & 0xFF);;
	outbuffer[3] = (buffer_checksum >> 8);;
    // transmit conversion results
    USBHIDTxBlocking();
}

/******************************************************************************
 * Function:        void SendVddVppUSB(void)
 *
 * Overview:        ADC converts VDD and VPP voltages and send results via USB.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Transmits HID Tx report with 4 bytes.
 *
 * Side Effects:    Stops and restarts Timer1/ADC voltage monitor.
 *
 * Note:            None
 *****************************************************************************/
void SendVddVppUSB(void)
{
    unsigned int adc_result;    

    VppVddADCTMR1_Stop();
    
    ADCConvert(ADC_CH1_VDD);
    adc_result = (ADRESH * 0x100) + ADRESL;
    adc_result = CalADCWord(adc_result);
	outbuffer[0] = (unsigned char) adc_result;
	outbuffer[1] = (unsigned char) (adc_result >> 8);

    ADCConvert(ADC_CH0_VPP);
    adc_result = (ADRESH * 0x100) + ADRESL;
    adc_result = CalADCWord(adc_result);
	outbuffer[2] = (unsigned char) adc_result;
	outbuffer[3] = (unsigned char) (adc_result >> 8);
    // transmit conversion results
    USBHIDTxBlocking();

    VppVddADCTmr1_Start();
} // end void SendVddVppUSB(void)


/******************************************************************************
 * Function:        void ADCConvert(unsigned char channel)
 *
 * Overview:        ADC converts the given channel.
 *
 * PreCondition:    None
 *
 * Input:           channel - channel to convert- bits must follow ADCON0
 *
 * Output:          conversion results in ADRESH and ADRESL
 *
 * Side Effects:    Uses ADC.  Leaves ADIF asserted.
 *
 * Note:            Expected to use default ADC settings from VppVddADCTmr1_Start()
 *                  May be changed by the calling routine if others are desired or to
 *                  enabled AN2-AN4 as analog pins.
 *****************************************************************************/
void ADCConvert(unsigned char channel)
{
    ADCON0 = (channel + 1);     // set channel and turn ADC on.
    ADCON0bits.GO = 1;          // begin conversion
    
    while (ADCON0bits.GO)       // wait for it to complete
    {
    }

}// void ADCConvert(unsigned char channel)


/******************************************************************************
 * Function:        void ReadUploadDataBuffer(void)
 *
 * Overview:        Sends data from upload data buffer over USB.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Transmits HID Tx report with data length and data.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void ReadUploadDataBuffer(void)
{
    unsigned char i, length;

    length = uploadbuf_mgmt.used_bytes;
    if (length > (BUF_SIZE - 1))        // limited to # bytes in USB report - length byte
    {
        length = (BUF_SIZE - 1);
    }

	outbuffer[0] = length;
    for (i = 1; i<= length; i++)
    {
	    outbuffer[i] = uc_upload_buffer[uploadbuf_mgmt.read_index++];
        if (uploadbuf_mgmt.read_index >= UPLOAD_SIZE)  // manage buffer wrap.
        {
            uploadbuf_mgmt.read_index = 0;
        }

    }

    uploadbuf_mgmt.used_bytes -= length;    // read out this many bytes.

    // transmit data
    USBHIDTxBlocking();
} // end void ReadUploadDataBuffer(void)


/******************************************************************************
 * Function:        void ReadUploadDataBufferNoLength(void)
 *
 * Overview:        Sends data from upload data buffer over USB,
 *                  but does not add a length byte.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Transmits HID Tx report with data only.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void ReadUploadDataBufferNoLength(void)
{
    unsigned char i, length;

    length = uploadbuf_mgmt.used_bytes;
    if (length > (BUF_SIZE))        // limited to # bytes in USB report
    {
        length = (BUF_SIZE);
    }

    for (i = 0; i < length; i++)
    {
	    outbuffer[i] = uc_upload_buffer[uploadbuf_mgmt.read_index++];
        if (uploadbuf_mgmt.read_index >= UPLOAD_SIZE)  // manage buffer wrap.
        {
            uploadbuf_mgmt.read_index = 0;
        }

    }

    uploadbuf_mgmt.used_bytes -= length;    // read out this many bytes.

    // transmit data
    USBHIDTxBlocking();
} // end void ReadUploadDataBufferNoLength(void)


/******************************************************************************
 * Function:        void RunScript(unsigned char scriptnumber, unsigned char repeat)
 *
 * Overview:        Runs a given script# from the Script Buffer "repeat" number of
 *                  times.
 *
 * PreCondition:    Must be a valid script in the script buffer
 *
 * Input:           scriptnumber = # of script to run
 *                  repeat = how many times to run the script
 *
 * Output:          Pk2Status.EmptyScript set if no script at given script#
 *
 * Side Effects:    Dependent on script being run.
 *
 * Note:            None
 *****************************************************************************/
void RunScript(unsigned char scriptnumber, unsigned char repeat)
{
    // check for valid script #
    if ((scriptnumber >= SCRIPT_ENTRIES) || (ScriptTable[scriptnumber].Length == 0))
    {
        Pk2Status.EmptyScript = 1;  // set error
        return;
    }

    do
    {
        ScriptEngine((uc_ScriptBuf_ptr + ScriptTable[scriptnumber].StartIndex) , ScriptTable[scriptnumber].Length);
        repeat--;
    } while (repeat > 0);

} // end void RunScript(unsigned char scriptnumber, unsigned char repeat)


/******************************************************************************
 * Function:        void ClearScriptTable(void)
 *
 * Overview:        Clears Script buffer by setting all Script Table length entries to zero.
 *
 * PreCondition:    None
 *
 * Input:           None.
 *
 * Output:          ScriptTable[x].Length = 0 for all valid x.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void ClearScriptTable(void)
{
    unsigned char i;

	for (i=0; i < SCRIPT_ENTRIES; i++) // init script table to empty.
	{
		ScriptTable[i].Length = 0;
	}
} // end void ClearScriptTable(void)

/******************************************************************************
 * Function:        void WriteDownloadDataBuffer(unsigned char *usbindex)
 *
 * Overview:        Writes a given # of bytes into the data download buffer.
 *
 * PreCondition:    None
 *
 * Input:           *usbindex - index to length of data in USB buffer
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
void WriteDownloadDataBuffer(unsigned char *usbindex)
{
    unsigned int i, numbytes;

    numbytes = inbuffer[(*usbindex)++] & 0xFF;   // i= # bytes data (length)

    if ((numbytes + downloadbuf_mgmt.used_bytes)  > DOWNLOAD_SIZE)     // not enough room for data
    {
        Pk2Status.DownloadOvrFlow = 1;
        return;
    }

    for (i = 0; i < numbytes; i++)
    {
        uc_download_buffer[downloadbuf_mgmt.write_index++] = inbuffer[(*usbindex)++];
        if (downloadbuf_mgmt.write_index >= DOWNLOAD_SIZE) // handle index wrap
        {
            downloadbuf_mgmt.write_index = 0;
        }
        downloadbuf_mgmt.used_bytes++;  // used another byte.
    }
} // end void WriteDownloadDataBuffer(unsigned char *usbindex)

/******************************************************************************
 * Function:        void StoreScriptInBuffer(unsigned char *usbindex)
 *
 * Overview:        Stores the script from USB buffer into Script Buffer & updates
 *                  the Script Table.
 *                  Prior script at the given script # is deleted and all following
 *                  scripts are moved up.  New script is appended at end.
 *
 * PreCondition:    None
 *
 * Input:           *usbindex - index to script # byte in USB buffer
 *
 * Output:          uc_script_buffer[] - updated
 *                  ScriptTable[] - updated
 *                  Pk2Status.ScriptBufOvrFlow - set if script length > remaining buffer
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void StoreScriptInBuffer(unsigned char *usbindex)
{
	int i;
	int LengthOfAllScripts;
	int Temp_1, Temp_2;

	Temp_1 = inbuffer[(*usbindex)+1];  	// Length of new script

	// First, make sure script length is valid
	if (Temp_1 > SCRIPT_MAXLEN)
	{
		Pk2Status.ScriptBufOvrFlow = 1;		// set error - script longer than max allowed
		return;
	}

	Temp_2 = inbuffer[*usbindex];		// Script# of new script 

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
	ScriptTable[Temp_2].Length = inbuffer[(*usbindex)+1];	// update Script Table Entry with new length.
	ScriptTable[Temp_2].StartIndex = LengthOfAllScripts;    // update entry with new index at end of buffer.
	*usbindex += 2;	// point to first byte of new script in USB buffer.
	for (i = 0; i < ScriptTable[Temp_2].Length; i++)
	{
		*(uc_ScriptBuf_ptr + LengthOfAllScripts + i) = 	inbuffer[(*usbindex)++];	
	}  

} // end void StoreScriptInBuffer(unsigned char *usbindex)

/******************************************************************************
 * Function:        void EnterBootloader(void)
 *
 * Overview:        Resets the 2550 into bootloader mode.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    Erases Flash location 0x7FFE then resets the 2550.
 *
 * Note:            None
 *****************************************************************************/
void EnterBootloader(void)
{
    INTCONbits.GIE = 0;             	// shut down global interrupts	
	USBSoftDetach();
	// erase location 0x7FFE
	EECON1 = 0b10000100;                // Setup writes: EEPGD=1, WREN=1
	*((rom far char *) 0x7FFE) = 0;
	EECON2 = 0x55;                      // Start Write
	EECON2 = 0xAA;
	EECON1bits.WR = 1;

	Delay10KTCYx(78);					// delay (~65 ms @ 48 MHz))
    Reset();
} // end void EnterBootloader(void)


/******************************************************************************
 * Function:        void SendStatusUSB(void)
 *
 * Overview:        Sends READ_STATUS response over USB.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Transmits HID Tx report with Pk2Status.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void SendStatusUSB(void)
{
    Pk2Status.StatusLow &= 0xF0;    // clear bits to be tested
    if (Vpp_ON_pin)         // active high
        Pk2Status.VppOn = 1;
    if (MCLR_TGT_pin)       // active high
        Pk2Status.VppGNDOn = 1;
    if (!Vdd_TGT_P_pin)     // active low
        Pk2Status.VddOn = 1;
    if (Vdd_TGT_N_pin)      // active high
        Pk2Status.VddGNDOn = 1;

	outbuffer[0] = Pk2Status.StatusLow;
	outbuffer[1] = Pk2Status.StatusHigh;

    // Now that it's in the USB buffer, clear errors & flags
    Pk2Status.StatusLow &= 0x8F;
    Pk2Status.StatusHigh &= 0x00;
    BUSY_LED = 0;                   // ensure it stops blinking at off.

    // transmit status
    USBHIDTxBlocking();
} // end void SendStatusUSB(void)

/******************************************************************************
 * Function:        void SendFWVersionUSB(void)
 *
 * Overview:        Sends firmware version over USB.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Transmits HID Tx report with 3-byte version #.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void SendFWVersionUSB(void)
{
	outbuffer[0] = MAJORVERSION;
	outbuffer[1] = MINORVERSION;
	outbuffer[2] = DOTVERSION;

    // transmit version number    
    USBHIDTxBlocking();

} // end void SendFWVersionUSB(void)


/******************************************************************************
 * Function:        void EnsureBOROff(void)
 *
 * Overview:        Reads CONFIG2L.  If any BOR bits are set, it re-writes
 *                  CONFIG2L with BOR disabled.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          May affect CONFIG2L BOREN1/0 value(s).
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
/*void EnsureBOROff(void)
{
    char cfgval;
    rom far char *cfg2l = cfg2l_address;

    cfgval = *cfg2l;
    
    if (((cfgval & cfg2l_mask) > 0 ) || ((cfgval & 0x20) == 0))
    {
        EECON1 = 0b11000100;        // Setup write: EEPGD=1, CFGS=1, WREN=1
        cfgval = 0x39;              // clear BOREN bits make sure VREG set
        *cfg2l = cfgval;
        EECON2 = 0x55;              // rewrite config word.
        EECON2 = 0xAA;
        EECON1bits.WR = 1;
        
    }

    TBLPTRU = 0;

}*/

//#####################################################################################################
//#####################################################################################################
//#####################################################################################################
//#######                                                                                       #######
//#######                                    SCRIPT ENGINE                                      #######
//#######                                                                                       #######
//#####################################################################################################
//#####################################################################################################
//#####################################################################################################

/******************************************************************************
 * Function:        void ScriptEngine(unsigned char *scriptstart_ptr, unsigned char scriptlength)
 *
 * Overview:        Executes the script pointed to by scriptstart_ptr from the Script Buffer
 *					Aborts if a control byte attempts to use a byte from an empty Download buffer
 *                  or store a byte in a full Upload Buffer.  Will not execute if 
 *                  Pk2Status.StatusHigh != 0 (a script error exists.)
 *
 * PreCondition:    None
 *
 * Input:           *scriptstart_ptr - Pointer to start of script
 *                  scriptlength - length of script
 *
 * Output:          uc_downloadbuf_read - advanced by the number of bytes read.
 *                  uc_upload_buffer[] - new data may be stored
 *                  uc_uploadbuf_write - advance by number of bytes written.
 *                  Pk2Status.StatusHigh - set if script error occurs
 *
 * Side Effects:    Uses Timer0.
 *
 * Note:            None
 *****************************************************************************/
void ScriptEngine(unsigned char *scriptstart_ptr, unsigned char scriptlength)
{
	unsigned char scriptindex = 0;
	unsigned char temp_byte;
    unsigned int temp_word;
	unsigned char loopcount, loopindex, loopbufferindex;
    unsigned int loopbuffercount;
    unsigned char *SFR_ptr;
	BOOL loopactive = 0;
    BOOL loopbufferactive = 0;

    INTCONbits.T0IE = 0; // ensure Timer0 interrupt diabled.
    T0CON = 0x07;       // 16-bit timer, 1:256 prescale.

	if ((scriptlength == 0) || (scriptlength > SCRIPT_MAXLEN))
	{
		Pk2Status.EmptyScript = 1;		// set error - script length out of bounds
		return;		
	}

	while (((Pk2Status.StatusHigh & STATUSHI_ERRMASK) == 0) && (scriptindex < scriptlength))
	{
        asm_temp1 = *(scriptstart_ptr + scriptindex); // script command

        if (asm_temp1 < 0xB3)
        {
            scriptindex = scriptlength;;
            continue;
        }
        if (asm_temp1 < 0xD5)
        {
			_asm
                bra ScriptJumpTable2
            _endasm
        }

        asm_temp1 -= 0xD5;  // D5 now zero
        asm_temp1 *= 2;     // address is 2x value
        asm_temp1 += 8;     // adjust for address difference between PCL instr

       _asm
                     movf      PCL, 0, 0             // load PCLATH and PCLATU by reading PCL
                     //movf      ASM_TEMP1_RAM, 0, 0     // in W
                     //addwf     PCL, 1, 0
                     addwf     ASM_TEMP1_RAM, 0, 0
                     btfsc     STATUS, 0, 0          // If a carry occurred 
                     incf      PCLATH, 1, 0
                     movwf     PCL, 0
                     bra       caseRD2_BITS_BUFFER
                     bra       caseRD2_BYTE_BUFFER 
                     bra       caseVISI24
                     bra       caseNOP24
                     bra       caseCOREINST24
                     bra       caseCOREINST18
                     bra       casePOP_DOWNLOAD 
                     bra       caseICSP_STATES_BUFFER
                     bra       caseLOOPBUFFER
                     bra       caseICDSLAVE_TX_BUF
                     bra       caseICDSLAVE_TX_LIT
                     bra       caseICDSLAVE_RX
                     bra       casePOKE_SFR
                     bra       casePEEK_SFR
                     bra       caseEXIT_SCRIPT 
                     bra       caseGOTO_INDEX
                     bra       caseIF_GT_GOTO
                     bra       caseIF_EQ_GOTO
                     bra       caseDELAY_SHORT
                     bra       caseDELAY_LONG
                     bra       caseLOOP
                     bra       caseSET_ICSP_SPEED
                     bra       caseREAD_BITS 
                     bra       caseREAD_BITS_BUFFER
                     bra       caseWRITE_BITS_BUFFER
                     bra       caseWRITE_BITS_LITERAL
                     bra       caseREAD_BYTE
                     bra       caseREAD_BYTE_BUFFER
                     bra       caseWRITE_BYTE_BUFFER
                     bra       caseWRITE_BYTE_LITERAL
                     bra       caseSET_ICSP_PINS
                     bra       caseBUSY_LED_OFF
                     bra       caseBUSY_LED_ON 
                     bra       caseMCLR_GND_OFF
                     bra       caseMCLR_GND_ON
                     bra       caseVPP_PWM_OFF
                     bra       caseVPP_PWM_ON
                     bra       caseVPP_OFF
                     bra       caseVPP_ON
                     bra       caseVDD_GND_OFF
                     bra       caseVDD_GND_ON
                     bra       caseVDD_OFF
                     bra       caseVDD_ON       //0xFF
            _endasm

		caseVDD_ON:
			Vdd_TGT_P = 0;
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseVDD_OFF:
			Vdd_TGT_P = 1;
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseVDD_GND_ON:
			Vdd_TGT_N = 1;
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseVDD_GND_OFF:
			Vdd_TGT_N = 0;
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseVPP_ON:
			Vpp_ON = 1;
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseVPP_OFF:
			Vpp_ON = 0;
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseVPP_PWM_ON:
			CCP2CON = VppPWM_Enable;
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseVPP_PWM_OFF:
			CCP2CON = VppPWM_Disable;
            Vpp_PUMP = 0;               // pass VDD
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseMCLR_GND_ON:
			MCLR_TGT = 1;
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseMCLR_GND_OFF:
			MCLR_TGT = 0;
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseBUSY_LED_ON:
			BUSY_LED = 1;
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseBUSY_LED_OFF:
			BUSY_LED = 0;
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseSET_ICSP_PINS:
			scriptindex++;
			icsp_pins = *(scriptstart_ptr + scriptindex);
			SetICSP_PinStates(icsp_pins);
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseWRITE_BYTE_LITERAL:
			scriptindex++;
            ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex), 8);
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseWRITE_BYTE_BUFFER:
            ShiftBitsOutICSP(ReadDownloadBuffer(), 8);
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseREAD_BYTE_BUFFER:
			WriteUploadBuffer(ShiftBitsInICSP(8));
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseREAD_BYTE:
			ShiftBitsInICSP(8);
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseWRITE_BITS_LITERAL:
            scriptindex++;
            ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex + 1), *(scriptstart_ptr + scriptindex));
			scriptindex+=2;
			_asm
                bra ScriptJumpEnd
            _endasm

		caseWRITE_BITS_BUFFER:
			scriptindex++;
            ShiftBitsOutICSP(ReadDownloadBuffer(), *(scriptstart_ptr + scriptindex));
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseREAD_BITS_BUFFER:
            scriptindex++;
			WriteUploadBuffer(ShiftBitsInICSP(*(scriptstart_ptr + scriptindex)));
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseREAD_BITS:
			scriptindex++;
			ShiftBitsInICSP(*(scriptstart_ptr + scriptindex));
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseSET_ICSP_SPEED:
			scriptindex++;
            icsp_baud = *(scriptstart_ptr + scriptindex);
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseLOOP:
			if (loopactive)
            {
                loopcount--;
                if (loopcount == 0)
                {
                    loopactive = 0;
                    scriptindex+=3;
                    _asm
                        bra ScriptJumpEnd
                    _endasm    
                }
                scriptindex = loopindex;
                _asm
                    bra ScriptJumpEnd
                _endasm
            }
            loopactive = 1;
			loopindex = scriptindex - *(scriptstart_ptr + scriptindex + 1);
            loopcount = *(scriptstart_ptr + scriptindex + 2);
            scriptindex = loopindex;
			_asm
                bra ScriptJumpEnd
            _endasm

		caseDELAY_SHORT:
			scriptindex++;
			ShortDelay(*(scriptstart_ptr + scriptindex));
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseDELAY_LONG:
			scriptindex++;
			LongDelay(*(scriptstart_ptr + scriptindex));
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseIF_EQ_GOTO:
            temp_byte = uc_upload_buffer[uploadbuf_mgmt.write_index - 1]; // last byte written
            if (temp_byte == *(scriptstart_ptr + scriptindex + 1))
            {
	            scriptindex = scriptindex + (signed char)*(scriptstart_ptr + scriptindex + 2);                       
            }
            else
            {
	            scriptindex+=3;
            }
        	_asm
                bra ScriptJumpEnd
            _endasm

        caseIF_GT_GOTO:
            temp_byte = uc_upload_buffer[uploadbuf_mgmt.write_index - 1]; // last byte written
            if (temp_byte > *(scriptstart_ptr + scriptindex + 1))
            {
	            scriptindex = scriptindex + (signed char)*(scriptstart_ptr + scriptindex + 2);                       
            }
            else
            {
	            scriptindex+=3;
            }
        	_asm
                bra ScriptJumpEnd
            _endasm

        caseGOTO_INDEX:
	        scriptindex = scriptindex + (signed char)*(scriptstart_ptr + scriptindex + 1);                       
        	_asm
                bra ScriptJumpEnd
            _endasm

        caseEXIT_SCRIPT:
            scriptindex = scriptlength;
            _asm
                bra ScriptJumpEnd
            _endasm

        casePOKE_SFR:
	        scriptindex++;
            SFR_ptr = (unsigned char *)0x0F00 + *(scriptstart_ptr + scriptindex++);
            *SFR_ptr = *(scriptstart_ptr + scriptindex++);
        	_asm
                bra ScriptJumpEnd
            _endasm

        casePEEK_SFR:
	        scriptindex++;
            SFR_ptr = (unsigned char *)0x0F00 + *(scriptstart_ptr + scriptindex);
            WriteUploadBuffer(*SFR_ptr);
	        //scriptindex++;
        	_asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseICDSLAVE_RX:
            scriptindex++;
            WriteUploadBuffer(ICDSlave_Receive());
            _asm
                bra ScriptJumpEnd
            _endasm

        caseICDSLAVE_TX_LIT:
            scriptindex++;
            ICDSlave_transmit(*(scriptstart_ptr + scriptindex++));
            _asm
                bra ScriptJumpEnd
            _endasm

        caseICDSLAVE_TX_BUF:
            scriptindex++;
            ICDSlave_transmit(ReadDownloadBuffer());
            _asm
                bra ScriptJumpEnd
            _endasm

		caseLOOPBUFFER:
			if (loopbufferactive)
            {
                loopbuffercount--;
                if (loopbuffercount == 0)
                {
                    loopbufferactive = 0;
                    scriptindex+=2;
                    _asm
                        bra ScriptJumpEnd
                    _endasm    
                }
                scriptindex = loopbufferindex;
                _asm
                    bra ScriptJumpEnd
                _endasm
            }
			loopbufferindex = scriptindex - *(scriptstart_ptr + scriptindex + 1);
            loopbuffercount = (unsigned int) ReadDownloadBuffer();  // low byte
            loopbuffercount += (256 * ReadDownloadBuffer());        // upper byte
            if (loopbuffercount == 0)
            { // value of "zero" 0x0000 means no loops.
                scriptindex+=2;
                _asm
                    bra ScriptJumpEnd
                _endasm
            }
            loopbufferactive = 1;
            scriptindex = loopbufferindex;
			_asm
                bra ScriptJumpEnd
            _endasm

        caseICSP_STATES_BUFFER:
            WriteUploadBuffer(GetICSP_PinStates());
	        //scriptindex++;
	        _asm
                bra ScriptIdxIncJumpEnd
            _endasm
           
        casePOP_DOWNLOAD:
            ReadDownloadBuffer();
	        //scriptindex++;
	        _asm
                bra ScriptIdxIncJumpEnd
            _endasm    

		caseVISI24:
			//scriptindex++;
			ShiftBitsOutICSP(1, 4);
			ShiftBitsOutICSP(0, 8);
			WriteUploadBuffer(ShiftBitsInPIC24(8));
			WriteUploadBuffer(ShiftBitsInPIC24(8));
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseNOP24:
			//scriptindex++;
			ShiftBitsOutICSP(0, 4);
			ShiftBitsOutICSP(0, 8);
			ShiftBitsOutICSP(0, 8);
			ShiftBitsOutICSP(0, 8);
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseCOREINST18:
			scriptindex++;
			ShiftBitsOutICSP(0, 4);
			ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex++), 8);
			ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex++), 8);
			_asm
                bra ScriptJumpEnd
            _endasm

		caseCOREINST24:
			scriptindex++;
			ShiftBitsOutICSP(0, 4);
			ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex++), 8);
			ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex++), 8);
			ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex++), 8);
			_asm
                bra ScriptJumpEnd
            _endasm

		caseRD2_BYTE_BUFFER:
			//scriptindex++;
			WriteUploadBuffer(ShiftBitsInPIC24(8));
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm 

        caseRD2_BITS_BUFFER: 
            scriptindex++;
			WriteUploadBuffer(ShiftBitsInPIC24(*(scriptstart_ptr + scriptindex)));
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm  

        ScriptIdxIncJumpEnd:
            scriptindex++;
        ScriptJumpEnd:
            continue;

        ScriptJumpTable2:

        asm_temp1 -= 0xB3;  // B3 now zero
        asm_temp1 *= 2;     // address is 2x value
        asm_temp1 += 8;     // adjust for address difference between PCL instr

       _asm
             movf      PCL, 0, 0             // load PCLATH and PCLATU by reading PCL
             //movf      ASM_TEMP1_RAM, 0, 0     // in W
             //addwf     PCL, 1, 0
             addwf     ASM_TEMP1_RAM, 0, 0
             btfsc     STATUS, 0, 0          // If a carry occurred 
             incf      PCLATH, 1, 0
             movwf     PCL, 0
             bra       caseJT2_PE_PROG_RESP //0xB3 
             bra       caseJT2_WAIT_PE_RESP
             bra       caseJT2_GET_PE_RESP
             bra       caseJT2_XFERINST_BUF
             bra       caseJT2_XFRFASTDAT_BUF
             bra       caseJT2_XFRFASTDAT_LIT
             bra       caseJT2_XFERDATA32_LIT
             bra       caseJT2_XFERDATA8_LIT
             bra       caseJT2_SENDCMD
             bra       caseJT2_SETMODE
             bra       caseUNIO_TX_RX
             bra       caseUNIO_TX
             bra       caseMEASURE_PULSE
             bra       caseICDSLAVE_TX_BUF_BL
             bra       caseICDSLAVE_TX_LIT_BL
             bra       caseICDSLAVE_RX_BL
             bra       caseSPI_RDWR_BYTE_BUF 
             bra       caseSPI_RDWR_BYTE_LIT
             bra       caseSPI_RD_BYTE_BUF
             bra       caseSPI_WR_BYTE_BUF
             bra       caseSPI_WR_BYTE_LIT
             bra       caseI2C_RD_BYTE_NACK
             bra       caseI2C_RD_BYTE_ACK
             bra       caseI2C_WR_BYTE_BUF
             bra       caseI2C_WR_BYTE_LIT 
             bra       caseI2C_STOP
             bra       caseI2C_START
             bra       caseAUX_STATE_BUFFER
             bra       caseSET_AUX
             bra       caseWRITE_BITS_BUF_HLD
             bra       caseWRITE_BITS_LIT_HLD 
             bra       caseCONST_WRITE_DL
             bra       caseWRITE_BUFBYTE_W 
             bra       caseWRITE_BUFWORD_W
        _endasm

        caseWRITE_BUFBYTE_W:
            scriptindex++;
            ShiftBitsOutICSP(0, 4); // six code
            ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex), 4); // W nibble
            ShiftBitsOutICSP(ReadDownloadBuffer(), 8); // literal LSB
            ShiftBitsOutICSP(0, 8); // literal MSB
            ShiftBitsOutICSP(0x2, 4); // opcode
	        //scriptindex++;
        	_asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseWRITE_BUFWORD_W:
            scriptindex++;
            ShiftBitsOutICSP(0, 4); // six code
            ShiftBitsOutICSP(*(scriptstart_ptr + scriptindex), 4); // W nibble
            ShiftBitsOutICSP(ReadDownloadBuffer(), 8); // literal LSB
            ShiftBitsOutICSP(ReadDownloadBuffer(), 8); // literal MSB
            ShiftBitsOutICSP(0x2, 4); // opcode
	        //scriptindex++;
        	_asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseCONST_WRITE_DL:
            scriptindex++;
            WriteByteDownloadBuffer(*(scriptstart_ptr + scriptindex));
            //scriptindex++;
	        _asm
                bra ScriptIdxIncJumpEnd
            _endasm 

		caseWRITE_BITS_LIT_HLD:
            scriptindex++;
            ShiftBitsOutICSPHold(*(scriptstart_ptr + scriptindex + 1), *(scriptstart_ptr + scriptindex));
			scriptindex+=2;
			_asm
                bra ScriptJumpEnd
            _endasm

		caseWRITE_BITS_BUF_HLD:
			scriptindex++;
            ShiftBitsOutICSPHold(ReadDownloadBuffer(), *(scriptstart_ptr + scriptindex));
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm      

		caseSET_AUX:
			scriptindex++;
			aux_pin = *(scriptstart_ptr + scriptindex);
			SetAUX_PinState(aux_pin);
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseAUX_STATE_BUFFER:
            WriteUploadBuffer(GetAUX_PinState());
	        //scriptindex++;
	        _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseI2C_START:
            I2C_Start();
	        //scriptindex++;
	        _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseI2C_STOP:
            I2C_Stop();
	        //scriptindex++;
	        _asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseI2C_WR_BYTE_LIT:
			scriptindex++;
            I2C_Write(*(scriptstart_ptr + scriptindex));
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

		caseI2C_WR_BYTE_BUF:
			//scriptindex++;
            I2C_Write(ReadDownloadBuffer());
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseI2C_RD_BYTE_ACK:
			WriteUploadBuffer(I2C_Read(ACK_BYTE));
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseI2C_RD_BYTE_NACK:
			WriteUploadBuffer(I2C_Read(NO_ACK_BYTE));
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseSPI_WR_BYTE_LIT:
            scriptindex++;
            SPI_ReadWrite(*(scriptstart_ptr + scriptindex));
			//scriptindex++;
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseSPI_WR_BYTE_BUF:
            //scriptindex++;
            SPI_ReadWrite(ReadDownloadBuffer());
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseSPI_RD_BYTE_BUF:
			WriteUploadBuffer(SPI_ReadWrite(0));
			//scriptindex++;
			_asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseSPI_RDWR_BYTE_LIT:
            scriptindex++;
            WriteUploadBuffer(SPI_ReadWrite(*(scriptstart_ptr + scriptindex)));
			//scriptindex++;
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseSPI_RDWR_BYTE_BUF:
            //scriptindex++;
            WriteUploadBuffer(SPI_ReadWrite(ReadDownloadBuffer()));
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseICDSLAVE_RX_BL:
            //scriptindex++;
            WriteUploadBuffer(ICDSlaveBL_Receive());
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseICDSLAVE_TX_LIT_BL:
            scriptindex++;
            ICDSlaveBL_transmit(*(scriptstart_ptr + scriptindex++));
            _asm
                bra ScriptJumpEnd
            _endasm

        caseICDSLAVE_TX_BUF_BL:
            //scriptindex++;
            ICDSlaveBL_transmit(ReadDownloadBuffer());
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm        

        caseMEASURE_PULSE:
            //scriptindex++;
            temp_word = MeasurePulse();
            WriteUploadBuffer(temp_word & 0xFF);
            WriteUploadBuffer(temp_word >> 8);
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseUNIO_TX:   
            scriptindex++;
            UNIO(*(scriptstart_ptr + scriptindex), *(scriptstart_ptr + scriptindex + 1), 0);
			scriptindex+= 2;
            _asm
                bra ScriptJumpEnd
            _endasm

        caseUNIO_TX_RX:   
            scriptindex++;
            UNIO(*(scriptstart_ptr + scriptindex), *(scriptstart_ptr + scriptindex + 1), *(scriptstart_ptr + scriptindex + 2));
			scriptindex += 3;
            _asm
                bra ScriptJumpEnd
            _endasm

        caseJT2_SETMODE:   
            scriptindex++;
            P32SetMode(*(scriptstart_ptr + scriptindex), *(scriptstart_ptr + scriptindex + 1));
			scriptindex += 2;
            _asm
                bra ScriptJumpEnd
            _endasm

        caseJT2_SENDCMD:   
            scriptindex++;
            P32SendCommand(*(scriptstart_ptr + scriptindex));
			//scriptindex++;
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseJT2_XFERDATA8_LIT:   
            scriptindex++;
            P32XferData8(*(scriptstart_ptr + scriptindex));
			//scriptindex++;
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseJT2_PE_PROG_RESP:
            //scriptindex++;
            P32GetPEResponse(0, 0);
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseJT2_WAIT_PE_RESP:
            //scriptindex++;
            P32GetPEResponse(0, 1);
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseJT2_XFERDATA32_LIT:   
            scriptindex++;
            P32XferData32(*(scriptstart_ptr + scriptindex + 3), *(scriptstart_ptr + scriptindex + 2), *(scriptstart_ptr + scriptindex + 1), *(scriptstart_ptr + scriptindex), 1);
			scriptindex += 4;
            _asm
                bra ScriptJumpEnd
            _endasm

        caseJT2_XFERINST_BUF:
            //scriptindex++;
            P32XferInstruction();
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseJT2_XFRFASTDAT_LIT:
            scriptindex++;
            P32XferFastData32(*(scriptstart_ptr + scriptindex + 3), *(scriptstart_ptr + scriptindex + 2), *(scriptstart_ptr + scriptindex + 1), *(scriptstart_ptr + scriptindex));
			scriptindex += 4;
            _asm
                bra ScriptJumpEnd
            _endasm

        caseJT2_XFRFASTDAT_BUF:
            //scriptindex++;
            P32XferFastData32(ReadDownloadBuffer(), ReadDownloadBuffer(), ReadDownloadBuffer(), ReadDownloadBuffer());
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm

        caseJT2_GET_PE_RESP:
            //scriptindex++;
            P32GetPEResponse(1, 1);
            _asm
                bra ScriptIdxIncJumpEnd
            _endasm

                            
	} // end;

} //end void ScriptEngine(unsigned char *scriptstart_ptr, unsigned char scriptlength)

/******************************************************************************
 * Function:        void WriteByteDownloadBuffer (unsigned char DataByte)
 *
 * Overview:        Puts a byte in the download buffer
 *
 * PreCondition:    None
 *
 * Input:           DataByte - Byte to be put at Write pointer
 *
 * Output:          write pointer and used_bytes updated.
 *
 * Side Effects:    
 *
 * Note:            None
 *****************************************************************************/
void WriteByteDownloadBuffer(unsigned char DataByte)
{
        uc_download_buffer[downloadbuf_mgmt.write_index++] = DataByte;
        if (downloadbuf_mgmt.write_index >= DOWNLOAD_SIZE) // handle index wrap
        {
            downloadbuf_mgmt.write_index = 0;
        }
        downloadbuf_mgmt.used_bytes++;  // used another byte.
}

/******************************************************************************
 * Function:        void LongDelay(unsigned char count)
 *
 * Overview:        Delays in increments of 5.46ms * count.
 * 
 * PreCondition:    None
 *
 * Input:           count - units of delay (5.46ms each)
 *
 * Output:          None.
 *
 * Side Effects:    Uses Timer0
 *
 * Note:            Uses Timer0 to allow more accurate timing with interrupts.
 *                  
 *****************************************************************************/
void LongDelay(unsigned char count)
{
    INTCONbits.T0IF = 0; //clear flag
    TMR0H = 0 - count;   
    
    TMR0L = 0;
    T0CONbits.TMR0ON = 1;
    while (INTCONbits.T0IF == 0)    // wait for rollover
    {
    }
    T0CONbits.TMR0ON = 0;   // shut off  
}

/******************************************************************************
 * Function:        void ICDSlave_transmit (unsigned char TransmitByte)
 *
 * Overview:        Handles handshake and slave transmission of a byte to debug exec.
 *
 * PreCondition:    None
 *
 * Input:           TransmitByte - byte to be clocked out by DE
 *
 * Output:          Pk2Status.ICDTimeOut if eight bits not clocked out in 500ms
 *
 * Side Effects:    Shuts off interrupts during execution - leaves ICDDATA pin as output.
 *
 * Note:            None
 *****************************************************************************/
void ICDSlave_transmit (unsigned char TransmitByte)
{
    //VppVddADCTMR1_Stop();           // Stop ADC - no interrupts
    INTCONbits.GIE = 0;             // disable all interrupts
    T1CONbits.TMR1ON = 0;           // pause ADC timer
    TMR0H = 0xA4;                   // 500ms till timer rolls over
    TMR0L = 0x73;
    INTCONbits.T0IF = 0;            // clear int flag
    T0CONbits.TMR0ON = 1;           // start timer

    while (!ICSPCLK_in && !INTCONbits.T0IF); // wait for clock high handshake or timeout
    if (!INTCONbits.T0IF)           // if timeout, skip this
    {
        ICSPDAT_out = 0;            // data output low
        tris_ICSPDAT = 0;
        Delay100TCYx(96);           // delay 800us to ensure DE sees it at slow speed.
        ICSPDAT_out = 1;            // data output high
        while (ICSPCLK_in && !INTCONbits.T0IF); // wait for clock to fall or timeout
        // leave data output
        if (!INTCONbits.T0IF)           // if timeout, skip this  
        {
            asm_temp1 = TransmitByte;
            _asm
             TXBit7H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit7H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 7, 0
                bsf     LATA, 2, 0      // data high
             TXBit7L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit7L

             TXBit6H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit6H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 6, 0
                bsf     LATA, 2, 0      // data high
             TXBit6L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit6L

             TXBit5H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit5H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 5, 0
                bsf     LATA, 2, 0      // data high
             TXBit5L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit5L

             TXBit4H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit4H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 4, 0
                bsf     LATA, 2, 0      // data high
             TXBit4L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit4L

             TXBit3H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit3H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 3, 0
                bsf     LATA, 2, 0      // data high
             TXBit3L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit3L

             TXBit2H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit2H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 2, 0
                bsf     LATA, 2, 0      // data high
             TXBit2L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit2L

             TXBit1H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit1H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 1, 0
                bsf     LATA, 2, 0      // data high
             TXBit1L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit1L

             TXBit0H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit0H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 0, 0
                bsf     LATA, 2, 0      // data high
             TXBit0L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit0L

             TXTimeoutExit:
                nop
            _endasm   
        }      
    }
    if (INTCONbits.T0IF)
        Pk2Status.ICDTimeOut = 1;   // Timeout error.
    T0CONbits.TMR0ON = 0;           // shut off timer
    INTCONbits.GIE = 1;             // re-enable all interrupts
    T1CONbits.TMR1ON = 1;           // restart ADC timer
    //VppVddADCTmr1_Start();          // restore ADC operation.
    //tris_ICSPDAT = 1;               // data back to input
            // - Leave Data as output or slow target clocks may miss last bit.

    return;
} // end void ICDSlave_transmit (unsigned char TransmitByte)


/******************************************************************************
 * Function:        unsigned char ICDSlave_Receive (void)
 *
 * Overview:        Handles handshake and slave reception of a byte from debug exec.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Pk2Status.ICDTimeOut if eight bits not received in 500ms
 *                  Returns byte value received.
 *
 * Side Effects:    Shuts off interrupts during execution
 *
 * Note:            None
 *****************************************************************************/
unsigned char ICDSlave_Receive (void)
{
    //VppVddADCTMR1_Stop();           // Stop ADC - no interrupts
    T1CONbits.TMR1ON = 0;           // pause ADC timer
    INTCONbits.GIE = 0;             // disable all interrupts
    TMR0H = 0xA4;                   // 500ms till timer rolls over
    TMR0L = 0x73;
    INTCONbits.T0IF = 0;            // clear int flag
    T0CONbits.TMR0ON = 1;           // start timer

    while (!ICSPCLK_in && !INTCONbits.T0IF); // wait for clock high handshake or timeout
    if (!INTCONbits.T0IF)           // if timeout, skip this
    {
        ICSPDAT_out = 0;            // data output low
        tris_ICSPDAT = 0;
        Delay100TCYx(96);           // delay 800us to ensure DE sees it at slow speed.
        ICSPDAT_out = 1;            // data output high
        while (ICSPCLK_in && !INTCONbits.T0IF); // wait for clock to fall or timeout
        tris_ICSPDAT = 1;           // data back to input
        if (!INTCONbits.T0IF)           // if timeout, skip this  
        {
            asm_temp1 = 0;
            _asm
             RXBit7H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit7H
             RXBit7L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit7L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 7, 0 // set bit if data high

             RXBit6H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit6H
             RXBit6L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit6L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 6, 0 // set bit if data high

             RXBit5H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit5H
             RXBit5L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit5L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 5, 0 // set bit if data high

             RXBit4H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit4H
             RXBit4L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit4L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 4, 0 // set bit if data high

             RXBit3H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit3H
             RXBit3L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit3L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 3, 0 // set bit if data high

             RXBit2H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit2H
             RXBit2L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit2L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 2, 0 // set bit if data high

             RXBit1H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit1H
             RXBit1L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit1L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 1, 0 // set bit if data high

             RXBit0H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit0H
             RXBit0L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit0L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 0, 0 // set bit if data high

             RXTimeoutExit:
                nop
            _endasm   
        }      
    }

    // Set the data line high to try to prevent floating around being seen as a handshake
    ICSPDAT_out = 1;            // data output high
    tris_ICSPDAT = 0;


    if (INTCONbits.T0IF)
        Pk2Status.ICDTimeOut = 1;   // Timeout error.
    T0CONbits.TMR0ON = 0;           // shut off timer
    INTCONbits.GIE = 1;             // re-enable all interrupts
    T1CONbits.TMR1ON = 1;           // restart ADC timer
    //VppVddADCTmr1_Start();          // restore ADC operation.

    return asm_temp1;
} // end unsigned char ICDSlave_Receive (void)

//############################

/******************************************************************************
 * Function:        void ICDSlaveBL_transmit (unsigned char TransmitByte)
 *
 * Overview:        Handles handshake and slave transmission of a byte to debug exec.
 *                  For PICkit 2 Baseline handshake!
 *
 * PreCondition:    None
 *
 * Input:           TransmitByte - byte to be clocked out by DE
 *
 * Output:          Pk2Status.ICDTimeOut if eight bits not clocked out in 500ms
 *
 * Side Effects:    Shuts off interrupts during execution - leaves ICDDATA pin as output.
 *
 * Note:            None
 *****************************************************************************/
void ICDSlaveBL_transmit (unsigned char TransmitByte)
{
    //VppVddADCTMR1_Stop();           // Stop ADC - no interrupts
    INTCONbits.GIE = 0;             // disable all interrupts
    T1CONbits.TMR1ON = 0;           // pause ADC timer
    TMR0H = 0xA4;                   // 500ms till timer rolls over
    TMR0L = 0x73;
    INTCONbits.T0IF = 0;            // clear int flag
    T0CONbits.TMR0ON = 1;           // start timer

    tris_ICSPDAT = 1; //output
    while (ICSPDAT_in && !INTCONbits.T0IF); // wait for data low
    if (!INTCONbits.T0IF)           // if timeout, skip this
    {
        tris_ICSPCLK = 0;
        ICSPCLK_out = 1;            // data output low
        Delay100TCYx(96);           // delay 800us to ensure DE sees it at slow speed.
        ICSPCLK_out = 0;            // data output high
        while (!ICSPDAT_in && !INTCONbits.T0IF); // wait for clock to fall or timeout
        tris_ICSPCLK = 1; // input
        tris_ICSPDAT = 0; //output
        if (!INTCONbits.T0IF)           // if timeout, skip this  
        {
            asm_temp1 = TransmitByte; 

           _asm
             TXBit7H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit7H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 7, 0
                bsf     LATA, 2, 0      // data high
             TXBit7L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit7L

             TXBit6H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit6H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 6, 0
                bsf     LATA, 2, 0      // data high
             TXBit6L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit6L

             TXBit5H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit5H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 5, 0
                bsf     LATA, 2, 0      // data high
             TXBit5L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit5L

             TXBit4H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit4H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 4, 0
                bsf     LATA, 2, 0      // data high
             TXBit4L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit4L

             TXBit3H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit3H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 3, 0
                bsf     LATA, 2, 0      // data high
             TXBit3L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit3L

             TXBit2H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit2H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 2, 0
                bsf     LATA, 2, 0      // data high
             TXBit2L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit2L

             TXBit1H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit1H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 1, 0
                bsf     LATA, 2, 0      // data high
             TXBit1L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit1L

             TXBit0H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     TXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     TXBit0H
                bcf     LATA, 2, 0      // data low unless bit high
                btfsc   ASM_TEMP1_RAM, 0, 0
                bsf     LATA, 2, 0      // data high
             TXBit0L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     TXBit0L

             TXTimeoutExit:
                nop
            _endasm   
        }      
    }
    if (INTCONbits.T0IF)
        Pk2Status.ICDTimeOut = 1;   // Timeout error.
    Delay100TCYx(96);           // delay 800us to ensure DE sees it at slow speed.
    T0CONbits.TMR0ON = 0;           // shut off timer
    INTCONbits.GIE = 1;             // re-enable all interrupts
    T1CONbits.TMR1ON = 1;           // restart ADC timer
    //VppVddADCTmr1_Start();          // restore ADC operation.
    tris_ICSPDAT = 1;               // data back to input
    tris_ICSPCLK = 0; // back to output
            // - Leave Data as output or slow target clocks may miss last bit.

    return;
} // end void ICDSlave_transmit (unsigned char TransmitByte)


/******************************************************************************
 * Function:        unsigned char ICDSlaveBL_Receive (void)
 *
 * Overview:        Handles handshake and slave reception of a byte from debug exec.
 *                  For Baseline handshake!
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Pk2Status.ICDTimeOut if eight bits not received in 500ms
 *                  Returns byte value received.
 *
 * Side Effects:    Shuts off interrupts during execution
 *
 * Note:            None
 *****************************************************************************/
unsigned char ICDSlaveBL_Receive (void)
{
    //VppVddADCTMR1_Stop();           // Stop ADC - no interrupts
    T1CONbits.TMR1ON = 0;           // pause ADC timer
    INTCONbits.GIE = 0;             // disable all interrupts
    TMR0H = 0xA4;                   // 500ms till timer rolls over
    TMR0L = 0x73;
    INTCONbits.T0IF = 0;            // clear int flag
    T0CONbits.TMR0ON = 1;           // start timer

    tris_ICSPDAT = 1; //inp

    while (ICSPDAT_in && !INTCONbits.T0IF); // wait for clock high handshake or timeout
    if (!INTCONbits.T0IF)           // if timeout, skip this
    {
        tris_ICSPCLK = 0;
        ICSPCLK_out = 1;            // data output low
        Delay100TCYx(96);           // delay 800us to ensure DE sees it at slow speed.
        ICSPCLK_out = 0;            // data output high
        while (!ICSPDAT_in && !INTCONbits.T0IF); // wait for clock to fall or timeout
        tris_ICSPDAT = 1;           // data back to input
        tris_ICSPCLK = 1; // input
        if (!INTCONbits.T0IF)           // if timeout, skip this  
        {
            asm_temp1 = 0;
            _asm
             RXBit7H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit7H
             RXBit7L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit7L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 7, 0 // set bit if data high

             RXBit6H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit6H
             RXBit6L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit6L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 6, 0 // set bit if data high

             RXBit5H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit5H
             RXBit5L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit5L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 5, 0 // set bit if data high

             RXBit4H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit4H
             RXBit4L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit4L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 4, 0 // set bit if data high

             RXBit3H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit3H
             RXBit3L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit3L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 3, 0 // set bit if data high

             RXBit2H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit2H
             RXBit2L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit2L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 2, 0 // set bit if data high

             RXBit1H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit1H
             RXBit1L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit1L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 1, 0 // set bit if data high

             RXBit0H:
                btfsc   INTCON, 3, 0    // test for timeout
                bra     RXTimeoutExit 
                btfss   PORTA, 3, 0     // wait for clock to go high
                bra     RXBit0H
             RXBit0L:
                btfsc   PORTA, 3, 0     // wait for clock to go low
                bra     RXBit0L
                btfsc   PORTA, 2, 0     // test data pin
                bsf     ASM_TEMP1_RAM, 0, 0 // set bit if data high

             RXTimeoutExit:
                nop
            _endasm   
        }      
    }


    if (INTCONbits.T0IF)
        Pk2Status.ICDTimeOut = 1;   // Timeout error.
    T0CONbits.TMR0ON = 0;           // shut off timer
    tris_ICSPDAT = 1;               // data back to input
    tris_ICSPCLK = 0; // back to output
    INTCONbits.GIE = 1;             // re-enable all interrupts
    T1CONbits.TMR1ON = 1;           // restart ADC timer
    //VppVddADCTmr1_Start();          // restore ADC operation.


    return asm_temp1;
} // end unsigned char ICDSlave_Receive (void)

/******************************************************************************
 * Function:        int MeasurePulse(void)
 *
 * Overview:        Measures a high pulse on the PGD pin
 *                  Has a 700ms timeout to detect rising edge of pulse
 *                  Measures up to a 700ms pulse.
 * 
 * PreCondition:    PGD must be set as input.
 *
 * Input:           none
 *
 * Output:          returns Pulse length count in 21.333us increments.
 *
 * Side Effects:    Uses Timer0; shuts off interrupts
 *
 * Note:            Offset in measurement of +12/-16 counts
 *                  
 *****************************************************************************/
int MeasurePulse(void)
{
    BOOL interrupts_on = 0;
    int retvalue;

    INTCONbits.T0IF = 0; //clear flag
    TMR0H = 0x80;   // roll over in 699 ms
    
    TMR0L = 0;
    T0CONbits.TMR0ON = 1;

	if (INTCONbits.GIE == 1)
		interrupts_on = 1;
	INTCONbits.GIE = 0;			// uninterruptable routine

    while (ICSPDAT_in == 1)     // wait for PGD low
    {
        if (INTCONbits.T0IF == 1)
        {
            T0CONbits.TMR0ON = 0;   // shut off timer
        	if (interrupts_on == 1)		// turn interrupts back on if enabled.	
        		INTCONbits.GIE = 1;
            return 0;               // timeout - no pulse start
        }
    }
    while (ICSPDAT_in == 0)     // wait for PGD high (Pulse Start)
    {
        if (INTCONbits.T0IF == 1)
        {
            T0CONbits.TMR0ON = 0;   // shut off timer
        	if (interrupts_on == 1)		// turn interrupts back on if enabled.	
        		INTCONbits.GIE = 1;
            return 0;               // timeout - no pulse start
        }
    }
    // reset timer
    TMR0H = 0x80;   // roll over in 699 ms
    TMR0L = 0; 
    INTCONbits.T0IF = 0; //clear flag
    while (ICSPDAT_in == 1)     // wait for PGD low (Pulse End)
    {
        if (INTCONbits.T0IF == 1)
        {
            T0CONbits.TMR0ON = 0;   // shut off timer
        	if (interrupts_on == 1)		// turn interrupts back on if enabled.	
        		INTCONbits.GIE = 1;
            return 0xFFFF;               // timeout - no pulse end
        }

    }   

    T0CONbits.TMR0ON = 0;   // shut off timer

	if (interrupts_on == 1)		// turn interrupts back on if enabled.	
		INTCONbits.GIE = 1;

    retvalue = TMR0L;
    retvalue += ((TMR0H & 0x7F) * 0x100); // mask off set MSb

    return retvalue;
}


/******************************************************************************
 * Function:        void ShortDelay(unsigned char count)
 *
 * Overview:        Delays in increments of 42.7us * count.
 * 
 * PreCondition:    None
 *
 * Input:           count - units of delay (42.7us each)
 *
 * Output:          None.
 *
 * Side Effects:    Uses Timer0
 *
 * Note:            Uses Timer0 to allow more accurate timing with interrupts.
 *                  
 *****************************************************************************/
void ShortDelay(unsigned char count)
{
    INTCONbits.T0IF = 0; //clear flag
    TMR0H = 0xFF;        // rolls over with when lower byte does

    // an inherent delay of about 12us exists between script entities.
    // round off with fudge factor to get about 21uS inherent delay
    Delay10TCYx(8);    

    if (count == 1)
    {
        return;     // just inherent delay + fudge factor. 
    }
    else
    {
        count--;        
    }        
    
    TMR0L = 0 - count;
    T0CONbits.TMR0ON = 1;
    while (INTCONbits.T0IF == 0)    // wait for rollover
    {
    }
    T0CONbits.TMR0ON = 0;   // shut off            
}

/******************************************************************************
 * Function:        unsigned char ShiftBitsInPIC24(unsigned char numbits)
 *
 * Overview:        Shifts in up to a byte of data.  Shifts in LSB first.
 *                  If less than 8 bits, return byte is right justified.
 *                  If full, sets error Pk2Status.UpLoadFull
 *                  Data is latched on Rising Edge of clock
 * PreCondition:    None
 *
 * Input:           numbits - # bits to shift in (max 8)
 *
 * Output:          returns bits right-justified.
 *
 * Side Effects:    Advances upload buffer write pointer, if err Pk2Status.StatusHigh != 0
 *
 * Note:            Assumes ICSPCLK is output.  Sets ICSPDAT to input then restores
 *                  previous state.
 *****************************************************************************/
unsigned char ShiftBitsInPIC24(unsigned char numbits)
{
	//BOOL interrupts_on = 0;
    char i;

	tris_ICSPDAT = 1;       // set input

	//if (INTCONbits.GIE == 1)
	//	interrupts_on = 1;
	//INTCONbits.GIE = 0;			// uninterruptable routine

	asm_temp1 = 0;              // bits get shifted in here.
    asm_temp2 = numbits;

    if (icsp_baud < 2)
    {
    	_asm
           READ8LOOPF:
    		btfsc	PORTA, 2, 0
    		bsf		ASM_TEMP1_RAM, 0, 0     // set bit 0 so gets shifted into MSB
    		bsf		LATA, 3, 0
            nop
            nop
            nop
            bcf     LATA, 3, 0
            nop
            rrncf   ASM_TEMP1_RAM, 1, 0
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     READ8LOOPF
    	_endasm
    }
    else
    {
        asm_temp3 = icsp_baud - 1;
    	_asm
           READLOOPS:
            movf    ASM_TEMP3_RAM, 0, 0    // delay 6 cycles per count
    		btfsc	PORTA, 2, 0
    		bsf		ASM_TEMP1_RAM, 0, 0     // set bit 0 so gets shifted into MSB
    		bsf		LATA, 3, 0
            nop
            nop
           RDELAYLOOPHI:
            nop
            nop
            nop
            decfsz  WREG, 1, 0
            bra     RDELAYLOOPHI
            bcf     LATA, 3, 0
            movf    ASM_TEMP3_RAM, 0, 0    // delay 6 cycles per count
           RDELAYLOOPLO:
            nop
            nop
            nop
            decfsz  WREG, 1, 0
            bra     RDELAYLOOPLO
            rrncf   ASM_TEMP1_RAM, 1, 0
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     READLOOPS
    	_endasm
    }


    asm_temp1 >>= (8 - numbits);    // right justify

	if (!(icsp_pins & 0x02))
			tris_ICSPDAT = 0;

	//if (interrupts_on == 1)		// turn interrupts back on if enabled.	
		//INTCONbits.GIE = 1;

    return asm_temp1;
}

/******************************************************************************
 * Function:        unsigned char ShiftBitsInICSP(unsigned char numbits)
 *
 * Overview:        Shifts in up to a byte of data.  Shifts in LSB first.
 *                  If less than 8 bits, return byte is right justified.
 *                  If full, sets error Pk2Status.UpLoadFull
 * PreCondition:    None
 *
 * Input:           numbits - # bits to shift in (max 8)
 *
 * Output:          returns bits right-justified.
 *
 * Side Effects:    Advances upload buffer write pointer, if err Pk2Status.StatusHigh != 0
 *
 * Note:            Assumes ICSPCLK is output.  Sets ICSPDAT to input then restores
 *                  previous state.
 *****************************************************************************/
unsigned char ShiftBitsInICSP(unsigned char numbits)
{
	//BOOL interrupts_on = 0;
    char i;

	tris_ICSPDAT = 1;       // set input

	//if (INTCONbits.GIE == 1)
	//	interrupts_on = 1;
	//INTCONbits.GIE = 0;			// uninterruptable routine

	asm_temp1 = 0;              // bits get shifted in here.
    asm_temp2 = numbits;

    if (icsp_baud < 2)
    {
    	_asm
           READ8LOOPF:
    		bsf		LATA, 3, 0
            nop
            nop
            nop
    		btfsc	PORTA, 2, 0
    		bsf		ASM_TEMP1_RAM, 0, 0     // set bit 0 so gets shifted into MSB
            bcf     LATA, 3, 0
            nop
            rrncf   ASM_TEMP1_RAM, 1, 0
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     READ8LOOPF
    	_endasm
    }
    else
    {
        asm_temp3 = icsp_baud - 1;
    	_asm
           READLOOPS:
    		bsf		LATA, 3, 0
            nop
            nop
            movf    ASM_TEMP3_RAM, 0, 0    // delay 6 cycles per count
           RDELAYLOOPHI:
            nop
            nop
            nop
            decfsz  WREG, 1, 0
            bra     RDELAYLOOPHI
    		btfsc	PORTA, 2, 0
    		bsf		ASM_TEMP1_RAM, 0, 0     // set bit 0 so gets shifted into MSB
            bcf     LATA, 3, 0
            movf    ASM_TEMP3_RAM, 0, 0    // delay 6 cycles per count
           RDELAYLOOPLO:
            nop
            nop
            nop
            decfsz  WREG, 1, 0
            bra     RDELAYLOOPLO
            rrncf   ASM_TEMP1_RAM, 1, 0
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     READLOOPS
    	_endasm
    }


    asm_temp1 >>= (8 - numbits);    // right justify

	if (!(icsp_pins & 0x02))
			tris_ICSPDAT = 0;

	//if (interrupts_on == 1)		// turn interrupts back on if enabled.	
		//INTCONbits.GIE = 1;

    return asm_temp1;
}

/******************************************************************************
 * Function:        void WriteUploadBuffer(unsigned char byte2write)
 *
 * Overview:        Attempts to write a byte to the upload buffer.
 *                  If full, sets error Pk2Status.UpLoadFull
 * PreCondition:    None
 *
 * Input:           byte2write - byte to be written
 *
 * Output:          uc_upload_buffer - byte written to end of buffer.
 *
 * Side Effects:    Advances download buffer write pointer, if err Pk2Status.StatusHigh != 0
 *
 * Note:            None
 *****************************************************************************/
void WriteUploadBuffer(unsigned char byte2write)
{

    if ((uploadbuf_mgmt.used_bytes + 1) > UPLOAD_SIZE)     // not enough room for data
    {
        Pk2Status.UpLoadFull = 1;
        return;
    }

    uc_upload_buffer[uploadbuf_mgmt.write_index++] = byte2write;
    if (uploadbuf_mgmt.write_index >= UPLOAD_SIZE) // handle index wrap
    {
        uploadbuf_mgmt.write_index = 0;
    }
    uploadbuf_mgmt.used_bytes++;  // used another byte.
}

/******************************************************************************
 * Function:        unsigned char ReadDownloadBuffer(void)
 *
 * Overview:        Attempts to pull a byte from the Download Buffer.
 *                  If empty, sets error Pk2Status.DownloadEmpty
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          Returns byte from top of buffer.
 *
 * Side Effects:    Advances download buffer read pointer, if err Pk2Status.StatusHigh != 0
 *
 * Note:            None
 *****************************************************************************/
unsigned char ReadDownloadBuffer(void)
{
    unsigned char readbyte;

    if (downloadbuf_mgmt.used_bytes == 0)
    {
        Pk2Status.DownloadEmpty = 1;
        return 0;
    } 

    readbyte = uc_download_buffer[downloadbuf_mgmt.read_index++];
    downloadbuf_mgmt.used_bytes--;        // just removed a byte.
    if (downloadbuf_mgmt.read_index >= DOWNLOAD_SIZE)   // circular buffer - handle wrap.
        downloadbuf_mgmt.read_index = 0; 

    return  readbyte;  

}

/******************************************************************************
 * Function:        void ShiftBitsOutICSP(unsigned char outputbyte, char numbits)
 *
 * Overview:        Shifts the given # bits out on the ICSP pins 
 *
 * PreCondition:    None
 *
 * Input:           outputbyte - byte to be shifted out LSB first
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            Assumes ICSP pins are already set to outputs.
 *****************************************************************************/
void ShiftBitsOutICSP(unsigned char outputbyte, char numbits)
{
	//BOOL interrupts_on = 0;
    char i;

	//if (INTCONbits.GIE == 1)
	//	interrupts_on = 1;
	//INTCONbits.GIE = 0;			// uninterruptable routine

	asm_temp1 = outputbyte;
    asm_temp2 = numbits;

    if (icsp_baud < 2)
    {
    	_asm
           WRITE8LOOPF:
    		btfss	ASM_TEMP1_RAM, 0, 0
    		bcf		LATA, 2, 0 
    		btfsc	ASM_TEMP1_RAM, 0, 0
    		bsf		LATA, 2, 0 
            nop
    		bsf		LATA, 3, 0
			nop
            bcf     LATA, 3, 0
            rrncf   ASM_TEMP1_RAM, 1, 0
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     WRITE8LOOPF
    	_endasm
    }
    else
    {
        asm_temp3 = icsp_baud - 1;
    	_asm
           WRITE8LOOPS:
    		btfss	ASM_TEMP1_RAM, 0, 0
    		bcf		LATA, 2, 0 
    		btfsc	ASM_TEMP1_RAM, 0, 0
    		bsf		LATA, 2, 0 
            movf    ASM_TEMP3_RAM, 0, 0    // delay 6 cycles per count
           DELAYLOOPHI:
            nop
            nop
            nop
            decfsz  WREG, 1, 0
            bra     DELAYLOOPHI
    		bsf		LATA, 3, 0
            movf    ASM_TEMP3_RAM, 0, 0    // delay 6 cycles per count
           DELAYLOOPLO:
            nop
            nop
            nop
            decfsz  WREG, 1, 0
            bra     DELAYLOOPLO
            bcf     LATA, 3, 0
            rrncf   ASM_TEMP1_RAM, 1, 0
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     WRITE8LOOPS
    	_endasm
    }

	//if (interrupts_on == 1)		// turn interrupts back on if enabled.	
	//	INTCONbits.GIE = 1;

}


/******************************************************************************
 * Function:        void ShiftBitsOutICSPHold(unsigned char outputbyte, char numbits)
 *
 * Overview:        Shifts the given # bits out on the ICSP pins 
 *                  Differs from ShiftBitsOutICSP in that the instead of
 *                  Setting data, delay, clock high, delay, clock low
 *                  This routine works as
 *                  Setting data, clock high, delay, clock low, delay
 *
 * PreCondition:    None
 *
 * Input:           outputbyte - byte to be shifted out LSB first
 *
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            Assumes ICSP pins are already set to outputs.
 *****************************************************************************/
void ShiftBitsOutICSPHold(unsigned char outputbyte, char numbits)
{
	//BOOL interrupts_on = 0;
    char i;

	//if (INTCONbits.GIE == 1)
	//	interrupts_on = 1;
	//INTCONbits.GIE = 0;			// uninterruptable routine

	asm_temp1 = outputbyte;
    asm_temp2 = numbits;

    if (icsp_baud < 2)
    {
    	_asm
           WRITE8LOOPF:
    		btfss	ASM_TEMP1_RAM, 0, 0
    		bcf		LATA, 2, 0 
    		btfsc	ASM_TEMP1_RAM, 0, 0
    		bsf		LATA, 2, 0 
    		bsf		LATA, 3, 0
			nop
            bcf     LATA, 3, 0
			nop
            rrncf   ASM_TEMP1_RAM, 1, 0
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     WRITE8LOOPF
    	_endasm
    }
    else
    {
        asm_temp3 = icsp_baud - 1;
    	_asm
           WRITE8LOOPS:
    		btfss	ASM_TEMP1_RAM, 0, 0
    		bcf		LATA, 2, 0 
    		btfsc	ASM_TEMP1_RAM, 0, 0
    		bsf		LATA, 2, 0 
            movf    ASM_TEMP3_RAM, 0, 0    // delay 6 cycles per count
            bsf     LATA, 3, 0
           DELAYLOOPHI:
            nop
            nop
            nop
            decfsz  WREG, 1, 0
            bra     DELAYLOOPHI
    		bcf		LATA, 3, 0
            movf    ASM_TEMP3_RAM, 0, 0    // delay 6 cycles per count
           DELAYLOOPLO:
            nop
            nop
            nop
            decfsz  WREG, 1, 0
            bra     DELAYLOOPLO
            rrncf   ASM_TEMP1_RAM, 1, 0
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     WRITE8LOOPS
    	_endasm
    }

	//if (interrupts_on == 1)		// turn interrupts back on if enabled.	
	//	INTCONbits.GIE = 1;

}


/******************************************************************************
 * Function:        void SetICSP_PinStates(unsigned char icsp_byte)
 *
 * Overview:        Sets the value and direction of the ICSP pins.
 *
 * PreCondition:    None
 *
 * Input:           icsp_byte - byte formated
 *   					<7 – 4> unused
 *   					<3> PGD logic level
 *   					<2> PGC logic level
 *   					<1> 1= PGD input, 0= output
 *    					<0> 1= PGC input, 0= output
 *
 *
 * Output:          Affects bits in TRISA and LATA
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void SetICSP_PinStates(unsigned char icsp_byte)
{
				// set ISCPCLK latch
				if (icsp_byte & 0x04)
					ICSPCLK_out = 1;	
				else
					ICSPCLK_out = 0;
				// set ISCDAT latch
				if (icsp_byte & 0x08)
					ICSPDAT_out = 1;	
				else
					ICSPDAT_out = 0;

				// set ISCPCLK direction
				if (icsp_byte & 0x01)
					tris_ICSPCLK = 1;	
				else
					tris_ICSPCLK = 0;
				// set ISCDAT direction
				if (icsp_byte & 0x02)
					tris_ICSPDAT = 1;	
				else
					tris_ICSPDAT = 0;
}

/******************************************************************************
 * Function:        unsigned char GetICSP_PinStates(void)
 *
 * Overview:        Gets the values of the ICSP pins.
 *
 * PreCondition:    None
 *
 * Input:           icsp_byte - byte formated
 *   					<7 – 4> unused
 *   					<3> PGD logic level
 *   					<2> PGC logic level
 *
 *
 * Output:          returns a byte with bits:
 *                      <1> PGD state
 *    					<0> PGC state
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
unsigned char GetICSP_PinStates(void)
{
    unsigned char states = 0;

    if (ICSPDAT_in == 1)
    {
        states |= 0x02;
    }
    if (ICSPCLK_in == 1)
    {
        states |= 0x01;
    }

    return states;
}

/******************************************************************************
 * Function:        void SetAUX_PinState(unsigned char aux_byte)
 *
 * Overview:        Sets the value and direction of the AUX pin.
 *
 * PreCondition:    None
 *
 * Input:           aux_byte - byte formated
 *   					<7 – 2> unused
 *   					<1> AUX logic level
 *    					<0> 1= AUX input, 0= output
 *
 *
 * Output:          Affects bits in TRISA and LATA
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void SetAUX_PinState(unsigned char aux_byte)
{
				// set AUX latch
				if (aux_byte & 0x02)
					AUX = 1;	
				else
					AUX = 0;

				// set AUX direction
				if (aux_byte & 0x01)
					tris_AUX = 1;	
				else
					tris_AUX = 0;
}

/******************************************************************************
 * Function:        unsigned char GetAUX_PinState(void)
 *
 * Overview:        Gets the values of the ICSP pins.
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          returns a byte with bits:
 *    					<1> AUX state
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
unsigned char GetAUX_PinState(void)
{
    unsigned char states = 0;

    if (AUX_in == 1)
    {
        states |= 0x01;
    }

    return states;
}


/******************************************************************************
 * Function:        void I2C_Start(void)
 *
 * Overview:        Creates I2C Start condition with PGC = SCL and AUX = SDA.
 *
 * PreCondition:    PGC = output, AUX = input
 *
 * Input:           None
 *
 * Output:          Affects bits in TRISA and LATA
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void I2C_Start(void)
{
    ICSPCLK_out = 1;            // SCL high
    Delay10TCYx(6);             // delay 5us
    AUX = 0;                    // ensure LAT bit is zero
    tris_AUX = 0;               // SDA low
    Delay10TCYx(6);             // delay 5us
    ICSPCLK_out = 0;            // SCL Low  
    Delay10TCYx(6);             // delay 5us
    tris_AUX = 1;               // SDA released
}

/******************************************************************************
 * Function:        void I2C_Stop(void)
 *
 * Overview:        Creates I2C Stop condition with PGC = SCL and AUX = SDA.
 *
 * PreCondition:    PGC = output, AUX = input
 *
 * Input:           None
 *
 * Output:          Affects bits in TRISA and LATA
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void I2C_Stop(void)
{
    ICSPCLK_out = 0;            // SCL low
    Delay10TCYx(6);             // delay 5us
    AUX = 0;                    // ensure LAT bit is zero
    tris_AUX = 0;               // SDA low
    Delay10TCYx(6);             // delay 5us
    ICSPCLK_out = 1;            // SCL high 
    Delay10TCYx(6);             // delay 5us
    tris_AUX = 1;               // SDA released
}

/******************************************************************************
 * Function:        void I2C_Write(unsigned char outputbyte)
 *
 * Overview:        Clocks out a byte with PGC = SCL and AUX = SDA.
 *                  Checks for ACK
 *
 * PreCondition:    PGC = output, AUX = input
 *
 * Input:           outputbyte = byte to be written MSB first
 *
 * Output:          Affects bits in TRISA and LATA
 *                  Pk2Status.ICDTimeOut set if NACK received
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
void I2C_Write(unsigned char outputbyte)
{
	asm_temp1 = outputbyte;
    asm_temp2 = 8;

    AUX = 0;                    // ensure LAT bit is zero

    if (icsp_baud < 2)
    { // 400kHz
    	_asm
           WRITE8LOOPF:
    		btfss	ASM_TEMP1_RAM, 7, 0
    		bcf		TRISA, 4, 0 
    		btfsc	ASM_TEMP1_RAM, 7, 0
    		bsf		TRISA, 4, 0 
            //nop
            //nop
            //nop
            //nop  
            //nop
            //nop  
            bra     N1
           N1:
            bra     N2
           N2:
            bra     N3
           N3:                         // 583ns setup time
    		bsf		LATA, 3, 0            // clock high time 833ns (10 inst)
			//nop
            //nop
            //nop
            //nop
            //nop
            //nop
            //nop
            //nop
            nop
            bra     N4
           N4:
            bra     N5
           N5:
            bra     N6
           N6:
            bra     N7
           N7:
            bcf     LATA, 3, 0            // clock low time 1667ns (20 inst)
			//nop
            //nop
            //nop
            //nop
            nop
            bra     N8
           N8:
            bra     N9
           N9:
            rlncf   ASM_TEMP1_RAM, 1, 0
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     WRITE8LOOPF
			bsf		TRISA, 4, 0           // release SDA for ACK
            //nop
            //nop
            //nop
            //nop
            //nop
            //nop
            //nop
            //nop
            //nop
            //nop  
            bra     N10
           N10:  
            bra     N11
           N11: 
            bra     N12
           N12:  
            bra     N13
           N13:    
            bra     N14
           N14:               
    		bsf		LATA, 3, 0            // ACK clock
			//nop
            //nop
            //nop
            //nop
            //nop
            //nop
            nop
            bra     N15
           N15:
            bra     N16
           N16:
            bra     N17
           N17:
            btfsc   PORTA, 4, 0
            bsf     ASM_TEMP2_RAM, 0, 0   // bit is ACK value. 
            bcf     LATA, 3, 0
                       
    	_endasm
    }
    else
    { // 100kHz at icsp_baud = 4
        asm_temp3 = (icsp_baud << 1) - 1;
    	_asm
           WRITE8LOOPS:
    		btfss	ASM_TEMP1_RAM, 7, 0
    		bcf		TRISA, 4, 0 
    		btfsc	ASM_TEMP1_RAM, 7, 0
    		bsf		TRISA, 4, 0 
            movf    ASM_TEMP3_RAM, 0, 0    // delay 10 cycles per count
           DELAYLOOPLO:
            //nop
            //nop   
            //nop
			//nop
            //nop
            //nop
            nop
            bra     N_1
           N_1:
            bra     N_2
           N_2:
            bra     N_3
           N_3: 
            decfsz  WREG, 1, 0
            bra     DELAYLOOPLO
    		bsf		LATA, 3, 0            
            movf    ASM_TEMP3_RAM, 0, 0    // delay 5 cycles per count
            //nop
            //nop
            //nop
            //nop
            bra     N_4
           N_4:
            bra     DELAYLOOPHI
           DELAYLOOPHI:
            //nop
            //nop
            bra     N_5
           N_5:
            decfsz  WREG, 1, 0
            bra     DELAYLOOPHI  
            bcf     LATA, 3, 0            
            nop
            rlncf   ASM_TEMP1_RAM, 1, 0
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     WRITE8LOOPS
			bsf		TRISA, 4, 0            // release SDA for ACK
            movf    ASM_TEMP3_RAM, 0, 0    // delay 10 cycles per count
            //nop
            //nop   
            //nop
			//nop
            bra     N_6
           N_6:
            bra     DELAYLOOPACKLO
           DELAYLOOPACKLO:
            //nop
            //nop   
            //nop
			//nop
            //nop
            //nop
            nop
            bra     N_7
           N_7:
            bra     N_8
           N_8:
            bra     N_9
           N_9: 
            decfsz  WREG, 1, 0
            bra     DELAYLOOPACKLO
    		bsf		LATA, 3, 0             // ACK clock
            movf    ASM_TEMP3_RAM, 0, 0    // delay 5 cycles per count
            //nop
            //nop
            bra     DELAYLOOPACKHI
           DELAYLOOPACKHI:
            //nop
            //nop
            bra     N_10
           N_10: 
            decfsz  WREG, 1, 0
            bra     DELAYLOOPACKHI  
            btfsc   PORTA, 4, 0
            bsf     ASM_TEMP2_RAM, 0, 0    // bit is ACK value. 
            bcf     LATA, 3, 0             // ACK clock low

    	_endasm
    }

    if (asm_temp2 > 0)
    {// NACK received
        Pk2Status.ICDTimeOut = 1;
    }

}


/******************************************************************************
 * Function:        unsigned char I2C_Read(unsigned char giveack)
 *
 * Overview:        Clocks in a byte with PGC = SCL and AUX = SDA.
 *                  Provides and ACK for the byte if giveack is 0.
 *
 * PreCondition:    PGC = output, AUX = input
 *
 * Input:           giveack - ACK the byte if 0, else NACK
 *
 * Output:          Affects bits in TRISA and LATA
 *                  returns byte read MSB first.
 *
 * Side Effects:    None
 *
 * Note:            None
 *****************************************************************************/
unsigned char I2C_Read(unsigned char giveack)
{
	asm_temp1 = 0;
    asm_temp2 = 8;
    asm_temp3 = 1;  // default speed setting for ACK

    tris_AUX = 1;                         // ensure TRIS bit is input

    if (icsp_baud < 2)
    { // 400kHz
    	_asm
           WRITE8LOOPF:
    		rlncf   ASM_TEMP1_RAM, 1, 0
    		//nop
    		//nop
    		//nop
            //nop
            //nop
            //nop   
            //nop
            //nop
            bra     R_1
           R_1:
            bra     R_2
           R_2:
            bra     R_3
           R_3:
            bra     R_4
           R_4:
            nop                           // 583ns setup time
    		bsf		LATA, 3, 0            // clock high time 833ns (10 inst)
			//nop
            //nop
            //nop
            //nop
            //nop
            //nop
            bra     R_5
           R_5:
            bra     R_6
           R_6:
            bra     R_7
           R_7:
            nop
            btfsc   PORTA, 4, 0           // Get bit
            bsf     ASM_TEMP1_RAM, 0, 0
            bcf     LATA, 3, 0            // clock low time 1667ns (20 inst)
			//nop
            //nop
            //nop
            //nop
            //nop
            //nop
            bra     R_8
           R_8:
            bra     R_9
           R_9:
            bra     R_10
           R_10:
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     WRITE8LOOPF                       
    	_endasm
    }
    else
    { // 100kHz at icsp_baud = 4
        asm_temp3 = (icsp_baud << 1) - 1;
    	_asm
           WRITE8LOOPS:
    		rlncf   ASM_TEMP1_RAM, 1, 0
    		//nop 
    		//nop
    		nop
            bra     R1
           R1:
            movf    ASM_TEMP3_RAM, 0, 0    // delay 10 cycles per count
           DELAYLOOPLO:
            //nop
            //nop   
            //nop
			//nop
            //nop
            //nop
            bra     R2
           R2:
            bra     R3
           R3:
            bra     R4
           R4:
            nop
            decfsz  WREG, 1, 0
            bra     DELAYLOOPLO
    		bsf		LATA, 3, 0            
            movf    ASM_TEMP3_RAM, 0, 0    // delay 5 cycles per count
            //nop
            //nop
            bra     DELAYLOOPHI
           DELAYLOOPHI:
            //nop
            //nop
            bra     R5
           R5:
            decfsz  WREG, 1, 0
            bra     DELAYLOOPHI  
            btfsc   PORTA, 4, 0           // Get bit
            bsf     ASM_TEMP1_RAM, 0, 0
            bcf     LATA, 3, 0            
            //nop
            //nop
            bra     R6
           R6:
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     WRITE8LOOPS
			nop
            movf    ASM_TEMP3_RAM, 0, 0    // delay 10 cycles per count
            //nop
            //nop   
            //nop
			//nop
            bra     R7
           R7:
            bra     DELAYLOOPSACKLO
           DELAYLOOPSACKLO:
            //nop
            //nop   
            //nop
			//nop
            //nop
            //nop
            bra     R8
           R8:
            bra     R9
           R9:
            bra     R10
           R10:
            nop
            decfsz  WREG, 1, 0
            bra     DELAYLOOPSACKLO

    	_endasm
    }

    // send ACK/NACK
    AUX = 0; // ensure LAT bit is clear
    if (giveack == 0)
    {
        tris_AUX = 0;   // output zero for ACK
    }

    _asm
            movf    ASM_TEMP3_RAM, 0, 0    // delay 10 cycles per count
            //nop
            //nop   
            //nop
			//nop
            bra     AN1
           AN1:
            bra     DELAYLOOPACKLO
           DELAYLOOPACKLO:
            //nop
            //nop   
            //nop
			//nop
            //nop
            //nop
            bra     AN2
           AN2:
            bra     AN3
           AN3:
            bra     AN4
           AN4:
            nop
            decfsz  WREG, 1, 0
            bra     DELAYLOOPACKLO
    		bsf		LATA, 3, 0             // ACK clock
            movf    ASM_TEMP3_RAM, 0, 0    // delay 5 cycles per count
            //nop
            //nop
            bra     DELAYLOOPACKHI
           DELAYLOOPACKHI:
            //nop
            //nop
            bra     AN5
           AN5:
            decfsz  WREG, 1, 0
            bra     DELAYLOOPACKHI  
            //nop
            //nop 
            bra     AN6
           AN6:
            bcf     LATA, 3, 0             // ACK clock low
            //nop
            //nop
            bra     AN7
           AN7:
            nop
            nop
  	_endasm

    tris_AUX = 1;       // back to input
    return asm_temp1;
}

/******************************************************************************
 * Function:        unsigned char SPI_ReadWrite(unsigned char outputbyte)
 *
 * Overview:        Shifts outputbyte out on the AUX pin with PGC as SCK
 *                  At the same time, bits on PGD are shifted in and
 *                  returned as the read value. 
 *
 * PreCondition:    PGC, AUX = output ; PGD = input
 *
 * Input:           outputbyte - byte to be shifted out MSb first on AUX
 *
 *
 * Output:          returns the byte shifted in MSb first from PGD
 *
 * Side Effects:    None
 *
 * Note:            Assumes ICSP pins are already set to outputs.
 *****************************************************************************/
unsigned char SPI_ReadWrite(unsigned char outputbyte)
{
	//BOOL interrupts_on = 0;
    char i;

	//if (INTCONbits.GIE == 1)
	//	interrupts_on = 1;
	//INTCONbits.GIE = 0;			// uninterruptable routine

	asm_temp1 = outputbyte;         // read byte is shifted in here as well
    asm_temp2 = 8;

    if (icsp_baud < 2)
    {
    	_asm
           WRITE8LOOPF:
    		btfss	ASM_TEMP1_RAM, 7, 0
    		bcf		LATA, 4, 0 
    		btfsc	ASM_TEMP1_RAM, 7, 0
    		bsf		LATA, 4, 0 
    		bsf		LATA, 3, 0
            rlncf   ASM_TEMP1_RAM, 1, 0
            bcf     ASM_TEMP1_RAM, 0, 0
            btfsc   PORTA, 2, 0
            bsf     ASM_TEMP1_RAM, 0, 0
            bcf     LATA, 3, 0
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     WRITE8LOOPF
    	_endasm
    }
    else
    {
        asm_temp3 = icsp_baud - 1;
    	_asm
           WRITE8LOOPS:
    		btfss	ASM_TEMP1_RAM, 7, 0
    		bcf		LATA, 4, 0 
    		btfsc	ASM_TEMP1_RAM, 7, 0
    		bsf		LATA, 4, 0 
            movf    ASM_TEMP3_RAM, 0, 0    // delay 6 cycles per count
           DELAYLOOPHI:
            nop
            nop
            nop
            decfsz  WREG, 1, 0
            bra     DELAYLOOPHI
    		bsf		LATA, 3, 0
            rlncf   ASM_TEMP1_RAM, 1, 0
            bcf     ASM_TEMP1_RAM, 0, 0
            btfsc   PORTA, 2, 0
            bsf     ASM_TEMP1_RAM, 0, 0
            movf    ASM_TEMP3_RAM, 0, 0    // delay 6 cycles per count
           DELAYLOOPLO:
            nop
            nop
            nop
            decfsz  WREG, 1, 0
            bra     DELAYLOOPLO
            bcf     LATA, 3, 0
            decfsz  ASM_TEMP2_RAM, 1, 0
            bra     WRITE8LOOPS
    	_endasm
    }

	//if (interrupts_on == 1)		// turn interrupts back on if enabled.	
	//	INTCONbits.GIE = 1;
    
    AUX = 0;        // leave low

    return asm_temp1;

}

/******************************************************************************
 * Function:        void EE_WriteByte(unsigned char byte_address, unsigned char write_byte)
 *
 * Overview:        Writes value write_byte to the internal EEPROM address byte_address. 
 *
 * PreCondition:    none
 *
 * Input:           byte_address - EEPROM address to be written to
 *                  write_byte - byte value to write to EEPROM
 *
 * Output:          Specified EE byte address is written with given value.
 *
 * Side Effects:    Interrupts are disabled during EE write sequence.
 *
 * Note:            
 *****************************************************************************/
void EE_WriteByte(unsigned char byte_address, unsigned char write_byte)
{
    BOOL interrupts_on = 0;

    EEADR = byte_address;
    EEDATA = write_byte;
    EECON1 = 0;
    EECON1bits.WREN = 1;

	if (INTCONbits.GIE == 1)
		interrupts_on = 1;
	INTCONbits.GIE = 0;			// uninterruptable routine

    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;

    if (interrupts_on == 1)		// turn interrupts back on if enabled.	
		INTCONbits.GIE = 1;

    while (EECON1bits.WR == 1); // wait for write to complete
}

/******************************************************************************
 * Function:        unsigned char EE_ReadByte(unsigned char byte_address)
 *
 * Overview:        Reads a byte from EEPROM at the given address. 
 *
 * PreCondition:    none
 *
 * Input:           byte_address - EEPROM address to be read from
 *
 * Output:          Returns value of EE byte at byte_address
 *
 * Side Effects:    None.
 *
 * Note:            
 *****************************************************************************/
unsigned char EE_ReadByte(unsigned char byte_address)
{
    EEADR = byte_address;
    EECON1 = 0;
    EECON1bits.RD = 1;
    return EEDATA;

}

/******************************************************************************
 * Function:        void UNIO (unsigned char device_addr, unsigned char txbytes, unsigned char rxbytes)
 *
 * Overview:        Executes the UNIO Start Header, then transmits "txbytes"
 *                  bytes from the Download Buffer.  If rxbytes is non-zero,
 *                  receives rxbytes into the Upload Buffer.  
 *                  NoMAK's the last byte. Byte is sent/ received MSb first.
 *
 * PreCondition:    None
 *
 * Input:           device_addr - the Device Address to RX after the header
 *                  txbytes - bytes to transmit from DL buffer
 *                  rxbytes - bytes to receive into UL buffer (0= None)
 *
 * Output:          None
 *
 * Side Effects:    Uses Timer0. AUX is set to Input at exit.  Interrupts
 *                  shut off during execution.  Uses bytes in DL Buffer
 *
 * Note:            None
 *****************************************************************************/
#define K_LATA_AUXMASK 0x10
#define K_100KHZ_TMR0_ADJ 198         // 100kHz : 256 - 60 + (adjust value = 2)
#define K_25KHZ_TMR0_ADJ 18           // 25kHz : 256 - 240 + (adjust value = 2)
void UNIO (unsigned char device_addr, unsigned char txbytes, unsigned char rxbytes)
{
    BOOL interrupts_on = 0;
    BOOL check_SAK = 0;     // don't check on start header
    char bitcount;
    unsigned char bitdelay;

    if (icsp_baud > 1)
    {   // slow : 25 kHz
        asm_temp3 = K_25KHZ_TMR0_ADJ;
        bitdelay = 15;
    }
    else
    {   // fast : 100 kHz
        asm_temp3 = K_100KHZ_TMR0_ADJ;
        bitdelay = 3;
    }

    // Set AUX pin to high output
    AUX = 1;
    tris_AUX = 0;
    Delay10TCYx(24);    // 2 * TSS

    txbytes+= 2;         // add start header & Device Address

    T0CON = 0x48;       // 8-bit timer, 1:1 prescale.

    // first byte is start header 
    asm_temp1 = 0x55;

    AUX = 0;
    TMR0L = 0;                  // preset timer
    INTCONbits.T0IF = 0;        // clear timer
    T0CONbits.TMR0ON = 1;       // start timer, first period counts as THDR

	if (INTCONbits.GIE == 1)
		interrupts_on = 1;
	INTCONbits.GIE = 0;			// uninterruptable routine

    do
    {
        txbytes--; // when 0, send NoMAK
        for (bitcount = 0; bitcount < 8; bitcount++)
        {
            _asm    // first part of bit period
                movf    LATA, 0, 0      // lata in w
                andlw   ~K_LATA_AUXMASK // clear output bit ('1' bit)
                btfss   ASM_TEMP1_RAM, 7, 0
                iorlw   K_LATA_AUXMASK  // set pin  ('0' bit)
             WAITTMR0:
                btfss   INTCON, 2, 0    // TMR0IF
                bra     WAITTMR0
                movwf   LATA, 0         // update output - start of bit
                movf    ASM_TEMP3_RAM, 0, 0 // TMR0 update adjustment
                addwf   TMR0L, 1, 0     // add to TMR0               
                bcf     TRISA, 4, 0     // AUX to output (from SAK)
            _endasm

            INTCONbits.T0IF = 0; //clear flag
            asm_temp1 <<= 1;            // rotate for next bit
            asm_temp2 = LATA;
            if ((txbytes == 0) && (rxbytes == 0))
                asm_temp2 |= K_LATA_AUXMASK; // NoMAK
            else
                asm_temp2 &= ~K_LATA_AUXMASK; // MAK

            _asm    // second part of bit period
             WAIT2TMR0:
                btfss   INTCON, 2, 0    // TMR0IF
                bra     WAIT2TMR0
                btg     LATA, 4, 0      // update output - 2nd half is NOT first
                movf    ASM_TEMP3_RAM, 0, 0 // TMR0 update adjustment
                addwf   TMR0L, 1, 0     // add to TMR0               
            _endasm

            INTCONbits.T0IF = 0; //clear flag
        }

        // MAK
        _asm    // first part of bit period
            movf    ASM_TEMP2_RAM, 0, 0      // to w
         WAITMTMR0:
            btfss   INTCON, 2, 0    // TMR0IF
            bra     WAITMTMR0
            movwf   LATA, 0         // update output - start of bit
            movf    ASM_TEMP3_RAM, 0, 0 // TMR0 update adjustment
            addwf   TMR0L, 1, 0     // add to TMR0               
        _endasm
        
         INTCONbits.T0IF = 0; //clear flag
         // Get next byte here
         if ((check_SAK) && (txbytes > 0))
            asm_temp1 = ReadDownloadBuffer();
         else
            asm_temp1 = device_addr;    // if not checking SAK, next byte is Device Address

        _asm    // second part of bit period
         WAITM2TMR0:
            btfss   INTCON, 2, 0    // TMR0IF
            bra     WAITM2TMR0
            btg     LATA, 4, 0      // update output - 2nd half is NOT first
            movf    ASM_TEMP3_RAM, 0, 0 // TMR0 update adjustment
            addwf   TMR0L, 1, 0     // add to TMR0               
        _endasm

        INTCONbits.T0IF = 0; //clear flag

        // SAK
        _asm    // wait for MAK period to end
         WAITSAK1:
            btfss   INTCON, 2, 0    // TMR0IF
            bra     WAITSAK1
            bsf     TRISA, 4, 0     // AUX pin to input for SAK
            movf    ASM_TEMP3_RAM, 0, 0 // TMR0 update adjustment
            addwf   TMR0L, 1, 0     // add to TMR0               
        _endasm
    
        INTCONbits.T0IF = 0; //clear flag

        // we've just started the first half of the SAK period.
        Delay10TCYx(bitdelay); // delay before getting state
        if (check_SAK && (AUX_in == 1))
        { // SAK should be zero.
            Pk2Status.ICDTimeOut = 1;   // bus error
            txbytes = 0; 
            rxbytes = 0;      
        }

        _asm    // wait for 2nd half of SAK period.
         WAITSAK2:
            btfss   INTCON, 2, 0    // TMR0IF
            bra     WAITSAK2
            nop                     // normal bit adjustment period.
            movf    ASM_TEMP3_RAM, 0, 0 // TMR0 update adjustment
            addwf   TMR0L, 1, 0     // add to TMR0               
        _endasm

        INTCONbits.T0IF = 0; //clear flag

        Delay10TCYx(bitdelay); // delay before getting state
        if (check_SAK && (AUX_in == 0))
        { // SAK should be one.
            Pk2Status.ICDTimeOut = 1;   // bus error
            txbytes = 0;  
            rxbytes = 0;     
        }

        check_SAK = 1;              // begin checking SAK after header byte.

    } while (txbytes > 0);

    // RECEIVE ---------------------------------------------------------
    while (rxbytes-- > 0)
    { // if there are any bytes to receive.
        // byte is received into asm_temp1
        asm_temp1 = 0;
        for (bitcount = 0; bitcount < 8; bitcount++)
        {
            _asm    // wait for previous period to end
             WAITRXBIT:
                btfss   INTCON, 2, 0    // TMR0IF
                bra     WAITRXBIT
                nop
                clrf    TMR0L, 0        // Set TMR0 as timeout.
            _endasm
        
            INTCONbits.T0IF = 0; //clear flag

            asm_temp2 = LATA;
            if (rxbytes == 0)
                asm_temp2 |= K_LATA_AUXMASK; // NoMAK
            else
                asm_temp2 &= ~K_LATA_AUXMASK; // MAK
    
            // we've just started the first half of the bit period.
            Delay10TCYx(bitdelay - 1); // delay before getting state
            
            if (AUX_in == 0)
            { // adjust TMR0 on '1' bit
                _asm    // wait for 2nd half of Bit period.
                 WAITRXADJ1:
                    btfsc   INTCON, 2, 0   // TMR0IF as timeout
                    bra     WAITRXADJ1_GO
                    btfss   PORTA, 4, 0    // wait for AUX = 1
                    bra     WAITRXADJ1
                 WAITRXADJ1_GO:
                    movf    ASM_TEMP3_RAM, 0, 0 // TMR0 update adjustment
                    movwf   TMR0L, 0            // to TMR0               
                _endasm
            }
            else
            { // adjust TMR0 on '0' bit
                _asm    // wait for 2nd half of Bit period.
                 WAITRXADJ0:
                    btfsc   INTCON, 2, 0   // TMR0IF as timeout
                    bra     WAITRXADJ0_GO
                    btfsc   PORTA, 4, 0    // wait for AUX = 0
                    bra     WAITRXADJ0
                 WAITRXADJ0_GO:
                    movf    ASM_TEMP3_RAM, 0, 0 // TMR0 update adjustment
                    movwf   TMR0L, 0            // to TMR0                  
                _endasm
            }

            INTCONbits.T0IF = 0;        //clear flag
    
            Delay10TCYx(bitdelay); // delay before getting data
            asm_temp1 <<= 1;            // rotate bits in towards MSb
            if ((AUX_in == 1))
            { 
                asm_temp1 += 1;         // set LSb = 1
            }
        }

        // MAK
        _asm    // first part of bit period
            movf    ASM_TEMP2_RAM, 0, 0      // to w
         WAITMRXTMR0:
            btfss   INTCON, 2, 0    // TMR0IF
            bra     WAITMRXTMR0
            movwf   LATA, 0         // update output - start of bit
            movf    ASM_TEMP3_RAM, 0, 0 // TMR0 update adjustment
            addwf   TMR0L, 1, 0     // add to TMR0               
            bcf     TRISA, 4, 0     // output for MAK
        _endasm
        
         INTCONbits.T0IF = 0; //clear flag
         // Save received byte here
         WriteUploadBuffer(asm_temp1);

        _asm    // second part of bit period
         WAITMRX2TMR0:
            btfss   INTCON, 2, 0    // TMR0IF
            bra     WAITMRX2TMR0
            btg     LATA, 4, 0      // update output - 2nd half is NOT first
            movf    ASM_TEMP3_RAM, 0, 0 // TMR0 update adjustment
            addwf   TMR0L, 1, 0     // add to TMR0               
        _endasm

        INTCONbits.T0IF = 0; //clear flag

        // SAK
        _asm    // wait for MAK period to end
         WAITSAK1:
            btfss   INTCON, 2, 0    // TMR0IF
            bra     WAITSAK1
            bsf     TRISA, 4, 0     // AUX pin to input for SAK
            movf    ASM_TEMP3_RAM, 0, 0 // TMR0 update adjustment
            addwf   TMR0L, 1, 0     // add to TMR0               
        _endasm
    
        INTCONbits.T0IF = 0; //clear flag

        // we've just started the first half of the SAK period.
        Delay10TCYx(bitdelay); // delay before getting state
        if (check_SAK && (AUX_in == 1))
        { // SAK should be zero.
            Pk2Status.ICDTimeOut = 1;   // bus error
            rxbytes = 0;      
        }

        _asm    // wait for 2nd half of SAK period.
         WAITSAK2:
            btfss   INTCON, 2, 0    // TMR0IF
            bra     WAITSAK2
            nop                     // normal bit adjustment period.
            movf    ASM_TEMP3_RAM, 0, 0 // TMR0 update adjustment
            addwf   TMR0L, 1, 0     // add to TMR0               
        _endasm

        INTCONbits.T0IF = 0; //clear flag

        Delay10TCYx(bitdelay); // delay before getting state
        if (check_SAK && (AUX_in == 0))
        { // SAK should be one.
            Pk2Status.ICDTimeOut = 1;   // bus error
            rxbytes = 0;     
        }

    }

	if (interrupts_on == 1)		// turn interrupts back on if enabled.	
		INTCONbits.GIE = 1;

    // reset Timer0 to Script Engine default.
    T0CON = 0x07;       // 16-bit timer, 1:256 prescale.

    tris_AUX = 1;       // back to input
}

/******************************************************************************
 * Function:        void P32SetMode (unsigned char numbits, unsigned char value)
 *
 * Overview:        Transmits the "numbits" LSbs of "value" as TMS bits on
 *                  JTAG 2W 4PH.  TDI = 0, TDO is ignored.
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           numbits - # of LSbs of "value" to tx as TMS bits
 *                  value - bits to transmit as TMS bits
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            numbits must be > 0
 *****************************************************************************/
void P32SetMode (unsigned char numbits, unsigned char value)
{
    unsigned char i;

    if (numbits == 0)
        return;
    asm_temp1 = 0;      // TDI
    asm_temp2 = value;  // TMS

    for (i=0; i < numbits; i++)
    {
        JTAG2W4PH();
        asm_temp2 >>= 1;
    }
}

/******************************************************************************
 * Function:        unsigned char P32DataIO (unsigned char numbits, unsigned char tdi, unsigned char tms)
 *
 * Overview:        Transmits up to 8 bits at a time on TDI/TMS and receives
 *                  up to 8 bits from TDO as the return value.
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           numbits - # bit sequences
 *                  tdi - value for TDI, LSb first
 *                  tms - value for TMS, LSb first
 *
 * Output:          Returns TDO data (note TDO bit x comes from sequence before
 *                  TDI bit x) shifted in from MSb
 *
 * Side Effects:    None
 *
 * Note:            numbits must be > 0
 *****************************************************************************/
unsigned char P32DataIO (unsigned char numbits, unsigned char tdi, unsigned char tms)
{
    unsigned char i;

    if (numbits == 0)
        return 0;

    asm_temp1 = tdi;
    asm_temp2 = tms;
    asm_temp3 = 0;

    for (i=0; i < numbits; i++)
    {
        asm_temp3 >>= 1;
        JTAG2W4PH();
        asm_temp1 >>= 1;
        asm_temp2 >>= 1;
    }

    return asm_temp3;
}

/******************************************************************************
 * Function:        void P32SendCommand (unsigned char command)
 *
 * Overview:        Transmits a 5 bit command via JTAG 2W 4PH
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           command - 5 bit command value is 5 LSbs
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Note:            Also sends TMS header and footer
 *****************************************************************************/
void P32SendCommand (unsigned char command)
{
    //TMS header 1100
    P32SetMode(4, 0x03); // sent LSb first

    // command itself
    P32DataIO(5, command, 0x10); // TMS on MSB

    // TMS footer 10
    P32SetMode(2, 0x01); // sent LSb first
    
}

/******************************************************************************
 * Function:        void P32XferData8 (unsigned char byte0)
 *
 * Overview:        Completes an 8-bit XferData psuedo op.
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           byte0 - data to send
 *
 * Output:          received byte in Upload buffer.
 *
 * Side Effects:    None
 *
 * Note:            Also sends TMS header and footer
 *****************************************************************************/
void P32XferData8 (unsigned char byte0)
{
    unsigned char rxbyte = 0;

    //TMS header 100
    P32SetMode(3, 0x01); // sent LSb first

    // TDO first bit received on last phase of setmode
    if (asm_temp3 & 0x80)
        rxbyte = 1;

    // data
    P32DataIO(8, byte0, 0x80); // TMS on last bit
    rxbyte |= (asm_temp3 << 1);
    WriteUploadBuffer(rxbyte);

    // TMS footer 10
    P32SetMode(2, 0x01); // sent LSb first    
}

/******************************************************************************
 * Function:        unsigned char P32XferData32(unsigned char byte3, unsigned char byte2, unsigned char byte1, unsigned char byte0, char rxdata)
 *
 * Overview:        Completes a 32-bit XferData psuedo op.  If rxdata > 0, places
 *                  received data in the Upload buffer.
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           byte0 - LSB of data to send
 *                  byte1
 *                  byte2
 *                  byte3
 *                  rxdata - if > 0, save received data
 *
 * Output:          four received bytes in Upload buffer (contigent on rxdata)
 *                  return value is the third of the 4 bytes (for PrACC bit)
 *
 * Side Effects:    None
 *
 * Note:            Also sends TMS header and footer
 *****************************************************************************/
unsigned char P32XferData32(unsigned char byte3, unsigned char byte2, unsigned char byte1, unsigned char byte0, char rxdata )
{
    unsigned char rxbyte = 0;
    unsigned char thirdbyte = 0;

    //TMS header 100
    P32SetMode(3, 0x01); // sent LSb first

    // TDO first bit received on last phase of setmode
    if (asm_temp3 & 0x80)
        rxbyte = 1;

    // data
    P32DataIO(8, byte0, 0);
    rxbyte |= (asm_temp3 << 1);
    if (rxdata)
        WriteUploadBuffer(rxbyte);
    rxbyte = 0;
    if (asm_temp3 & 0x80)
        rxbyte = 1;

    P32DataIO(8, byte1, 0);
    rxbyte |= (asm_temp3 << 1);
    if (rxdata)
        WriteUploadBuffer(rxbyte);
    rxbyte = 0;
    if (asm_temp3 & 0x80)
        rxbyte = 1;

    P32DataIO(8, byte2, 0);
    rxbyte |= (asm_temp3 << 1);
    if (rxdata)
        WriteUploadBuffer(rxbyte);
    thirdbyte = rxbyte;
    rxbyte = 0;
    if (asm_temp3 & 0x80)
        rxbyte = 1;

    P32DataIO(8, byte3, 0x80); // TMS on last bit
    rxbyte |= (asm_temp3 << 1);
    if (rxdata)
        WriteUploadBuffer(rxbyte);

    // TMS footer 10
    P32SetMode(2, 0x01); // sent LSb first    

    return thirdbyte;
}

/******************************************************************************
 * Function:        void P32XferFastData32(unsigned char byte3, unsigned char byte2, unsigned char byte1, unsigned char byte0
 *
 * Overview:        Completes a 32-bit XferFastData psuedo op.  If PrAcc = 0
 *                  sets Pk2Status.ICDTimeout
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           byte0 - LSB of data to send
 *                  byte1
 *                  byte2
 *                  byte3
 *
 * Output:          None
 *
 * Side Effects:    May set Pk2Status.ICDTimeout
 *
 * Note:            Also sends TMS header and footer
 *****************************************************************************/
void P32XferFastData32(unsigned char byte3, unsigned char byte2, unsigned char byte1, unsigned char byte0)
{
    //TMS header 100
    P32SetMode(3, 0x01); // sent LSb first

    // PrAcc first bit received on last phase of setmode
    if ((asm_temp3 & 0x80) == 0)
    { // unrecoverable error.
        P32SetMode(5, 0x1F);
        Pk2Status.ICDTimeOut = 1;
        return;
    }

    // PrAcc bit
    P32SetMode(1, 0);

    // data
    P32DataIO(8, byte0, 0);
    P32DataIO(8, byte1, 0);
    P32DataIO(8, byte2, 0);
    P32DataIO(8, byte3, 0x80); // TMS on last bit

    // TMS footer 10
    P32SetMode(2, 0x01); // sent LSb first    
}

/******************************************************************************
 * Function:        void P32XferInstruction(void)
 *
 * Overview:        Completes the entire XferInstruction psuedo op.
 *                  Has a timeout on PrACC bit check, which sets Pk2Status.ICDTimeout
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           uses 4 bytes from the Download Buffer
 *
 * Output:          None
 *
 * Side Effects:    May set Pk2Status.ICDTimeout, PK2Status.DownloadEmpty
 *
 * Note:            Uses Timer0
 *****************************************************************************/
void P32XferInstruction(void)
{
    unsigned char praccbyte;

    TMR0H = 0x00;                   // 1.4s till timer rolls over
    TMR0L = 0x00;
    INTCONbits.T0IF = 0;            // clear int flag
    T0CONbits.TMR0ON = 1;           // start timer

    // This may be extraneous
    //P32SendCommand(0x05);           // MTAP_SW_ETAP
    //P32SetMode(6, 0x1F);

    P32SendCommand(0x0A);           // ETAP_CONTROL
    do
    {
        praccbyte = P32XferData32(0x00, 0x04, 0xD0, 0x00, 0);
    } while (((praccbyte & 0x04) == 0) && (INTCONbits.T0IF == 0));
    if (INTCONbits.T0IF)
    {
        Pk2Status.ICDTimeOut = 1;   // Timeout error.
        T0CONbits.TMR0ON = 0;       // shut off timer
        return;
    }
    P32SendCommand(0x09);           // ETAP_DATA
    // actual instruction:
    P32XferData32(ReadDownloadBuffer(), ReadDownloadBuffer(), ReadDownloadBuffer(), ReadDownloadBuffer(), 0);
    P32SendCommand(0x0A);           // ETAP_CONTROL
    P32XferData32(0x00, 0x00, 0xC0, 0x00, 0);

    T0CONbits.TMR0ON = 0;           // shut off timer
}

/******************************************************************************
 * Function:        void P32GetPEResponse(unsigned char savedata, unsigned char execute)
 *
 * Overview:        Completes the entire GET PE RESPONSE psuedo op.
 *                  Has a timeout on PrACC bit check, which sets Pk2Status.ICDTimeout
 *
 * PreCondition:    Assumes PGC = output low
 *
 * Input:           savedata - if true (> 0), response stored in Upload buffer, else trashed
 *                  noexecute - if true (> 0), the "tell CPU to execute instruction" cmds are skipped
 *
 * Output:          4 bytes in Upload Buffer
 *
 * Side Effects:    May set Pk2Status.ICDTimeout, PK2Status.UploadFull
 *
 * Note:            Uses Timer0
 *****************************************************************************/
void P32GetPEResponse(unsigned char savedata, unsigned char execute)
{
    unsigned char praccbyte;

    TMR0H = 0x00;                   // 1.4s till timer rolls over
    TMR0L = 0x00;
    INTCONbits.T0IF = 0;            // clear int flag
    T0CONbits.TMR0ON = 1;           // start timer

    P32SendCommand(0x0A);           // ETAP_CONTROL
    do
    {
        praccbyte = P32XferData32(0x00, 0x04, 0xD0, 0x00, 0);
    } while (((praccbyte & 0x04) == 0) && (INTCONbits.T0IF == 0));
    if (INTCONbits.T0IF)
    {
        Pk2Status.ICDTimeOut = 1;   // Timeout error.
        T0CONbits.TMR0ON = 0;       // shut off timer
        return;
    }

        P32SendCommand(0x09);           // ETAP_DATA
        // actual instruction:
        P32XferData32(0, 0, 0, 0, savedata);   // response in upload buffer
    
    if (execute)
    {
        P32SendCommand(0x0A);           // ETAP_CONTROL
        P32XferData32(0x00, 0x00, 0xC0, 0x00, 0);
    }

    T0CONbits.TMR0ON = 0;           // shut off timer

}


/******************************************************************************
 * Function:        void LogicAnalyzer(void)
 *
 * Overview:        
 *
 * PreCondition:    
 *
 * Input:           asm_temp1 = TrigMask 
 *                  asm_temp3 = TrigStates
 *                  asm_temp5 = EdgeMask
 *                  asm_temp6 = TrigCount
 *                  asm_temp7 = PostTrigCountL
 *                  asm_temp8 = PostTrigCountH + 1
 *                  asm_temp11 = SampleRateFactor
 *                  asm_temp12 = EdgeRising
 *
 * Output:          asm_temp1 = TrigLocL
 *                  asm_temp2 = TrigLocH
 *
 * Side Effects:    Sets PGD, PGC, & AUX pins to inputs, clears script table
 *
 * Note:            Uses ScriptBuffer RAM 0x600 to 0x7FF
 *****************************************************************************/
void LogicAnalyzer(void)
{
    unsigned char fsr0l_save, fsr0h_save;

    INTCONbits.GIE = 0;     // interrupts off

    // stop ADC interrupts
    VppVddADCTMR1_Stop();

    tris_ICSPDAT = 1;               // RA2 Input 
    tris_ICSPCLK = 1;               // RA3 Input 
    tris_AUX = 1;                   // RA4 Input 

    ClearScriptTable(); // script buffer is being used by logic analyzer

    BUSY_LED = 1; // BUSY LED On
    
    fsr0l_save = FSR0L;
    fsr0h_save = FSR0H;

    if (asm_temp12)
        asm_temp12 = 0; // don't invert for rising
    else
        asm_temp12 = asm_temp5; // set to mask so it inverts edge bits.

    LOGICPREP();        // clear RAM, create TrigMaskSwap & TrigStatesSwap
    if (asm_temp11 > 0)
        ANALYZER_SLOWRATE();        // < 1MHz rate, rising or falling
    else if (asm_temp12)
        ANALYZER_FALLING_1MHZ(); 
    else
        ANALYZER_RISING_1MHZ();

    FSR0L = fsr0l_save;
    FSR0H = fsr0h_save;

    BUSY_LED = 0; // BUSY LED Off

    // Restart ADC monitoring 
    VppVddADCTmr1_Start();

    INTCONbits.GIE = 1;     // interrupts on

    // Send response
    outbuffer[0] = asm_temp1;
    outbuffer[1] = asm_temp2;
    // transmit results
    USBHIDTxBlocking();

}

/** EOF pickit.c *********************************************************/

