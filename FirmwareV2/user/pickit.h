/*********************************************************************
 *
 *                  Microchip PICkit 2 Flash Starter Kit
 *
 *********************************************************************
 * FileName:        pickit.h
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
 ********************************************************************/

#ifndef PICKIT_H
#define PICKIT_H

/** D E F I N I T I O N S ****************************************************/
#define MAJORVERSION    2
#define MINORVERSION    32
#define DOTVERSION      0

#define BUF_SIZE        64			// USB buffers

#define SCRIPTBUF_SIZE	768			// Script buffer size
#define SCRIPT_ENTRIES	32			// Script Table entries
#define SCRIPT_MAXLEN	61			// maximum length of script
#define SCRIPTRSV_SIZE	0			// size of reserved memory at end of script buffer (may be used for canned scripts)
#define SCRIPTBUFSPACE	(SCRIPTBUF_SIZE - SCRIPTRSV_SIZE)

#define DOWNLOAD_SIZE	256			// download buffer size
#define UPLOAD_SIZE		128			// upload buffer size

// CONFIG2L definitions
#define cfg2l_address   0x300002
#define cfg2l_mask      0x06        // CONFIG2L & cfg2l_mask = 0 if BOR bits are correct

// ADC Channels
#define ADC_CH0_VPP     0x00
#define ADC_CH1_VDD     0x04
#define ADC_CH2_DAT     0x08
#define ADC_CH3_CLK     0x0C
#define ADC_CH4_AUX     0x10

// PIC18F2550 EEPROM MAP
// Voltage calibration EEPROM locations
#define ADC_CAL_L       0x00
#define ADC_CAL_H       0x01
#define CPP_OFFSET      0x02
#define CPP_CAL         0x03
#define PK2GO_KEY1      0x04
#define PK2GO_KEY2      0x05
#define PK2GO_KEY3      0x06
#define PK2GO_MEM       0x07    // 0 = 128K; 1 = 256K
#define UNIT_ID         0xF0    // Locations 0xF0 - 0xFF reserved for Unit ID.

//#define ANALOG              'A'
//#define WRITECONFIG         'C'     // Set Program Counter to 0x2000 (Config Mem)
//#define CMDTABLE            'c'     // Download command table
//#define WRITEDATA           'D'     // Write data memory
//#define WRITEDATAEXT        'd'     // Write data memory externally timed
//#define ERASEPGM            'E'     // Bulk erase program memory
//#define ERASEDATA           'e'     // Bulk erase data memory
//#define RECEIVE             'g'     // Get n bytes from flash device
//#define INCADDRESS          'I'     // Increment address
//#define VDDVPPSTATUS		'M'		// Get status byte and Vdd, Vpp ADC Conversions WEK
//#define WRITEnWORDSEXT      'n'     // Midrange write by n, externally timed
//#define WRITEnWORDSINT      'N'     // Midrange write by n, internally timed
//#define ENTERPROGVDD1ST     'O'     // Enter programming mode
//#define ENTERPROG           'P'     // Enter programming mode
//#define EXITPROG            'p'     // Exit programming mode
//#define READPGM             'R'     // Read program memory
//#define READDATA            'r'     // Read EE data memory
//#define CHECKSUM            'S'     // Calculate checksum
//#define SETVDDVPP           's'     // Set VDD and VPP voltages. WEK
//#define TRANSFER            't'     // Transfer n bytes to flash device
//#define POWERCTRL           'V'     // Turn Vdd on/off; turn 5kHz square wave on/off
#define GETVERSION          'v'     // Get firmware version number
//#define WRITEPGM            'W'     // Write program memory
//#define WRITEPGMEXT         'w'     // Write 1 word to program memory, externally timed
//#define WRITE4WORDS         '4'     // Write 4 words to program memory
#define BOOTMODE            'B'     // Enter bootloader mode
//#define PIC18FREAD          0x80    // Read n bytes starting at address
//#define PIC18FEEDATAREAD    0x81
//#define PIC18FWRITE1        0x82
//#define PIC18FWRITE2        0x83
//#define PIC18FWRITECONFIG   0x84
//#define PIC18FERASE         0x85
//#define PIC18FERASE1        0x86
//#define PIC18FEEDATAWRITE   0x87
//#define	PIC18J_ENTERPROG	0x90
//#define	PIC18J_EXITPROG		0x91
//#define PIC18J_ERASE		0x92
//#define PIC18J_BUFFER32		0x93
//#define PIC18J_WRITE64		0x94
#define	NO_OPERATION		'Z'		// does nothing	 

#define SETVDD				0xA0
#define SETVPP				0xA1
#define READ_STATUS 		0xA2
#define READ_VOLTAGES		0xA3
#define DOWNLOAD_SCRIPT		0xA4
#define RUN_SCRIPT			0xA5
#define EXECUTE_SCRIPT		0xA6
#define CLR_DOWNLOAD_BUFFER	0xA7
#define DOWNLOAD_DATA		0xA8
#define CLR_UPLOAD_BUFFER	0xA9
#define UPLOAD_DATA			0xAA
#define CLR_SCRIPT_BUFFER	0xAB
#define UPLOAD_DATA_NOLEN   0xAC
#define END_OF_BUFFER       0xAD
#define RESET               0xAE
#define SCRIPT_BUFFER_CHKSM 0xAF
#define SET_VOLTAGE_CALS    0xB0
#define WR_INTERNAL_EE      0xB1
#define RD_INTERNAL_EE      0xB2
#define ENTER_UART_MODE     0xB3
#define EXIT_UART_MODE      0xB4
#define ENTER_LEARN_MODE    0xB5
#define EXIT_LEARN_MODE     0xB6
#define ENABLE_PK2GO_MODE   0xB7
#define LOGIC_ANALYZER_GO   0xB8
#define COPY_RAM_UPLOAD     0xB9
// META-COMMANDS
#define READ_OSCCAL         0x80
#define WRITE_OSCCAL        0x81
#define START_CHECKSUM      0x82
#define VERIFY_CHECKSUM     0x83
#define CHECK_DEVICE_ID     0x84
#define READ_BANDGAP        0x85
#define WRITE_CFG_BANDGAP   0x86
#define CHANGE_CHKSM_FRMT   0x87


// SCRIPT CONTROL BYTE DEFINITIONS
#define	VDD_ON				0xFF
#define VDD_OFF				0xFE 
#define VDD_GND_ON			0xFD
#define VDD_GND_OFF			0xFC
#define VPP_ON				0xFB
#define VPP_OFF				0xFA
#define VPP_PWM_ON			0xF9
#define VPP_PWM_OFF			0xF8
#define MCLR_GND_ON			0xF7
#define MCLR_GND_OFF		0xF6
#define BUSY_LED_ON			0xF5
#define BUSY_LED_OFF		0xF4
#define SET_ICSP_PINS		0xF3
#define WRITE_BYTE_LITERAL	0xF2
#define WRITE_BYTE_BUFFER   0xF1
#define READ_BYTE_BUFFER    0xF0
#define READ_BYTE  			0xEF
#define WRITE_BITS_LITERAL	0xEE
#define WRITE_BITS_BUFFER	0xED
#define READ_BITS_BUFFER	0xEC
#define READ_BITS			0xEB
#define SET_ICSP_SPEED      0xEA
#define LOOP				0xE9
#define DELAY_LONG 			0xE8
#define DELAY_SHORT			0xE7
#define IF_EQ_GOTO			0xE6
#define IF_GT_GOTO			0xE5
#define GOTO_INDEX	        0xE4
#define EXIT_SCRIPT			0xE3
#define PEEK_SFR			0xE2
#define POKE_SFR			0xE1
#define ICDSLAVE_RX         0xE0
#define ICDSLAVE_TX_LIT     0xDF
#define ICDSLAVE_TX_BUF     0xDE
#define LOOPBUFFER			0xDD
#define ICSP_STATES_BUFFER  0xDC
#define POP_DOWNLOAD		0xDB
#define COREINST18          0xDA
#define COREINST24			0xD9
#define NOP24               0xD8
#define VISI24        		0xD7
#define RD2_BYTE_BUFFER		0xD6
#define RD2_BITS_BUFFER		0xD5
#define WRITE_BUFWORD_W		0xD4
#define WRITE_BUFBYTE_W		0xD3
#define CONST_WRITE_DL		0xD2
#define WRITE_BITS_LIT_HLD  0xD1
#define WRITE_BITS_BUF_HLD	0xD0
#define SET_AUX				0xCF
#define AUX_STATE_BUFFER	0xCE
#define I2C_START			0xCD
#define I2C_STOP			0xCC
#define I2C_WR_BYTE_LIT		0xCB
#define I2C_WR_BYTE_BUF		0xCA
#define I2C_RD_BYTE_ACK		0xC9
#define I2C_RD_BYTE_NACK	0xC8
#define SPI_WR_BYTE_LIT		0xC7
#define SPI_WR_BYTE_BUF		0xC6
#define SPI_RD_BYTE_BUF		0xC5
#define SPI_RDWR_BYTE_LIT   0xC4
#define SPI_RDWR_BYTE_BUF	0xC3
#define ICDSLAVE_RX_BL      0xC2
#define ICDSLAVE_TX_LIT_BL  0xC1
#define ICDSLAVE_TX_BUF_BL  0xC0
#define MEASURE_PULSE       0xBF
#define UNIO_TX             0xBE
#define UNIO_TX_RX          0xBD
#define JT2_SETMODE         0xBC
#define JT2_SENDCMD         0xBB
#define JT2_XFERDATA8_LIT   0xBA
#define JT2_XFERDATA32_LIT  0xB9
#define JT2_XFRFASTDAT_LIT  0xB8
#define JT2_XFRFASTDAT_BUF  0xB7
#define JT2_XFERINST_BUF    0xB6
#define JT2_GET_PE_RESP     0xB5
#define JT2_WAIT_PE_RESP    0xB4
#define JT2_PE_PROG_RESP    0xB3

#define ACK_BYTE            0x00
#define NO_ACK_BYTE         0x0F

#define STATUSHI_ERRMASK    0xFE

#define PK2GO_MODE_OFF      0x00
#define PK2GO_MODE_LEARN    0x01
#define PK2GO_MODE_GO       0x02

/** E X T E R N S ***************************************************/
extern struct {
	unsigned char   VddThreshold;	// error detect threshold
	unsigned char   VppThreshold;	// error detect threshold
    unsigned char   VddErrCount;
    unsigned char   VppErrCount;
    unsigned int    BlinkClount;    // counter for blinking Busy on VDD/VPP error
} VddVppLevels;

extern union {		// Status bits
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

extern struct {
	unsigned char CCPRSetPoint;
	unsigned char UppperLimit;
	unsigned char LowerLimit;
} Vpp_PWM;

extern near struct {
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

extern near unsigned char PK2Go_Mode;
extern char inbuffer[BUF_SIZE];
extern char outbuffer[BUF_SIZE]; 

extern struct {
    unsigned char	write_index;        // buffer write index
    unsigned char	read_index;			// buffer read index
    unsigned int    used_bytes;         // # bytes in buffer
} downloadbuf_mgmt; 

extern struct {
    unsigned char	write_index;		// buffer write index
    unsigned char	read_index;			// buffer read index
    unsigned int    used_bytes;         // # bytes in buffer
} uploadbuf_mgmt; 

extern unsigned char 	*uc_ScriptBuf_ptr;

extern struct {						// Script table - keeps track of scripts in the Script Buffer.
	unsigned char	Length;
	int	StartIndex;	// offset from uc_script_buffer[0] of beginning of script.
} ScriptTable[SCRIPT_ENTRIES];

extern unsigned char 	uc_download_buffer[DOWNLOAD_SIZE];	// Download Data Buffer
extern unsigned char 	uc_upload_buffer[UPLOAD_SIZE];

extern near unsigned char asm_temp1;
extern near unsigned char asm_temp2;
extern near unsigned char asm_temp3;
extern near unsigned char asm_temp4;
extern near unsigned char asm_temp5;
extern near unsigned char asm_temp6;
extern near unsigned char asm_temp7;
extern near unsigned char asm_temp8;

/** P U B L I C  P R O T O T Y P E S *****************************************/
void PICkitInit(void);
void ProcessIO(void);

void ClearUploadBuffer(void);
void ClearDownloadBuffer(void);
void ReadUploadDataBufferNoLength(void);
void ClearScriptTable(void);
void ReadUploadDataBuffer(void);
void ScriptEngine(unsigned char *scriptstart_ptr, unsigned char scriptlength);
void RunScript(unsigned char scriptnumber, unsigned char repeat);
void CalAndSetCCP1(unsigned char ccp1_upper, unsigned char ccp1_lower);
unsigned char CalThresholdByte(unsigned char rawValue);

void EE_WriteByte(unsigned char byte_address, unsigned char write_byte);
unsigned char EE_ReadByte(unsigned char byte_address);

#endif // PICKIT_H
