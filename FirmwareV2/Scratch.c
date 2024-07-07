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

#define tris_BUSY_LED       TRISCbits.TRISC0    // RC0 Output
#define BUSY_LED            LATCbits.LATC0

extern volatile near unsigned char       PIE1;
extern volatile near struct {
  unsigned TMR1IE:1;
  unsigned TMR2IE:1;
  unsigned CCP1IE:1;
  unsigned SSPIE:1;
  unsigned TXIE:1;
  unsigned RCIE:1;
  unsigned ADIE:1;
} PIE1bits;
extern volatile near unsigned char       PIR1;
extern volatile near struct {
  unsigned TMR1IF:1;
  unsigned TMR2IF:1;
  unsigned CCP1IF:1;
  unsigned SSPIF:1;
  unsigned TXIF:1;
  unsigned RCIF:1;
  unsigned ADIF:1;
} PIR1bits;
extern volatile near unsigned char       PIE2;
extern volatile near union {
  struct {
    unsigned CCP2IE:1;
    unsigned TMR3IE:1;
    unsigned LVDIE:1;
    unsigned BCLIE:1;
    unsigned EEIE:1;
    unsigned USBIE:1;
    unsigned CMIE:1;
    unsigned OSCFIE:1;
  };
  struct {
    unsigned :2;
    unsigned HLVDIE:1;
  };
} PIE2bits;
extern volatile near unsigned char       PIR2;
extern volatile near union {
  struct {
    unsigned CCP2IF:1;
    unsigned TMR3IF:1;
    unsigned LVDIF:1;
    unsigned BCLIF:1;
    unsigned EEIF:1;
    unsigned USBIF:1;
    unsigned CMIF:1;
    unsigned OSCFIF:1;
  };
  struct {
    unsigned :2;
    unsigned HLVDIF:1;
  };
} PIR2bits;
extern volatile near unsigned char       IPR2;
extern volatile near union {
  struct {
    unsigned CCP2IP:1;
    unsigned TMR3IP:1;
    unsigned LVDIP:1;
    unsigned BCLIP:1;
    unsigned EEIP:1;
    unsigned USBIP:1;
    unsigned CMIP:1;
    unsigned OSCFIP:1;
  };
  struct {
    unsigned :2;
    unsigned HLVDIP:1;
  };
} IPR2bits;

extern volatile near unsigned char       T1CON;
extern volatile near union {
  struct {
    unsigned TMR1ON:1;
    unsigned TMR1CS:1;
    unsigned T1SYNC:1;
    unsigned T1OSCEN:1;
    unsigned T1CKPS0:1;
    unsigned T1CKPS1:1;
    unsigned T1RUN:1;
    unsigned RD16:1;
  };
  struct {
    unsigned :2;
    unsigned NOT_T1SYNC:1;
  };
} T1CONbits;
extern volatile near unsigned char       TMR1L;
extern volatile near unsigned char       TMR1H;
extern volatile near unsigned char       T3CON;
extern volatile near union {
  struct {
    unsigned TMR3ON:1;
    unsigned TMR3CS:1;
    unsigned T3SYNC:1;
    unsigned T3CCP1:1;
    unsigned T3CKPS0:1;
    unsigned T3CKPS1:1;
    unsigned T3CCP2:1;
    unsigned RD16:1;
  };
  struct {
    unsigned :2;
    unsigned T3NSYNC:1;
  };
  struct {
    unsigned :2;
    unsigned NOT_T3SYNC:1;
  };
} T3CONbits;
extern volatile near unsigned char       TMR3L;
extern volatile near unsigned char       TMR3H;
extern volatile near unsigned char       CMCON;
extern volatile near struct {
  unsigned CM0:1;
  unsigned CM1:1;
  unsigned CM2:1;
  unsigned CIS:1;
  unsigned C1INV:1;
  unsigned C2INV:1;
  unsigned C1OUT:1;
  unsigned C2OUT:1;
} CMCONbits;
extern volatile near unsigned char       CVRCON;
extern volatile near union {
  struct {
    unsigned CVR0:1;
    unsigned CVR1:1;
    unsigned CVR2:1;
    unsigned CVR3:1;
    unsigned CVREF:1;
    unsigned CVRR:1;
    unsigned CVROE:1;
    unsigned CVREN:1;
  };
  struct {
    unsigned :4;
    unsigned CVRSS:1;
  };
} CVRCONbits;

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
    unsigned char   TimerBaudLoadL;      // timer value to load for particular baud rate
    unsigned char   TimerBaudLoadH;
    unsigned char   RXbits;             // # bits received during byte reception
    unsigned char   RXbyte;             // Byte value being received
    unsigned char   TXbits;             // # bits send during byte transmission
    unsigned char   TXByte;             // Byte value being sent
    unsigned char   LastRXByte;         // Received byte to be saved
    unsigned char   NewRX;              // flag for new data in LastRXByte
} UARTStatus;

void ExitUARTMode(void)

void ICDSlaveBL_transmit (unsigned char TransmitByte)
unsigned char ICDSlaveBL_Receive (void)

#define ICDSLAVE_RX_BL      0xC2
#define ICDSLAVE_TX_LIT_BL  0xC1
#define ICDSLAVE_TX_BUF_BL  0xC0