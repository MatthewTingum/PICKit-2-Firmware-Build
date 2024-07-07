/*********************************************************************
 *
 *                Microchip USB C18 Firmware Version 1.0
 *
 *********************************************************************
 * FileName:        usb9.c
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
 * Rawin Rojvanit       11/19/04    Original.
 ********************************************************************/

/******************************************************************************
 * -usb9.c-
 * This is a mini-version of the actual usb9.c from the Microchip USB library.
 * Many functions are commented out or modified to make the USB stack smaller
 *****************************************************************************/

/** I N C L U D E S **********************************************************/
#include <p18cxxx.h>
#include "system\typedefs.h"
#include "system\usb\usb.h"
#include "io_cfg.h"                     // Required for self_power status

/** V A R I A B L E S ********************************************************/
#pragma udata

/** P R I V A T E  P R O T O T Y P E S ***************************************/
void USBStdGetDscHandler(void);
void USBStdSetCfgHandler(void);
void USBStdGetStatusHandler(void);
void USBStdFeatureReqHandler(void);

/** D E C L A R A T I O N S **************************************************/
#pragma code
/******************************************************************************
 * Function:        void USBCheckStdRequest(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine checks the setup data packet to see if it
 *                  knows how to handle it
 *
 * Note:            None
 *****************************************************************************/
void USBCheckStdRequest(void)
{   
    if(SetupPkt.RequestType != STANDARD) return;
    
    switch(SetupPkt.bRequest)
    {
        case SET_ADR:
            ctrl_trf_session_owner = MUID_USB9;
            usb_device_state = ADR_PENDING_STATE;       // Update state only
            /* See USBCtrlTrfInHandler() in usbctrltrf.c for the next step */
            break;
        case GET_DSC:
            ctrl_trf_session_owner = MUID_USB9;
            if(SetupPkt.bDscType == DSC_DEV)
            {
                    pSrc.bRom = (rom byte*)&device_dsc;
                    wCount._word = sizeof(device_dsc);          // Set data count
            } 
            else if(SetupPkt.bDscType == DSC_CFG)
            {
                    pSrc.bRom = *(USB_CD_Ptr+SetupPkt.bDscIndex);
                    wCount._word = *(pSrc.wRom+1);              // Set data count
            }   
            else if(SetupPkt.bDscType == DSC_STR)
            {
                   if (SetupPkt.bDscIndex == SERIAL_UNITID_DSC)
                    { // serial number string
                        pSrc.bRam = unit_id_usb_buf;
                        wCount._word = *pSrc.bRam;
                        usb_stat.ctrl_trf_mem = _RAM;
                        break;
                    }
                    pSrc.bRom = *(USB_SD_Ptr+SetupPkt.bDscIndex);
                    wCount._word = *pSrc.bRom;                  // Set data count
            }
            else
                ctrl_trf_session_owner = MUID_NULL;
            
            usb_stat.ctrl_trf_mem = _ROM;                       // Set memory type 
            break;
        case SET_CFG:
            ctrl_trf_session_owner = MUID_USB9;
            //mDisableEP1to15();                          // See usbdrv.h
            //ClearArray((byte*)&usb_alt_intf,MAX_NUM_INT);
            usb_active_cfg = SetupPkt.bCfgValue;
            if(SetupPkt.bCfgValue == 0)
                usb_device_state = ADDRESS_STATE;
            else
            {
                usb_device_state = CONFIGURED_STATE;
        
                /* Modifiable Section */
                
                #if defined(USB_USE_HID)                // See autofiles\usbcfg.h
                HIDInitEP();
                #endif
                
                /* End modifiable section */
        
            }//end if(SetupPkt.bcfgValue == 0)
            break;
        case GET_CFG:
            ctrl_trf_session_owner = MUID_USB9;
            pSrc.bRam = (byte*)&usb_active_cfg;         // Set Source
            usb_stat.ctrl_trf_mem = _RAM;               // Set memory type
            LSB(wCount) = 1;                            // Set data count
            break;
//        case GET_STATUS:
//            USBStdGetStatusHandler();
//            break;
//        case CLR_FEATURE:
//        case SET_FEATURE:
//            USBStdFeatureReqHandler();
//            break;
//        case GET_INTF:
//            ctrl_trf_session_owner = MUID_USB9;
//            pSrc.bRam = (byte*)&usb_alt_intf+SetupPkt.bIntfID;  // Set source
//            usb_stat.ctrl_trf_mem = _RAM;               // Set memory type
//            LSB(wCount) = 1;                            // Set data count
//            break;
//        case SET_INTF:
//            ctrl_trf_session_owner = MUID_USB9;
//            usb_alt_intf[SetupPkt.bIntfID] = SetupPkt.bAltID;
//            break;
//        case SET_DSC:
//        case SYNCH_FRAME:
        default:
            break;
    }//end switch
    
}//end USBCheckStdRequest


/** EOF usb9.c ***************************************************************/
