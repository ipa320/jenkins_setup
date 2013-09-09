// DLL class definitions for access to USB HID devices
//
// (C) 2001-2003 Copyright Cleware GmbH
// All rights reserved
//
// History:
// 05.01.2001	ws	Initial coding
// 17.07.2001	ws	cleanup interface
// 10.12.2001	ws	cleanup interface again, basic class to hide implementation details
// 13.12.2001	ws	introduced versionNumber and virtual destructor
// 23.05.2002	ws	added switch access
// ...
// 03.02.2003	ws	added switch version 10 
// 04.08.2003	ws	added humidity 
// 21.01.2004	ws	fixed some humidity problems 
//		 2004	ws	added contact + io16
// 05.02.2005	ws	added ADC08-Support 



// The following ifdef block is the standard way of creating macros which make exporting 
// from a DLL simpler. All files within this DLL are compiled with the USBACCESS_EXPORTS
// symbol defined on the command line. this symbol should not be defined on any project
// that uses this DLL. This way any other project whose source files include this file see 
// USBACCESS_API functions as being imported from a DLL, wheras this DLL sees symbols
// defined with this macro as being exported.


#ifndef __USBACCESS_H__
#define __USBACCESS_H__

typedef int HANDLE ;

#ifdef USBACCESS_EXPORTS
#define USBACCESS_API __declspec(dllexport)
#else
#define USBACCESS_API __declspec(dllimport)
#endif

const int USBaccessVersion = 330 ;

class CUSBaccess {
	public:
		enum USBactions {		LEDs=0, EEwrite=1, EEread=2, Reset=3, KeepCalm=4, GetInfo=5, 
								StartMeasuring=6,		// USB-Humidity
								Configure=7,			// USB-IO16-V10, USB-Counter-V05
								RunPoint=10				// USB-Encoder
								} ;
		enum USBInfoType {		OnlineTime=1, OnlineCount=2, ManualTime=3, ManualCount=4 } ;
		enum LED_IDs {			LED_0=0, LED_1=1, LED_2=2, LED_3=3 } ;
		enum COUNTER_IDs {		COUNTER_0=0, COUNTER_1=1 } ;
		enum SWITCH_IDs {		SWITCH_0=0x10, SWITCH_1=0x11, SWITCH_2=0x12, SWITCH_3=0x13,
								SWITCH_4=0x14, SWITCH_5=0x15, SWITCH_6=0x16, SWITCH_7=0x17,
								SWITCH_8=0x18, SWITCH_9=0x19, SWITCH_10=0x1a, SWITCH_11=0x1b,
								SWITCH_12=0x1c, SWITCH_13=0x1d, SWITCH_14=0x1e, SWITCH_15=0x1f
								} ;
		enum USBtype_enum {		ILLEGAL_DEVICE=0,
								LED_DEVICE=0x01,
								POWER_DEVICE=0x02,
								WATCHDOG_DEVICE=0x05,
								AUTORESET_DEVICE=0x06,
								WATCHDOGXP_DEVICE=0x07,
								SWITCH1_DEVICE=0x08,
								SWITCH2_DEVICE=0x09, SWITCH3_DEVICE=0x0a, SWITCH4_DEVICE=0x0b,
								SWITCH5_DEVICE=0x0c, SWITCH6_DEVICE=0x0d, SWITCH7_DEVICE=0x0e, SWITCH8_DEVICE=0x0f,
								TEMPERATURE_DEVICE=0x10, 
								TEMPERATURE2_DEVICE=0x11,
								TEMPERATURE5_DEVICE=0x15, 
								HUMIDITY1_DEVICE=0x20,
								SWITCHX_DEVICE=0x28,		// new switch 3,4,8
								CONTACT00_DEVICE=0x30, CONTACT01_DEVICE=0x31, CONTACT02_DEVICE=0x32, CONTACT03_DEVICE=0x33, 
								CONTACT04_DEVICE=0x34, CONTACT05_DEVICE=0x35, CONTACT06_DEVICE=0x36, CONTACT07_DEVICE=0x37, 
								CONTACT08_DEVICE=0x38, CONTACT09_DEVICE=0x39, CONTACT10_DEVICE=0x3a, CONTACT11_DEVICE=0x3b, 
								CONTACT12_DEVICE=0x3c, CONTACT13_DEVICE=0x3d, CONTACT14_DEVICE=0x3e, CONTACT15_DEVICE=0x3f, 
								F4_DEVICE=0x40, 
								KEYC01_DEVICE=0x41, KEYC16_DEVICE=0x42,
								ADC0800_DEVICE=0x50, ADC0801_DEVICE=0x51, ADC0802_DEVICE=0x52, ADC0803_DEVICE=0x53, 
								COUNTER00_DEVICE=0x60, 
								ENCODER01_DEVICE=0x80,
								BUTTON_NODEVICE=0x1000
								} ;
	private:
		class CUSBaccessBasic *	X	;	// avoid export of internal USB variables

	public:
		CUSBaccess() ;
		virtual ~CUSBaccess() ;		// maybe used as base class

		virtual int			OpenCleware() ;			// returns number of found Cleware devices
		virtual int			CloseCleware() ;		// close all Cleware devices
		virtual int			Recover(int devNum) ;	// try to find disconnected devices, returns true if succeeded
		virtual HANDLE		GetHandle(int deviceNo) ;
		virtual int			GetValue(int deviceNo, unsigned char *buf, int bufsize) ;
		virtual int			SetValue(int deviceNo, unsigned char *buf, int bufsize) ;
		virtual int			SetLED(int deviceNo, enum LED_IDs Led, int value) ;	// value: 0=off 7=medium 15=highlight
		virtual int			SetSwitch(int deviceNo, enum SWITCH_IDs Switch, int On) ;	//	On: 0=off, 1=on
		virtual int			GetSwitch(int deviceNo, enum SWITCH_IDs Switch) ;			//	On: 0=off, 1=on, -1=error
		virtual int			GetSeqSwitch(int deviceNo, enum SWITCH_IDs Switch, int seqNum) ;		//	On: 0=off, 1=on, -1=error
		virtual int			GetSwitchConfig(int deviceNo, int *switchCount, int *buttonAvailable) ;
		virtual int			GetTemperature(int deviceNo, double *Temperature, int *timeID) ;
		virtual int			GetHumidity(int deviceNo, double *Humidity, int *timeID) ;
		virtual int			ResetDevice(int deviceNo) ;
		virtual int			StartDevice(int deviceNo) ;	
		virtual int			CalmWatchdog(int deviceNo, int minutes, int minutes2restart) ;
		virtual int			GetVersion(int deviceNo) ;
		virtual int			GetUSBType(int deviceNo) ;
		virtual int			GetSerialNumber(int deviceNo) ;
		virtual int			GetDLLVersion() { return USBaccessVersion ; }
		virtual int			GetManualOnCount(int deviceNo) ;		// returns how often switch is manually turned on
		virtual int			GetManualOnTime(int deviceNo) ;			// returns how long (seconds) switch is manually turned on
		virtual int			GetOnlineOnCount(int deviceNo) ;		// returns how often switch is turned on by USB command
		virtual int			GetOnlineOnTime(int deviceNo) ;			// returns how long (seconds) switch is turned on by USB command
		virtual int			GetMultiSwitch(int deviceNo, unsigned long int *mask, unsigned long int *value, int seqNumber) ;
		virtual int			SetMultiSwitch(int deviceNo, unsigned long int value) ;
		virtual int			SetMultiConfig(int deviceNo, unsigned long int directions) ;
		virtual int			GetCounter(int deviceNo, enum COUNTER_IDs counterID) ;	// COUNTER_IDs ununsed until now
		virtual int			SetCounter(int deviceNo, int counter, enum COUNTER_IDs counterID) ;	//  -1=error, COUNTER_IDs ununsed until now
		virtual int			SyncDevice(int deviceNo, unsigned long int mask) ;	
		virtual void		DebugWrite(char *s) ;
		virtual void		DebugWrite(char *f, int a1) ;
		virtual void		DebugWrite(char *f, int a1, int a2) ;
		virtual void		DebugWrite(char *f, int a1, int a2, int a3) ;
		virtual void		DebugWrite(char *f, int a1, int a2, int a3, int a4) ;
		virtual void		Sleep(int ms) { usleep(ms * 1000) ; }	// for Linux
	} ;

#endif // __USBACCESS_H__
