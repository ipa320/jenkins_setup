/* USBswitchCmd [-n device] [0 | 1] [-d]
 *           -n device   use device with this serial number
 *           0 | 1       turns switch off(0) or on(1)
 *           -d          print debug infos
 *           -w          make use of the watchdog
 *           -s          secure switching - wait and ask if switching was done
 *           -r          read the current setting
 *           -t          reseT the device
 *           -i nnn      interval test, turn endless on/off and wait nnn ms between state change
 *           -# switch#  select switch for multiple switch device, first=0
 *           -v          print version
 *           -h          print command usage

 *
 * (C) Copyright 2003-2005 Cleware GmbH
 *
 * Version     Date     Comment
 *   1.0    03.04.2003	Initial coding
 *   3.0    0x.0x.2005
 *
 */

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include "USBaccess.h"

#define Sleep(ms) usleep( (ms) * 1000)

int 
main(int argc, char* argv[]) {
	CUSBaccess CWusb ;
	
	printf("Start USB Access Example!\n") ;
//	int USBcount = CWusb.OpenCleware() ;
//	printf("OpenCleware fand %d Geräte\n", USBcount) ;

//	CWusb.CloseCleware() ;

	return 0;
	}

