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
#include <vector>
#include "USBaccess.h"

#define Sleep(ms) usleep( (ms) * 1000)

int 
main(int argc, char* argv[]) {
	CUSBaccess CWusb ;
	
	printf("Start USB Access Example!\n") ;
	int USBcount = CWusb.OpenCleware() ;
	printf("OpenCleware found %d devices\n", USBcount) ;

	USBcount = 2; //TODO remove

	for (int devID=0 ; devID < USBcount ; devID++) {
		int devType = CWusb.GetUSBType(devID) ;
		printf("Device %d: Type=%d, Version=%d, SerNum=%d\n", devID,
					devType, CWusb.GetVersion(devID),
					CWusb.GetSerialNumber(devID)) ;

		devType = CUSBaccess::SWITCH1_DEVICE; //TODO remove

		// TODO: implement ampel logic with results of jenkins here
		std::vector<bool> status(3, false);
		status[0] = true; // red
		status[1] = false; // yellow
		status[2] = false; // green


		// TODO: how to distinguish the three lights (red, yellow, green)?
		if (devType == CUSBaccess::SWITCH1_DEVICE) {
			printf("old switch setting = %d\n", CWusb.GetSwitch(devID, CUSBaccess::SWITCH_0)) ;
			
			printf("turn light on\n");
			CWusb.SetSwitch(devID, CUSBaccess::SWITCH_0, 1) ;
			usleep(1000000); // wait 1 sec
			printf("turn light off\n");
			CWusb.SetSwitch(devID, CUSBaccess::SWITCH_0, 0) ;
			break ;
		}
		else
			continue ;
	}

	CWusb.CloseCleware() ;

	return 0;
	}

