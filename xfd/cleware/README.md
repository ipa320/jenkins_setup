## Install cleware ampel

### Install clewarecontrol for linux
	cd clewarecontrol
	tar -xzvf clewarecontrol-2.5.tgz
	cd clewarecontrol-2.5
	sudo make install

### Copy udev rule
	sudo cp 99-hidraw-permissions.rules /etc/udev/rules.d
	sudo /etc/init.d/udev restart
Plug the ampel out an in again.

### Start ample script
	./AmpelControl.py -u http://build.care-o-bot.org:8080/view/All
