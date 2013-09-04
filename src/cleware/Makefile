all:
	gcc -g -DCLEWARELINUX -c USBaccessBasic.c -o USBaccessBasic.o
	g++ -g -DCLEWARELINUX -c USBaccess.cpp -o USBaccess.o
	g++ -g -DCLEWARELINUX -c Ampel.cpp -o Ampel.o
	g++ -g USBaccessBasic.o USBaccess.o Ampel.o -o Ampel

clean:
	rm Ampel *.o
