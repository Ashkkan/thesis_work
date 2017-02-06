#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <unistd.h>
#include <stdio.h>

int main(int argc, char **argv)
{
	int fd, fd2, i;
	char bytes[5];
	
	uint16_t value[4];
	value[0]=50;
	value[1]=100;
	value[2]=1000;
	value[3]=2000;
	
	//bytes[0]=(char)(value & 0xFF);
	
	if ((fd = serialOpen("/dev/ttyS0",115200)) < 0){
		fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
		return 1;		
		}
	
	if ((fd2 = serialOpen("/dev/ttyUSB0",115200)) < 0){
		fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
		return 1;		
		}
			
	if (wiringPiSetup() == -1){
		fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
		return 1;
	}
	
	for (i=0;i<10;i++){
		//printf("Bytes available: %i\n", serialDataAvail(fd));
		serialPutchar(fd, value[1]);
		printf("Value no %i: %i\n", i, serialGetchar(fd2));
		//serialFlush(fd);
		sleep(1);
	}
	
	serialFlush(fd);
	serialClose(fd);
	serialFlush(fd2);
	serialClose(fd2);
	
	return 0;
}

