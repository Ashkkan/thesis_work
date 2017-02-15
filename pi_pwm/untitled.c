/* 
 */  
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <unistd.h>

int main(int argc, char **argv)
{
	int fd, i;
	uint8_t data8[4];
	data8[0]=0;
	data8[1]=127;
	data8[2]=250;
	data8[3]=254;
		
	if ((fd = serialOpen("/dev/ttyUSB0",115200)) < 0){
		fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
		return 1;		
		}
			
	if (wiringPiSetup() == -1){
		fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
		return 1;
	}
	
	for (i=0;i<100;i++){
		//printf("Bytes available: %i\n", serialDataAvail(fd));
		//serialPutchar(fd2, value[3] & 0xff);	//lower
		//serialPutchar(fd2, value[3] >> 8);		//higher
		if (data8[0]>=0 && data8[0] <= 254 && data8[1]>=0 && data8[1] <= 254
		&& data8[2]>=0 && data8[2] <= 254 && data8[3]>=0 && data8[3] <= 254){
		serialPutchar(fd, 255);
		serialPutchar(fd, data8[0]++);
		serialPutchar(fd, data8[1]++);
		serialPutchar(fd, data8[2]++);
		serialPutchar(fd, data8[3]);
		}	
		else
			printf("BAD PWM VALUES\n");
			
		sleep(1);
	}
	 
	serialFlush(fd);
	serialClose(fd);
	
	return 0;
}
