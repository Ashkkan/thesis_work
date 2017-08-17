#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "wiringPi.h"
#include "wiringSerial.h"

int main(int argc, char **argv)
{
	int fd, count, i, n;
	unsigned int nextTime;
	char ch[100];
	char NMEA[6];
	char char_one[1];
	int finish;
	
	//for(i=0;i<100;i++)
		//ch[i] = argv[1][i];
		
	if ((fd = serialOpen("/dev/ttyUSB0", 9600)) < 0){
		fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
		return 1;		
		}
	
	if (wiringPiSetup() == -1){
		fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
		return 1;
	}
		
	while(1) {
		for ( i = 1; i < 5; i++) {
			NMEA[i-1] = NMEA[i];
		}
		NMEA[4] = serialGetchar(fd);
		
		while ( strcmp(NMEA, "$GPRMC") ) {
			printf("---$GPRMC---");
			
			char_one = serialGetchar(fd);
			
			if ( strcmp(char_one, "$") ) {
				printf("it is $\n");
			}
			
			//while ( !strcmp(char_one[0] = serialGetchar(fd), "$") ) {
				//printf("%c", serialGetchar(fd));
			//}
			
			//if ( 1 ) break;
			//else printf("%c", serialGetchar(fd));
		}
		
		
		//n = strlen(ch);
	
		//for(i=0;i<n;i++){
			//serialPutchar(fd, ch[i]);
			//printf("%d\n", i);
		//}
		
		//for(i=0;i<n;i++){
			//printf("%c", serialGetchar(fd));
		//}
		//printf("\n- %i -\n", n);
	
		//printf("%c", serialGetchar(fd));
		//printf("\n-----\n", n);
	}	

	
	return 0;
}
