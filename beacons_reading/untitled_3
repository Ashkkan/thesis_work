#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>

int main(int argc, char **argv)
{
	int fd, count, i, n;
	unsigned int nextTime;
	char ch[100];
	
	for(i=0;i<100;i++)
		ch[i] = argv[1][i];
		
	if ((fd = serialOpen("dev/ttyAMA0",115200)) < 0){
		fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
		return 1;		
		}
	
	if (wiringPiSetup() == -1){
		fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
		return 1;
	}
		
	n = strlen(ch);
	
	for(i=0;i<n;i++){
		serialPutchar(fd, ch[i]);
		printf("%d\n", i);
	}
	
	for(i=0;i<n;i++){
		printf("it is %c\n", serialGetchar(fd));
	}
	
	return 0;
}
