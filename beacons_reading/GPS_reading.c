#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "wiringPi.h"
#include "wiringSerial.h"

int main(int argc, char **argv)
{
	int fd, i, j, k, l,  decimalFlag, comma_read;
	//char ch[100], char_one[1];
	char NMEA[6], char_two[2] = { 'G', '\0' }, validaty[2] = { '\0','\0' };
	char time[11] = {'\0','\0','\0','\0','\0','\0','\0','\0','\0','\0','\0'}, latitude_char[8], longitude_char[9], altitude_char[10];
	NMEA[5] = '\0'; latitude_char[7] = '\0'; longitude_char[8] = '\0';
	double latitude=0, longitude=0, latMinute=0, lonMinute=0, altitude=0;
	
	if ((fd = serialOpen("/dev/ttyUSB0", 9600)) < 0) {
		fprintf(stderr, "Unable to open serial device: %s\n", strerror (errno));
		return 1;		
	}
	
	if (wiringPiSetup() == -1) {
		fprintf(stdout, "Unable to start wiringPi: %s\n", strerror(errno));
		return 1;
	}
		
	while(1) {
		for ( i = 1; i < 5; i++) {
			NMEA[i-1] = NMEA[i];
		}
		NMEA[4] = serialGetchar(fd);
		

		//printf("%c", NMEA[4]);
		
		//printf("--- %c %c %c %c %c ---\n", NMEA[0], NMEA[1], NMEA[2], NMEA[3], NMEA[4]);
		
		//char_one[0] = NMEA[0];
		//NMEA[0] = (char)NMEA[0];
			
		//if ( strcmp(&char_two[0], "G" ) == 0 ) {
			//printf("dollars\n");
		//}	



		if ( 1 && strcmp(&NMEA[0], "GPRMC" ) == 0 ) {
			printf("-------GPRMC------");
			i=0;	//reset comma counter
			while ( strcmp(&char_two[0], "$" ) != 0 ) {
				
				char_two[0] = serialGetchar(fd);	//read again
				if ( strcmp(&char_two[0], "," ) == 0 ) {
					i++;					//next comma
					if ( 1 || (i != 1 && i != 2 )) {			// present the ones that are not taken care of yet!
						printf("\n%i->  ", i);
					}
				}
				else if ( strcmp(&char_two[0], "$" ) == 0 ) {
					printf("------------------\n");
				}
				else {
					printf("%c", char_two[0]);
				}
				
				
				if ( i==1 ) {	
					for ( j = 0; j < 10; j++) {
						time[j] = serialGetchar(fd);
					}
					printf("Time: %s", time);
					//for ( j = 0; j < 4; j++) {
						//serialGetchar(fd);
					//}
				}
				if ( i==2 ) {
					validaty[0] = serialGetchar(fd);
					printf("Validaty: %s", validaty);
				}
				if ( i==3 ) {
					char latitude_temp[8] = {'5','2','1','2','.','6','7','\0'};	//52 deg and 12.67 min 
					
					for ( j = 0; j < 7; j++) {
						//latitude_char[j] = serialGetchar(fd);
						latitude_char[j] = latitude_temp[j];
						if (j==0) latitude  = 10*(latitude_char[j]-'0');
						if (j==1) latitude +=     latitude_char[j]-'0';
						if (j==2) latMinute = 10*(latitude_char[j]-'0');
						if (j==3) latMinute+=    (latitude_char[j]-'0');
						if (j==5) latMinute+= .1*(latitude_char[j]-'0');
						if (j==6) latMinute+=.01*(latitude_char[j]-'0');
					}
					latitude += latMinute/60;
					printf("latitude_char: %s and latitude in dec deg: %f", latitude_char, latitude);
				}
				if ( i==4 ) {
					char latSign_temp[2] = {'N','\0'};
					char_two[0] = latSign_temp[0];
					//char_two[0] = serialGetchar(fd);
					if ( strcmp(&char_two[0], "S" ) == 0 ) latitude *= -1;
					printf("char_two: %s and latitude is %f", char_two, latitude);
				}
				if ( i==5 ) {
					char longitude_temp[9] = {'0','0','0','5','2','.','4','4','\0'};	//000 deg and 52.44 min
					
					for ( j = 0; j < 8; j++) {
						//latitude_char[j] = serialGetchar(fd);
						longitude_char[j] = longitude_temp[j];
						if (j==0) longitude =100*(longitude_char[j]-'0');
						if (j==1) longitude+= 10*(longitude_char[j]-'0');
						if (j==2) longitude+=     longitude_char[j]-'0';
						if (j==3) lonMinute = 10*(longitude_char[j]-'0');
						if (j==4) lonMinute+=    (longitude_char[j]-'0');
						if (j==6) lonMinute+= .1*(longitude_char[j]-'0');
						if (j==7) lonMinute+=.01*(longitude_char[j]-'0');
					}
					longitude += lonMinute/60;
					printf("longitude_char: %s and longitude in dec deg: %f", longitude_char, longitude);
				}	
				if ( i==6 ) {
					char lonSign_temp[2] = {'W','\0'};
					char_two[0] = lonSign_temp[0];
					//char_two[0] = serialGetchar(fd);
					if ( strcmp(&char_two[0], "W" ) == 0 ) longitude *= -1;
					printf("char_two: %s and longitude is %f", char_two, longitude);
				}
				
			}
			char_two[0] = '\0';	//Reset
		}
		
		if ( 1 && strcmp(&NMEA[0], "GPGGA" ) == 0 ) {
			printf("-------GPGGA------");
			i=0;	//reset comma counter
			comma_read = 0;
			while ( strcmp(&char_two[0], "$" ) != 0 ) {
				char_two[0] = serialGetchar(fd);	//read again
				
				if ( strcmp(&char_two[0], "," ) == 0 || comma_read==1 ) {
					i++;					//next comma
					comma_read=0;
					if ( 1 || (i != 1 && i != 2 )) {			// present the ones that are not taken care of yet!
						printf("\n%i->  ", i);
					}
				}
				else if ( strcmp(&char_two[0], "$" ) == 0 ) {
					printf("------------------\n");
				}
				else {
					printf("%c", char_two[0]);
				}
				
				if ( i==1 ) {	
					for ( j = 0; j < 10; j++) {
						time[j] = serialGetchar(fd);
					}
					printf("Time: %s", time);
					//for ( j = 0; j < 4; j++) {
						//serialGetchar(fd);
					//}
				}
				if ( i==2 ) {
					char latitude_temp[8] = {'5','2','1','2','.','6','7','\0'};	//52 deg and 12.67 min 
					
					for ( j = 0; j < 7; j++) {
						//latitude_char[j] = serialGetchar(fd);
						latitude_char[j] = latitude_temp[j];
						if (j==0) latitude  = 10*(latitude_char[j]-'0');
						if (j==1) latitude +=     latitude_char[j]-'0';
						if (j==2) latMinute = 10*(latitude_char[j]-'0');
						if (j==3) latMinute+=    (latitude_char[j]-'0');
						if (j==5) latMinute+= .1*(latitude_char[j]-'0');
						if (j==6) latMinute+=.01*(latitude_char[j]-'0');
					}
					latitude += latMinute/60;
					printf("latitude_char: %s and latitude in dec deg: %f", latitude_char, latitude);
				}
				if ( i==3 ) {
					char latSign_temp[2] = {'N','\0'};
					char_two[0] = latSign_temp[0];
					//char_two[0] = serialGetchar(fd);
					if ( strcmp(&char_two[0], "S" ) == 0 ) latitude *= -1;
					printf("char_two: %s and latitude is %f", char_two, latitude);
				}
				if ( i==4 ) {
					char longitude_temp[9] = {'0','0','0','5','2','.','4','4','\0'};	//000 deg and 52.44 min
					
					for ( j = 0; j < 8; j++) {
						//latitude_char[j] = serialGetchar(fd);
						longitude_char[j] = longitude_temp[j];
						if (j==0) longitude =100*(longitude_char[j]-'0');
						if (j==1) longitude+= 10*(longitude_char[j]-'0');
						if (j==2) longitude+=     longitude_char[j]-'0';
						if (j==3) lonMinute = 10*(longitude_char[j]-'0');
						if (j==4) lonMinute+=    (longitude_char[j]-'0');
						if (j==6) lonMinute+= .1*(longitude_char[j]-'0');
						if (j==7) lonMinute+=.01*(longitude_char[j]-'0');
					}
					longitude += lonMinute/60;
					printf("longitude_char: %s and longitude in dec deg: %f", longitude_char, longitude);
				}	
				if ( i==5 ) {
					char lonSign_temp[2] = {'W','\0'};
					char_two[0] = lonSign_temp[0];
					//char_two[0] = serialGetchar(fd);
					if ( strcmp(&char_two[0], "W" ) == 0 ) longitude *= -1;
					printf("char_two: %s and longitude is %f", char_two, longitude);
				}
				if ( i==6 ) {
					validaty[0] = serialGetchar(fd);
					printf("Validaty: %s", validaty);
				}
				if ( i==9 ) {
					char altitude_temp[8] = {'2','8','1','.','3','4',',','\0'};
					altitude=0; decimalFlag=0; j=0; k=0; l=0;
					while ( comma_read == 0 ) {
						//char_two[0] = serialGetchar(fd);
						if ( l<7 ) {
							char_two[0] = altitude_temp[l];
							l++;
						}
						if ( strcmp(&char_two[0], "." ) == 0 ) { decimalFlag = 1; altitude_char[j] = char_two[0]; }
						else if ( strcmp(&char_two[0], "," ) == 0 ) { comma_read=1; altitude_char[j+k+1] = '\0';}
						else if ( decimalFlag == 0 ) {
							altitude_char[j] = char_two[0];
							j++;
						}
						else if ( decimalFlag == 1 ) {
							altitude_char[j+k+1] = char_two[0];
							k++;
						}
					}
					
					for ( l=0; l<j; l++ ) {
						altitude += (altitude_char[l]-'0')*pow(10,j-l-1);
						//printf("altitude=%f at j=%i", altitude, l);
					}
					for ( l=0; l<k; l++ ) {
						altitude += (altitude_char[l+j+1]-'0')*pow(10,-(l+1));
						//printf("altitude=%f at k=%i", altitude, l);
					}
					printf("altitude: %f, altitude_char: %s", altitude, altitude_char);
				}
				if ( i==10 ) {
					char_two[0] = serialGetchar(fd);
					if ( strcmp(&char_two[0], "M" ) != 0 ) { printf("Altitude is NOT in meters!!!"); }
					else printf("%s", char_two);
				}
				
			}
			char_two[0] = '\0';	//Reset
		}
		
		
		
		
		
		//printf("OUTSIDE\n");
	}	

	
	return 0;
}
