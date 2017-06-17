#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <errno.h>
#include <semaphore.h>
#include <math.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>	
#include <stdio.h>

struct Params {
	double kappa;
	double niters;
	};
	
static void fmpc(struct Params*);

int main(int argc, char **argv)
{
	struct Params params;
	
	fmpc(&params);
	
	printf("%f\n", params.kappa);
	printf("%f\n", params.niters);
	
	return 0;
}

static void fmpc(struct Params *par){
	par->kappa=5;
}
