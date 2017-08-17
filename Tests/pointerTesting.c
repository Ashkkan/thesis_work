


#include <stdio.h>

int main(int argc, char **argv)
{
	int a[4] = { 1, 2, 3, 4 };
	int *dptr, *dptr1;
	
	dptr = a;
	dptr1 = dptr;
	dptr1++;
	
	printf("a[0] = %i", *dptr1);
	return 0;
}

