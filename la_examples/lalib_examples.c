#include <stdio.h>
#include "blas.h"
#include "lapack.h"

void printmat(double *A, int m, int n);

int main(int argc, char **argv)
{
	int i, j, SIZE = 3; 
	double A[SIZE*SIZE];

	for( i = 0; i < SIZE; i++ ) {
		for( j = 0; j < SIZE; j++ ) {
			A[j*SIZE+i] = (double) (i+j+1)*(2*i-j); 
		}
	}
	
	double B[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
	double C[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0  };
	
	printmat(A, SIZE, SIZE);
	
	int info = 0; 
	int lworkspace = SIZE;
	int ipiv [SIZE];
	double workspace [lworkspace];
	
	F77_CALL(dgetrf)(&SIZE, &SIZE, A, &SIZE, ipiv, &info);
	F77_CALL(dgetri)(&SIZE, A, &SIZE, ipiv, workspace, &lworkspace, &info);
	if ( info != 0 ) printf("UNSECCESSFUL INVERSION");
	
	printf("\n");
	printmat(A, SIZE, SIZE);
	
	return 0;
}

void printmat(double *A, int m, int n)
{
    double *dptr;
    int j, i;
    dptr = A;
    for (j = 0; j < m; j++)
    {
        for (i = 0; i < n; i++)
        {
            printf("%5.4f\t", *(dptr+m*i+j));
        }
        printf("\n");
    }
    return;
}
