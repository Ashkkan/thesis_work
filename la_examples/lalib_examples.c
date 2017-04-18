#include <stdio.h>
#include "blas.h"
#include "lapack.h"

const int ione = 1;
const int itwo = 2;
const int ithree = 3;
const int iseven = 7;
const double fone = 1;
const double ftwo = 2;
const double fzero = 0;
const double fmone = -1;

void printmat(double *A, int m, int n);

int main(int argc, char **argv)
{
	int i, j, m, n, k, SIZE = 3; 
	double A[SIZE*SIZE];

	for( i = 0; i < SIZE; i++ ) {
		for( j = 0; j < SIZE; j++ ) {
			A[j*SIZE+i] = (double) (i+j+1)*(2*i-j); 
		}
	}
	
	double B[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
	double C[3*4] = { 1, 2, 3, 4, 1, 2, 3, 4, 1, 2, 3, 4 };
	double result[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,  0 };
	double eyem[3*3] = { 1, 0, 0, 0, 1, 0, 0, 0, 1};
	double eyen[4*4] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
	m = 3; n = 4;
 	
	printmat(eyen, n, n);
	
	int info = 0; 
	int lworkspace = SIZE;
	int ipiv [SIZE];
	double workspace [lworkspace];
	
	F77_CALL(dgetrf)(&SIZE, &SIZE, A, &SIZE, ipiv, &info);
	F77_CALL(dgetri)(&SIZE, A, &SIZE, ipiv, workspace, &lworkspace, &info);
	if ( info != 0 ) printf("UNSECCESSFUL INVERSION");
	
	
	F77_CALL(dgemm)("n","t",&n,&m,&n,&fone,eyen,&n,C,&m,&fzero,result,&n);
	
	//printf("\n");
	//printmat(A, SIZE, SIZE);
	
		printf("\n");
	printmat(C, m, n);
	
	printf("\n");
	printmat(result, n, m);
	
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
