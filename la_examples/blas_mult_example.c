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
void blas_mult(int n, int m, int k, double** a, double** b, double** c,
double alpha, double beta);

int main(int argc, char **argv)
{
	double A[9] = { 1, 2, 3, 4, 5, 6, 7, 8, 9 };
	double B[6] = { 1, 3, 5, 2, 4, 6 };
	double C[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	double D[2] = { 1, 2 };
	double y[3] = { 0, 0, 0};
	
	int n = 3;
	int m = 2;
	
	/* 					 &n  &m            &n     &ione             &ione */ //always true
	F77_CALL(dgemv)("n", &n, &m, &fone, B, &n, D ,&ione, &fzero, y, &ione);
	
	F77_CALL(dgemm)("t","t",&n,&n,&n,&fone,A,&n,B,&n,&fzero,C,&n);
	
	printmat(y, n, 1);

	//blas_mult(3, 3, 3, *A[0], *B[0], *C[0], 1, 1);
	return 0;
}

/* void blas_mult(int n, int m, int k, double** a, double** b, double** c,
double alpha, double beta) {
	cblas_dgemm(CblasRowMajor,CblasNoTrans, CblasNoTrans, 
	m, n, k, alpha, a[0], k, b[0], n, beta, c[0], n);
} */

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


