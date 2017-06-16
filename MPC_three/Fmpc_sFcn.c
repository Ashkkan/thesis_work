#define S_FUNCTION_NAME  Fmpc_sFcn
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "fmpc_step_simulink_c.c"

const int paramsNum = 2;

/*================*
 * Build checking *
 *================*/

// ==================ASHKAN BEGINS 1===========================
// // #include "sfun_slutils.c"
// 
// enum {PARAM = 0, NUM_PARAMS};
// 
// #define PARAM_ARG ssGetSFcnParam(S, PARAM)
// 
// #define EDIT_OK(S, ARG) \
// (!((ssGetSimMode(S) == SS_SIMMODE_SIZES_CALL_ONLY) && mxIsEmpty(ARG)))
// 
// 
// #ifdef MATLAB_MEX_FILE
// #define MDL_CHECK_PARAMETERS
// /* Function: mdlCheckParameters =============================================
//  * Abstract:
//  *    Verify parameter settings.
//  */
// static void mdlCheckParameters(SimStruct *S)
// {
//     if(EDIT_OK(S, PARAM_ARG)){
//         /* Check that parameter value is not empty*/
//         if( mxIsEmpty(PARAM_ARG) ) {
//             ssSetErrorStatus(S, "Invalid parameter specified. The parameter "
//             "must be non-empty");
//             return;
//         }
//     }
// } /* end mdlCheckParameters */
// #endif
// ==================ASHKAN ENDS 1===========================

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *   Setup sizes of the various vectors.
 */
static void mdlInitializeSizes(SimStruct *S)
{
    const mxArray   *prhs[paramsNum];
    int n, m ,T, nd;
    
    prhs[0] = ssGetSFcnParam(S,0); //sys
    prhs[1] = ssGetSFcnParam(S,1); //params
    n =  (int)mxGetScalar(mxGetField(prhs[0],0,"n" ));
    m =  (int)mxGetScalar(mxGetField(prhs[0],0,"m" ));
//     nd = (int)mxGetScalar(mxGetField(prhs[0],0,"nd"));
    T  = (int)mxGetScalar(mxGetField(prhs[1],0,"T" ));
//     n=n+nd;
    
    ssSetNumSFcnParams(S, paramsNum);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }
    
    /* Allow signal dimensions greater than 2 */
//     ssAllowSignalsWithMoreThan2D(S);

    if (!ssSetNumInputPorts(S, 1)) return;
    ssSetInputPortWidth(S, 0, n*T + n*T + n + n*n + n*m + 1+n+n); // DYNAMICALLY_SIZED can be also used instead as a flag
    ssSetInputPortDirectFeedThrough(S, 0, 1);

    if (!ssSetNumOutputPorts(S,1)) return;
//     ssSetOutputPortWidth(S, 0, 120+30+1);
    ssSetOutputPortWidth(S, 0, n*T + m*T + 1 ); // DYNAMICALLY_SIZED can be also used instead as a flag
    
    ssSetNumSampleTimes(S, 1);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}

/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    y = 3*u!
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    InputRealPtrsType uPtrs  = ssGetInputPortRealSignalPtrs(S,0);
//     double           *uPtrs  = ssGetInputPortRealSignalPtrs(S,0);
//     const double            *u     = ssGetInputPortRealSignal(S, 0);
    double            *y     = ssGetOutputPortRealSignal(S,0);
//     int_T             uWidth = ssGetInputPortWidth(S,0);
    int             yWidth = ssGetOutputPortWidth(S,0);

    const mxArray   *prhs[paramsNum];
    mxArray         *plhs[120+30+12];
    
    prhs[0] = ssGetSFcnParam(S,0); //sys
    prhs[1] = ssGetSFcnParam(S,1); //params

	/* problem setup */
    int i, j, m, n, nz, T, niters, nd, k;
    double kappa;
    double *dptr, *dptr1, *dptr2;
    const double *Cdptr;
    double *A, *B, *At, *Bt, *Q, *R, *Qf, *xmax, *xmin, *umax, *umin, *x;
    double *zmax, *zmin, *zmaxp, *zminp, *X, *U, *z, *eyen, *eyem, *x0;
    double *X0, *U0;
    int agent_mode = 0;
//     double *A_temp, *B_temp;
    double *telapsed;
    clock_t t1, t2;
//   
//     A = mxGetPr(mxGetField(prhs[0],0,"A"));
//     B = mxGetPr(mxGetField(prhs[0],0,"B"));
    Q = mxGetPr(mxGetField(prhs[0],0,"Q"));
    R = mxGetPr(mxGetField(prhs[0],0,"R"));
    Qf = mxGetPr(mxGetField(prhs[1],0,"Qf"));
    xmax = mxGetPr(mxGetField(prhs[0],0,"xmax"));
    xmin = mxGetPr(mxGetField(prhs[0],0,"xmin"));
    umax = mxGetPr(mxGetField(prhs[0],0,"umax"));
    umin = mxGetPr(mxGetField(prhs[0],0,"umin"));
    n = (int)mxGetScalar(mxGetField(prhs[0],0,"n"));
//     nd = (int)mxGetScalar(mxGetField(prhs[0],0,"nd"));
    m = (int)mxGetScalar(mxGetField(prhs[0],0,"m"));
    T = (int)mxGetScalar(mxGetField(prhs[1],0,"T"));
    kappa = (double)mxGetScalar(mxGetField(prhs[1],0,"kappa"));
    niters = (int)mxGetScalar(mxGetField(prhs[1],0,"niters"));
    quiet = (int)mxGetScalar(mxGetField(prhs[1],0,"quiet"));
//     n=n+nd;
    nz = T*(n+m);
//     
    X0 = malloc(sizeof(double)*n*T);
    U0 = malloc(sizeof(double)*m*T);
    x0 = malloc(sizeof(double)*n*1);
    /* REMEMBER TO FREE THE MEMOMRY!!! */
    A = malloc(sizeof(double)*n*n);
    B = malloc(sizeof(double)*n*(m));
    xmax = malloc(sizeof(double)*n);
    xmin = malloc(sizeof(double)*n);
// 
	/* Reading from the input */
    Cdptr = uPtrs[0];
    for (i = 0; i < T; i++) {   //col-major X0_temp
        for (j = 0; j < n; j++) {
            X0[i*n+j] = *Cdptr++;
//             *y++ = X0_temp[i*n+j];
//             Cdptr++;
        }
    }
    for (i = 0; i < T; i++) {   //col-major U0_temp
        for (j = 0; j < m; j++) {
            U0[i*m+j] = *Cdptr++;
//             *y++ = U0_temp[i*m+j];
//             printf("%f\n", U0_temp[i*m+j]);
//             Cdptr++;
        }
        for (j = 0; j < n-m; j++) { //forward the zero-padded part of the coloumn
//             *y++ = *Cdptr;
//             printf("%f\n", *Cdptr);
//             if ( ~isnan(*Cdptr) ) printf("ERROR in input reading!!!\n");
//             Cdptr++; //it must be NAN
            Cdptr++;
        }
    }
    for (j = 0; j < n; j++) {   //col-major x0
        x0[j] = *Cdptr++;
//         *y++ = x0[i*m+j];
//         printf("%f\n", x0[i*m+j]);
//         Cdptr++;
    }
	/* Reading from exact linearizzation
    REMEMBER TO FREE THE MEMOMRY!!! */
    for (i = 0; i < (n)*(n); i++) {   //col-major A
        A[i] = *Cdptr++;
    }
// 
    for (i = 0; i < (n)*m; i++) {   //col-major B
        B[i] = *Cdptr++;
    }
//     
    agent_mode = *Cdptr++;
//     printf("agent_mode %i \n", agent_mode);
    for (i = 0; i < n; i++) {
        xmin[i] = *Cdptr++;
//         printf("xmin %i is %f\n", i, xmin[i]);
    }
    for (i = 0; i < n; i++) {
        xmax[i] = *Cdptr++;
//         printf("xmax %i is %f\n", i, xmax[i]);
    }
//     printf("---\n"); 
//  
    /* outputs */
    X = malloc(sizeof(double)*n*T);
    U = malloc(sizeof(double)*m*T);
    telapsed = malloc(sizeof(double)*1);

    At = malloc(sizeof(double)*n*n);
    Bt = malloc(sizeof(double)*n*m);
    eyen = malloc(sizeof(double)*n*n);
    eyem = malloc(sizeof(double)*m*m);
    z = malloc(sizeof(double)*nz);
    x = malloc(sizeof(double)*n);
    zmax = malloc(sizeof(double)*nz);
    zmin = malloc(sizeof(double)*nz);
    zmaxp = malloc(sizeof(double)*nz);
    zminp = malloc(sizeof(double)*nz);
//     
    /* eyen, eyem */
    dptr = eyen;
    for (i = 0; i < n*n; i++)
    {
        *dptr = 0;
        dptr++;
    }
    dptr = dptr-n*n;
    for (i = 0; i < n; i++)
    {
        *dptr = 1;
        dptr = dptr+n+1;
    }
// 
    dptr = eyem;
    for (i = 0; i < m*m; i++)
    {
        *dptr = 0;
        dptr++;
    }
    dptr = dptr-m*m;
    for (i = 0; i < m; i++)
    {
        *(dptr+i*m+i) = 1;
    }
    dptr = x; dptr1 = x0;
    for (i = 0; i < n; i++)
    {
        *dptr = *dptr1;
        dptr++; dptr1++;
    }
    dptr = z;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *dptr = *(U0+i*m+j);
            dptr++;
        }
        for (j = 0; j < n; j++)
        {
            *dptr = *(X0+i*n+j);
            dptr++; 
        }
    }  
    /* At, Bt */
    F77_CALL(dgemm)("t","n",&n,&n,&n,&fone,A,&n,eyen,&n,&fzero,At,&n);
    F77_CALL(dgemm)("n","t",&m,&n,&m,&fone,eyem,&m,B,&n,&fzero,Bt,&m);
// 
    /* zmax, zmin */
    dptr1 = zmax;
    dptr2 = zmin;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *dptr1 = *(umax+j);
            *dptr2 = *(umin+j);
            dptr1++; dptr2++;
        }
        for (j = 0; j < n; j++)
        {
            *dptr1 = *(xmax+j);
            *dptr2 = *(xmin+j);
            dptr1++; dptr2++;
        }
    }  
// 
    /* zmaxp, zminp */
    for (i = 0; i < nz; i++) zminp[i] = zmin[i] + 0.01*(zmax[i]-zmin[i]);
    for (i = 0; i < nz; i++) zmaxp[i] = zmax[i] - 0.01*(zmax[i]-zmin[i]);
// 
    /* project z */
    for (i = 0; i < nz; i++) z[i] = z[i] > zmaxp[i] ? zmaxp[i] : z[i];
    for (i = 0; i < nz; i++) z[i] = z[i] < zminp[i] ? zminp[i] : z[i];
//     
//     printf("A\n");
//     printmat(A, n, n);
//     printf("B\n");
//     printmat(B, n, m);
//     printf("At\n");
//     printmat(At, n, n);
//     printf("Bt\n");
//     printmat(Bt, m, n);
//     printf("n = %i | nd = %i | m = %i | T = %i | niters = %i | kappa = %f\n", n, nd, m, T, niters, kappa);
//     
//     printf("eyen\n");
//     printmat(eyen, n, n);
//     printf("eyem\n");
//     printmat(eyem, m, m);
//     printf("Q\n");
//     printmat(Q, n, n);
//     printf("R\n");
//     printmat(R, m, m);
//     printf("Qf\n");
//     printmat(Qf, n, n);
// //     
//     printf("zmax\n");
//     printmat(zmax, n+m, T);
//     printf("zmin\n");
//     printmat(zmin, n+m, T);
// // 
//     printf("x\n");
//     printmat(x, n, 1);
//     printf("z\n");
//     printmat(z, n+m, 1);
// 
    t1 = clock();
    fmpcsolve(A,B,At,Bt,eyen,eyem,Q,R,Qf,zmax,zmin,x,z,T,n,m,nz,niters,kappa);
    t2 = clock();
    *telapsed = (double)(t2-t1)/(CLOCKS_PER_SEC);
//     
    dptr = z;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *(U+i*m+j) = *dptr;
            *y++ = *dptr;//output
//             *y++ = 0;//output
            dptr++;
        }
        for (j = 0; j < n; j++)
        {
            *(X+i*n+j) = *dptr;
            *y++ = *dptr;//output
//             *y++ = 0;//output
            dptr++;
        }
    }
    *y++ = *telapsed;//output
// 
//     for (i = 0; i < yWidth; i++) {
//             *y++ = i+1;
//     }
//     
    free(At); free(Bt); free(eyen); free(eyem);
    free(z); free(x); free(zmax); free(zmin);
    free(A); free(B);
    free(xmin); free(xmax);
    return;
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{
}



#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
