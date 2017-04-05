void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    /* problem setup */
    int i, j, m, n, nz, T, niters;
    double kappa;
    double *dptr, *dptr1, *dptr2;
    double *A, *B, *At, *Bt, *Q, *R, *Qf, *xmax, *xmin, *umax, *umin, *x;
    double *zmax, *zmin, *zmaxp, *zminp, *X, *U, *z, *eyen, *eyem, *x0;
    double *X0, *U0;
    double *telapsed;
    clock_t t1, t2;

    A = mxGetPr(mxGetField(prhs[0],0,"A"));
    B = mxGetPr(mxGetField(prhs[0],0,"B"));
    Q = mxGetPr(mxGetField(prhs[0],0,"Q"));
    R = mxGetPr(mxGetField(prhs[0],0,"R"));
    Qf = mxGetPr(mxGetField(prhs[1],0,"Qf"));
    xmax = mxGetPr(mxGetField(prhs[0],0,"xmax"));
    xmin = mxGetPr(mxGetField(prhs[0],0,"xmin"));
    umax = mxGetPr(mxGetField(prhs[0],0,"umax"));
    umin = mxGetPr(mxGetField(prhs[0],0,"umin"));
    n = (int)mxGetScalar(mxGetField(prhs[0],0,"n"));
    m = (int)mxGetScalar(mxGetField(prhs[0],0,"m"));
    T = (int)mxGetScalar(mxGetField(prhs[1],0,"T"));
    kappa = (double)mxGetScalar(mxGetField(prhs[1],0,"kappa"));
    niters = (int)mxGetScalar(mxGetField(prhs[1],0,"niters"));
    quiet = (int)mxGetScalar(mxGetField(prhs[1],0,"quiet"));

    X0 = mxGetPr(prhs[2]);
    U0 = mxGetPr(prhs[3]);
    x0 = mxGetPr(prhs[4]);

    nz = T*(n+m);

    /* outputs */
    plhs[0] = mxCreateDoubleMatrix(n,T,mxREAL);
    plhs[1] = mxCreateDoubleMatrix(m,T,mxREAL);
    plhs[2] = mxCreateDoubleMatrix(1,1,mxREAL);
    X = mxGetPr(plhs[0]);
    U = mxGetPr(plhs[1]);
    telapsed = mxGetPr(plhs[2]);

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

    /* zmaxp, zminp */
    for (i = 0; i < nz; i++) zminp[i] = zmin[i] + 0.01*(zmax[i]-zmin[i]);
    for (i = 0; i < nz; i++) zmaxp[i] = zmax[i] - 0.01*(zmax[i]-zmin[i]);

    /* project z */
    for (i = 0; i < nz; i++) z[i] = z[i] > zmaxp[i] ? zmaxp[i] : z[i];
    for (i = 0; i < nz; i++) z[i] = z[i] < zminp[i] ? zminp[i] : z[i];

    t1 = clock();
    fmpcsolve(A,B,At,Bt,eyen,eyem,Q,R,Qf,zmax,zmin,x,z,T,n,m,nz,niters,kappa);
    t2 = clock();
    *telapsed = (double)(t2-t1)/(CLOCKS_PER_SEC);

    dptr = z;
    for (i = 0; i < T; i++)
    {
        for (j = 0; j < m; j++)
        {
            *(U+i*m+j) = *dptr;
            dptr++;
        }
        for (j = 0; j < n; j++)
        {
            *(X+i*n+j) = *dptr;
            dptr++; 
        }
    }  

    free(At); free(Bt); free(eyen); free(eyem);
    free(z); free(x); free(zmax); free(zmin);
    return;
}
