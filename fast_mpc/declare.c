/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: declare.c
 *
 * MATLAB Coder version            : 3.2
 * C/C++ source code generated on  : 05-Apr-2017 10:22:37
 */

/* Include Files */
#include "rt_nonfinite.h"
#include "declare.h"

/* Function Definitions */

/*
 * rand('state',0); % make
 *  k = 1;          % spring constant
 *  lam = 0;        % damping constant
 *  a = -2*k;
 *  b = -2*lam;
 *  c = k;
 *  d = lam;
 * Arguments    : double unused
 *                double A[144]
 *                double B[36]
 *                double Q[144]
 *                double R[9]
 *                double Qf[144]
 *                double xmax[12]
 *                double xmin[12]
 *                double umax[3]
 *                double umin[3]
 *                double *n
 *                double *m
 *                double *T
 *                double *kappa
 *                double *niters
 *                boolean_T *quiet
 *                double X0[120]
 *                double U0[30]
 *                double x0[12]
 * Return Type  : void
 */
void declare(double unused, double A[144], double B[36], double Q[144], double
             R[9], double Qf[144], double xmax[12], double xmin[12], double
             umax[3], double umin[3], double *n, double *m, double *T, double
             *kappa, double *niters, boolean_T *quiet, double X0[120], double
             U0[30], double x0[12])
{
  static const double dv0[144] = { 0.7627, 0.1149, 0.0025, 0.0, 0.0, 0.0,
    -0.8994, 0.4202, 0.0193, 0.0002, 0.0, 0.0, 0.1149, 0.7652, 0.1149, 0.0025,
    0.0, 0.0, 0.4202, -0.8801, 0.4205, 0.0193, 0.0002, 0.0, 0.0025, 0.1149,
    0.7652, 0.1149, 0.0025, 0.0, 0.0193, 0.4205, -0.8801, 0.4205, 0.0193, 0.0002,
    0.0, 0.0025, 0.1149, 0.7652, 0.1149, 0.0025, 0.0002, 0.0193, 0.4205, -0.8801,
    0.4205, 0.0193, 0.0, 0.0, 0.0025, 0.1149, 0.7652, 0.1149, 0.0, 0.0002,
    0.0193, 0.4205, -0.8801, 0.4202, 0.0, 0.0, 0.0, 0.0025, 0.1149, 0.7627, 0.0,
    0.0, 0.0002, 0.0193, 0.4202, -0.8994, 0.4596, 0.0198, 0.0003, 0.0, 0.0, 0.0,
    0.7627, 0.1149, 0.0025, 0.0, 0.0, 0.0, 0.0198, 0.4599, 0.0198, 0.0003, 0.0,
    0.0, 0.1149, 0.7652, 0.1149, 0.0025, 0.0, 0.0, 0.0003, 0.0198, 0.4599,
    0.0198, 0.0003, 0.0, 0.0025, 0.1149, 0.7652, 0.1149, 0.0025, 0.0, 0.0,
    0.0003, 0.0198, 0.4599, 0.0198, 0.0003, 0.0, 0.0025, 0.1149, 0.7652, 0.1149,
    0.0025, 0.0, 0.0, 0.0003, 0.0198, 0.4599, 0.0198, 0.0, 0.0, 0.0025, 0.1149,
    0.7652, 0.1149, 0.0, 0.0, 0.0, 0.0003, 0.0198, 0.4596, 0.0, 0.0, 0.0, 0.0025,
    0.1149, 0.7627 };

  static const double dv1[36] = { 0.1174, -0.1174, -0.0025, -0.0, -0.0, -0.0,
    0.4398, -0.4401, -0.0196, -0.0002, -0.0, -0.0, 0.0, 0.0025, 0.1199, 0.0,
    -0.1199, -0.0025, 0.0003, 0.0198, 0.4596, 0.0, -0.4596, -0.0198, 0.0, 0.0,
    0.0025, 0.1199, 0.0, -0.1199, 0.0, 0.0003, 0.0198, 0.4596, 0.0, -0.4594 };

  int i;
  (void)unused;
  memcpy(&A[0], &dv0[0], 144U * sizeof(double));
  memcpy(&B[0], &dv1[0], 36U * sizeof(double));
  *n = 12.0;

  /*  state dimension */
  *m = 3.0;

  /*  input dimension */
  /*  Acts = [zeros(n/2) eye(n/2); */
  /*      [a,c,0,0,0,0,b,d,0,0,0,0; */
  /*       c,a,c,0,0,0,d,b,d,0,0,0; */
  /*       0,c,a,c,0,0,0,d,b,d,0,0; */
  /*       0,0,c,a,c,0,0,0,d,b,d,0; */
  /*       0,0,0,c,a,c,0,0,0,d,b,d; */
  /*       0,0,0,0,c,a,0,0,0,0,d,b]]; */
  /*   */
  /*  Bcts = [zeros(n/2,m); */
  /*      [1, 0, 0; */
  /*      -1, 0, 0; */
  /*       0, 1, 0; */
  /*       0, 0, 1; */
  /*       0,-1, 0; */
  /*       0, 0,-1]]; */
  /*  % convert to discrete-time system */
  /*  ts = 0.5;       % sampling time */
  /*  A = expm(ts*Acts); */
  /*  B = (Acts\(expm(ts*Acts)-eye(n)))*Bcts; */
  /*  objective matrices */
  memset(&Q[0], 0, 144U * sizeof(double));
  for (i = 0; i < 12; i++) {
    Q[i + 12 * i] = 1.0;
  }

  memset(&R[0], 0, 9U * sizeof(double));
  for (i = 0; i < 3; i++) {
    R[i + 3 * i] = 1.0;
  }

  /*  state and control limits */
  for (i = 0; i < 12; i++) {
    xmin[i] = -4.0;
    xmax[i] = 4.0;
  }

  for (i = 0; i < 3; i++) {
    umin[i] = -0.5;
    umax[i] = 0.5;
  }

  /*  process noise trajectory */
  /*  nsteps = 100; */
  /*  w = 2*rand(n,nsteps)-1; w(1:n/2,:) = 0; w = 0.5*w; */
  /*  W = (1/12)*diag([0;0;0;0;0;0;1;1;1;1;1;1]); */
  /*  initial state */
  memset(&x0[0], 0, 12U * sizeof(double));

  /*  % system description */
  /*  sys.A = A ; */
  /*  sys.B = B; */
  /*  sys.xmax = xmax; */
  /*  sys.xmin = xmin; */
  /*  sys.umax = umax; */
  /*  sys.umin = umin; */
  /*  sys.n = n; */
  /*  sys.m = m; */
  /*  sys.Q = Q; */
  /*  sys.R = R; */
  /*  fast MPC parameters */
  *T = 10.0;

  /*  time horizon */
  memcpy(&Qf[0], &Q[0], 144U * sizeof(double));

  /*  final state cost */
  *kappa = 0.01;

  /*  barrier parameter */
  *niters = 5.0;

  /*  number of newton steps */
  *quiet = false;

  /*  % allocate history matrices */
  /*  Xhist = zeros(n,nsteps);  % state */
  /*  Uhist = zeros(m,nsteps);  % input */
  /*  Jhist = zeros(1,nsteps);  % stage cost */
  /*  thist = zeros(1,nsteps);  % fmpc run time */
  /*  set up initial state and input trajectories */
  memset(&X0[0], 0, 120U * sizeof(double));
  memset(&U0[0], 0, 30U * sizeof(double));

  /*  x = x0; */
}

/*
 * File trailer for declare.c
 *
 * [EOF]
 */
