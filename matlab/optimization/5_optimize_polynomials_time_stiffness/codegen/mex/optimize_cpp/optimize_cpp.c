/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp.c
 *
 * Code generation for function 'optimize_cpp'
 *
 */

/* Include files */
#include "optimize_cpp.h"
#include "fmincon.h"
#include "linspace.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "tic.h"
#include "toc.h"
#include "mwmathutil.h"

/* Variable Definitions */
static emlrtRTEInfo emlrtRTEI = { 111, /* lineNo */
  5,                                   /* colNo */
  "constraints",                       /* fName */
  "/home/mfocchi/Dropbox/RESEARCH/climbingrobotnotes/matlab/optimize_polynomials_time_stiffness/constraints.m"/* pName */
};

/* Function Definitions */
void anon(const real_T p0[3], const real_T pf[3], real_T Fun_max, real_T Fr_max,
          real_T mu, const emxArray_real_T *x, emxArray_real_T *varargout_1)
{
  emxArray_boolean_T *b_x;
  emxArray_real_T *E;
  emxArray_real_T *b_time;
  emxArray_real_T *l;
  emxArray_real_T *p;
  emxArray_real_T *phi;
  emxArray_real_T *phid;
  emxArray_real_T *r;
  emxArray_real_T *sigma;
  emxArray_real_T *theta;
  emxArray_real_T *thetad;
  const mxArray *b_m;
  const mxArray *b_y;
  real_T K;
  real_T a;
  real_T absxk;
  real_T b_a;
  real_T b_scale;
  real_T c_scale;
  real_T scale;
  real_T t;
  real_T *pData;
  int32_T iv[2];
  int32_T b_i;
  int32_T i;
  int32_T k;
  int32_T nx;
  boolean_T exitg1;
  boolean_T y;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&b_time, 2, true);
  emxInit_real_T(&E, 2, true);

  /*  ineq are <= 0 */
  linspace(x->data[0], N, b_time);
  K = x->data[13];

  /*   */
  /*  check they are column vectors */
  /*  parametrizzation with sin theta sing phi */
  k = E->size[0] * E->size[1];
  E->size[0] = 1;
  E->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(E, k);
  nx = b_time->size[1];
  for (k = 0; k < nx; k++) {
    scale = b_time->data[k];
    E->data[k] = scale * scale;
  }

  emxInit_real_T(&l, 2, true);
  k = l->size[0] * l->size[1];
  l->size[0] = 1;
  l->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(l, k);
  nx = b_time->size[1];
  for (k = 0; k < nx; k++) {
    l->data[k] = muDoubleScalarPower(b_time->data[k], 3.0);
  }

  emxInit_real_T(&theta, 2, true);
  k = theta->size[0] * theta->size[1];
  theta->size[0] = 1;
  theta->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(theta, k);
  b_scale = x->data[1];
  scale = x->data[2];
  absxk = x->data[3];
  t = x->data[4];
  nx = b_time->size[0] * b_time->size[1];
  for (k = 0; k < nx; k++) {
    theta->data[k] = ((b_scale + scale * b_time->data[k]) + absxk * E->data[k])
      + t * l->data[k];
  }

  emxInit_real_T(&thetad, 2, true);
  a = 2.0 * x->data[3];
  b_a = 3.0 * x->data[4];
  k = thetad->size[0] * thetad->size[1];
  thetad->size[0] = 1;
  thetad->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(thetad, k);
  b_scale = x->data[2];
  nx = b_time->size[0] * b_time->size[1];
  for (k = 0; k < nx; k++) {
    thetad->data[k] = (b_scale + a * b_time->data[k]) + b_a * E->data[k];
  }

  emxInit_real_T(&phi, 2, true);
  k = phi->size[0] * phi->size[1];
  phi->size[0] = 1;
  phi->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(phi, k);
  b_scale = x->data[5];
  scale = x->data[6];
  absxk = x->data[7];
  t = x->data[8];
  nx = b_time->size[0] * b_time->size[1];
  for (k = 0; k < nx; k++) {
    phi->data[k] = ((b_scale + scale * b_time->data[k]) + absxk * E->data[k]) +
      t * l->data[k];
  }

  emxInit_real_T(&phid, 2, true);
  a = 2.0 * x->data[7];
  b_a = 3.0 * x->data[8];
  k = phid->size[0] * phid->size[1];
  phid->size[0] = 1;
  phid->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(phid, k);
  b_scale = x->data[6];
  nx = b_time->size[0] * b_time->size[1];
  for (k = 0; k < nx; k++) {
    phid->data[k] = (b_scale + a * b_time->data[k]) + b_a * E->data[k];
  }

  k = l->size[0] * l->size[1];
  l->size[0] = 1;
  l->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(l, k);
  b_scale = x->data[9];
  scale = x->data[10];
  absxk = x->data[11];
  t = x->data[12];
  nx = b_time->size[0] * b_time->size[1] - 1;
  for (k = 0; k <= nx; k++) {
    l->data[k] = ((b_scale + scale * b_time->data[k]) + absxk * E->data[k]) + t *
      l->data[k];
  }

  a = 2.0 * x->data[11];
  b_a = 3.0 * x->data[12];
  k = b_time->size[0] * b_time->size[1];
  i = b_time->size[0] * b_time->size[1];
  b_time->size[0] = 1;
  emxEnsureCapacity_real_T(b_time, i);
  b_scale = x->data[10];
  nx = k - 1;
  for (k = 0; k <= nx; k++) {
    b_time->data[k] = (b_scale + a * b_time->data[k]) + b_a * E->data[k];
  }

  k = E->size[0] * E->size[1];
  E->size[0] = 1;
  E->size[1] = theta->size[1];
  emxEnsureCapacity_real_T(E, k);
  nx = theta->size[0] * theta->size[1];
  for (k = 0; k < nx; k++) {
    E->data[k] = theta->data[k];
  }

  nx = theta->size[1];
  for (k = 0; k < nx; k++) {
    E->data[k] = muDoubleScalarSin(E->data[k]);
  }

  emxInit_real_T(&sigma, 2, true);
  k = sigma->size[0] * sigma->size[1];
  sigma->size[0] = 1;
  sigma->size[1] = phi->size[1];
  emxEnsureCapacity_real_T(sigma, k);
  nx = phi->size[0] * phi->size[1];
  for (k = 0; k < nx; k++) {
    sigma->data[k] = phi->data[k];
  }

  nx = phi->size[1];
  for (k = 0; k < nx; k++) {
    sigma->data[k] = muDoubleScalarCos(sigma->data[k]);
  }

  nx = phi->size[1];
  for (k = 0; k < nx; k++) {
    phi->data[k] = muDoubleScalarSin(phi->data[k]);
  }

  emxInit_real_T(&r, 2, true);
  k = r->size[0] * r->size[1];
  r->size[0] = 1;
  r->size[1] = theta->size[1];
  emxEnsureCapacity_real_T(r, k);
  nx = theta->size[0] * theta->size[1];
  for (k = 0; k < nx; k++) {
    r->data[k] = theta->data[k];
  }

  nx = theta->size[1];
  for (k = 0; k < nx; k++) {
    r->data[k] = muDoubleScalarCos(r->data[k]);
  }

  emxInit_real_T(&p, 2, true);
  k = p->size[0] * p->size[1];
  p->size[0] = 3;
  p->size[1] = l->size[1];
  emxEnsureCapacity_real_T(p, k);
  nx = l->size[1];
  for (k = 0; k < nx; k++) {
    p->data[3 * k] = l->data[k] * E->data[k] * sigma->data[k];
  }

  nx = l->size[1];
  for (k = 0; k < nx; k++) {
    p->data[3 * k + 1] = l->data[k] * E->data[k] * phi->data[k];
  }

  nx = l->size[1];
  for (k = 0; k < nx; k++) {
    p->data[3 * k + 2] = -l->data[k] * r->data[k];
  }

  emxFree_real_T(&r);

  /*  number of constraints */
  /*  size not known */
  k = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[0] = 1;
  nx = (int32_T)(((((N - 1.0) + N) + 2.0 * N) + 3.0) + 3.0);
  varargout_1->size[1] = nx;
  emxEnsureCapacity_real_T(varargout_1, k);
  for (k = 0; k < nx; k++) {
    varargout_1->data[k] = 0.0;
  }

  k = E->size[0] * E->size[1];
  E->size[0] = 1;
  E->size[1] = (int32_T)N;
  emxEnsureCapacity_real_T(E, k);
  nx = (int32_T)N;
  for (k = 0; k < nx; k++) {
    E->data[k] = 0.0;
  }

  k = sigma->size[0] * sigma->size[1];
  sigma->size[0] = 1;
  sigma->size[1] = (int32_T)N;
  emxEnsureCapacity_real_T(sigma, k);
  nx = (int32_T)N;
  for (k = 0; k < nx; k++) {
    sigma->data[k] = 0.0;
  }

  k = (int32_T)N;
  for (b_i = 0; b_i < k; b_i++) {
    /*  these are 8 constraints  */
    scale = theta->data[b_i];
    a = muDoubleScalarSin(scale);
    absxk = l->data[b_i];
    b_a = absxk - l_uncompressed;
    t = thetad->data[b_i];
    b_scale = phid->data[b_i];
    c_scale = b_time->data[b_i];
    scale = ((m * (absxk * absxk) / 2.0 * (t * t + a * a * (b_scale * b_scale))
              + m * (c_scale * c_scale) / 2.0) - m * g * absxk *
             muDoubleScalarCos(scale)) + K * (b_a * b_a) / 2.0;
    E->data[b_i] = scale;
    sigma->data[b_i] = x->data[(int32_T)(num_params + ((real_T)b_i + 1.0)) - 1];
    if (b_i + 1U >= 2U) {
      i = phi->size[0] * phi->size[1];
      phi->size[0] = 1;
      nx = varargout_1->size[1];
      phi->size[1] = varargout_1->size[1] + 1;
      emxEnsureCapacity_real_T(phi, i);
      for (i = 0; i < nx; i++) {
        phi->data[i] = varargout_1->data[i];
      }

      phi->data[varargout_1->size[1]] = muDoubleScalarAbs(scale - E->data[b_i -
        1]) - sigma->data[b_i];
      i = varargout_1->size[0] * varargout_1->size[1];
      varargout_1->size[0] = 1;
      varargout_1->size[1] = phi->size[1];
      emxEnsureCapacity_real_T(varargout_1, i);
      nx = phi->size[0] * phi->size[1];
      for (i = 0; i < nx; i++) {
        varargout_1->data[i] = phi->data[i];
      }
    }

    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }

  emxFree_real_T(&sigma);
  emxFree_real_T(&phid);
  emxFree_real_T(&thetad);
  emxFree_real_T(&theta);
  emxFree_real_T(&b_time);

  /*  constraint to do not enter the wall, p_x >=0 */
  k = (int32_T)N;
  for (b_i = 0; b_i < k; b_i++) {
    i = varargout_1->size[1];
    nx = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, nx);
    varargout_1->data[i] = -p->data[3 * b_i];
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }

  /*  constraints on retraction force   -Fr_max < Fr = -K*(l-luncompr) < 0  */
  k = E->size[0] * E->size[1];
  E->size[0] = 1;
  E->size[1] = (int32_T)N;
  emxEnsureCapacity_real_T(E, k);
  nx = (int32_T)N;
  for (k = 0; k < nx; k++) {
    E->data[k] = 0.0;
  }

  k = (int32_T)N;
  for (b_i = 0; b_i < k; b_i++) {
    scale = -K * (l->data[b_i] - l_uncompressed);
    E->data[b_i] = scale;
    i = varargout_1->size[1];
    nx = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, nx);
    varargout_1->data[i] = scale;

    /*  Fr <0 */
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }

  k = (int32_T)N;
  for (b_i = 0; b_i < k; b_i++) {
    i = varargout_1->size[1];
    nx = varargout_1->size[0] * varargout_1->size[1];
    varargout_1->size[1]++;
    emxEnsureCapacity_real_T(varargout_1, nx);
    varargout_1->data[i] = -Fr_max - E->data[b_i];

    /*  -Fr_max -Fr <0 */
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }

  emxFree_real_T(&E);

  /* eval trajectory */
  /*  parametrizzation with sin theta sing phi */
  /* evaluate inpulse ( the integral of the gaussian is 1)  */
  scale = m * x->data[9];
  absxk = scale * x->data[2] / T_th;
  scale = scale * muDoubleScalarSin(x->data[1]) * x->data[6] / T_th;
  k = phi->size[0] * phi->size[1];
  phi->size[0] = 1;
  phi->size[1] = varargout_1->size[1] + 1;
  emxEnsureCapacity_real_T(phi, k);
  nx = varargout_1->size[1];
  for (k = 0; k < nx; k++) {
    phi->data[k] = varargout_1->data[k];
  }

  phi->data[varargout_1->size[1]] = muDoubleScalarSqrt(absxk * absxk + scale *
    scale) - Fun_max;
  k = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[0] = 1;
  varargout_1->size[1] = phi->size[1];
  emxEnsureCapacity_real_T(varargout_1, k);
  nx = phi->size[0] * phi->size[1];
  for (k = 0; k < nx; k++) {
    varargout_1->data[k] = phi->data[k];
  }

  k = varargout_1->size[1];
  i = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i);
  varargout_1->data[k] = -absxk;
  k = phi->size[0] * phi->size[1];
  phi->size[0] = 1;
  phi->size[1] = varargout_1->size[1] + 1;
  emxEnsureCapacity_real_T(phi, k);
  nx = varargout_1->size[1];
  for (k = 0; k < nx; k++) {
    phi->data[k] = varargout_1->data[k];
  }

  phi->data[varargout_1->size[1]] = muDoubleScalarAbs(scale) - mu * absxk;
  k = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[0] = 1;
  varargout_1->size[1] = phi->size[1];
  emxEnsureCapacity_real_T(varargout_1, k);
  nx = phi->size[0] * phi->size[1];
  for (k = 0; k < nx; k++) {
    varargout_1->data[k] = phi->data[k];
  }

  /*  initial final point */
  scale = 3.3121686421112381E-170;
  k = varargout_1->size[1];
  i = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i);
  b_scale = 3.3121686421112381E-170;
  c_scale = 3.3121686421112381E-170;
  absxk = muDoubleScalarAbs(p->data[0] - p0[0]);
  if (absxk > 3.3121686421112381E-170) {
    a = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    a = t * t;
  }

  absxk = muDoubleScalarAbs(p->data[3 * (p->size[1] - 1)] - pf[0]);
  if (absxk > 3.3121686421112381E-170) {
    b_a = 1.0;
    b_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    b_a = t * t;
  }

  absxk = muDoubleScalarAbs(pf[0]);
  if (absxk > 3.3121686421112381E-170) {
    K = 1.0;
    c_scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    K = t * t;
  }

  absxk = muDoubleScalarAbs(p->data[1] - p0[1]);
  if (absxk > scale) {
    t = scale / absxk;
    a = a * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    a += t * t;
  }

  absxk = muDoubleScalarAbs(p->data[3 * (p->size[1] - 1) + 1] - pf[1]);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_a = b_a * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_a += t * t;
  }

  absxk = muDoubleScalarAbs(pf[1]);
  if (absxk > c_scale) {
    t = c_scale / absxk;
    K = K * t * t + 1.0;
    c_scale = absxk;
  } else {
    t = absxk / c_scale;
    K += t * t;
  }

  absxk = muDoubleScalarAbs(p->data[2] - p0[2]);
  if (absxk > scale) {
    t = scale / absxk;
    a = a * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    a += t * t;
  }

  absxk = muDoubleScalarAbs(p->data[3 * (p->size[1] - 1) + 2] - pf[2]);
  emxFree_real_T(&p);
  if (absxk > b_scale) {
    t = b_scale / absxk;
    b_a = b_a * t * t + 1.0;
    b_scale = absxk;
  } else {
    t = absxk / b_scale;
    b_a += t * t;
  }

  absxk = muDoubleScalarAbs(pf[2]);
  if (absxk > c_scale) {
    t = c_scale / absxk;
    K = K * t * t + 1.0;
    c_scale = absxk;
  } else {
    t = absxk / c_scale;
    K += t * t;
  }

  a = scale * muDoubleScalarSqrt(a);
  scale = num_params + N;
  varargout_1->data[k] = a - x->data[(int32_T)(scale + 1.0) - 1];
  b_a = b_scale * muDoubleScalarSqrt(b_a);
  k = varargout_1->size[1];
  i = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[1]++;
  emxEnsureCapacity_real_T(varargout_1, i);
  varargout_1->data[k] = b_a - x->data[(int32_T)(scale + 2.0) - 1];
  K = c_scale * muDoubleScalarSqrt(K);
  k = phi->size[0] * phi->size[1];
  phi->size[0] = 1;
  phi->size[1] = varargout_1->size[1] + 1;
  emxEnsureCapacity_real_T(phi, k);
  nx = varargout_1->size[1];
  for (k = 0; k < nx; k++) {
    phi->data[k] = varargout_1->data[k];
  }

  phi->data[varargout_1->size[1]] = muDoubleScalarAbs(K - l->data[l->size[1] - 1])
    - x->data[(int32_T)(scale + 3.0) - 1];
  k = varargout_1->size[0] * varargout_1->size[1];
  varargout_1->size[0] = 1;
  varargout_1->size[1] = phi->size[1];
  emxEnsureCapacity_real_T(varargout_1, k);
  nx = phi->size[0] * phi->size[1];
  emxFree_real_T(&l);
  for (k = 0; k < nx; k++) {
    varargout_1->data[k] = phi->data[k];
  }

  emxFree_real_T(&phi);
  emxInit_boolean_T(&b_x, 2, true);

  /*  not ok this is too restrictive */
  /*  eq= [eq norm(p_0 - p0)  ]; */
  /*  eq= [eq norm(p_f - pf) ]; */
  /*  eq= [eq abs(norm(pf) - l_f)  ]; */
  k = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = varargout_1->size[1];
  emxEnsureCapacity_boolean_T(b_x, k);
  nx = varargout_1->size[0] * varargout_1->size[1];
  for (k = 0; k < nx; k++) {
    b_x->data[k] = muDoubleScalarIsInf(varargout_1->data[k]);
  }

  y = false;
  nx = 1;
  exitg1 = false;
  while ((!exitg1) && (nx <= b_x->size[1])) {
    if (!b_x->data[nx - 1]) {
      nx++;
    } else {
      y = true;
      exitg1 = true;
    }
  }

  emxFree_boolean_T(&b_x);
  if (y) {
    b_y = NULL;
    iv[0] = varargout_1->size[0];
    iv[1] = varargout_1->size[1];
    b_m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
    pData = emlrtMxGetPr(b_m);
    k = 0;
    for (b_i = 0; b_i < varargout_1->size[1]; b_i++) {
      pData[k] = varargout_1->data[b_i];
      k++;
    }

    emlrtAssign(&b_y, b_m);
    emlrtDisplayR2012b(b_y, "ineq", &emlrtRTEI, emlrtRootTLSGlobal);
  }

  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

real_T b_anon(const emxArray_real_T *x)
{
  emxArray_real_T *b_time;
  emxArray_real_T *b_tmp;
  emxArray_real_T *l;
  emxArray_real_T *phid;
  emxArray_real_T *theta;
  emxArray_real_T *thetad;
  real_T b_x;
  real_T c_x;
  real_T d_x;
  real_T e_x;
  real_T varargout_1;
  int32_T i;
  int32_T k;
  int32_T nx;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&b_time, 2, true);
  emxInit_real_T(&b_tmp, 2, true);
  linspace(x->data[0], N, b_time);

  /*  parametrizzation with sin(theta) sin(phi) */
  i = b_tmp->size[0] * b_tmp->size[1];
  b_tmp->size[0] = 1;
  b_tmp->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(b_tmp, i);
  nx = b_time->size[1];
  for (k = 0; k < nx; k++) {
    b_x = b_time->data[k];
    b_tmp->data[k] = b_x * b_x;
  }

  emxInit_real_T(&l, 2, true);
  i = l->size[0] * l->size[1];
  l->size[0] = 1;
  l->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(l, i);
  nx = b_time->size[1];
  for (k = 0; k < nx; k++) {
    l->data[k] = muDoubleScalarPower(b_time->data[k], 3.0);
  }

  emxInit_real_T(&theta, 2, true);
  i = theta->size[0] * theta->size[1];
  theta->size[0] = 1;
  theta->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(theta, i);
  c_x = x->data[1];
  b_x = x->data[2];
  d_x = x->data[3];
  e_x = x->data[4];
  nx = b_time->size[0] * b_time->size[1];
  for (i = 0; i < nx; i++) {
    theta->data[i] = ((c_x + b_x * b_time->data[i]) + d_x * b_tmp->data[i]) +
      e_x * l->data[i];
  }

  i = l->size[0] * l->size[1];
  l->size[0] = 1;
  l->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(l, i);
  c_x = x->data[9];
  b_x = x->data[10];
  d_x = x->data[11];
  e_x = x->data[12];
  nx = b_time->size[0] * b_time->size[1] - 1;
  for (i = 0; i <= nx; i++) {
    l->data[i] = ((c_x + b_x * b_time->data[i]) + d_x * b_tmp->data[i]) + e_x *
      l->data[i];
  }

  emxInit_real_T(&thetad, 2, true);
  e_x = 2.0 * x->data[3];
  b_x = 3.0 * x->data[4];
  i = thetad->size[0] * thetad->size[1];
  thetad->size[0] = 1;
  thetad->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(thetad, i);
  c_x = x->data[2];
  nx = b_time->size[0] * b_time->size[1];
  for (i = 0; i < nx; i++) {
    thetad->data[i] = (c_x + e_x * b_time->data[i]) + b_x * b_tmp->data[i];
  }

  emxInit_real_T(&phid, 2, true);
  e_x = 2.0 * x->data[7];
  b_x = 3.0 * x->data[8];
  i = phid->size[0] * phid->size[1];
  phid->size[0] = 1;
  phid->size[1] = b_time->size[1];
  emxEnsureCapacity_real_T(phid, i);
  c_x = x->data[6];
  nx = b_time->size[0] * b_time->size[1];
  for (i = 0; i < nx; i++) {
    phid->data[i] = (c_x + e_x * b_time->data[i]) + b_x * b_tmp->data[i];
  }

  e_x = 2.0 * x->data[11];
  b_x = 3.0 * x->data[12];
  i = b_time->size[0] * b_time->size[1];
  nx = b_time->size[0] * b_time->size[1];
  b_time->size[0] = 1;
  emxEnsureCapacity_real_T(b_time, nx);
  c_x = x->data[10];
  nx = i - 1;
  for (i = 0; i <= nx; i++) {
    b_time->data[i] = (c_x + e_x * b_time->data[i]) + b_x * b_tmp->data[i];
  }

  /* count row wise how many elemnts are lower than zero */
  /*  be careful there are only N values in this vector the path migh be */
  /*  underestimated! */
  /*  diff(X); */
  /*  diff(Y); */
  /*  diff(Z); */
  i = b_tmp->size[0] * b_tmp->size[1];
  b_tmp->size[0] = 1;
  nx = (int32_T)muDoubleScalarFloor(N - 1.0);
  b_tmp->size[1] = nx + 1;
  emxEnsureCapacity_real_T(b_tmp, i);
  for (i = 0; i <= nx; i++) {
    b_tmp->data[i] = x->data[(int32_T)(num_params + (real_T)(i + 1)) - 1];
  }

  if (((int32_T)muDoubleScalarFloor(N - 1.0) + 1 == 0) || ((int32_T)
       muDoubleScalarFloor(N - 1.0) + 1 == 0)) {
    d_x = 0.0;
  } else {
    d_x = x->data[(int32_T)(num_params + 1.0) - 1];
    for (k = 2; k <= nx + 1; k++) {
      d_x += b_tmp->data[k - 1];
    }
  }

  emxFree_real_T(&b_tmp);
  b_x = (num_params + N) + 1.0;
  if (b_x > x->size[1]) {
    i = -1;
    nx = -1;
  } else {
    i = (int32_T)b_x - 2;
    nx = x->size[1] - 1;
  }

  /* Ekin0cost= w5 * (    (m*l(1)^2/2).*( thetad(1)^2 + sin(theta(1))^2 *phid(1)^2 )  + (m*ld(1)^2/2)   ); */
  e_x = muDoubleScalarSin(theta->data[theta->size[1] - 1]);

  /* fprintf('cost comparison p0: %5.2f  pf: %5.2f  lf: %5.2f  slack_cost: %5.2f  Ekin0_cost: %5.2f \n' , norm(p_0 - p0), norm(p_f - pf),  abs(norm(pf) - l_f), sum(x(num_params+1:num_params+N)), (m*l(1)^2/2).*( thetad(1)^2 + sin(theta(1))^2 *phid(1)^2 )); */
  nx -= i;
  emxFree_real_T(&theta);
  if (nx == 0) {
    b_x = 0.0;
  } else {
    b_x = x->data[i + 1];
    for (k = 2; k <= nx; k++) {
      b_x += x->data[i + k];
    }
  }

  varargout_1 = ((x->data[0] + w5 * (m * (l->data[l->size[1] - 1] * l->data
    [l->size[1] - 1]) / 2.0 * (thetad->data[thetad->size[1] - 1] * thetad->
    data[thetad->size[1] - 1] + e_x * e_x * (phid->data[phid->size[1] - 1] *
    phid->data[phid->size[1] - 1])) + m * (b_time->data[b_time->size[1] - 1] *
    b_time->data[b_time->size[1] - 1]) / 2.0)) + w3 * d_x) + w4 * b_x;

  /*       coste =    sigma_final_initial  + slack_cost ; */
  emxFree_real_T(&phid);
  emxFree_real_T(&thetad);
  emxFree_real_T(&l);
  emxFree_real_T(&b_time);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
  return varargout_1;
}

void emlrt_checkEscapedGlobals(void)
{
  m_dirty |= m_dirty >> 1U;
  g_dirty |= g_dirty >> 1U;
  T_th_dirty |= T_th_dirty >> 1U;
  w1_dirty |= w1_dirty >> 1U;
  w2_dirty |= w2_dirty >> 1U;
  w3_dirty |= w3_dirty >> 1U;
  w4_dirty |= w4_dirty >> 1U;
  w5_dirty |= w5_dirty >> 1U;
  N_dirty |= N_dirty >> 1U;
  num_params_dirty |= num_params_dirty >> 1U;
  l_uncompressed_dirty |= l_uncompressed_dirty >> 1U;
}

void optimize_cpp(real_T l_0, real_T theta0, real_T phi0, const real_T pf[3],
                  real_T Fun_max, real_T Fr_max, real_T mu, real_T
                  *number_of_converged_solutions, real_T *initial_kin_energy,
                  real_T *final_kin_energy, real_T *opt_Fun, real_T *opt_Fut,
                  real_T *opt_K, real_T *opt_Tf)
{
  cell_wrap_6 this_tunableEnvironment[2];
  emxArray_real_T *T_pend;
  emxArray_real_T *c_T_pend;
  emxArray_real_T *r;
  emxArray_real_T *x;
  real_T pd[3000];
  real_T b_time[1000];
  real_T this_tunableEnvironment_f2[3];
  real_T EXITFLAG;
  real_T a;
  real_T b_T_pend;
  real_T b_a;
  real_T b_expl_temp;
  real_T b_x;
  real_T c_x;
  real_T d;
  real_T d1;
  real_T d2;
  real_T d3;
  real_T d4;
  real_T d5;
  real_T d_expl_temp;
  real_T d_x;
  real_T e_expl_temp;
  real_T e_x;
  real_T expl_temp;
  real_T f_expl_temp;
  real_T f_x;
  real_T g_expl_temp;
  real_T g_x;
  real_T h_x;
  real_T i_x;
  real_T j_x;
  real_T k_x;
  int32_T i;
  int32_T loop_ub;
  char_T c_expl_temp[3];
  boolean_T problem_solved;
  emlrtHeapReferenceStackEnterFcnR2012b(emlrtRootTLSGlobal);
  emxInit_real_T(&T_pend, 2, true);
  m = 5.0;
  m_dirty |= 1U;
  g = 9.81;
  g_dirty |= 1U;
  T_th = 0.05;
  T_th_dirty |= 1U;
  w1 = 1.0;
  w1_dirty |= 1U;

  /*  green initial cost (not used) */
  w2 = 1.0;
  w2_dirty |= 1U;

  /* red final cost (not used) */
  w3 = 1.0;
  w3_dirty |= 1U;

  /*  energy weight E */
  w4 = 10.0;
  w4_dirty |= 1U;

  /*  slacks initial / final  */
  w5 = 0.01;
  w5_dirty |= 1U;

  /* ekinf */
  N = 10.0;
  N_dirty |= 1U;

  /*  energy constraints */
  num_params = 14.0;
  num_params_dirty |= 1U;

  /*  time + poly + K  */
  b_T_pend = l_0 * muDoubleScalarSin(theta0);
  this_tunableEnvironment[0].f1[0] = b_T_pend * muDoubleScalarCos(phi0);
  this_tunableEnvironment[0].f1[1] = b_T_pend * muDoubleScalarSin(phi0);
  this_tunableEnvironment[0].f1[2] = -l_0 * muDoubleScalarCos(theta0);
  l_uncompressed = l_0;
  l_uncompressed_dirty |= 1U;

  /* pendulum period */
  b_T_pend = 6.2831853071795862 * muDoubleScalarSqrt(l_0 / g) / 4.0;

  /*  half period */
  /*  more meaninguful init */
  /* params0 = 0.1*ones(1,num_params); */
  /*  % does not always satisfy bounds */
  tic();
  this_tunableEnvironment[1].f1[0] = pf[0];
  this_tunableEnvironment_f2[0] = pf[0];
  this_tunableEnvironment[1].f1[1] = pf[1];
  this_tunableEnvironment_f2[1] = pf[1];
  this_tunableEnvironment[1].f1[2] = pf[2];
  this_tunableEnvironment_f2[2] = pf[2];
  i = T_pend->size[0] * T_pend->size[1];
  T_pend->size[0] = 1;
  T_pend->size[1] = (int32_T)N + 17;
  emxEnsureCapacity_real_T(T_pend, i);
  T_pend->data[0] = b_T_pend;
  T_pend->data[1] = theta0;
  T_pend->data[2] = 0.01;
  T_pend->data[3] = 0.0;
  T_pend->data[4] = 0.0;
  T_pend->data[5] = phi0;
  T_pend->data[6] = 0.01;
  T_pend->data[7] = 0.0;
  T_pend->data[8] = 0.0;
  T_pend->data[9] = l_0;
  T_pend->data[10] = 0.01;
  T_pend->data[11] = 0.0;
  T_pend->data[12] = 0.0;
  T_pend->data[13] = 6.0;
  loop_ub = (int32_T)N;
  for (i = 0; i < loop_ub; i++) {
    T_pend->data[i + 14] = 0.0;
  }

  emxInit_real_T(&r, 2, true);
  T_pend->data[(int32_T)N + 14] = 0.0;
  T_pend->data[(int32_T)N + 15] = 0.0;
  T_pend->data[(int32_T)N + 16] = 0.0;
  i = r->size[0] * r->size[1];
  r->size[0] = 1;
  r->size[1] = (int32_T)N + 17;
  emxEnsureCapacity_real_T(r, i);
  r->data[0] = 0.01;
  for (i = 0; i < 8; i++) {
    r->data[i + 1] = -10.0;
  }

  r->data[9] = -30.0;
  r->data[10] = -30.0;
  r->data[11] = -30.0;
  r->data[12] = -30.0;
  r->data[13] = 0.1;
  loop_ub = (int32_T)N;
  for (i = 0; i < loop_ub; i++) {
    r->data[i + 14] = 0.0;
  }

  emxInit_real_T(&c_T_pend, 2, true);
  r->data[(int32_T)N + 14] = 0.0;
  r->data[(int32_T)N + 15] = 0.0;
  r->data[(int32_T)N + 16] = 0.0;
  i = c_T_pend->size[0] * c_T_pend->size[1];
  c_T_pend->size[0] = 1;
  c_T_pend->size[1] = (int32_T)N + 17;
  emxEnsureCapacity_real_T(c_T_pend, i);
  c_T_pend->data[0] = b_T_pend * 2.0;
  for (i = 0; i < 8; i++) {
    c_T_pend->data[i + 1] = 10.0;
  }

  c_T_pend->data[9] = 30.0;
  c_T_pend->data[10] = 30.0;
  c_T_pend->data[11] = 30.0;
  c_T_pend->data[12] = 30.0;
  c_T_pend->data[13] = 20.0;
  loop_ub = (int32_T)N;
  for (i = 0; i < loop_ub; i++) {
    c_T_pend->data[i + 14] = 100.0;
  }

  emxInit_real_T(&x, 2, true);
  c_T_pend->data[(int32_T)N + 14] = 100.0;
  c_T_pend->data[(int32_T)N + 15] = 100.0;
  c_T_pend->data[(int32_T)N + 16] = 100.0;
  fmincon(this_tunableEnvironment, T_pend, r, c_T_pend, this_tunableEnvironment
          [0].f1, this_tunableEnvironment_f2, Fun_max, Fr_max, mu, x, &b_T_pend,
          &EXITFLAG, &expl_temp, &b_expl_temp, c_expl_temp, &d_expl_temp,
          &e_expl_temp, &f_expl_temp, &g_expl_temp);
  toc();

  /* eval trajectory */
  b_time[999] = x->data[0];
  b_time[0] = 0.0;
  emxFree_real_T(&c_T_pend);
  emxFree_real_T(&r);
  emxFree_real_T(&T_pend);
  if (0.0 == -x->data[0]) {
    for (loop_ub = 0; loop_ub < 998; loop_ub++) {
      b_time[loop_ub + 1] = x->data[0] * ((2.0 * ((real_T)loop_ub + 2.0) -
        1001.0) / 999.0);
    }
  } else if ((x->data[0] < 0.0) && (muDoubleScalarAbs(x->data[0]) >
              8.9884656743115785E+307)) {
    b_T_pend = x->data[0] / 999.0;
    for (loop_ub = 0; loop_ub < 998; loop_ub++) {
      b_time[loop_ub + 1] = b_T_pend * ((real_T)loop_ub + 1.0);
    }
  } else {
    b_T_pend = x->data[0] / 999.0;
    for (loop_ub = 0; loop_ub < 998; loop_ub++) {
      b_time[loop_ub + 1] = ((real_T)loop_ub + 1.0) * b_T_pend;
    }
  }

  /*  check they are column vectors */
  /*  parametrizzation with sin theta sing phi */
  expl_temp = x->data[1];
  b_expl_temp = x->data[2];
  d_expl_temp = x->data[3];
  e_expl_temp = x->data[4];
  f_expl_temp = 2.0 * x->data[3];
  g_expl_temp = 3.0 * x->data[4];
  b_x = x->data[2];
  c_x = x->data[5];
  d_x = x->data[6];
  e_x = x->data[7];
  f_x = x->data[8];
  a = 2.0 * x->data[7];
  b_a = 3.0 * x->data[8];
  g_x = x->data[6];
  h_x = x->data[9];
  i_x = x->data[10];
  j_x = x->data[11];
  k_x = x->data[12];

  /* disp(strcat('Initial velocity is [theta0, phi0]:',num2str(thetad(1)),"   ", num2str(phid(1))) ); */
  /*  velocity */
  for (loop_ub = 0; loop_ub < 1000; loop_ub++) {
    b_T_pend = b_time[loop_ub];
    d = b_T_pend * b_T_pend;
    d1 = muDoubleScalarPower(b_T_pend, 3.0);
    d2 = ((expl_temp + b_expl_temp * b_T_pend) + d_expl_temp * d) + e_expl_temp *
      d1;
    d3 = (b_x + f_expl_temp * b_T_pend) + g_expl_temp * d;
    d4 = ((c_x + d_x * b_T_pend) + e_x * d) + f_x * d1;
    d5 = (g_x + a * b_T_pend) + b_a * d;
    b_T_pend = ((h_x + i_x * b_T_pend) + j_x * d) + k_x * d1;
    b_time[loop_ub] = b_T_pend;
    d = muDoubleScalarSin(d2);
    d2 = muDoubleScalarCos(d2);
    d1 = muDoubleScalarCos(d4);
    d4 = muDoubleScalarSin(d4);
    pd[3 * loop_ub] = d2 * d1 * d3 * b_T_pend - d4 * d * d5 * b_T_pend;
    pd[3 * loop_ub + 1] = d2 * d4 * d3 * b_T_pend + d1 * d * d5 * b_T_pend;
    pd[3 * loop_ub + 2] = d * d3 * b_T_pend;
  }

  /*  diff(X); */
  /*  diff(Y); */
  /*  diff(Z); */
  /*  check length is always l */
  /*      a = vecnorm(p) */
  /*      a -  ones(1,length(a))*l */
  /*  init struct foc C++ code generation */
  /*  Calculating and ploting the total Energy from the new fit: theta, thetad and phid */
  /*  kinetic energy at the beginning */
  b_T_pend = m / 2.0;
  for (loop_ub = 0; loop_ub < 1000; loop_ub++) {
    if (*emlrtBreakCheckR2012bFlagVar != 0) {
      emlrtBreakCheckR2012b(emlrtRootTLSGlobal);
    }
  }

  /* compare for sanity check should be equal to  E.Ekin0 */
  /* E.Ekinfangles=  (m*l(end)^2/2).*(thetad(end)^2 + sin(theta(end))^2 *phid(end)^2); */
  *opt_Tf = x->data[0];
  *opt_K = x->data[13];

  /* eval trajectory */
  /*  parametrizzation with sin theta sing phi */
  /* evaluate inpulse ( the integral of the gaussian is 1)  */
  if ((EXITFLAG == 1.0) || (EXITFLAG == 2.0)) {
    problem_solved = true;
  } else {
    problem_solved = false;
  }

  *number_of_converged_solutions = rtNaN;
  *initial_kin_energy = rtNaN;
  *final_kin_energy = rtNaN;
  *opt_Fun = rtNaN;
  *opt_Fut = rtNaN;
  if (problem_solved) {
    *number_of_converged_solutions = 1.0;

    /*   */
    *initial_kin_energy = (b_T_pend * pd[0] * pd[0] + b_T_pend * pd[1] * pd[1])
      + b_T_pend * pd[2] * pd[2];
    *final_kin_energy = (b_T_pend * pd[2997] * pd[2997] + b_T_pend * pd[2998] *
                         pd[2998]) + b_T_pend * pd[2999] * pd[2999];
    b_T_pend = m * x->data[9];
    *opt_Fut = b_T_pend * muDoubleScalarSin(x->data[1]) * x->data[6] / T_th;
    *opt_Fun = b_T_pend * x->data[2] / T_th;
  }

  emxFree_real_T(&x);
  emlrtHeapReferenceStackLeaveFcnR2012b(emlrtRootTLSGlobal);
}

/* End of code generation (optimize_cpp.c) */
