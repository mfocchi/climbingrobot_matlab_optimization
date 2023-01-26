/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * toc.c
 *
 * Code generation for function 'toc'
 *
 */

/* Include files */
#include "toc.h"
#include "_coder_optimize_cpp_api.h"
#include "optimize_cpp.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_mexutil.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"
#include "emlrt.h"
#include <stdio.h>

/* Variable Definitions */
static emlrtMCInfo emlrtMCI = { 64,    /* lineNo */
  18,                                  /* colNo */
  "fprintf",                           /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/eml/lib/matlab/iofun/fprintf.m"/* pName */
};

/* Function Declarations */
static const mxArray *feval(const mxArray *b, const mxArray *c, const mxArray *d,
  const mxArray *e, emlrtMCInfo *location);

/* Function Definitions */
static const mxArray *feval(const mxArray *b, const mxArray *c, const mxArray *d,
  const mxArray *e, emlrtMCInfo *location)
{
  const mxArray *pArrays[4];
  const mxArray *b_m;
  pArrays[0] = b;
  pArrays[1] = c;
  pArrays[2] = d;
  pArrays[3] = e;
  return emlrtCallMATLABR2012b(emlrtRootTLSGlobal, 1, &b_m, 4, pArrays, "feval",
    true, location);
}

void toc(void)
{
  static const int32_T iv[2] = { 1, 7 };

  static const int32_T iv1[2] = { 1, 28 };

  static const char_T b_u[28] = { 'E', 'l', 'a', 'p', 's', 'e', 'd', ' ', 't',
    'i', 'm', 'e', ' ', 'i', 's', ' ', '%', 'f', ' ', 's', 'e', 'c', 'o', 'n',
    'd', 's', '\\', 'n' };

  static const char_T u[7] = { 'f', 'p', 'r', 'i', 'n', 't', 'f' };

  emlrtTimespec tnow;
  const mxArray *b_m;
  const mxArray *b_y;
  const mxArray *m1;
  const mxArray *m2;
  const mxArray *m3;
  const mxArray *m4;
  const mxArray *y;
  real_T tstart_tv_nsec;
  real_T tstart_tv_sec;
  b_timeKeeper(&tstart_tv_sec, &tstart_tv_nsec);
  emlrt_checkEscapedGlobals();
  emlrtClockGettimeMonotonic(&tnow);
  b_m = NULL;
  m1 = NULL;
  m2 = NULL;
  m3 = NULL;
  y = NULL;
  m4 = emlrtCreateCharArray(2, &iv[0]);
  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 7, m4, &u[0]);
  emlrtAssign(&y, m4);
  emlrtAssign(&b_m, y);
  emlrtAssign(&m1, emlrt_marshallOut(1.0));
  b_y = NULL;
  m4 = emlrtCreateCharArray(2, &iv1[0]);
  emlrtInitCharArrayR2013a(emlrtRootTLSGlobal, 28, m4, &b_u[0]);
  emlrtAssign(&b_y, m4);
  emlrtAssign(&m2, b_y);
  emlrtAssign(&m3, emlrt_marshallOut((tnow.tv_sec - tstart_tv_sec) +
    (tnow.tv_nsec - tstart_tv_nsec) / 1.0E+9));
  emlrt_synchGlobalsToML();
  emlrt_marshallIn(feval(emlrtAlias(b_m), emlrtAlias(m1), emlrtAlias(m2),
    emlrtAlias(m3), &emlrtMCI), "<output of feval>");
  emlrt_synchGlobalsFromML();
  emlrtDestroyArray(&b_m);
  emlrtDestroyArray(&m1);
  emlrtDestroyArray(&m2);
  emlrtDestroyArray(&m3);
}

/* End of code generation (toc.c) */
