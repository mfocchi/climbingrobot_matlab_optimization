/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_terminate.c
 *
 * Code generation for function 'optimize_cpp_terminate'
 *
 */

/* Include files */
#include "optimize_cpp_terminate.h"
#include "_coder_optimize_cpp_api.h"
#include "_coder_optimize_cpp_mex.h"
#include "optimize_cpp_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
void optimize_cpp_atexit(void)
{
  mexFunctionCreateRootTLS();
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrt_synchGlobalsFromML();
  emlrt_synchGlobalsToML();
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void optimize_cpp_terminate(void)
{
  emlrtLeaveRtStackR2012b(emlrtRootTLSGlobal);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/* End of code generation (optimize_cpp_terminate.c) */
