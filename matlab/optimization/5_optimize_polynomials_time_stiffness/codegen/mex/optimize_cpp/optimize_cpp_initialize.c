/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp_initialize.c
 *
 * Code generation for function 'optimize_cpp_initialize'
 *
 */

/* Include files */
#include "optimize_cpp_initialize.h"
#include "_coder_optimize_cpp_api.h"
#include "_coder_optimize_cpp_mex.h"
#include "optimize_cpp_data.h"
#include "rt_nonfinite.h"
#include "timeKeeper.h"

/* Function Declarations */
static void optimize_cpp_once(void);

/* Function Definitions */
static void optimize_cpp_once(void)
{
  mex_InitInfAndNan();
  savedTime_not_empty_init();
  l_uncompressed = 3.0;
  l_uncompressed_dirty = 1U;
  num_params = 14.0;
  num_params_dirty = 1U;
  N = 10.0;
  N_dirty = 1U;
  w5 = 0.01;
  w5_dirty = 1U;
  w4 = 10.0;
  w4_dirty = 1U;
  w3 = 1.0;
  w3_dirty = 1U;
  w2 = 1.0;
  w2_dirty = 1U;
  w1 = 1.0;
  w1_dirty = 1U;
  T_th = 0.05;
  T_th_dirty = 1U;
  g = 9.81;
  g_dirty = 1U;
  m = 5.0;
  m_dirty = 1U;
  emlrtSetGlobalSyncFcn(emlrtRootTLSGlobal, (void (*)(const void *))&
                        emlrt_synchGlobalsToML);
}

void optimize_cpp_initialize(void)
{
  mexFunctionCreateRootTLS();
  emlrtBreakCheckR2012bFlagVar = emlrtGetBreakCheckFlagAddressR2012b();
  emlrtClearAllocCountR2012b(emlrtRootTLSGlobal, false, 0U, 0);
  emlrtEnterRtStackR2012b(emlrtRootTLSGlobal);
  emlrtLicenseCheckR2012b(emlrtRootTLSGlobal, "optimization_toolbox", 2);
  if (emlrtFirstTimeR2012b(emlrtRootTLSGlobal)) {
    optimize_cpp_once();
  }
}

/* End of code generation (optimize_cpp_initialize.c) */
