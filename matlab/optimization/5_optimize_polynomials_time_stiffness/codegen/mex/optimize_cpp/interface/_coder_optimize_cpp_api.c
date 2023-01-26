/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_optimize_cpp_api.c
 *
 * Code generation for function '_coder_optimize_cpp_api'
 *
 */

/* Include files */
#include "_coder_optimize_cpp_api.h"
#include "optimize_cpp.h"
#include "optimize_cpp_data.h"
#include "optimize_cpp_mexutil.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static real_T (*c_emlrt_marshallIn(const mxArray *pf, const char_T *identifier))
  [3];
static real_T (*d_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier
  *parentId))[3];
static real_T (*f_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[3];

/* Function Definitions */
static real_T (*c_emlrt_marshallIn(const mxArray *pf, const char_T *identifier))
  [3]
{
  emlrtMsgIdentifier thisId;
  real_T (*y)[3];
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = d_emlrt_marshallIn(emlrtAlias(pf), &thisId);
  emlrtDestroyArray(&pf);
  return y;
}
  static real_T (*d_emlrt_marshallIn(const mxArray *u, const emlrtMsgIdentifier *
  parentId))[3]
{
  real_T (*y)[3];
  y = f_emlrt_marshallIn(emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*f_emlrt_marshallIn(const mxArray *src, const emlrtMsgIdentifier *
  msgId))[3]
{
  static const int32_T dims[2] = { 1, 3 };

  real_T (*ret)[3];
  emlrtCheckBuiltInR2012b(emlrtRootTLSGlobal, msgId, src, "double", false, 2U,
    dims);
  ret = (real_T (*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
  void MEXGlobalSyncInFunction(void)
{
  const mxArray *tmp;

  /* Marshall in global variables */
  tmp = emlrtGetGlobalVariable("m");
  if (tmp != NULL) {
    m = emlrt_marshallIn(tmp, "m");
    m_dirty = 0U;
  }

  tmp = emlrtGetGlobalVariable("g");
  if (tmp != NULL) {
    g = emlrt_marshallIn(tmp, "g");
    g_dirty = 0U;
  }

  tmp = emlrtGetGlobalVariable("T_th");
  if (tmp != NULL) {
    T_th = emlrt_marshallIn(tmp, "T_th");
    T_th_dirty = 0U;
  }

  tmp = emlrtGetGlobalVariable("w1");
  if (tmp != NULL) {
    w1 = emlrt_marshallIn(tmp, "w1");
    w1_dirty = 0U;
  }

  tmp = emlrtGetGlobalVariable("w2");
  if (tmp != NULL) {
    w2 = emlrt_marshallIn(tmp, "w2");
    w2_dirty = 0U;
  }

  tmp = emlrtGetGlobalVariable("w3");
  if (tmp != NULL) {
    w3 = emlrt_marshallIn(tmp, "w3");
    w3_dirty = 0U;
  }

  tmp = emlrtGetGlobalVariable("w4");
  if (tmp != NULL) {
    w4 = emlrt_marshallIn(tmp, "w4");
    w4_dirty = 0U;
  }

  tmp = emlrtGetGlobalVariable("w5");
  if (tmp != NULL) {
    w5 = emlrt_marshallIn(tmp, "w5");
    w5_dirty = 0U;
  }

  tmp = emlrtGetGlobalVariable("N");
  if (tmp != NULL) {
    N = emlrt_marshallIn(tmp, "N");
    N_dirty = 0U;
  }

  tmp = emlrtGetGlobalVariable("num_params");
  if (tmp != NULL) {
    num_params = emlrt_marshallIn(tmp, "num_params");
    num_params_dirty = 0U;
  }

  tmp = emlrtGetGlobalVariable("l_uncompressed");
  if (tmp != NULL) {
    l_uncompressed = emlrt_marshallIn(tmp, "l_uncompressed");
    l_uncompressed_dirty = 0U;
  }
}

void MEXGlobalSyncOutFunction(boolean_T skipDirtyCheck)
{
  /* Marshall out global variables */
  if (skipDirtyCheck || (m_dirty != 0U)) {
    emlrtPutGlobalVariable("m", emlrt_marshallOut(m));
  }

  if (skipDirtyCheck || (g_dirty != 0U)) {
    emlrtPutGlobalVariable("g", emlrt_marshallOut(g));
  }

  if (skipDirtyCheck || (T_th_dirty != 0U)) {
    emlrtPutGlobalVariable("T_th", emlrt_marshallOut(T_th));
  }

  if (skipDirtyCheck || (w1_dirty != 0U)) {
    emlrtPutGlobalVariable("w1", emlrt_marshallOut(w1));
  }

  if (skipDirtyCheck || (w2_dirty != 0U)) {
    emlrtPutGlobalVariable("w2", emlrt_marshallOut(w2));
  }

  if (skipDirtyCheck || (w3_dirty != 0U)) {
    emlrtPutGlobalVariable("w3", emlrt_marshallOut(w3));
  }

  if (skipDirtyCheck || (w4_dirty != 0U)) {
    emlrtPutGlobalVariable("w4", emlrt_marshallOut(w4));
  }

  if (skipDirtyCheck || (w5_dirty != 0U)) {
    emlrtPutGlobalVariable("w5", emlrt_marshallOut(w5));
  }

  if (skipDirtyCheck || (N_dirty != 0U)) {
    emlrtPutGlobalVariable("N", emlrt_marshallOut(N));
  }

  if (skipDirtyCheck || (num_params_dirty != 0U)) {
    emlrtPutGlobalVariable("num_params", emlrt_marshallOut(num_params));
  }

  if (skipDirtyCheck || (l_uncompressed_dirty != 0U)) {
    emlrtPutGlobalVariable("l_uncompressed", emlrt_marshallOut(l_uncompressed));
  }
}

void emlrt_synchGlobalsFromML(void)
{
  MEXGlobalSyncInFunction();
  m_dirty = 0U;
  g_dirty = 0U;
  T_th_dirty = 0U;
  w1_dirty = 0U;
  w2_dirty = 0U;
  w3_dirty = 0U;
  w4_dirty = 0U;
  w5_dirty = 0U;
  N_dirty = 0U;
  num_params_dirty = 0U;
  l_uncompressed_dirty = 0U;
}

void emlrt_synchGlobalsToML(void)
{
  MEXGlobalSyncOutFunction(false);
}

void optimize_cpp_api(const mxArray * const prhs[7], int32_T nlhs, const mxArray
                      *plhs[7])
{
  real_T (*pf)[3];
  real_T Fr_max;
  real_T Fun_max;
  real_T final_kin_energy;
  real_T initial_kin_energy;
  real_T l_0;
  real_T mu;
  real_T number_of_converged_solutions;
  real_T opt_Fun;
  real_T opt_Fut;
  real_T opt_K;
  real_T opt_Tf;
  real_T phi0;
  real_T theta0;

  /* Marshall function inputs */
  l_0 = emlrt_marshallIn(emlrtAliasP(prhs[0]), "l_0");
  theta0 = emlrt_marshallIn(emlrtAliasP(prhs[1]), "theta0");
  phi0 = emlrt_marshallIn(emlrtAliasP(prhs[2]), "phi0");
  pf = c_emlrt_marshallIn(emlrtAlias(prhs[3]), "pf");
  Fun_max = emlrt_marshallIn(emlrtAliasP(prhs[4]), "Fun_max");
  Fr_max = emlrt_marshallIn(emlrtAliasP(prhs[5]), "Fr_max");
  mu = emlrt_marshallIn(emlrtAliasP(prhs[6]), "mu");

  /* Marshall in global variables */
  MEXGlobalSyncInFunction();

  /* Invoke the target function */
  optimize_cpp(l_0, theta0, phi0, *pf, Fun_max, Fr_max, mu,
               &number_of_converged_solutions, &initial_kin_energy,
               &final_kin_energy, &opt_Fun, &opt_Fut, &opt_K, &opt_Tf);

  /* Marshall out global variables */
  MEXGlobalSyncOutFunction(true);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(number_of_converged_solutions);
  if (nlhs > 1) {
    plhs[1] = emlrt_marshallOut(initial_kin_energy);
  }

  if (nlhs > 2) {
    plhs[2] = emlrt_marshallOut(final_kin_energy);
  }

  if (nlhs > 3) {
    plhs[3] = emlrt_marshallOut(opt_Fun);
  }

  if (nlhs > 4) {
    plhs[4] = emlrt_marshallOut(opt_Fut);
  }

  if (nlhs > 5) {
    plhs[5] = emlrt_marshallOut(opt_K);
  }

  if (nlhs > 6) {
    plhs[6] = emlrt_marshallOut(opt_Tf);
  }
}

/* End of code generation (_coder_optimize_cpp_api.c) */
