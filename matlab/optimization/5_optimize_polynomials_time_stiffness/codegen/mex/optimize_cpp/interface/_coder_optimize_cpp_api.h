/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * _coder_optimize_cpp_api.h
 *
 * Code generation for function '_coder_optimize_cpp_api'
 *
 */

#pragma once

/* Include files */
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void MEXGlobalSyncInFunction(void);
void MEXGlobalSyncOutFunction(boolean_T skipDirtyCheck);
void emlrt_synchGlobalsFromML(void);
void emlrt_synchGlobalsToML(void);
void optimize_cpp_api(const mxArray * const prhs[7], int32_T nlhs, const mxArray
                      *plhs[7]);

/* End of code generation (_coder_optimize_cpp_api.h) */
