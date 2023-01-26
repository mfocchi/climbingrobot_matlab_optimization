/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * optimize_cpp.h
 *
 * Code generation for function 'optimize_cpp'
 *
 */

#pragma once

/* Include files */
#include "optimize_cpp_types.h"
#include "rtwtypes.h"
#include "emlrt.h"
#include "mex.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Function Declarations */
void anon(const real_T p0[3], const real_T pf[3], real_T Fun_max, real_T Fr_max,
          real_T mu, const emxArray_real_T *x, emxArray_real_T *varargout_1);
real_T b_anon(const emxArray_real_T *x);
void emlrt_checkEscapedGlobals(void);
void optimize_cpp(real_T l_0, real_T theta0, real_T phi0, const real_T pf[3],
                  real_T Fun_max, real_T Fr_max, real_T mu, real_T
                  *number_of_converged_solutions, real_T *initial_kin_energy,
                  real_T *final_kin_energy, real_T *opt_Fun, real_T *opt_Fut,
                  real_T *opt_K, real_T *opt_Tf);

/* End of code generation (optimize_cpp.h) */
