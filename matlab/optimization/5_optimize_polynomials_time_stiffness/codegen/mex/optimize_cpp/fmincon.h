/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * fmincon.h
 *
 * Code generation for function 'fmincon'
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
void fmincon(const cell_wrap_6 fun_tunableEnvironment[2], const emxArray_real_T *
             x0, const emxArray_real_T *lb, const emxArray_real_T *ub, const
             real_T nonlcon_tunableEnvironment_f1[3], const real_T
             nonlcon_tunableEnvironment_f2[3], real_T
             nonlcon_tunableEnvironment_f3, real_T nonlcon_tunableEnvironment_f4,
             real_T nonlcon_tunableEnvironment_f5, emxArray_real_T *x, real_T
             *fval, real_T *exitflag, real_T *output_iterations, real_T
             *output_funcCount, char_T output_algorithm[3], real_T
             *output_constrviolation, real_T *output_stepsize, real_T
             *output_lssteplength, real_T *output_firstorderopt);

/* End of code generation (fmincon.h) */
