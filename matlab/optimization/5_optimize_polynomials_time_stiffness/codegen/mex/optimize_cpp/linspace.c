/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * linspace.c
 *
 * Code generation for function 'linspace'
 *
 */

/* Include files */
#include "linspace.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Function Definitions */
void linspace(real_T d2, real_T n, emxArray_real_T *y)
{
  real_T delta1;
  int32_T k;
  int32_T y_tmp;
  if (!(n >= 0.0)) {
    y->size[0] = 1;
    y->size[1] = 0;
  } else {
    delta1 = muDoubleScalarFloor(n);
    y_tmp = y->size[0] * y->size[1];
    y->size[0] = 1;
    y->size[1] = (int32_T)delta1;
    emxEnsureCapacity_real_T(y, y_tmp);
    if ((int32_T)delta1 >= 1) {
      y_tmp = (int32_T)delta1 - 1;
      y->data[(int32_T)delta1 - 1] = d2;
      if (y->size[1] >= 2) {
        y->data[0] = 0.0;
        if (y->size[1] >= 3) {
          if ((0.0 == -d2) && ((int32_T)delta1 > 2)) {
            for (k = 2; k <= y_tmp; k++) {
              y->data[k - 1] = d2 * ((real_T)(((k << 1) - (int32_T)delta1) - 1) /
                ((real_T)(int32_T)delta1 - 1.0));
            }

            if (((int32_T)delta1 & 1) == 1) {
              y->data[(int32_T)delta1 >> 1] = 0.0;
            }
          } else if ((d2 < 0.0) && (muDoubleScalarAbs(d2) >
                      8.9884656743115785E+307)) {
            delta1 = d2 / ((real_T)y->size[1] - 1.0);
            y_tmp = y->size[1];
            for (k = 0; k <= y_tmp - 3; k++) {
              y->data[k + 1] = delta1 * ((real_T)k + 1.0);
            }
          } else {
            delta1 = d2 / ((real_T)y->size[1] - 1.0);
            y_tmp = y->size[1];
            for (k = 0; k <= y_tmp - 3; k++) {
              y->data[k + 1] = ((real_T)k + 1.0) * delta1;
            }
          }
        }
      }
    }
  }
}

/* End of code generation (linspace.c) */
