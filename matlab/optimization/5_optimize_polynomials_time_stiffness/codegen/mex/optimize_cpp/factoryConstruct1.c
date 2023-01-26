/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * factoryConstruct1.c
 *
 * Code generation for function 'factoryConstruct1'
 *
 */

/* Include files */
#include "factoryConstruct1.h"
#include "optimize_cpp_emxutil.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "mwmathutil.h"

/* Function Definitions */
void b_factoryConstruct(const cell_wrap_6 objfun_tunableEnvironment[2], const
  real_T nonlin_tunableEnvironment_f1[3], const real_T
  nonlin_tunableEnvironment_f2[3], real_T nonlin_tunableEnvironment_f3, real_T
  nonlin_tunableEnvironment_f4, real_T nonlin_tunableEnvironment_f5, int32_T
  nVar, int32_T mCineq, const emxArray_real_T *ub, e_struct_T *obj)
{
  int32_T idx;
  boolean_T b;
  obj->objfun.tunableEnvironment[0] = objfun_tunableEnvironment[0];
  obj->objfun.tunableEnvironment[1] = objfun_tunableEnvironment[1];
  obj->nonlin.tunableEnvironment.f1[0] = nonlin_tunableEnvironment_f1[0];
  obj->nonlin.tunableEnvironment.f2[0] = nonlin_tunableEnvironment_f2[0];
  obj->nonlin.tunableEnvironment.f1[1] = nonlin_tunableEnvironment_f1[1];
  obj->nonlin.tunableEnvironment.f2[1] = nonlin_tunableEnvironment_f2[1];
  obj->nonlin.tunableEnvironment.f1[2] = nonlin_tunableEnvironment_f1[2];
  obj->nonlin.tunableEnvironment.f2[2] = nonlin_tunableEnvironment_f2[2];
  obj->nonlin.tunableEnvironment.f3 = nonlin_tunableEnvironment_f3;
  obj->nonlin.tunableEnvironment.f4 = nonlin_tunableEnvironment_f4;
  obj->nonlin.tunableEnvironment.f5 = nonlin_tunableEnvironment_f5;
  obj->f_1 = 0.0;
  idx = obj->cIneq_1->size[0];
  obj->cIneq_1->size[0] = mCineq;
  emxEnsureCapacity_real_T(obj->cIneq_1, idx);
  obj->f_2 = 0.0;
  idx = obj->cIneq_2->size[0];
  obj->cIneq_2->size[0] = mCineq;
  emxEnsureCapacity_real_T(obj->cIneq_2, idx);
  obj->nVar = nVar;
  obj->mIneq = mCineq;
  obj->mEq = 0;
  obj->numEvals = 0;
  obj->SpecifyObjectiveGradient = false;
  obj->SpecifyConstraintGradient = false;
  idx = obj->hasLB->size[0];
  obj->hasLB->size[0] = nVar;
  emxEnsureCapacity_boolean_T(obj->hasLB, idx);
  idx = obj->hasUB->size[0];
  obj->hasUB->size[0] = nVar;
  emxEnsureCapacity_boolean_T(obj->hasUB, idx);
  obj->FiniteDifferenceType = 0;
  b = false;
  idx = 0;
  if (1 <= nVar) {
    obj->hasLB->data[0] = true;
    obj->hasUB->data[0] = ((!muDoubleScalarIsInf(ub->data[0])) &&
      (!muDoubleScalarIsNaN(ub->data[0])));
    b = true;
    idx = 1;
  }

  while (idx + 1 <= nVar) {
    obj->hasLB->data[idx] = true;
    obj->hasUB->data[idx] = ((!muDoubleScalarIsInf(ub->data[idx])) &&
      (!muDoubleScalarIsNaN(ub->data[idx])));
    idx++;
  }

  obj->hasBounds = b;
}

/* End of code generation (factoryConstruct1.c) */
