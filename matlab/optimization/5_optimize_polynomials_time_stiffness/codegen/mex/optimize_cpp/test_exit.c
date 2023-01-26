/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * test_exit.c
 *
 * Code generation for function 'test_exit'
 *
 */

/* Include files */
#include "test_exit.h"
#include "computeComplError.h"
#include "computeGradLag.h"
#include "computeLambdaLSQ.h"
#include "ixamax.h"
#include "optimize_cpp_internal_types.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "updateWorkingSetForNewQP.h"
#include "xcopy.h"
#include "mwmathutil.h"

/* Function Definitions */
void test_exit(struct_T *Flags, c_struct_T *memspace, k_struct_T *MeritFunction,
               const emxArray_real_T *fscales_cineq_constraint, j_struct_T
               *WorkingSet, d_struct_T *TrialState, f_struct_T *QRManager, const
               emxArray_real_T *lb, const emxArray_real_T *ub)
{
  emxArray_real_T *gradLag;
  real_T d;
  real_T feasError;
  real_T nlpComplErrorTmp;
  real_T optimRelativeFactor;
  int32_T idx;
  int32_T idx_max;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mLambda;
  int32_T mUB;
  int32_T nVar;
  boolean_T dxTooSmall;
  boolean_T exitg1;
  boolean_T guard1 = false;
  boolean_T isFeasible;
  nVar = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mIneq = WorkingSet->sizes[2];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mLambda = ((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes[3])
    + WorkingSet->sizes[4];
  computeGradLag(TrialState->gradLag, WorkingSet->ldA, WorkingSet->nVar,
                 TrialState->grad, WorkingSet->sizes[2], WorkingSet->Aineq,
                 WorkingSet->indexFixed, WorkingSet->sizes[0],
                 WorkingSet->indexLB, WorkingSet->sizes[3], WorkingSet->indexUB,
                 WorkingSet->sizes[4], TrialState->lambdasqp);
  idx_max = ixamax(WorkingSet->nVar, TrialState->grad);
  optimRelativeFactor = muDoubleScalarMax(1.0, muDoubleScalarAbs
    (TrialState->grad->data[idx_max - 1]));
  if (muDoubleScalarIsInf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }

  feasError = 0.0;
  idx_max = WorkingSet->sizes[2];
  for (idx = 0; idx < idx_max; idx++) {
    feasError = muDoubleScalarMax(feasError, TrialState->cIneq->data[idx]);
  }

  for (idx = 0; idx < mLB; idx++) {
    idx_max = WorkingSet->indexLB->data[idx] - 1;
    feasError = muDoubleScalarMax(feasError, lb->data[idx_max] -
      TrialState->xstarsqp->data[idx_max]);
  }

  for (idx = 0; idx < mUB; idx++) {
    idx_max = WorkingSet->indexUB->data[idx] - 1;
    feasError = muDoubleScalarMax(feasError, TrialState->xstarsqp->data[idx_max]
      - ub->data[idx_max]);
  }

  MeritFunction->nlpPrimalFeasError = feasError;
  if (TrialState->sqpIterations == 0) {
    MeritFunction->feasRelativeFactor = muDoubleScalarMax(1.0, feasError);
  }

  isFeasible = (feasError <= 0.0001 * MeritFunction->feasRelativeFactor);
  idx_max = WorkingSet->nVar;
  gradLag = TrialState->gradLag;
  dxTooSmall = true;
  feasError = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= idx_max - 1)) {
    dxTooSmall = ((!muDoubleScalarIsInf(gradLag->data[idx])) &&
                  (!muDoubleScalarIsNaN(gradLag->data[idx])));
    if (!dxTooSmall) {
      exitg1 = true;
    } else {
      feasError = muDoubleScalarMax(feasError, muDoubleScalarAbs(gradLag->
        data[idx]));
      idx++;
    }
  }

  Flags->gradOK = dxTooSmall;
  MeritFunction->nlpDualFeasError = feasError;
  if (!Flags->gradOK) {
    Flags->done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = computeComplError(fscales_cineq_constraint,
      TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq,
      WorkingSet->indexLB, WorkingSet->sizes[3], lb, WorkingSet->indexUB,
      WorkingSet->sizes[4], ub, TrialState->lambdasqp, WorkingSet->sizes[0] + 1);
    MeritFunction->firstOrderOpt = muDoubleScalarMax
      (MeritFunction->nlpDualFeasError, MeritFunction->nlpComplError);
    if (TrialState->sqpIterations > 1) {
      computeGradLag(memspace->workspace_double, WorkingSet->ldA,
                     WorkingSet->nVar, TrialState->grad, WorkingSet->sizes[2],
                     WorkingSet->Aineq, WorkingSet->indexFixed,
                     WorkingSet->sizes[0], WorkingSet->indexLB,
                     WorkingSet->sizes[3], WorkingSet->indexUB,
                     WorkingSet->sizes[4], TrialState->lambdasqp_old);
      idx_max = WorkingSet->nVar;
      gradLag = memspace->workspace_double;
      feasError = 0.0;
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (idx <= idx_max - 1)) {
        dxTooSmall = ((!muDoubleScalarIsInf(gradLag->data[idx])) &&
                      (!muDoubleScalarIsNaN(gradLag->data[idx])));
        if (!dxTooSmall) {
          exitg1 = true;
        } else {
          feasError = muDoubleScalarMax(feasError, muDoubleScalarAbs
            (gradLag->data[idx]));
          idx++;
        }
      }

      nlpComplErrorTmp = computeComplError(fscales_cineq_constraint,
        TrialState->xstarsqp, WorkingSet->sizes[2], TrialState->cIneq,
        WorkingSet->indexLB, WorkingSet->sizes[3], lb, WorkingSet->indexUB,
        WorkingSet->sizes[4], ub, TrialState->lambdasqp_old, WorkingSet->sizes[0]
        + 1);
      d = muDoubleScalarMax(feasError, nlpComplErrorTmp);
      if (d < muDoubleScalarMax(MeritFunction->nlpDualFeasError,
           MeritFunction->nlpComplError)) {
        MeritFunction->nlpDualFeasError = feasError;
        MeritFunction->nlpComplError = nlpComplErrorTmp;
        MeritFunction->firstOrderOpt = d;
        c_xcopy(mLambda, TrialState->lambdasqp_old, TrialState->lambdasqp);
      } else {
        c_xcopy(mLambda, TrialState->lambdasqp, TrialState->lambdasqp_old);
      }
    } else {
      c_xcopy(mLambda, TrialState->lambdasqp, TrialState->lambdasqp_old);
    }

    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-6 *
                       optimRelativeFactor) && (MeritFunction->nlpComplError <=
         1.0E-6 * optimRelativeFactor)) {
      Flags->done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags->done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags->done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        guard1 = false;
        if (TrialState->sqpIterations > 0) {
          dxTooSmall = true;
          idx = 0;
          exitg1 = false;
          while ((!exitg1) && (idx <= nVar - 1)) {
            if (1.0E-6 * muDoubleScalarMax(1.0, muDoubleScalarAbs
                 (TrialState->xstarsqp->data[idx])) <= muDoubleScalarAbs
                (TrialState->delta_x->data[idx])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              idx++;
            }
          }

          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags->stepType != 2) {
                Flags->stepType = 2;
                Flags->failedLineSearch = false;
                Flags->stepAccepted = false;
                guard1 = true;
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = -2;
              }
            } else {
              idx_max = WorkingSet->nActiveConstr;
              if (WorkingSet->nActiveConstr > 0) {
                updateWorkingSetForNewQP(TrialState->xstarsqp, WorkingSet,
                  WorkingSet->sizes[2], TrialState->mNonlinIneq,
                  TrialState->cIneq, WorkingSet->sizes[3], lb, WorkingSet->
                  sizes[4], ub, WorkingSet->sizes[0]);
                computeLambdaLSQ(nVar, idx_max, QRManager, WorkingSet->ATwset,
                                 TrialState->grad, TrialState->lambda,
                                 memspace->workspace_double);
                idx_max = mFixed + 1;
                for (idx = idx_max; idx <= mFixed; idx++) {
                  TrialState->lambda->data[idx - 1] = -TrialState->lambda->
                    data[idx - 1];
                }

                sortLambdaQP(TrialState->lambda, WorkingSet->nActiveConstr,
                             WorkingSet->sizes, WorkingSet->isActiveIdx,
                             WorkingSet->Wid, WorkingSet->Wlocalidx,
                             memspace->workspace_double);
                computeGradLag(memspace->workspace_double, WorkingSet->ldA, nVar,
                               TrialState->grad, mIneq, WorkingSet->Aineq,
                               WorkingSet->indexFixed, mFixed,
                               WorkingSet->indexLB, mLB, WorkingSet->indexUB,
                               mUB, TrialState->lambda);
                gradLag = memspace->workspace_double;
                feasError = 0.0;
                idx = 0;
                exitg1 = false;
                while ((!exitg1) && (idx <= nVar - 1)) {
                  dxTooSmall = ((!muDoubleScalarIsInf(gradLag->data[idx])) &&
                                (!muDoubleScalarIsNaN(gradLag->data[idx])));
                  if (!dxTooSmall) {
                    exitg1 = true;
                  } else {
                    feasError = muDoubleScalarMax(feasError, muDoubleScalarAbs
                      (gradLag->data[idx]));
                    idx++;
                  }
                }

                nlpComplErrorTmp = computeComplError(fscales_cineq_constraint,
                  TrialState->xstarsqp, mIneq, TrialState->cIneq,
                  WorkingSet->indexLB, mLB, lb, WorkingSet->indexUB, mUB, ub,
                  TrialState->lambda, mFixed + 1);
                if ((feasError <= 1.0E-6 * optimRelativeFactor) &&
                    (nlpComplErrorTmp <= 1.0E-6 * optimRelativeFactor)) {
                  MeritFunction->nlpDualFeasError = feasError;
                  MeritFunction->nlpComplError = nlpComplErrorTmp;
                  MeritFunction->firstOrderOpt = muDoubleScalarMax(feasError,
                    nlpComplErrorTmp);
                  c_xcopy(mLambda, TrialState->lambda, TrialState->lambdasqp);
                  Flags->done = true;
                  TrialState->sqpExitFlag = 1;
                } else {
                  Flags->done = true;
                  TrialState->sqpExitFlag = 2;
                }
              } else {
                Flags->done = true;
                TrialState->sqpExitFlag = 2;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }

        if (guard1) {
          if (TrialState->sqpIterations >= 400) {
            Flags->done = true;
            TrialState->sqpExitFlag = 0;
          } else {
            if (TrialState->FunctionEvaluations >= 10000) {
              Flags->done = true;
              TrialState->sqpExitFlag = 0;
            }
          }
        }
      }
    }
  }
}

/* End of code generation (test_exit.c) */
