/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * driver.c
 *
 * Code generation for function 'driver'
 *
 */

/* Include files */
#include "driver.h"
#include "BFGSUpdate.h"
#include "computeConstraints_.h"
#include "computeDeltaLag.h"
#include "computeForwardDifferences.h"
#include "computeGradLag.h"
#include "computeLinearResiduals.h"
#include "ixamax.h"
#include "optimize_cpp.h"
#include "optimize_cpp_internal_types.h"
#include "optimize_cpp_types.h"
#include "rt_nonfinite.h"
#include "saveJacobian.h"
#include "step.h"
#include "test_exit.h"
#include "updateWorkingSetForNewQP.h"
#include "xaxpy.h"
#include "xcopy.h"
#include "mwmathutil.h"

/* Function Definitions */
void driver(emxArray_real_T *Hessian, const emxArray_real_T *lb, const
            emxArray_real_T *ub, d_struct_T *TrialState, k_struct_T
            *MeritFunction, const g_struct_T *FcnEvaluator, e_struct_T
            *FiniteDifferences, c_struct_T *memspace, j_struct_T *WorkingSet,
            f_struct_T *QRManager, h_struct_T *CholManager, i_struct_T
            *QPObjective, const emxArray_real_T *fscales_cineq_constraint)
{
  static const char_T qpoptions_SolverName[7] = { 'f', 'm', 'i', 'n', 'c', 'o',
    'n' };

  b_struct_T b_expl_temp;
  b_struct_T expl_temp;
  emxArray_real_T *gradLag;
  struct_T Flags;
  real_T alpha;
  real_T optimRelativeFactor;
  real_T phi_alpha;
  int32_T b_mIneq;
  int32_T b_mLB;
  int32_T b_mUB;
  int32_T b_nVar;
  int32_T exitg2;
  int32_T idx;
  int32_T idx_max;
  int32_T mConstr;
  int32_T mFixed;
  int32_T mIneq;
  int32_T mLB;
  int32_T mLinIneq;
  int32_T mUB;
  int32_T nVar;
  int32_T qpoptions_MaxIterations;
  boolean_T exitg1;
  boolean_T gradOK;
  boolean_T isFeasible;
  boolean_T tooSmallX;
  nVar = WorkingSet->nVar;
  mFixed = WorkingSet->sizes[0];
  mIneq = WorkingSet->sizes[2];
  mLB = WorkingSet->sizes[3];
  mUB = WorkingSet->sizes[4];
  mConstr = ((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes[3])
    + WorkingSet->sizes[4];
  mLinIneq = WorkingSet->sizes[2] - TrialState->mNonlinIneq;
  idx_max = ((WorkingSet->sizes[2] + WorkingSet->sizes[3]) + WorkingSet->sizes[4])
    + (WorkingSet->sizes[0] << 1);
  qpoptions_MaxIterations = 10 * muIntScalarMax_sint32(WorkingSet->nVar, idx_max);
  TrialState->steplength = 1.0;
  Flags.fevalOK = true;
  Flags.stepAccepted = false;
  Flags.failedLineSearch = false;
  Flags.stepType = 1;
  b_mLB = WorkingSet->sizes[3];
  b_mUB = WorkingSet->sizes[4];
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

  phi_alpha = 0.0;
  b_mIneq = WorkingSet->sizes[2];
  for (idx = 0; idx < b_mIneq; idx++) {
    phi_alpha = muDoubleScalarMax(phi_alpha, TrialState->cIneq->data[idx]);
  }

  for (idx = 0; idx < b_mLB; idx++) {
    idx_max = WorkingSet->indexLB->data[idx] - 1;
    phi_alpha = muDoubleScalarMax(phi_alpha, lb->data[idx_max] -
      TrialState->xstarsqp->data[idx_max]);
  }

  for (idx = 0; idx < b_mUB; idx++) {
    idx_max = WorkingSet->indexUB->data[idx] - 1;
    phi_alpha = muDoubleScalarMax(phi_alpha, TrialState->xstarsqp->data[idx_max]
      - ub->data[idx_max]);
  }

  MeritFunction->nlpPrimalFeasError = phi_alpha;
  MeritFunction->feasRelativeFactor = muDoubleScalarMax(1.0, phi_alpha);
  isFeasible = (phi_alpha <= 0.0001 * MeritFunction->feasRelativeFactor);
  b_nVar = WorkingSet->nVar;
  gradLag = TrialState->gradLag;
  gradOK = true;
  phi_alpha = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= b_nVar - 1)) {
    gradOK = ((!muDoubleScalarIsInf(gradLag->data[idx])) &&
              (!muDoubleScalarIsNaN(gradLag->data[idx])));
    if (!gradOK) {
      exitg1 = true;
    } else {
      phi_alpha = muDoubleScalarMax(phi_alpha, muDoubleScalarAbs(gradLag->
        data[idx]));
      idx++;
    }
  }

  Flags.gradOK = gradOK;
  MeritFunction->nlpDualFeasError = phi_alpha;
  if (!gradOK) {
    Flags.done = true;
    if (isFeasible) {
      TrialState->sqpExitFlag = 2;
    } else {
      TrialState->sqpExitFlag = -2;
    }
  } else {
    MeritFunction->nlpComplError = 0.0;
    MeritFunction->firstOrderOpt = muDoubleScalarMax
      (MeritFunction->nlpDualFeasError, 0.0);
    c_xcopy(((WorkingSet->sizes[0] + WorkingSet->sizes[2]) + WorkingSet->sizes[3])
            + WorkingSet->sizes[4], TrialState->lambdasqp,
            TrialState->lambdasqp_old);
    if (isFeasible && (MeritFunction->nlpDualFeasError <= 1.0E-6 *
                       optimRelativeFactor)) {
      Flags.done = true;
      TrialState->sqpExitFlag = 1;
    } else {
      Flags.done = false;
      if (isFeasible && (TrialState->sqpFval < -1.0E+20)) {
        Flags.done = true;
        TrialState->sqpExitFlag = -3;
      } else {
        if (TrialState->FunctionEvaluations >= 10000) {
          Flags.done = true;
          TrialState->sqpExitFlag = 0;
        }
      }
    }
  }

  saveJacobian(TrialState, WorkingSet->nVar, WorkingSet->sizes[2],
               WorkingSet->Aineq, TrialState->iNonIneq0, WorkingSet->ldA);
  TrialState->sqpFval_old = TrialState->sqpFval;
  b_nVar = TrialState->xstarsqp->size[1];
  xcopy(TrialState->xstarsqp->size[1], TrialState->xstarsqp,
        TrialState->xstarsqp_old);
  for (idx_max = 0; idx_max < b_nVar; idx_max++) {
    TrialState->grad_old->data[idx_max] = TrialState->grad->data[idx_max];
  }

  c_xcopy(TrialState->mIneq, TrialState->cIneq, TrialState->cIneq_old);
  if (!Flags.done) {
    TrialState->sqpIterations = 1;
  }

  while (!Flags.done) {
    while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
      updateWorkingSetForNewQP(TrialState->xstarsqp, WorkingSet, mIneq,
        TrialState->mNonlinIneq, TrialState->cIneq, mLB, lb, mUB, ub, mFixed);
      expl_temp.IterDisplayQP = false;
      expl_temp.RemainFeasible = false;
      expl_temp.ProbRelTolFactor = 1.0;
      expl_temp.ConstrRelTolFactor = 1.0;
      expl_temp.PricingTolerance = 0.0;
      expl_temp.ObjectiveLimit = rtMinusInf;
      expl_temp.ConstraintTolerance = 0.0001;
      expl_temp.OptimalityTolerance = 2.2204460492503131E-14;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = qpoptions_MaxIterations;
      for (idx_max = 0; idx_max < 7; idx_max++) {
        expl_temp.SolverName[idx_max] = qpoptions_SolverName[idx_max];
      }

      b_expl_temp = expl_temp;
      Flags.stepAccepted = step(&Flags.stepType, Hessian, lb, ub, TrialState,
        MeritFunction, memspace, WorkingSet, QRManager, CholManager, QPObjective,
        &b_expl_temp);
      if (Flags.stepAccepted) {
        for (idx_max = 0; idx_max < nVar; idx_max++) {
          TrialState->xstarsqp->data[idx_max] += TrialState->delta_x->
            data[idx_max];
        }

        phi_alpha = b_anon(TrialState->xstarsqp);
        idx_max = 1;
        if (muDoubleScalarIsInf(phi_alpha) || muDoubleScalarIsNaN(phi_alpha)) {
          if (muDoubleScalarIsNaN(phi_alpha)) {
            idx_max = -6;
          } else if (phi_alpha < 0.0) {
            idx_max = -4;
          } else {
            idx_max = -5;
          }
        }

        TrialState->sqpFval = phi_alpha;
        if (idx_max == 1) {
          idx_max = computeConstraints_
            (FcnEvaluator->nonlcon.tunableEnvironment.f1,
             FcnEvaluator->nonlcon.tunableEnvironment.f2,
             FcnEvaluator->nonlcon.tunableEnvironment.f3,
             FcnEvaluator->nonlcon.tunableEnvironment.f4,
             FcnEvaluator->nonlcon.tunableEnvironment.f5, FcnEvaluator->mCineq,
             TrialState->xstarsqp, TrialState->cIneq, TrialState->iNonIneq0);
        }

        Flags.fevalOK = (idx_max == 1);
        TrialState->FunctionEvaluations++;
        computeLinearResiduals(TrialState->xstarsqp, nVar, TrialState->cIneq,
          mLinIneq, WorkingSet->Aineq, WorkingSet->ldA);
        if (Flags.fevalOK) {
          optimRelativeFactor = 0.0;
          for (idx = 0; idx < mIneq; idx++) {
            if (TrialState->cIneq->data[idx] > 0.0) {
              optimRelativeFactor += TrialState->cIneq->data[idx];
            }
          }

          MeritFunction->phiFullStep = phi_alpha + MeritFunction->penaltyParam *
            optimRelativeFactor;
        } else {
          MeritFunction->phiFullStep = rtInf;
        }
      }

      if ((Flags.stepType == 1) && Flags.stepAccepted && Flags.fevalOK &&
          (MeritFunction->phi < MeritFunction->phiFullStep) &&
          (TrialState->sqpFval < TrialState->sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          isFeasible = true;
        } else {
          isFeasible = false;
        }

        gradOK = Flags.fevalOK;
        b_nVar = WorkingSet->nVar;
        b_mIneq = TrialState->mIneq;
        b_mLB = TrialState->mIneq - TrialState->mNonlinIneq;
        alpha = 1.0;
        b_mUB = 1;
        phi_alpha = MeritFunction->phiFullStep;
        c_xcopy(WorkingSet->nVar, TrialState->delta_x, TrialState->searchDir);
        do {
          exitg2 = 0;
          if (TrialState->FunctionEvaluations < 10000) {
            if (gradOK && (phi_alpha <= MeritFunction->phi + alpha * 0.0001 *
                           MeritFunction->phiPrimePlus)) {
              exitg2 = 1;
            } else {
              alpha *= 0.7;
              for (idx = 0; idx < b_nVar; idx++) {
                TrialState->delta_x->data[idx] = alpha * TrialState->xstar->
                  data[idx];
              }

              if (isFeasible) {
                xaxpy(b_nVar, alpha * alpha, TrialState->socDirection,
                      TrialState->delta_x);
              }

              tooSmallX = true;
              idx = 0;
              exitg1 = false;
              while ((!exitg1) && (idx <= b_nVar - 1)) {
                if (1.0E-6 * muDoubleScalarMax(1.0, muDoubleScalarAbs
                     (TrialState->xstarsqp->data[idx])) <= muDoubleScalarAbs
                    (TrialState->delta_x->data[idx])) {
                  tooSmallX = false;
                  exitg1 = true;
                } else {
                  idx++;
                }
              }

              if (tooSmallX) {
                b_mUB = -2;
                exitg2 = 1;
              } else {
                for (idx = 0; idx < b_nVar; idx++) {
                  TrialState->xstarsqp->data[idx] = TrialState->
                    xstarsqp_old->data[idx] + TrialState->delta_x->data[idx];
                }

                phi_alpha = b_anon(TrialState->xstarsqp);
                idx_max = 1;
                if (muDoubleScalarIsInf(phi_alpha) || muDoubleScalarIsNaN
                    (phi_alpha)) {
                  if (muDoubleScalarIsNaN(phi_alpha)) {
                    idx_max = -6;
                  } else if (phi_alpha < 0.0) {
                    idx_max = -4;
                  } else {
                    idx_max = -5;
                  }
                }

                TrialState->sqpFval = phi_alpha;
                if (idx_max == 1) {
                  idx_max = computeConstraints_
                    (FcnEvaluator->nonlcon.tunableEnvironment.f1,
                     FcnEvaluator->nonlcon.tunableEnvironment.f2,
                     FcnEvaluator->nonlcon.tunableEnvironment.f3,
                     FcnEvaluator->nonlcon.tunableEnvironment.f4,
                     FcnEvaluator->nonlcon.tunableEnvironment.f5,
                     FcnEvaluator->mCineq, TrialState->xstarsqp,
                     TrialState->cIneq, TrialState->iNonIneq0);
                }

                computeLinearResiduals(TrialState->xstarsqp, b_nVar,
                  TrialState->cIneq, b_mLB, WorkingSet->Aineq, WorkingSet->ldA);
                TrialState->FunctionEvaluations++;
                gradOK = (idx_max == 1);
                if (gradOK) {
                  optimRelativeFactor = 0.0;
                  for (idx = 0; idx < b_mIneq; idx++) {
                    if (TrialState->cIneq->data[idx] > 0.0) {
                      optimRelativeFactor += TrialState->cIneq->data[idx];
                    }
                  }

                  phi_alpha += MeritFunction->penaltyParam * optimRelativeFactor;
                } else {
                  phi_alpha = rtInf;
                }
              }
            }
          } else {
            b_mUB = 0;
            exitg2 = 1;
          }
        } while (exitg2 == 0);

        Flags.fevalOK = gradOK;
        TrialState->steplength = alpha;
        if (b_mUB > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }

    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      for (idx = 0; idx < nVar; idx++) {
        TrialState->xstarsqp->data[idx] = TrialState->xstarsqp_old->data[idx] +
          TrialState->delta_x->data[idx];
      }

      for (idx = 0; idx < mConstr; idx++) {
        TrialState->lambdasqp->data[idx] += TrialState->steplength *
          (TrialState->lambda->data[idx] - TrialState->lambdasqp->data[idx]);
      }

      TrialState->sqpFval_old = TrialState->sqpFval;
      b_nVar = TrialState->xstarsqp->size[1];
      xcopy(TrialState->xstarsqp->size[1], TrialState->xstarsqp,
            TrialState->xstarsqp_old);
      for (idx_max = 0; idx_max < b_nVar; idx_max++) {
        TrialState->grad_old->data[idx_max] = TrialState->grad->data[idx_max];
      }

      c_xcopy(TrialState->mIneq, TrialState->cIneq, TrialState->cIneq_old);
      Flags.gradOK = computeForwardDifferences(FiniteDifferences,
        TrialState->sqpFval, TrialState->cIneq, TrialState->iNonIneq0,
        TrialState->xstarsqp, TrialState->grad, WorkingSet->Aineq,
        TrialState->iNonIneq0, lb, ub);
      TrialState->FunctionEvaluations += FiniteDifferences->numEvals;
    } else {
      TrialState->sqpFval = TrialState->sqpFval_old;
      xcopy(TrialState->xstarsqp->size[1], TrialState->xstarsqp_old,
            TrialState->xstarsqp);
      c_xcopy(TrialState->mIneq, TrialState->cIneq_old, TrialState->cIneq);
    }

    test_exit(&Flags, memspace, MeritFunction, fscales_cineq_constraint,
              WorkingSet, TrialState, QRManager, lb, ub);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      computeDeltaLag(nVar, WorkingSet->ldA, TrialState->mNonlinIneq,
                      TrialState->delta_gradLag, TrialState->grad,
                      WorkingSet->Aineq, TrialState->iNonIneq0,
                      TrialState->grad_old, TrialState->JacCineqTrans_old,
                      TrialState->lambdasqp, mFixed + TrialState->iNonIneq0);
      saveJacobian(TrialState, nVar, mIneq, WorkingSet->Aineq,
                   TrialState->iNonIneq0, WorkingSet->ldA);
      BFGSUpdate(nVar, Hessian, TrialState->delta_x, TrialState->delta_gradLag,
                 memspace->workspace_double);
      TrialState->sqpIterations++;
    }
  }
}

/* End of code generation (driver.c) */
