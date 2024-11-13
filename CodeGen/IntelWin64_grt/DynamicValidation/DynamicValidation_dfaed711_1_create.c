/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'DynamicValidation/Dynamic_Model_PID/SPM MODEL/Solver Configuration'.
 */

#include "pm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "pm_default_allocator.h"
#include "sm_ssci_NeDaePrivateData.h"
#include "sm_CTarget.h"

PmfMessageId sm_ssci_recordRunTimeError(
  const char *errorId, const char *errorMsg, NeuDiagnosticManager* mgr);

#define pm_allocator_alloc(_allocator, _m, _n) ((_allocator)->mCallocFcn((_allocator), (_m), (_n)))
#define PM_ALLOCATE_ARRAY(_name, _type, _size, _allocator)\
 _name = (_type *) pm_allocator_alloc(_allocator, sizeof(_type), _size)
#define pm_size_to_int(_size)          ((int32_T) (_size))

PmIntVector *pm_create_int_vector(size_t, PmAllocator *);
int_T pm_create_int_vector_fields (PmIntVector *, size_t, PmAllocator *);
int_T pm_create_real_vector_fields(PmRealVector *, size_t, PmAllocator *);
int_T pm_create_char_vector_fields(PmCharVector *, size_t, PmAllocator *);
int_T pm_create_bool_vector_fields(PmBoolVector *, size_t, PmAllocator *);
void pm_rv_equals_rv(const PmRealVector *, const PmRealVector *);
extern const NeAssertData DynamicValidation_dfaed711_1_assertData[];
extern const NeZCData DynamicValidation_dfaed711_1_ZCData[];
void DynamicValidation_dfaed711_1_computeRuntimeParameters(
  const double *runtimeRootVariables,
  double *runtimeParameters);
void DynamicValidation_dfaed711_1_validateRuntimeParameters(
  const double *runtimeParameters,
  int32_T *assertSatisfactionFlags);
void DynamicValidation_dfaed711_1_computeAsmRuntimeDerivedValues(
  const double *runtimeParameters,
  RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle);
void DynamicValidation_dfaed711_1_computeSimRuntimeDerivedValues(
  const double *runtimeParameters,
  RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle);
void DynamicValidation_dfaed711_1_initializeGeometries(
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle);
PmfMessageId DynamicValidation_dfaed711_1_compDerivs(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId DynamicValidation_dfaed711_1_numJacPerturbLoBounds(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId DynamicValidation_dfaed711_1_numJacPerturbHiBounds(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId DynamicValidation_dfaed711_1_checkDynamics(
  const RuntimeDerivedValuesBundle *,
  const double *,
  const double *, const double *, const double *,
  const double *,
  const int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId DynamicValidation_dfaed711_1_compOutputsDyn(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId DynamicValidation_dfaed711_1_compOutputsKin(
  const RuntimeDerivedValuesBundle *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId DynamicValidation_dfaed711_1_compOutputs (
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId DynamicValidation_dfaed711_1_computeAsmModeVector(
  const double *, const double *, const double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId DynamicValidation_dfaed711_1_computeSimModeVector(
  const double *, const double *, const double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId DynamicValidation_dfaed711_1_computeZeroCrossings(
  const RuntimeDerivedValuesBundle *,
  const double *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId DynamicValidation_dfaed711_1_recordLog(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
void DynamicValidation_dfaed711_1_setTargets(
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  CTarget *targets);
void DynamicValidation_dfaed711_1_resetAsmStateVector(const void *mech, double
  *stateVector);
void DynamicValidation_dfaed711_1_resetSimStateVector(const void *mech, double
  *stateVector);
void DynamicValidation_dfaed711_1_initializeTrackedAngleState(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *modeVector,
  const double *motionData,
  double *stateVector);
void DynamicValidation_dfaed711_1_computeDiscreteState(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  double *stateVector);
void DynamicValidation_dfaed711_1_adjustPosition(
  const void *mech,
  const double *dofDeltas,
  double *stateVector);
void DynamicValidation_dfaed711_1_perturbAsmJointPrimitiveState(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void DynamicValidation_dfaed711_1_perturbSimJointPrimitiveState(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void DynamicValidation_dfaed711_1_perturbFlexibleBodyState(
  const void *mech,
  size_t stageIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void DynamicValidation_dfaed711_1_computePosDofBlendMatrix(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *stateVector,
  int partialType,
  double *matrix);
void DynamicValidation_dfaed711_1_computeVelDofBlendMatrix(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *stateVector,
  int partialType,
  double *matrix);
void DynamicValidation_dfaed711_1_projectPartiallyTargetedPos(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *origStateVector,
  int partialType,
  double *stateVector);
void DynamicValidation_dfaed711_1_propagateMotion(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const double *stateVector,
  double *motionData);
size_t DynamicValidation_dfaed711_1_computeAssemblyError(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  size_t constraintIdx,
  const int *modeVector,
  const double *motionData,
  double *error);
size_t DynamicValidation_dfaed711_1_computeAssemblyJacobian(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  size_t constraintIdx,
  boolean_T forVelocitySatisfaction,
  const double *stateVector,
  const int *modeVector,
  const double *motionData,
  double *J);
size_t DynamicValidation_dfaed711_1_computeFullAssemblyJacobian(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const double *stateVector,
  const int *modeVector,
  const double *motionData,
  double *J);
boolean_T DynamicValidation_dfaed711_1_isInKinematicSingularity(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  size_t constraintIdx,
  const int *modeVector,
  const double *motionData);
void DynamicValidation_dfaed711_1_convertStateVector(
  const void *asmMech,
  const RuntimeDerivedValuesBundle *asmRuntimeDerivedValuesBundle,
  const void *simMech,
  const double *asmStateVector,
  const int *asmModeVector,
  const int *simModeVector,
  double *simStateVector);
void DynamicValidation_dfaed711_1_constructStateVector(
  const void *mech,
  const double *solverStateVector,
  const double *u,
  const double *uDot,
  const double *discreteStateVector,
  double *fullStateVector);
void DynamicValidation_dfaed711_1_extractSolverStateVector(
  const void *mech,
  const double *fullStateVector,
  double *solverStateVector);
boolean_T DynamicValidation_dfaed711_1_isPositionViolation(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const double *stateVector,
  const int *modeVector);
boolean_T DynamicValidation_dfaed711_1_isVelocityViolation(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const double *stateVector,
  const int *modeVector);
PmfMessageId DynamicValidation_dfaed711_1_projectStateSim(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const int *modeVector,
  double *stateVector,
  void *neDiagMgr);
void DynamicValidation_dfaed711_1_computeConstraintError(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const double *stateVector,
  const int *modeVector,
  double *error);
void DynamicValidation_dfaed711_1_resetModeVector(const void *mech, int
  *modeVector);
boolean_T DynamicValidation_dfaed711_1_hasJointDisToNormModeChange(
  const void *mech,
  const int *prevModeVector,
  const int *modeVector);
PmfMessageId DynamicValidation_dfaed711_1_performJointDisToNormModeChange(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const int *prevModeVector,
  const int *modeVector,
  const double *inputVector,
  double *stateVector,
  void *neDiagMgr);
void DynamicValidation_dfaed711_1_onModeChangedCutJoints(
  const void *mech,
  const int *prevModeVector,
  const int *modeVector,
  double *stateVector);
PmfMessageId DynamicValidation_dfaed711_1_assemble(const double *u, double *udot,
  double *x,
  NeuDiagnosticManager *neDiagMgr)
{
  (void) x;
  (void) u;
  (void) udot;
  (void) neDiagMgr;
  return NULL;
}

static
  void dae_cg_setParameters_function(const NeDae *dae,
  const NeParameterBundle *paramBundle)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  const double *runtimeRootVariables = paramBundle->mRealParameters.mX;
  if (smData->mRuntimeParameterScalars.mN == 0)
    return;
  DynamicValidation_dfaed711_1_computeRuntimeParameters(
    runtimeRootVariables,
    smData->mRuntimeParameterScalars.mX);
  DynamicValidation_dfaed711_1_computeAsmRuntimeDerivedValues(
    smData->mRuntimeParameterScalars.mX,
    &dae->mPrivateData->mAsmRuntimeDerivedValuesBundle);
  DynamicValidation_dfaed711_1_computeSimRuntimeDerivedValues(
    smData->mRuntimeParameterScalars.mX,
    &dae->mPrivateData->mSimRuntimeDerivedValuesBundle);
  DynamicValidation_dfaed711_1_initializeGeometries
    (&smData->mSimRuntimeDerivedValuesBundle);
  sm_core_computeRedundantConstraintEquations(
    &dae->mPrivateData->mSimulationDelegate,
    &smData->mSimRuntimeDerivedValuesBundle);

#if 0

  {
    size_t i;
    const size_t n = smData->mSimulationDelegate.mRunTimeEnabledEquations.mSize;
    pmf_printf("\nRuntime Enabled Equations (%lu)\n", n);
    for (i = 0; i < n; ++i)
      pmf_printf("  %2lu:  %d\n", i,
                 smData->mSimulationDelegate.mRunTimeEnabledEquations.mValues[i]);
  }

#endif

}

static
  PmfMessageId dae_cg_pAssert_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  const double *runtimeParams = smData->mRuntimeParameterScalars.mX;
  int32_T *assertSatisfactionFlags = daeMethodOutput->mPASSERT.mX;
  (void) systemInput;
  (void) neDiagMgr;
  DynamicValidation_dfaed711_1_validateRuntimeParameters(
    runtimeParams, assertSatisfactionFlags);
  return NULL;
}

static
  PmfMessageId dae_cg_deriv_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  if (smData->mCachedDerivativesAvailable)
    memcpy(daeMethodOutput->mXP0.mX, smData->mCachedDerivatives.mX,
           12 * sizeof(real_T));
  else
    errorId = DynamicValidation_dfaed711_1_compDerivs(
      &smData->mSimRuntimeDerivedValuesBundle,
      smData->mSimulationDelegate
      .mRunTimeEnabledEquations.mValues,
      systemInput->mX.mX,
      systemInput->mM.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mV.mX + 3,
      systemInput->mD.mX,
      daeMethodOutput->mXP0.mX,
      &errorResult,
      neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_numJacPerturbLoBounds_method(
  const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  errorId = DynamicValidation_dfaed711_1_numJacPerturbLoBounds(
    &smData->mSimRuntimeDerivedValuesBundle,
    smData->mSimulationDelegate
    .mRunTimeEnabledEquations.mValues,
    systemInput->mX.mX,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mV.mX + 3,
    systemInput->mD.mX,
    daeMethodOutput->mNUMJAC_DX_LO.mX,
    &errorResult,
    neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_numJacPerturbHiBounds_method(
  const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  errorId = DynamicValidation_dfaed711_1_numJacPerturbHiBounds(
    &smData->mSimRuntimeDerivedValuesBundle,
    smData->mSimulationDelegate
    .mRunTimeEnabledEquations.mValues,
    systemInput->mX.mX,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mV.mX + 3,
    systemInput->mD.mX,
    daeMethodOutput->mNUMJAC_DX_HI.mX,
    &errorResult,
    neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_compOutputs_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  PmfMessageId errorId = NULL;
  NeDaePrivateData *smData = dae->mPrivateData;
  int derivErr = 0;
  double errorResult = 0.0;
  errorId = DynamicValidation_dfaed711_1_compOutputs(
    &smData->mSimRuntimeDerivedValuesBundle,
    smData->mSimulationDelegate
    .mRunTimeEnabledEquations.mValues,
    systemInput->mX.mX,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mV.mX + 3,
    systemInput->mD.mX,
    smData->mCachedDerivatives.mX, daeMethodOutput->mY.mX,
    &derivErr, &errorResult, neDiagMgr);
  smData->mCachedDerivativesAvailable = (derivErr == 0);
  return errorId;
}

static
  PmfMessageId dae_cg_mode_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  errorId = DynamicValidation_dfaed711_1_computeSimModeVector(
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mV.mX + 3,
    daeMethodOutput->mMODE.mX,
    &errorResult,
    neDiagMgr);
  memcpy(smData->mCachedModeVector.mX, daeMethodOutput->mMODE.mX,
         0 * sizeof(int32_T));
  return errorId;
}

static
  PmfMessageId dae_cg_zeroCrossing_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  double errorResult = 0.0;
  return
    DynamicValidation_dfaed711_1_computeZeroCrossings(
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mX.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mV.mX + 3,
    systemInput->mD.mX,
    daeMethodOutput->mZC.mX,
    &errorResult,
    neDiagMgr);
}

static
  void dae_cg_setupLoggerFcn(const NeDae *dae,
  NeLoggerBuilder *neLoggerBuilder)
{
  (void) dae;
  (void) neLoggerBuilder;
}

static
  PmfMessageId dae_cg_recordLog_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  PmRealVector *output,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  double *fullStateVector = smData->mSimulationFullStateVector.mX;
  DynamicValidation_dfaed711_1_constructStateVector(
    NULL,
    systemInput->mX.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mD.mX,
    fullStateVector);
  errorId = DynamicValidation_dfaed711_1_recordLog(
    &smData->mSimRuntimeDerivedValuesBundle,
    smData->mSimulationDelegate
    .mRunTimeEnabledEquations.mValues,
    fullStateVector,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mV.mX + 3,
    output->mX,
    &errorResult,
    neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_project_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  return
    sm_core_projectState(
    false,
    &smData->mSimulationDelegate,
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mD.mX,
    systemInput->mX.mX, neDiagMgr);
}

static
  PmfMessageId dae_cg_check_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  if (smData->mNumConstraintEqns > 0)
    errorId = sm_core_projectState(
      false,
      &smData->mSimulationDelegate,
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mM.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mD.mX,
      systemInput->mX.mX, neDiagMgr);
  if (errorId == NULL) {
    double result = 0.0;
    errorId = DynamicValidation_dfaed711_1_checkDynamics(
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mX.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mV.mX + 3,
      systemInput->mD.mX,
      systemInput->mM.mX,
      &result, neDiagMgr);
  }

  return errorId;
}

static
  PmfMessageId dae_cg_CIC_MODE_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  const size_t mvSize = smData->mModeVectorSize;
  boolean_T modeChanged = false;
  if (mvSize > 0) {
    errorId = DynamicValidation_dfaed711_1_computeSimModeVector(
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mV.mX + 3,
      systemInput->mM.mX,
      &errorResult,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;

    {
      size_t i;
      for (i = 0; i < mvSize; ++i)
        if (systemInput->mM.mX[i] != smData->mCachedModeVector.mX[i]) {
          modeChanged = true;
          break;
        }
    }
  }

  if (modeChanged) {
    errorId = sm_core_onModeChanged(
      &smData->mSimulationDelegate,
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mD.mX,
      smData->mCachedModeVector.mX,
      systemInput->mM.mX,
      systemInput->mX.mX,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;
    memcpy(smData->mCachedModeVector.mX, systemInput->mM.mX,
           0 * sizeof(int32_T));
  }

  errorId =
    sm_core_projectState(
    true,
    &smData->mSimulationDelegate,
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 3,
    systemInput->mD.mX,
    systemInput->mX.mX, neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_assemble_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  const SmMechanismDelegate *delegate = &smData->mAssemblyDelegate;
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle =
    &smData->mAsmRuntimeDerivedValuesBundle;
  PmfMessageId errorId = NULL;
  size_t i;
  double errorResult = 0.0;
  const size_t numTargets = 24;
  unsigned int asmStatus = 0;
  double *assemblyFullStateVector = smData->mAssemblyFullStateVector.mX;
  double *simulationFullStateVector = smData->mSimulationFullStateVector.mX;
  (*delegate->mSetTargets)(runtimeDerivedValuesBundle, smData->mTargets);

  {
    const double *u = systemInput->mU.mX;
    const double *uDot = u + smData->mInputVectorSize;
    CTarget *target = smData->mTargets + smData->mNumInternalTargets;
    for (i = 0; i < smData->mNumInputMotionPrimitives; ++i) {
      const size_t inputOffset = smData->mMotionInputOffsets.mX[i];
      (target++)->mValue[0] = u [inputOffset];
      (target++)->mValue[0] = uDot[inputOffset];
    }
  }

  if (smData->mAssemblyModeVector.mN > 0) {
    errorId = DynamicValidation_dfaed711_1_computeAsmModeVector(
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mV.mX + 3,
      smData->mAssemblyModeVector.mX,
      &errorResult,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;
  }

  sm_core_computeStateVector(
    delegate, runtimeDerivedValuesBundle, smData->mAssemblyModeVector.mX,
    numTargets, smData->mTargets, assemblyFullStateVector);
  asmStatus = sm_core_checkAssembly(
    delegate, runtimeDerivedValuesBundle, assemblyFullStateVector,
    smData->mAssemblyModeVector.mX,
    NULL, NULL, NULL);
  if (asmStatus != 1) {
    return sm_ssci_recordRunTimeError(
      "physmod:sm:ssci:core:dae:dae:assemblyFailure",
      asmStatus == 2 ?
      "Model not assembled. The following violation occurred: Position Violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
      :
      (asmStatus == 3 ?
       "Model not assembled. The following violation occurred: Velocity Violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
       :
       "Model not assembled. The following violation occurred: Singularity Violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."),
      neDiagMgr);
  }

#if 0

  DynamicValidation_dfaed711_1_checkTargets(
    &smData->mSimRuntimeDerivedValuesBundle,
    assemblyFullStateVector);

#endif

  if (smData->mModeVectorSize > 0) {
    errorId = DynamicValidation_dfaed711_1_computeSimModeVector(
      systemInput->mU.mX,
      systemInput->mU.mX + 3,
      systemInput->mV.mX + 3,
      systemInput->mM.mX,
      &errorResult,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;
    memcpy(smData->mCachedModeVector.mX, systemInput->mM.mX,
           0 * sizeof(int32_T));
  }

  (*delegate->mConvertStateVector)(
    NULL, runtimeDerivedValuesBundle, NULL, assemblyFullStateVector,
    smData->mAssemblyModeVector.mX, systemInput->mM.mX,
    simulationFullStateVector);
  for (i = 0; i < smData->mStateVectorSize; ++i)
    systemInput->mX.mX[i] = simulationFullStateVector[smData->
      mStateVectorMap.mX[i]];
  memcpy(systemInput->mD.mX,
         simulationFullStateVector +
         smData->mFullStateVectorSize - smData->mDiscreteStateSize,
         smData->mDiscreteStateSize * sizeof(double));
  return errorId;
}

typedef struct {
  size_t first;
  size_t second;
} SizePair;

static void checkMemAllocStatus(int_T status)
{
  (void) status;
}

static
  PmCharVector cStringToCharVector(const char *src)
{
  const size_t n = strlen(src);
  PmCharVector charVect;
  const int_T status =
    pm_create_char_vector_fields(&charVect, n + 1, pm_default_allocator());
  checkMemAllocStatus(status);
  strcpy(charVect.mX, src);
  return charVect;
}

static
  void initBasicAttributes(NeDaePrivateData *smData)
{
  size_t i;
  smData->mStateVectorSize = 12;
  smData->mFullStateVectorSize = 18;
  smData->mDiscreteStateSize = 0;
  smData->mModeVectorSize = 0;
  smData->mNumZeroCrossings = 0;
  smData->mInputVectorSize = 3;
  smData->mOutputVectorSize = 17;
  smData->mNumConstraintEqns = 10;
  smData->mFundamentalSampleTime = +0.000000000000000000e+00;
  for (i = 0; i < 4; ++i)
    smData->mChecksum[i] = 0;
}

static
  void initStateVector(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  const int32_T stateVectorMap[12] = {
    2, 3, 4, 5, 8, 9, 12, 13, 14, 15,
    16, 17
  };

  const CTarget targets[24] = {
    { 0, 40, 0, false, 0, 2, "deg", false, true, +1.000000000000000000e+00,
      false, 1, { +2.220446049250313081e-16, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 40, 0, false, 0, 2, "deg/s", true, true, +1.000000000000000000e+00,
      false, 1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 41, 0, false, 0, 1, "deg", false, true, +1.000000000000000000e+00,
      false, 1, { -7.848053195848782304e-03, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 41, 0, false, 0, 0, "1", true, true, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 42, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 42, 0, false, 0, 0, "1", true, true, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 72, 0, false, 0, 2, "deg", false, true, +1.000000000000000000e+00,
      false, 1, { +4.440892098500626162e-16, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 72, 0, false, 0, 2, "deg/s", true, true, +1.000000000000000000e+00,
      false, 1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 73, 0, false, 0, 1, "deg", false, true, +1.000000000000000000e+00,
      false, 1, { +1.562948260724305038e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 73, 0, false, 0, 0, "1", true, true, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 74, 0, false, 0, 1, "deg", false, true, +1.000000000000000000e+00,
      false, 1, { +1.567641070009739623e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 74, 0, false, 0, 0, "1", true, true, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 108, 0, false, 0, 2, "deg", false, true, +1.000000000000000000e+00,
      false, 1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 108, 0, false, 0, 2, "deg/s", true, true, +1.000000000000000000e+00,
      false, 1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 109, 0, false, 0, 1, "deg", false, true, +1.000000000000000000e+00,
      false, 1, { -2.864422827130083338e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 109, 0, false, 0, 0, "1", true, true, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 110, 0, false, 0, 0, "1", false, true, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 110, 0, false, 0, 0, "1", true, true, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 40, 0, false, 0, 3, "", false, false, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 40, 0, false, 0, 3, "", true, false, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 72, 0, false, 0, 3, "", false, false, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 72, 0, false, 0, 3, "", true, false, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 108, 0, false, 0, 3, "", false, false, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } },

    { 0, 108, 0, false, 0, 3, "", true, false, +1.000000000000000000e+00, false,
      1, { +0.000000000000000000e+00, +0.000000000000000000e+00,
        +0.000000000000000000e+00, +0.000000000000000000e+00 }, { +
        0.000000000000000000e+00 } }
  };

  const size_t numTargets = 24;
  int_T status;
  size_t i;
  status = pm_create_real_vector_fields(
    &smData->mAssemblyFullStateVector, 18, alloc);
  checkMemAllocStatus(status);
  status = pm_create_real_vector_fields(
    &smData->mSimulationFullStateVector, 18, alloc);
  checkMemAllocStatus(status);
  status = pm_create_int_vector_fields(
    &smData->mStateVectorMap, smData->mStateVectorSize, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mStateVectorMap.mX, stateVectorMap,
         smData->mStateVectorSize * sizeof(int32_T));
  smData->mNumInternalTargets = 18;
  smData->mNumInputMotionPrimitives = 3;
  PM_ALLOCATE_ARRAY(smData->mTargets, CTarget, numTargets, alloc);
  for (i = 0; i < numTargets; ++i)
    sm_compiler_CTarget_copy(targets + i, smData->mTargets + i);
}

static void initAsserts(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  int_T status = 0;
  smData->mNumParamAsserts = 0;
  smData->mParamAssertObjects = NULL;
  smData->mParamAssertPaths = NULL;
  smData->mParamAssertDescriptors = NULL;
  smData->mParamAssertMessages = NULL;
  smData->mParamAssertMessageIds = NULL;
  status = pm_create_bool_vector_fields(
    &smData->mParamAssertIsWarnings, smData->mNumParamAsserts, alloc);
  checkMemAllocStatus(status);
  if (smData->mNumParamAsserts > 0) {
    const NeAssertData *ad = DynamicValidation_dfaed711_1_assertData;
    size_t i;
    PM_ALLOCATE_ARRAY(smData->mParamAssertObjects,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertPaths,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertDescriptors,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertMessages,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertMessageIds,
                      PmCharVector, 0, alloc);
    for (i = 0; i < smData->mNumParamAsserts; ++i, ++ad) {
      smData->mParamAssertObjects [i] = cStringToCharVector(ad->mObject );
      smData->mParamAssertPaths [i] = cStringToCharVector(ad->mPath );
      smData->mParamAssertDescriptors[i] = cStringToCharVector(ad->mDescriptor);
      smData->mParamAssertMessages [i] = cStringToCharVector(ad->mMessage );
      smData->mParamAssertMessageIds [i] = cStringToCharVector(ad->mMessageID );
      smData->mParamAssertIsWarnings.mX[i] = ad->mIsWarn;
    }
  }
}

static
  void initModeVector(NeDaePrivateData *smData)
{
  {
    size_t i;
    const int_T status = pm_create_int_vector_fields(
      &smData->mAssemblyModeVector, 0,
      pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mAssemblyModeVector.mN; ++i)
      smData->mAssemblyModeVector.mX[i] = 0;
  }

  {
    size_t i;
    const int_T status = pm_create_int_vector_fields(
      &smData->mCachedModeVector, 0, pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mModeVectorSize; ++i)
      smData->mCachedModeVector.mX[i] = 0;
  }
}

static void initZeroCrossings(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  int_T status = 0;
  smData->mZeroCrossingObjects = NULL;
  smData->mZeroCrossingPaths = NULL;
  smData->mZeroCrossingDescriptors = NULL;
  status = pm_create_int_vector_fields(
    &smData->mZeroCrossingTypes, 0, alloc);
  checkMemAllocStatus(status);
  if (smData->mNumZeroCrossings > 0) {
    const NeZCData *zcd = DynamicValidation_dfaed711_1_ZCData;
    size_t i;
    PM_ALLOCATE_ARRAY(smData->mZeroCrossingObjects,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mZeroCrossingPaths,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mZeroCrossingDescriptors,
                      PmCharVector, 0, alloc);
    for (i = 0; i < smData->mNumZeroCrossings; ++i, ++zcd) {
      smData->mZeroCrossingObjects [i] = cStringToCharVector(zcd->mObject);
      smData->mZeroCrossingPaths [i] = cStringToCharVector(zcd->mPath );
      smData->mZeroCrossingDescriptors[i] = cStringToCharVector(zcd->mDescriptor);
      smData->mZeroCrossingTypes.mX[i] = zcd->mType;
    }
  }
}

static
  void initVariables(NeDaePrivateData *smData)
{
  const char *varFullPaths[12] = {
    "Dynamic_Model_PID.SPM_MODEL.LEG_1.R12.Rz.q",
    "Dynamic_Model_PID.SPM_MODEL.LEG_1.R12.Rz.w",
    "Dynamic_Model_PID.SPM_MODEL.LEG_1.R13.Rz.q",
    "Dynamic_Model_PID.SPM_MODEL.LEG_1.R13.Rz.w",
    "Dynamic_Model_PID.SPM_MODEL.LEG_2.R22.Rz.q",
    "Dynamic_Model_PID.SPM_MODEL.LEG_2.R22.Rz.w",
    "Dynamic_Model_PID.SPM_MODEL.LEG_3.R32.Rz.q",
    "Dynamic_Model_PID.SPM_MODEL.LEG_3.R32.Rz.w",
    "Dynamic_Model_PID.SPM_MODEL.LEG_2.R23.Rz.q",
    "Dynamic_Model_PID.SPM_MODEL.LEG_2.R23.Rz.w",
    "Dynamic_Model_PID.SPM_MODEL.LEG_3.R33.Rz.q",
    "Dynamic_Model_PID.SPM_MODEL.LEG_3.R33.Rz.w"
  };

  const char *varObjects[12] = {
    "DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 1/R12",
    "DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 1/R12",
    "DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 1/R13",
    "DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 1/R13",
    "DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 2/R22",
    "DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 2/R22",
    "DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 3/R32",
    "DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 3/R32",
    "DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 2/R23",
    "DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 2/R23",
    "DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 3/R33",
    "DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 3/R33"
  };

  smData->mNumVarScalars = 12;
  smData->mVarFullPaths = NULL;
  smData->mVarObjects = NULL;
  if (smData->mNumVarScalars > 0) {
    size_t s;
    PmAllocator *alloc = pm_default_allocator();
    PM_ALLOCATE_ARRAY(smData->mVarFullPaths, PmCharVector, 12, alloc);
    PM_ALLOCATE_ARRAY(smData->mVarObjects, PmCharVector, 12, alloc);
    for (s = 0; s < smData->mNumVarScalars; ++s) {
      smData->mVarFullPaths[s] = cStringToCharVector(varFullPaths[s]);
      smData->mVarObjects[s] = cStringToCharVector(varObjects[s]);
    }
  }
}

static
  void initRuntimeParameters(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  int_T status = 0;
  size_t i = 0;
  const int32_T *rtpRootVarRows = NULL;
  const int32_T *rtpRootVarCols = NULL;
  const char **rtpFullPaths = NULL;
  smData->mNumRtpRootVars = 0;
  status = pm_create_int_vector_fields(
    &smData->mRtpRootVarRows, smData->mNumRtpRootVars, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mRtpRootVarRows.mX, rtpRootVarRows,
         smData->mNumRtpRootVars * sizeof(int32_T));
  status = pm_create_int_vector_fields(
    &smData->mRtpRootVarCols, smData->mNumRtpRootVars, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mRtpRootVarCols.mX, rtpRootVarCols,
         smData->mNumRtpRootVars * sizeof(int32_T));
  smData->mRtpFullPaths = NULL;
  if (smData->mNumRtpRootVars > 0) {
    size_t v;
    PM_ALLOCATE_ARRAY(smData->mRtpFullPaths, PmCharVector, 0, alloc);
    for (v = 0; v < smData->mNumRtpRootVars; ++v) {
      smData->mRtpFullPaths[v] = cStringToCharVector(rtpFullPaths[v]);
    }
  }

  smData->mNumRuntimeRootVarScalars = 0;
  status = pm_create_real_vector_fields(
    &smData->mRuntimeParameterScalars, 0,
    alloc);
  checkMemAllocStatus(status);
  for (i = 0; i < smData->mRuntimeParameterScalars.mN; ++i)
    smData->mRuntimeParameterScalars.mX[i] = 0.0;
  sm_core_RuntimeDerivedValuesBundle_create(
    &smData->mAsmRuntimeDerivedValuesBundle,
    0,
    0);
  sm_core_RuntimeDerivedValuesBundle_create(
    &smData->mSimRuntimeDerivedValuesBundle,
    0,
    0);
}

static
  void initIoInfoHelper(
  size_t n,
  const char *portPathsSource[],
  const char *unitsSource[],
  const SizePair dimensions[],
  boolean_T doInputs,
  NeDaePrivateData *smData)
{
  PmCharVector *portPaths = NULL;
  PmCharVector *units = NULL;
  SscIoInfo *infos = NULL;
  if (n > 0) {
    size_t s;
    PmAllocator *alloc = pm_default_allocator();
    PM_ALLOCATE_ARRAY(portPaths, PmCharVector, n, alloc);
    PM_ALLOCATE_ARRAY(units, PmCharVector, n, alloc);
    PM_ALLOCATE_ARRAY(infos, SscIoInfo, n, alloc);
    for (s = 0; s < n; ++s) {
      portPaths[s] = cStringToCharVector(portPathsSource[s]);
      units[s] = cStringToCharVector(unitsSource[s]);

      {
        SscIoInfo *info = infos + s;
        info->mName = info->mIdentifier = portPaths[s].mX;
        ssc_array_size_set_scalar(&info->mSize);
        ssc_array_size_set_dim(&info->mSize, 0, dimensions[s].first);
        ssc_array_size_set_dim(&info->mSize, 1, dimensions[s].second);
        info->mUnit = units[s].mX;
      }
    }
  }

  if (doInputs) {
    smData->mNumInputs = n;
    smData->mInputPortPaths = portPaths;
    smData->mInputUnits = units;
    smData->mInputInfos = infos;
  } else {
    smData->mNumOutputs = n;
    smData->mOutputPortPaths = portPaths;
    smData->mOutputUnits = units;
    smData->mOutputInfos = infos;
  }
}

static
  void initIoInfo(NeDaePrivateData *smData)
{
  const char *inputPortPaths[3] = {
    "Dynamic_Model_PID.SPM_MODEL.LEG_1.R11.qi",
    "Dynamic_Model_PID.SPM_MODEL.LEG_2.R21.qi",
    "Dynamic_Model_PID.SPM_MODEL.LEG_3.R31.qi"
  };

  const char *inputUnits[3] = {
    "rad",
    "rad",
    "rad"
  };

  const SizePair inputDimensions[3] = {
    { 1, 1 }, { 1, 1 }, { 1, 1 }
  };

  const char *outputPortPaths[14] = {
    "Dynamic_Model_PID.SPM_MODEL.LEG_1.R11.q",
    "Dynamic_Model_PID.SPM_MODEL.LEG_1.R11.w",
    "Dynamic_Model_PID.SPM_MODEL.LEG_1.R11.t",
    "Dynamic_Model_PID.SPM_MODEL.LEG_2.R21.q",
    "Dynamic_Model_PID.SPM_MODEL.LEG_2.R21.w",
    "Dynamic_Model_PID.SPM_MODEL.LEG_2.R21.t",
    "Dynamic_Model_PID.SPM_MODEL.LEG_3.R31.q",
    "Dynamic_Model_PID.SPM_MODEL.LEG_3.R31.w",
    "Dynamic_Model_PID.SPM_MODEL.LEG_3.R31.t",
    "Dynamic_Model_PID.SPM_MODEL.LEG_3.PROXIMAL3.Transform_Sensor.q",
    "Dynamic_Model_PID.SPM_MODEL.PLATFORM.EA_MEAS.Q",
    "Dynamic_Model_PID.SPM_MODEL.PLATFORM.W_MEAS.wx",
    "Dynamic_Model_PID.SPM_MODEL.PLATFORM.W_MEAS.wy",
    "Dynamic_Model_PID.SPM_MODEL.PLATFORM.W_MEAS.wz"
  };

  const char *outputUnits[14] = {
    "rad",
    "rad/s",
    "kg*m^2/s^2",
    "rad",
    "rad/s",
    "kg*m^2/s^2",
    "rad",
    "rad/s",
    "kg*m^2/s^2",
    "rad",
    "1",
    "rad/s",
    "rad/s",
    "rad/s"
  };

  const SizePair outputDimensions[14] = {
    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },

    { 1, 1 }, { 1, 1 }, { 1, 1 }, { 1, 1 },

    { 1, 1 }, { 1, 1 }, { 4, 1 }, { 1, 1 },

    { 1, 1 }, { 1, 1 }
  };

  initIoInfoHelper(3, inputPortPaths, inputUnits, inputDimensions,
                   true, smData);
  initIoInfoHelper(14, outputPortPaths, outputUnits, outputDimensions,
                   false, smData);
}

static
  void initInputDerivs(NeDaePrivateData *smData)
{
  const int32_T numInputDerivs[3] = {
    2, 2, 2
  };

  PmAllocator *alloc = pm_default_allocator();
  const int_T status = pm_create_int_vector_fields(
    &smData->mNumInputDerivs, smData->mInputVectorSize, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mNumInputDerivs.mX, numInputDerivs,
         3 * sizeof(int32_T));
  smData->mInputOrder = 2;
}

static
  void initDirectFeedthrough(NeDaePrivateData *smData)
{
  const boolean_T directFeedthroughVector[3] = {
    true, true, true
  };

  const boolean_T directFeedthroughMatrix[102] = {
    true, true, true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true,
    true, true, true, true, true, true, true, true, true, true,
    true, true
  };

  PmAllocator *alloc = pm_default_allocator();

  {
    const int_T status = pm_create_bool_vector_fields(
      &smData->mDirectFeedthroughVector, 3, alloc);
    checkMemAllocStatus(status);
    memcpy(smData->mDirectFeedthroughVector.mX, directFeedthroughVector,
           3 * sizeof(boolean_T));
  }

  {
    const int_T status = pm_create_bool_vector_fields(
      &smData->mDirectFeedthroughMatrix, 102, alloc);
    checkMemAllocStatus(status);
    memcpy(smData->mDirectFeedthroughMatrix.mX, directFeedthroughMatrix,
           102 * sizeof(boolean_T));
  }
}

static
  void initOutputDerivProc(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  const int32_T outputFunctionMap[17] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0
  };

  smData->mOutputFunctionMap = pm_create_int_vector(17, alloc);
  memcpy(smData->mOutputFunctionMap->mX, outputFunctionMap,
         17 * sizeof(int32_T));
  smData->mNumOutputClasses = 1;
  smData->mHasKinematicOutputs = true;
  smData->mHasDynamicOutputs = true;
  smData->mIsOutputClass0Dynamic = false;
  smData->mDoComputeDynamicOutputs = false;
  smData->mCachedDerivativesAvailable = false;

  {
    size_t i = 0;
    const int_T status = pm_create_real_vector_fields(
      &smData->mCachedDerivatives, 12, pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mCachedDerivatives.mN; ++i)
      smData->mCachedDerivatives.mX[i] = 0.0;
  }
}

#if 0

static void initializeSizePairVector(const SmSizePair *data,
  SmSizePairVector *vector)
{
  const size_t n = sm_core_SmSizePairVector_size(vector);
  size_t i;
  for (i = 0; i < n; ++i, ++data)
    sm_core_SmSizePairVector_setValue(vector, i, data++);
}

#endif

static
  void initAssemblyDelegate(SmMechanismDelegate *delegate)
{
  SmMechanismDelegateScratchpad *scratchpad = NULL;
  const SmSizePair jointToStageIdx[7] = {
    { 40, 0 }, { 41, 1 }, { 72, 2 }, { 73, 3 }, { 74, 4 }, { 108, 5 },

    { 109, 6 }
  };

  const size_t primitiveIndices[7 + 1] = {
    0, 1, 2, 3, 4, 5, 6, 7
  };

  const SmSizePair stateOffsets[7] = {
    { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }, { 8, 9 }, { 10, 11 },

    { 12, 13 }
  };

  const SmSizePair dofOffsets[7] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 5 }, { 5, 6 },

    { 6, 7 }
  };

  const size_t *flexibleStages = NULL;
  const size_t remodIndices[7] = {
    0, 2, 4, 6, 8, 10, 12
  };

  const size_t equationsPerConstraint[2] = {
    5, 5
  };

  const int32_T hasAllVelocityDisabledEquations[2] = {
    0, 0
  };

  const int32_T runtimeEnabledEquations[10] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1
  };

  const size_t dofToVelSlot[7] = {
    1, 3, 5, 7, 9, 11, 13
  };

  const size_t constraintDofs[10] = {
    0, 1, 2, 3, 4, 5, 6, 2, 3, 4
  };

  const size_t constraintDofOffsets[2 + 1] = {
    0, 5, 10
  };

  const size_t Jm = 10;
  const size_t Jn = 7;
  SmSizePair zeroSizePair;
  zeroSizePair.mFirst = zeroSizePair.mSecond = 0;
  sm_core_MechanismDelegate_allocScratchpad(delegate);
  scratchpad = delegate->mScratchpad;
  delegate->mTargetStrengthFree = 0;
  delegate->mTargetStrengthSuggested = 1;
  delegate->mTargetStrengthDesired = 2;
  delegate->mTargetStrengthRequired = 3;
  delegate->mConsistencyTol = +1.000000000000000082e-05;
  delegate->mTreeJointDof = 7;
  delegate->mDof = 7;
  delegate->mStateSize = 18;
  delegate->mContinuousStateSize = 18;
  delegate->mModeVectorSize = 0;
  delegate->mNumStages = 7;
  delegate->mNumConstraints = 2;
  delegate->mNumAllConstraintEquations = 10;
  sm_core_SmSizePairVector_create(
    &delegate->mJointToStageIdx, 7, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mJointToStageIdx),
         jointToStageIdx, 7 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mPrimitiveIndices, delegate->mNumStages + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mPrimitiveIndices),
         primitiveIndices, (delegate->mNumStages + 1) * sizeof(size_t));
  sm_core_SmSizePairVector_create(
    &delegate->mStateOffsets, 7, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mStateOffsets),
         stateOffsets, 7 * sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mDofOffsets, 7, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mDofOffsets),
         dofOffsets, 7 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mFlexibleStages, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mFlexibleStages),
         flexibleStages, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mRemodIndices, 7, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mRemodIndices),
         remodIndices, 7 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mEquationsPerConstraint, delegate->mNumConstraints, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mEquationsPerConstraint),
         equationsPerConstraint, delegate->mNumConstraints * sizeof(size_t));
  sm_core_SmIntVector_create(
    &delegate->mHasAllVelocityDisabledEquations, delegate->mNumConstraints, 0);
  memcpy(
         sm_core_SmIntVector_nonConstValues
         (&delegate->mHasAllVelocityDisabledEquations),
         hasAllVelocityDisabledEquations, delegate->mNumConstraints * sizeof
         (int32_T));
  sm_core_SmIntVector_create(
    &delegate->mRunTimeEnabledEquations,
    delegate->mNumAllConstraintEquations, 0);
  memcpy(
         sm_core_SmIntVector_nonConstValues(&delegate->mRunTimeEnabledEquations),
         runtimeEnabledEquations, delegate->mNumAllConstraintEquations *
         sizeof(int32_T));
  sm_core_SmSizeTVector_create(
    &delegate->mDofToVelSlot, delegate->mDof, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mDofToVelSlot),
         dofToVelSlot, delegate->mDof * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofs, 10, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofs),
         constraintDofs, 10 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofOffsets, delegate->mNumConstraints + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofOffsets),
         constraintDofOffsets, (delegate->mNumConstraints + 1) * sizeof(size_t));
  sm_core_SmBoundedSet_create(&scratchpad->mPosRequired, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mPosDesired, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggested, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mPosFree, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mPosNonRequired, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggAndFree, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mVelRequired, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mVelDesired, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggested, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mVelFree, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mVelNonRequired, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggAndFree, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mConstraintFilter, 2);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveConstraints, 2);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs0, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mNewConstraints, 2);
  sm_core_SmBoundedSet_create(&scratchpad->mNewDofs, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mUnsatisfiedConstraints, 2);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveConstraintsVect,
    2, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveDofsVect, 7, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mFullDofToActiveDof, 7, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyPosTargetedPrims, 7, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyVelTargetedPrims, 7, &zeroSizePair);
  sm_core_SmSizeTVector_create(&scratchpad->mPosPartialTypes, 7, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mVelPartialTypes, 7, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mPartiallyActivePrims, 7, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mBaseFrameVelOffsets, 0, &zeroSizePair);
  sm_core_SmSizePairVector_create(&scratchpad->mCvQuaternionVelOffsets,
    0,
    &zeroSizePair);
  sm_core_SmRealVector_create(&scratchpad->mCvQuaternionAzimuthValues,
    0, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mInitialState, 18, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mStartState, 18, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mTestState, 18, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mFullStateVector, 18, 0.0);
  sm_core_SmIntVector_create(&scratchpad->mModeVector, 0, 0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianRowMaj, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobian, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianPrimSubmatrix, Jm * 6, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintNonhomoTerms, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mBestConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mDeltas,
    Jn * (Jm <= Jn ? Jm : Jn), 0.0);
  sm_core_SmRealVector_create(&scratchpad->mSvdWork, 287, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchScaledDeltaVect, 7, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchTestStateVect, 18, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mLineSearchErrorVect, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mActiveDofVelsVect, 7, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mVelSystemRhs, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mMotionData, 126, 0.0);
  delegate->mSetTargets = DynamicValidation_dfaed711_1_setTargets;
  delegate->mResetStateVector = DynamicValidation_dfaed711_1_resetAsmStateVector;
  delegate->mInitializeTrackedAngleState =
    DynamicValidation_dfaed711_1_initializeTrackedAngleState;
  delegate->mComputeDiscreteState =
    DynamicValidation_dfaed711_1_computeDiscreteState;
  delegate->mAdjustPosition = DynamicValidation_dfaed711_1_adjustPosition;
  delegate->mPerturbJointPrimitiveState =
    DynamicValidation_dfaed711_1_perturbAsmJointPrimitiveState;
  delegate->mPerturbFlexibleBodyState = NULL;
  delegate->mComputePosDofBlendMatrix =
    DynamicValidation_dfaed711_1_computePosDofBlendMatrix;
  delegate->mComputeVelDofBlendMatrix =
    DynamicValidation_dfaed711_1_computeVelDofBlendMatrix;
  delegate->mProjectPartiallyTargetedPos =
    DynamicValidation_dfaed711_1_projectPartiallyTargetedPos;
  delegate->mPropagateMotion = DynamicValidation_dfaed711_1_propagateMotion;
  delegate->mComputeAssemblyError =
    DynamicValidation_dfaed711_1_computeAssemblyError;
  delegate->mComputeAssemblyJacobian =
    DynamicValidation_dfaed711_1_computeAssemblyJacobian;
  delegate->mComputeFullAssemblyJacobian =
    DynamicValidation_dfaed711_1_computeFullAssemblyJacobian;
  delegate->mIsInKinematicSingularity =
    DynamicValidation_dfaed711_1_isInKinematicSingularity;
  delegate->mConvertStateVector =
    DynamicValidation_dfaed711_1_convertStateVector;
  delegate->mConstructStateVector = NULL;
  delegate->mExtractSolverStateVector = NULL;
  delegate->mIsPositionViolation = NULL;
  delegate->mIsVelocityViolation = NULL;
  delegate->mProjectStateSim = NULL;
  delegate->mComputeConstraintError = NULL;
  delegate->mResetModeVector = NULL;
  delegate->mHasJointDisToNormModeChange = NULL;
  delegate->mPerformJointDisToNormModeChange = NULL;
  delegate->mOnModeChangedCutJoints = NULL;
  delegate->mMech = NULL;
}

static
  void initSimulationDelegate(SmMechanismDelegate *delegate)
{
  SmMechanismDelegateScratchpad *scratchpad = NULL;
  const SmSizePair jointToStageIdx[7] = {
    { 40, 0 }, { 41, 1 }, { 42, 2 }, { 72, 3 }, { 73, 4 }, { 108, 5 },

    { 109, 6 }
  };

  const size_t primitiveIndices[7 + 1] = {
    0, 1, 2, 3, 4, 5, 6, 7
  };

  const SmSizePair stateOffsets[7] = {
    { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }, { 8, 9 }, { 10, 11 },

    { 12, 13 }
  };

  const SmSizePair dofOffsets[7] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 5 }, { 5, 6 },

    { 6, 7 }
  };

  const size_t *flexibleStages = NULL;
  const size_t remodIndices[7] = {
    0, 2, 4, 6, 8, 10, 12
  };

  const size_t equationsPerConstraint[2] = {
    5, 5
  };

  const int32_T hasAllVelocityDisabledEquations[2] = {
    0, 0
  };

  const int32_T runtimeEnabledEquations[10] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1
  };

  const size_t dofToVelSlot[7] = {
    1, 3, 5, 7, 9, 11, 13
  };

  const size_t constraintDofs[10] = {
    3, 4, 0, 1, 2, 5, 6, 0, 1, 2
  };

  const size_t constraintDofOffsets[2 + 1] = {
    0, 5, 10
  };

  const size_t Jm = 10;
  const size_t Jn = 7;
  SmSizePair zeroSizePair;
  zeroSizePair.mFirst = zeroSizePair.mSecond = 0;
  sm_core_MechanismDelegate_allocScratchpad(delegate);
  scratchpad = delegate->mScratchpad;
  delegate->mTargetStrengthFree = 0;
  delegate->mTargetStrengthSuggested = 1;
  delegate->mTargetStrengthDesired = 2;
  delegate->mTargetStrengthRequired = 3;
  delegate->mConsistencyTol = +1.000000000000000082e-05;
  delegate->mTreeJointDof = 7;
  delegate->mDof = 7;
  delegate->mStateSize = 18;
  delegate->mContinuousStateSize = 18;
  delegate->mModeVectorSize = 0;
  delegate->mNumStages = 7;
  delegate->mNumConstraints = 2;
  delegate->mNumAllConstraintEquations = 10;
  sm_core_SmSizePairVector_create(
    &delegate->mJointToStageIdx, 7, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mJointToStageIdx),
         jointToStageIdx, 7 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mPrimitiveIndices, delegate->mNumStages + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mPrimitiveIndices),
         primitiveIndices, (delegate->mNumStages + 1) * sizeof(size_t));
  sm_core_SmSizePairVector_create(
    &delegate->mStateOffsets, 7, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mStateOffsets),
         stateOffsets, 7 * sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mDofOffsets, 7, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mDofOffsets),
         dofOffsets, 7 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mFlexibleStages, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mFlexibleStages),
         flexibleStages, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mRemodIndices, 7, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mRemodIndices),
         remodIndices, 7 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mEquationsPerConstraint, delegate->mNumConstraints, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mEquationsPerConstraint),
         equationsPerConstraint, delegate->mNumConstraints * sizeof(size_t));
  sm_core_SmIntVector_create(
    &delegate->mHasAllVelocityDisabledEquations, delegate->mNumConstraints, 0);
  memcpy(sm_core_SmIntVector_nonConstValues
         (&delegate->mHasAllVelocityDisabledEquations),
         hasAllVelocityDisabledEquations, delegate->mNumConstraints * sizeof
         (int32_T));
  sm_core_SmIntVector_create(
    &delegate->mRunTimeEnabledEquations,
    delegate->mNumAllConstraintEquations, 0);
  memcpy(
         sm_core_SmIntVector_nonConstValues(&delegate->mRunTimeEnabledEquations),
         runtimeEnabledEquations, delegate->mNumAllConstraintEquations *
         sizeof(int32_T));
  sm_core_SmSizeTVector_create(
    &delegate->mDofToVelSlot, delegate->mDof, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mDofToVelSlot),
         dofToVelSlot, delegate->mDof * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofs, 10, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofs),
         constraintDofs, 10 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofOffsets, delegate->mNumConstraints + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofOffsets),
         constraintDofOffsets, (delegate->mNumConstraints + 1) * sizeof(size_t));
  sm_core_SmBoundedSet_create(&scratchpad->mPosRequired, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mPosDesired, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggested, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mPosFree, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mPosNonRequired, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggAndFree, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mVelRequired, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mVelDesired, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggested, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mVelFree, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mVelNonRequired, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggAndFree, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mConstraintFilter, 2);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveConstraints, 2);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs0, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mNewConstraints, 2);
  sm_core_SmBoundedSet_create(&scratchpad->mNewDofs, 7);
  sm_core_SmBoundedSet_create(&scratchpad->mUnsatisfiedConstraints, 2);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveConstraintsVect,
    2, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveDofsVect, 7, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mFullDofToActiveDof, 7, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyPosTargetedPrims, 7, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyVelTargetedPrims, 7, &zeroSizePair);
  sm_core_SmSizeTVector_create(&scratchpad->mPosPartialTypes, 7, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mVelPartialTypes, 7, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mPartiallyActivePrims, 7, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mBaseFrameVelOffsets, 0, &zeroSizePair);
  sm_core_SmSizePairVector_create(&scratchpad->mCvQuaternionVelOffsets,
    0,
    &zeroSizePair);
  sm_core_SmRealVector_create(&scratchpad->mCvQuaternionAzimuthValues,
    0, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mInitialState, 18, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mStartState, 18, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mTestState, 18, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mFullStateVector, 18, 0.0);
  sm_core_SmIntVector_create(&scratchpad->mModeVector, 0, 0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianRowMaj, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobian, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianPrimSubmatrix, Jm * 6, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintNonhomoTerms, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mBestConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mDeltas,
    Jn * (Jm <= Jn ? Jm : Jn), 0.0);
  sm_core_SmRealVector_create(&scratchpad->mSvdWork, 287, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchScaledDeltaVect, 7, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchTestStateVect, 18, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mLineSearchErrorVect, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mActiveDofVelsVect, 7, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mVelSystemRhs, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mMotionData, 126, 0.0);
  delegate->mSetTargets = NULL;
  delegate->mResetStateVector = DynamicValidation_dfaed711_1_resetSimStateVector;
  delegate->mInitializeTrackedAngleState = NULL;
  delegate->mComputeDiscreteState = NULL;
  delegate->mAdjustPosition = NULL;
  delegate->mPerturbJointPrimitiveState =
    DynamicValidation_dfaed711_1_perturbSimJointPrimitiveState;
  delegate->mPerturbFlexibleBodyState =
    DynamicValidation_dfaed711_1_perturbFlexibleBodyState;
  delegate->mComputePosDofBlendMatrix = NULL;
  delegate->mComputeVelDofBlendMatrix = NULL;
  delegate->mProjectPartiallyTargetedPos = NULL;
  delegate->mPropagateMotion = NULL;
  delegate->mComputeAssemblyError = NULL;
  delegate->mComputeAssemblyJacobian = NULL;
  delegate->mComputeFullAssemblyJacobian = NULL;
  delegate->mIsInKinematicSingularity = NULL;
  delegate->mConvertStateVector = NULL;
  delegate->mConstructStateVector =
    DynamicValidation_dfaed711_1_constructStateVector;
  delegate->mExtractSolverStateVector =
    DynamicValidation_dfaed711_1_extractSolverStateVector;
  delegate->mIsPositionViolation =
    DynamicValidation_dfaed711_1_isPositionViolation;
  delegate->mIsVelocityViolation =
    DynamicValidation_dfaed711_1_isVelocityViolation;
  delegate->mProjectStateSim = DynamicValidation_dfaed711_1_projectStateSim;
  delegate->mComputeConstraintError =
    DynamicValidation_dfaed711_1_computeConstraintError;
  delegate->mResetModeVector = DynamicValidation_dfaed711_1_resetModeVector;
  delegate->mHasJointDisToNormModeChange =
    DynamicValidation_dfaed711_1_hasJointDisToNormModeChange;
  delegate->mPerformJointDisToNormModeChange =
    DynamicValidation_dfaed711_1_performJointDisToNormModeChange;
  delegate->mOnModeChangedCutJoints =
    DynamicValidation_dfaed711_1_onModeChangedCutJoints;
  delegate->mMech = NULL;
}

static
  void initMechanismDelegates(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  const int32_T motionInputOffsets[3] = {
    0, 1, 2
  };

  int_T status = 0;
  initAssemblyDelegate(&smData->mAssemblyDelegate);
  initSimulationDelegate(&smData->mSimulationDelegate);
  status = pm_create_int_vector_fields(
    &smData->mMotionInputOffsets, smData->mNumInputMotionPrimitives, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mMotionInputOffsets.mX, motionInputOffsets,
         3 * sizeof(int32_T));
}

static
  void initComputationFcnPtrs(NeDaePrivateData *smData)
{
  smData->mSetParametersFcn = dae_cg_setParameters_function;
  smData->mPAssertFcn = dae_cg_pAssert_method;
  smData->mDerivativeFcn = dae_cg_deriv_method;
  smData->mNumJacPerturbLoBoundsFcn = dae_cg_numJacPerturbLoBounds_method;
  smData->mNumJacPerturbHiBoundsFcn = dae_cg_numJacPerturbHiBounds_method;
  smData->mOutputFcn = dae_cg_compOutputs_method;
  smData->mModeFcn = dae_cg_mode_method;
  smData->mZeroCrossingFcn = dae_cg_zeroCrossing_method;
  smData->mProjectionFcn = dae_cg_project_solve;
  smData->mCIC_MODE_Fcn = dae_cg_CIC_MODE_solve;
  smData->mCheckFcn =
    (smData->mStateVectorSize == 0) ? dae_cg_check_solve : NULL;
  smData->mAssemblyFcn = dae_cg_assemble_solve;
  smData->mSetupLoggerFcn = dae_cg_setupLoggerFcn;
  smData->mLogFcn = dae_cg_recordLog_method;
  smData->mResidualsFcn = NULL;
  smData->mLinearizeFcn = NULL;
  smData->mGenerateFcn = NULL;
}

static
  void initLiveLinkToSm(NeDaePrivateData *smData)
{
  smData->mLiveSmLink = NULL;
  smData->mLiveSmLink_destroy = NULL;
  smData->mLiveSmLink_copy = NULL;
}

void DynamicValidation_dfaed711_1_NeDaePrivateData_create(NeDaePrivateData
  *smData)
{
  initBasicAttributes (smData);
  initStateVector (smData);
  initAsserts (smData);
  initModeVector (smData);
  initZeroCrossings (smData);
  initVariables (smData);
  initRuntimeParameters (smData);
  initIoInfo (smData);
  initInputDerivs (smData);
  initDirectFeedthrough (smData);
  initOutputDerivProc (smData);
  initMechanismDelegates (smData);
  initComputationFcnPtrs (smData);
  initLiveLinkToSm (smData);
}
