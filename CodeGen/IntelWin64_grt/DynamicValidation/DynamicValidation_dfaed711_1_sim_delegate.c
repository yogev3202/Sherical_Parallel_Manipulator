/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'DynamicValidation/Dynamic_Model_PID/SPM MODEL/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ne_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"

void DynamicValidation_dfaed711_1_resetSimStateVector(const void *mech, double
  *state)
{
  double xx[1];
  (void) mech;
  xx[0] = 0.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = xx[0];
  state[4] = xx[0];
  state[5] = xx[0];
  state[6] = xx[0];
  state[7] = xx[0];
  state[8] = xx[0];
  state[9] = xx[0];
  state[10] = xx[0];
  state[11] = xx[0];
  state[12] = xx[0];
  state[13] = xx[0];
  state[14] = xx[0];
  state[15] = xx[0];
  state[16] = xx[0];
  state[17] = xx[0];
}

static void perturbSimJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbSimJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[1] = state[1] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_1_0(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbSimJointPrimitiveState_1_0v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[3] = state[3] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_2_0(double mag, double *state)
{
  state[4] = state[4] + mag;
}

static void perturbSimJointPrimitiveState_2_0v(double mag, double *state)
{
  state[4] = state[4] + mag;
  state[5] = state[5] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_3_0(double mag, double *state)
{
  state[6] = state[6] + mag;
}

static void perturbSimJointPrimitiveState_3_0v(double mag, double *state)
{
  state[6] = state[6] + mag;
  state[7] = state[7] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_4_0(double mag, double *state)
{
  state[8] = state[8] + mag;
}

static void perturbSimJointPrimitiveState_4_0v(double mag, double *state)
{
  state[8] = state[8] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_5_0(double mag, double *state)
{
  state[10] = state[10] + mag;
}

static void perturbSimJointPrimitiveState_5_0v(double mag, double *state)
{
  state[10] = state[10] + mag;
  state[11] = state[11] - 0.875 * mag;
}

static void perturbSimJointPrimitiveState_6_0(double mag, double *state)
{
  state[12] = state[12] + mag;
}

static void perturbSimJointPrimitiveState_6_0v(double mag, double *state)
{
  state[12] = state[12] + mag;
  state[13] = state[13] - 0.875 * mag;
}

void DynamicValidation_dfaed711_1_perturbSimJointPrimitiveState(const void *mech,
  size_t stageIdx, size_t primIdx, double mag, boolean_T doPerturbVelocity,
  double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch ((stageIdx * 6 + primIdx) * 2 + (doPerturbVelocity ? 1 : 0))
  {
   case 0:
    perturbSimJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbSimJointPrimitiveState_0_0v(mag, state);
    break;

   case 12:
    perturbSimJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbSimJointPrimitiveState_1_0v(mag, state);
    break;

   case 24:
    perturbSimJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbSimJointPrimitiveState_2_0v(mag, state);
    break;

   case 36:
    perturbSimJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbSimJointPrimitiveState_3_0v(mag, state);
    break;

   case 48:
    perturbSimJointPrimitiveState_4_0(mag, state);
    break;

   case 49:
    perturbSimJointPrimitiveState_4_0v(mag, state);
    break;

   case 60:
    perturbSimJointPrimitiveState_5_0(mag, state);
    break;

   case 61:
    perturbSimJointPrimitiveState_5_0v(mag, state);
    break;

   case 72:
    perturbSimJointPrimitiveState_6_0(mag, state);
    break;

   case 73:
    perturbSimJointPrimitiveState_6_0v(mag, state);
    break;
  }
}

void DynamicValidation_dfaed711_1_perturbFlexibleBodyState(const void *mech,
  size_t stageIdx, double mag, boolean_T doPerturbVelocity, double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch (stageIdx * 2 + (doPerturbVelocity ? 1 : 0))
  {
  }
}

void DynamicValidation_dfaed711_1_constructStateVector(const void *mech, const
  double *solverState, const double *u, const double *uDot, double
  *discreteState, double *fullState)
{
  (void) mech;
  (void) discreteState;
  fullState[0] = u[0];
  fullState[1] = uDot[0];
  fullState[2] = solverState[0];
  fullState[3] = solverState[1];
  fullState[4] = solverState[2];
  fullState[5] = solverState[3];
  fullState[6] = u[1];
  fullState[7] = uDot[1];
  fullState[8] = solverState[4];
  fullState[9] = solverState[5];
  fullState[10] = u[2];
  fullState[11] = uDot[2];
  fullState[12] = solverState[6];
  fullState[13] = solverState[7];
  fullState[14] = solverState[8];
  fullState[15] = solverState[9];
  fullState[16] = solverState[10];
  fullState[17] = solverState[11];
}

void DynamicValidation_dfaed711_1_extractSolverStateVector(const void *mech,
  const double *fullState, double *solverState)
{
  (void) mech;
  solverState[0] = fullState[2];
  solverState[1] = fullState[3];
  solverState[2] = fullState[4];
  solverState[3] = fullState[5];
  solverState[4] = fullState[8];
  solverState[5] = fullState[9];
  solverState[6] = fullState[12];
  solverState[7] = fullState[13];
  solverState[8] = fullState[14];
  solverState[9] = fullState[15];
  solverState[10] = fullState[16];
  solverState[11] = fullState[17];
}

boolean_T DynamicValidation_dfaed711_1_isPositionViolation(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const
  double *state, const int *modeVector)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  int ii[1];
  double xx[75];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) modeVector;
  xx[0] = 0.6190277160392075;
  xx[1] = - 0.2167287412203158;
  xx[2] = 0.7124684524173637;
  xx[3] = 0.2494434681733462;
  xx[4] = 0.5;
  xx[5] = xx[4] * state[0];
  xx[6] = sin(xx[5]);
  xx[7] = cos(xx[5]);
  xx[8] = 5.906528705024172e-8 * xx[6];
  xx[9] = 0.7816132178294208 * xx[6];
  xx[10] = 0.6237633988254963 * xx[6];
  pm_math_Quaternion_compose_ra(xx + 0, xx + 7, xx + 11);
  xx[0] = - 0.9578985788971721;
  xx[1] = 0.01596084651607334;
  xx[2] = 0.2160322563859199;
  xx[3] = 0.188429371718097;
  xx[5] = xx[4] * state[2];
  xx[6] = sin(xx[5]);
  xx[7] = cos(xx[5]);
  xx[8] = - (0.503898206037895 * xx[6]);
  xx[9] = 0.8089781762292558 * xx[6];
  xx[10] = - (0.3027224939388848 * xx[6]);
  pm_math_Quaternion_compose_ra(xx + 0, xx + 7, xx + 15);
  pm_math_Quaternion_compose_ra(xx + 11, xx + 15, xx + 0);
  xx[5] = - 0.7391371617538293;
  xx[6] = - 0.2713596951978788;
  xx[7] = - 0.5768318010060487;
  xx[8] = - 0.2174976902974528;
  xx[9] = xx[4] * state[4];
  xx[10] = sin(xx[9]);
  xx[19] = cos(xx[9]);
  xx[20] = - (0.3618351326747742 * xx[10]);
  xx[21] = 0.342020353294726 * xx[10];
  xx[22] = - (0.8672355012880755 * xx[10]);
  pm_math_Quaternion_compose_ra(xx + 5, xx + 19, xx + 23);
  pm_math_Quaternion_compose_ra(xx + 0, xx + 23, xx + 5);
  xx[19] = 0.1645059501899856;
  xx[20] = 0.932950220907751;
  xx[21] = - 0.05560783598731543;
  xx[22] = 0.3153560626293339;
  pm_math_Quaternion_compose_ra(xx + 5, xx + 19, xx + 27);
  xx[9] = 2.0;
  xx[10] = 1.0;
  xx[19] = (xx[28] * xx[30] + xx[27] * xx[29]) * xx[9];
  xx[20] = xx[9] * (xx[29] * xx[30] - xx[27] * xx[28]);
  xx[21] = xx[10] - (xx[28] * xx[28] + xx[29] * xx[29]) * xx[9];
  xx[27] = 0.324402019972231;
  xx[28] = 0.1793201049669474;
  xx[29] = 0.8745212078931046;
  xx[30] = 0.3127623480180516;
  xx[22] = xx[4] * state[6];
  xx[31] = sin(xx[22]);
  xx[32] = cos(xx[22]);
  xx[33] = 0.5165599445270539 * xx[31];
  xx[34] = 0.7400480272337564 * xx[31];
  xx[35] = 0.4306910041986394 * xx[31];
  pm_math_Quaternion_compose_ra(xx + 27, xx + 32, xx + 36);
  xx[27] = - 0.8846589215343961;
  xx[28] = - 0.3902684207978444;
  xx[29] = 0.2533894697105629;
  xx[30] = - 0.02937565177755097;
  xx[22] = xx[4] * state[8];
  xx[31] = sin(xx[22]);
  xx[32] = cos(xx[22]);
  xx[33] = - (0.5038982060378953 * xx[31]);
  xx[34] = 0.8089781762292564 * xx[31];
  xx[35] = - (0.3027224939388842 * xx[31]);
  pm_math_Quaternion_compose_ra(xx + 27, xx + 32, xx + 40);
  pm_math_Quaternion_compose_ra(xx + 36, xx + 40, xx + 27);
  xx[22] = 0.300048825040154;
  xx[31] = 0.7582880608547207;
  xx[32] = xx[22];
  xx[33] = xx[31];
  xx[34] = 0.2970285731203685;
  xx[35] = 0.4967332746125315;
  pm_math_Quaternion_compose_ra(xx + 27, xx + 32, xx + 44);
  xx[32] = xx[47] * xx[47];
  xx[33] = xx[45] * xx[46];
  xx[34] = xx[44] * xx[47];
  xx[48] = xx[10] - (xx[46] * xx[46] + xx[32]) * xx[9];
  xx[49] = (xx[33] + xx[34]) * xx[9];
  xx[50] = xx[9] * (xx[45] * xx[47] - xx[44] * xx[46]);
  xx[51] = xx[9] * (xx[33] - xx[34]);
  xx[52] = xx[10] - (xx[45] * xx[45] + xx[32]) * xx[9];
  xx[53] = (xx[46] * xx[47] + xx[44] * xx[45]) * xx[9];
  xx[32] = 0.01027766876427663;
  xx[33] = 1.904031651085619e-3;
  xx[34] = - 0.01346562362772443;
  pm_math_Quaternion_xform_ra(xx + 5, xx + 32, xx + 44);
  xx[32] = - 3.265857732358156e-3;
  xx[33] = 0.01115673029435595;
  xx[34] = - 7.827670717550887e-3;
  pm_math_Quaternion_xform_ra(xx + 23, xx + 32, xx + 54);
  xx[23] = - (0.02695619245963533 + xx[54]);
  xx[24] = 0.01378863199166597 - xx[55];
  xx[25] = 3.369121440618853e-3 - xx[56];
  pm_math_Quaternion_xform_ra(xx + 0, xx + 23, xx + 32);
  xx[0] = - 6.076258344851112e-3;
  xx[1] = - 7.677368824970993e-3;
  xx[2] = 7.914929897510211e-3;
  pm_math_Quaternion_xform_ra(xx + 15, xx + 0, xx + 23);
  xx[0] = 1.706386300196451e-9 - xx[23];
  xx[1] = 0.01104873622831247 - xx[24];
  xx[2] = 0.01832347446050686 - xx[25];
  pm_math_Quaternion_xform_ra(xx + 11, xx + 0, xx + 15);
  xx[0] = - 2.009348425691778e-9;
  xx[1] = 3.882107956182032e-4;
  xx[2] = - 0.01888260476507436;
  pm_math_Quaternion_xform_ra(xx + 11, xx + 0, xx + 23);
  xx[0] = xx[32] + xx[15] - xx[23];
  xx[1] = - 1.754291965803085e-3;
  xx[2] = 9.46128575715573e-3;
  xx[3] = - 5.462379018955248e-3;
  pm_math_Quaternion_xform_ra(xx + 27, xx + 1, xx + 11);
  xx[1] = - 6.076258598061595e-3;
  xx[2] = - 7.677368418456878e-3;
  xx[3] = 7.9149297453912e-3;
  pm_math_Quaternion_xform_ra(xx + 40, xx + 1, xx + 26);
  xx[1] = 0.01659132440963332 - xx[26];
  xx[2] = 8.500421875005637e-3 - xx[27];
  xx[3] = 8.386468547111092e-3 - xx[28];
  pm_math_Quaternion_xform_ra(xx + 36, xx + 1, xx + 26);
  xx[1] = - 0.01544153522065833;
  xx[2] = 1.74063061481753e-3;
  xx[3] = - 4.362197173551173e-3;
  pm_math_Quaternion_xform_ra(xx + 36, xx + 1, xx + 40);
  xx[1] = xx[33] + xx[16] - xx[24] - 0.04200000000000426;
  xx[2] = xx[34] + xx[17] - xx[25];
  xx[14] = 0.3676062374005039;
  xx[15] = - 0.4418868990508547;
  xx[16] = - 0.6040401092638356;
  xx[17] = 0.552030043631683;
  pm_math_Quaternion_compose_ra(xx + 5, xx + 14, xx + 32);
  xx[14] = (xx[33] * xx[35] + xx[32] * xx[34]) * xx[9];
  xx[15] = xx[9] * (xx[34] * xx[35] - xx[32] * xx[33]);
  xx[16] = xx[10] - (xx[33] * xx[33] + xx[34] * xx[34]) * xx[9];
  xx[32] = 0.4437459878328764;
  xx[33] = 0.1803099382577183;
  xx[34] = 0.7643348123670592;
  xx[35] = 0.4317060563062962;
  xx[3] = xx[4] * state[10];
  xx[17] = sin(xx[3]);
  xx[36] = cos(xx[3]);
  xx[37] = 0.6587699864704037 * xx[17];
  xx[38] = 0.5622364142279257 * xx[17];
  xx[39] = 0.4999123117526796 * xx[17];
  pm_math_Quaternion_compose_ra(xx + 32, xx + 36, xx + 54);
  xx[32] = - 0.2207829197706712;
  xx[33] = - 0.6845661841871489;
  xx[34] = - 0.01617467817412936;
  xx[35] = 0.6945231613063365;
  xx[3] = xx[4] * state[12];
  xx[4] = sin(xx[3]);
  xx[36] = cos(xx[3]);
  xx[37] = - (0.5038982060378495 * xx[4]);
  xx[38] = 0.8089781762292761 * xx[4];
  xx[39] = - (0.3027224939389068 * xx[4]);
  pm_math_Quaternion_compose_ra(xx + 32, xx + 36, xx + 58);
  pm_math_Quaternion_compose_ra(xx + 54, xx + 58, xx + 32);
  xx[36] = xx[22];
  xx[37] = xx[31];
  xx[38] = 0.2970285731203686;
  xx[39] = 0.4967332746125314;
  pm_math_Quaternion_compose_ra(xx + 32, xx + 36, xx + 22);
  xx[3] = xx[25] * xx[25];
  xx[4] = xx[23] * xx[24];
  xx[17] = xx[22] * xx[25];
  xx[29] = xx[10] - (xx[24] * xx[24] + xx[3]) * xx[9];
  xx[30] = (xx[4] + xx[17]) * xx[9];
  xx[31] = xx[9] * (xx[23] * xx[25] - xx[22] * xx[24]);
  xx[36] = xx[9] * (xx[4] - xx[17]);
  xx[37] = xx[10] - (xx[23] * xx[23] + xx[3]) * xx[9];
  xx[38] = (xx[24] * xx[25] + xx[22] * xx[23]) * xx[9];
  xx[22] = - 0.01680036206308408;
  xx[23] = 1.904183946736232e-3;
  xx[24] = - 2.167844671666771e-3;
  pm_math_Quaternion_xform_ra(xx + 5, xx + 22, xx + 62);
  xx[3] = - 1.75429196580309e-3;
  xx[4] = 9.461285757155713e-3;
  xx[5] = - 5.462379018955269e-3;
  pm_math_Quaternion_xform_ra(xx + 32, xx + 3, xx + 6);
  xx[3] = - 6.076258262701207e-3;
  xx[4] = - 7.677368956859708e-3;
  xx[5] = 7.91492994686379e-3;
  pm_math_Quaternion_xform_ra(xx + 58, xx + 3, xx + 22);
  xx[3] = 7.869797269250363e-3 - xx[22];
  xx[4] = 0.01668052845419594 - xx[23];
  xx[5] = 2.96461486266998e-3 - xx[24];
  pm_math_Quaternion_xform_ra(xx + 54, xx + 3, xx + 22);
  xx[3] = - 6.483073904707459e-4;
  xx[4] = - 0.01395135758917342;
  xx[5] = 3.551989250699576e-3;
  pm_math_Quaternion_xform_ra(xx + 54, xx + 3, xx + 32);
  xx[65] = fabs(pm_math_Vector3_dot_ra(xx + 19, xx + 48));
  xx[66] = fabs(pm_math_Vector3_dot_ra(xx + 19, xx + 51));
  xx[67] = fabs(xx[44] + xx[0] - (xx[11] + xx[26] - xx[40]));
  xx[68] = fabs(xx[45] + xx[1] - (xx[12] + xx[27] - xx[41]) +
                0.03750000000000427);
  xx[69] = fabs(xx[46] + xx[2] - (xx[13] + xx[28] - xx[42]));
  xx[70] = fabs(pm_math_Vector3_dot_ra(xx + 14, xx + 29));
  xx[71] = fabs(pm_math_Vector3_dot_ra(xx + 14, xx + 36));
  xx[72] = fabs(xx[62] + xx[0] - (xx[6] + xx[22] - xx[32]));
  xx[73] = fabs(xx[63] + xx[1] - (xx[7] + xx[23] - xx[33]) + 0.03300000000000426);
  xx[74] = fabs(xx[64] + xx[2] - (xx[8] + xx[24] - xx[34]));
  ii[0] = 65;

  {
    int ll;
    for (ll = 66; ll < 75; ++ll)
      if (xx[ll] > xx[ii[0]])
        ii[0] = ll;
  }

  ii[0] -= 65;
  xx[0] = xx[65 + (ii[0])];
  return xx[0] > 1.0e-5;
}

boolean_T DynamicValidation_dfaed711_1_isVelocityViolation(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const
  double *state, const int *modeVector)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  int ii[1];
  double xx[98];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) modeVector;
  xx[0] = 0.6190277160392075;
  xx[1] = - 0.2167287412203158;
  xx[2] = 0.7124684524173637;
  xx[3] = 0.2494434681733462;
  xx[4] = 0.5;
  xx[5] = xx[4] * state[0];
  xx[6] = 5.906528705024172e-8;
  xx[7] = sin(xx[5]);
  xx[8] = 0.7816132178294208;
  xx[9] = 0.6237633988254963;
  xx[10] = cos(xx[5]);
  xx[11] = xx[6] * xx[7];
  xx[12] = xx[8] * xx[7];
  xx[13] = xx[9] * xx[7];
  pm_math_Quaternion_compose_ra(xx + 0, xx + 10, xx + 14);
  xx[0] = - 0.9578985788971721;
  xx[1] = 0.01596084651607334;
  xx[2] = 0.2160322563859199;
  xx[3] = 0.188429371718097;
  xx[5] = xx[4] * state[2];
  xx[7] = 0.503898206037895;
  xx[10] = sin(xx[5]);
  xx[11] = 0.8089781762292558;
  xx[12] = 0.3027224939388848;
  xx[18] = cos(xx[5]);
  xx[19] = - (xx[7] * xx[10]);
  xx[20] = xx[11] * xx[10];
  xx[21] = - (xx[12] * xx[10]);
  pm_math_Quaternion_compose_ra(xx + 0, xx + 18, xx + 22);
  pm_math_Quaternion_compose_ra(xx + 14, xx + 22, xx + 0);
  xx[18] = - 0.7391371617538293;
  xx[19] = - 0.2713596951978788;
  xx[20] = - 0.5768318010060487;
  xx[21] = - 0.2174976902974528;
  xx[5] = xx[4] * state[4];
  xx[10] = 0.3618351326747742;
  xx[13] = sin(xx[5]);
  xx[26] = 0.342020353294726;
  xx[27] = 0.8672355012880755;
  xx[28] = cos(xx[5]);
  xx[29] = - (xx[10] * xx[13]);
  xx[30] = xx[26] * xx[13];
  xx[31] = - (xx[27] * xx[13]);
  pm_math_Quaternion_compose_ra(xx + 18, xx + 28, xx + 32);
  pm_math_Quaternion_compose_ra(xx + 0, xx + 32, xx + 18);
  xx[28] = xx[6] * state[1];
  xx[29] = xx[8] * state[1];
  xx[30] = xx[9] * state[1];
  pm_math_Quaternion_inverseXform_ra(xx + 22, xx + 28, xx + 36);
  xx[39] = xx[36] - xx[7] * state[3];
  xx[40] = xx[37] + xx[11] * state[3];
  xx[41] = xx[38] - xx[12] * state[3];
  pm_math_Quaternion_inverseXform_ra(xx + 32, xx + 39, xx + 5);
  xx[11] = xx[5] - xx[10] * state[5];
  xx[12] = xx[6] + xx[26] * state[5];
  xx[13] = xx[7] - xx[27] * state[5];
  xx[5] = 0.5701273767950668;
  xx[6] = - 0.3420242615573683;
  xx[7] = - 0.7469766922300273;
  pm_math_Vector3_cross_ra(xx + 11, xx + 5, xx + 8);
  pm_math_Quaternion_xform_ra(xx + 18, xx + 8, xx + 5);
  xx[8] = 1.0;
  xx[42] = 0.324402019972231;
  xx[43] = 0.1793201049669474;
  xx[44] = 0.8745212078931046;
  xx[45] = 0.3127623480180516;
  xx[9] = xx[4] * state[6];
  xx[10] = 0.5165599445270539;
  xx[26] = sin(xx[9]);
  xx[27] = 0.7400480272337564;
  xx[31] = 0.4306910041986394;
  xx[46] = cos(xx[9]);
  xx[47] = xx[10] * xx[26];
  xx[48] = xx[27] * xx[26];
  xx[49] = xx[31] * xx[26];
  pm_math_Quaternion_compose_ra(xx + 42, xx + 46, xx + 50);
  xx[42] = - 0.8846589215343961;
  xx[43] = - 0.3902684207978444;
  xx[44] = 0.2533894697105629;
  xx[45] = - 0.02937565177755097;
  xx[9] = xx[4] * state[8];
  xx[26] = 0.5038982060378953;
  xx[36] = sin(xx[9]);
  xx[37] = 0.8089781762292564;
  xx[38] = 0.3027224939388842;
  xx[46] = cos(xx[9]);
  xx[47] = - (xx[26] * xx[36]);
  xx[48] = xx[37] * xx[36];
  xx[49] = - (xx[38] * xx[36]);
  pm_math_Quaternion_compose_ra(xx + 42, xx + 46, xx + 54);
  pm_math_Quaternion_compose_ra(xx + 50, xx + 54, xx + 42);
  xx[9] = 0.300048825040154;
  xx[36] = 0.7582880608547207;
  xx[46] = xx[9];
  xx[47] = xx[36];
  xx[48] = 0.2970285731203685;
  xx[49] = 0.4967332746125315;
  pm_math_Quaternion_compose_ra(xx + 42, xx + 46, xx + 58);
  xx[46] = xx[61] * xx[61];
  xx[47] = 2.0;
  xx[48] = xx[59] * xx[60];
  xx[49] = xx[58] * xx[61];
  xx[62] = xx[8] - (xx[60] * xx[60] + xx[46]) * xx[47];
  xx[63] = (xx[48] + xx[49]) * xx[47];
  xx[64] = xx[47] * (xx[59] * xx[61] - xx[58] * xx[60]);
  xx[65] = xx[10] * state[7];
  xx[66] = xx[27] * state[7];
  xx[67] = xx[31] * state[7];
  pm_math_Quaternion_inverseXform_ra(xx + 54, xx + 65, xx + 68);
  xx[71] = xx[68] - xx[26] * state[9];
  xx[72] = xx[69] + xx[37] * state[9];
  xx[73] = xx[70] - xx[38] * state[9];
  xx[10] = 0.3300601612855789;
  xx[26] = 0.7485549122714541;
  xx[68] = xx[10];
  xx[69] = xx[26];
  xx[70] = 0.5750876743996639;
  pm_math_Vector3_cross_ra(xx + 71, xx + 68, xx + 74);
  pm_math_Quaternion_xform_ra(xx + 42, xx + 74, xx + 68);
  xx[74] = 0.1645059501899856;
  xx[75] = 0.932950220907751;
  xx[76] = - 0.05560783598731543;
  xx[77] = 0.3153560626293339;
  pm_math_Quaternion_compose_ra(xx + 18, xx + 74, xx + 78);
  xx[74] = (xx[79] * xx[81] + xx[78] * xx[80]) * xx[47];
  xx[75] = xx[47] * (xx[80] * xx[81] - xx[78] * xx[79]);
  xx[76] = xx[8] - (xx[79] * xx[79] + xx[80] * xx[80]) * xx[47];
  xx[77] = xx[47] * (xx[48] - xx[49]);
  xx[78] = xx[8] - (xx[59] * xx[59] + xx[46]) * xx[47];
  xx[79] = (xx[60] * xx[61] + xx[58] * xx[59]) * xx[47];
  xx[58] = 0.1523779706481012;
  xx[59] = - 0.643489458684202;
  xx[60] = 0.7501348349620082;
  pm_math_Vector3_cross_ra(xx + 71, xx + 58, xx + 80);
  pm_math_Quaternion_xform_ra(xx + 42, xx + 80, xx + 58);
  xx[80] = 0.01027766876427663;
  xx[81] = 1.904031651085619e-3;
  xx[82] = - 0.01346562362772443;
  pm_math_Vector3_cross_ra(xx + 11, xx + 80, xx + 83);
  pm_math_Quaternion_xform_ra(xx + 18, xx + 83, xx + 80);
  xx[83] = 0.01500104515676648;
  xx[84] = 1.380515327287848e-10;
  xx[85] = - 1.593463070824624e-9;
  pm_math_Quaternion_xform_ra(xx + 14, xx + 83, xx + 86);
  xx[83] = - 4.078893315886357e-3;
  xx[84] = - 5.827739056241386e-3;
  xx[85] = - 8.784172772109595e-3;
  pm_math_Quaternion_xform_ra(xx + 22, xx + 83, xx + 89);
  xx[83] = - 6.076258344851112e-3;
  xx[84] = - 7.677368824970993e-3;
  xx[85] = 7.914929897510211e-3;
  pm_math_Quaternion_xform_ra(xx + 22, xx + 83, xx + 92);
  xx[22] = 1.706386300196451e-9 - xx[92];
  xx[23] = 0.01104873622831247 - xx[93];
  xx[24] = 0.01832347446050686 - xx[94];
  pm_math_Vector3_cross_ra(xx + 28, xx + 22, xx + 83);
  xx[22] = xx[89] * state[3] + xx[83];
  xx[23] = xx[90] * state[3] + xx[84];
  xx[24] = xx[91] * state[3] + xx[85];
  pm_math_Quaternion_xform_ra(xx + 14, xx + 22, xx + 27);
  xx[14] = - 6.9982898852701e-3;
  xx[15] = 5.850496230747418e-8;
  xx[16] = 2.919907170843508e-3;
  pm_math_Quaternion_xform_ra(xx + 32, xx + 14, xx + 22);
  xx[14] = - 3.265857732358156e-3;
  xx[15] = 0.01115673029435595;
  xx[16] = - 7.827670717550887e-3;
  pm_math_Quaternion_xform_ra(xx + 32, xx + 14, xx + 83);
  xx[14] = - (0.02695619245963533 + xx[83]);
  xx[15] = 0.01378863199166597 - xx[84];
  xx[16] = 3.369121440618853e-3 - xx[85];
  pm_math_Vector3_cross_ra(xx + 39, xx + 14, xx + 30);
  xx[14] = xx[22] * state[5] + xx[30];
  xx[15] = xx[23] * state[5] + xx[31];
  xx[16] = xx[24] * state[5] + xx[32];
  pm_math_Quaternion_xform_ra(xx + 0, xx + 14, xx + 22);
  xx[0] = xx[86] * state[1] + xx[27] + xx[22];
  xx[1] = - 1.754291965803085e-3;
  xx[2] = 9.46128575715573e-3;
  xx[3] = - 5.462379018955248e-3;
  pm_math_Vector3_cross_ra(xx + 71, xx + 1, xx + 14);
  pm_math_Quaternion_xform_ra(xx + 42, xx + 14, xx + 1);
  xx[14] = 3.977909360125871e-3;
  xx[15] = 4.397193980568331e-3;
  xx[16] = - 0.012326617731341;
  pm_math_Quaternion_xform_ra(xx + 50, xx + 14, xx + 30);
  xx[14] = - 4.078893315886373e-3;
  xx[15] = - 5.827739056241395e-3;
  xx[16] = - 8.784172772109623e-3;
  pm_math_Quaternion_xform_ra(xx + 54, xx + 14, xx + 33);
  xx[14] = - 6.076258598061595e-3;
  xx[15] = - 7.677368418456878e-3;
  xx[16] = 7.9149297453912e-3;
  pm_math_Quaternion_xform_ra(xx + 54, xx + 14, xx + 37);
  xx[14] = 0.01659132440963332 - xx[37];
  xx[15] = 8.500421875005637e-3 - xx[38];
  xx[16] = 8.386468547111092e-3 - xx[39];
  pm_math_Vector3_cross_ra(xx + 65, xx + 14, xx + 37);
  xx[14] = xx[33] * state[9] + xx[37];
  xx[15] = xx[34] * state[9] + xx[38];
  xx[16] = xx[35] * state[9] + xx[39];
  pm_math_Quaternion_xform_ra(xx + 50, xx + 14, xx + 33);
  xx[14] = xx[87] * state[1] + xx[28] + xx[23];
  xx[15] = xx[88] * state[1] + xx[29] + xx[24];
  xx[22] = - 0.9319675119375604;
  xx[23] = - 0.3420158151110814;
  xx[24] = - 0.1202569703044931;
  pm_math_Vector3_cross_ra(xx + 11, xx + 22, xx + 27);
  pm_math_Quaternion_xform_ra(xx + 18, xx + 27, xx + 22);
  xx[37] = 0.4437459878328764;
  xx[38] = 0.1803099382577183;
  xx[39] = 0.7643348123670592;
  xx[40] = 0.4317060563062962;
  xx[16] = xx[4] * state[10];
  xx[17] = 0.6587699864704037;
  xx[25] = sin(xx[16]);
  xx[27] = 0.5622364142279257;
  xx[28] = 0.4999123117526796;
  xx[41] = cos(xx[16]);
  xx[42] = xx[17] * xx[25];
  xx[43] = xx[27] * xx[25];
  xx[44] = xx[28] * xx[25];
  pm_math_Quaternion_compose_ra(xx + 37, xx + 41, xx + 48);
  xx[37] = - 0.2207829197706712;
  xx[38] = - 0.6845661841871489;
  xx[39] = - 0.01617467817412936;
  xx[40] = 0.6945231613063365;
  xx[16] = xx[4] * state[12];
  xx[4] = 0.5038982060378495;
  xx[25] = sin(xx[16]);
  xx[29] = 0.8089781762292761;
  xx[41] = 0.3027224939389068;
  xx[42] = cos(xx[16]);
  xx[43] = - (xx[4] * xx[25]);
  xx[44] = xx[29] * xx[25];
  xx[45] = - (xx[41] * xx[25]);
  pm_math_Quaternion_compose_ra(xx + 37, xx + 42, xx + 52);
  pm_math_Quaternion_compose_ra(xx + 48, xx + 52, xx + 37);
  xx[42] = xx[9];
  xx[43] = xx[36];
  xx[44] = 0.2970285731203686;
  xx[45] = 0.4967332746125314;
  pm_math_Quaternion_compose_ra(xx + 37, xx + 42, xx + 83);
  xx[9] = xx[86] * xx[86];
  xx[16] = xx[84] * xx[85];
  xx[25] = xx[83] * xx[86];
  xx[42] = xx[8] - (xx[85] * xx[85] + xx[9]) * xx[47];
  xx[43] = (xx[16] + xx[25]) * xx[47];
  xx[44] = xx[47] * (xx[84] * xx[86] - xx[83] * xx[85]);
  xx[65] = xx[17] * state[11];
  xx[66] = xx[27] * state[11];
  xx[67] = xx[28] * state[11];
  pm_math_Quaternion_inverseXform_ra(xx + 52, xx + 65, xx + 71);
  xx[87] = xx[71] - xx[4] * state[13];
  xx[88] = xx[72] + xx[29] * state[13];
  xx[89] = xx[73] - xx[41] * state[13];
  xx[27] = xx[10];
  xx[28] = xx[26];
  xx[29] = 0.5750876743996637;
  pm_math_Vector3_cross_ra(xx + 87, xx + 27, xx + 71);
  pm_math_Quaternion_xform_ra(xx + 37, xx + 71, xx + 26);
  xx[90] = 0.3676062374005039;
  xx[91] = - 0.4418868990508547;
  xx[92] = - 0.6040401092638356;
  xx[93] = 0.552030043631683;
  pm_math_Quaternion_compose_ra(xx + 18, xx + 90, xx + 94);
  xx[71] = (xx[95] * xx[97] + xx[94] * xx[96]) * xx[47];
  xx[72] = xx[47] * (xx[96] * xx[97] - xx[94] * xx[95]);
  xx[73] = xx[8] - (xx[95] * xx[95] + xx[96] * xx[96]) * xx[47];
  xx[90] = xx[47] * (xx[16] - xx[25]);
  xx[91] = xx[8] - (xx[84] * xx[84] + xx[9]) * xx[47];
  xx[92] = (xx[85] * xx[86] + xx[83] * xx[84]) * xx[47];
  xx[8] = 0.1523779706481014;
  xx[9] = - 0.6434894586842019;
  xx[10] = 0.7501348349620083;
  pm_math_Vector3_cross_ra(xx + 87, xx + 8, xx + 45);
  pm_math_Quaternion_xform_ra(xx + 37, xx + 45, xx + 8);
  xx[45] = - 0.01680036206308408;
  xx[46] = 1.904183946736232e-3;
  xx[47] = - 2.167844671666771e-3;
  pm_math_Vector3_cross_ra(xx + 11, xx + 45, xx + 83);
  pm_math_Quaternion_xform_ra(xx + 18, xx + 83, xx + 11);
  xx[16] = - 1.75429196580309e-3;
  xx[17] = 9.461285757155713e-3;
  xx[18] = - 5.462379018955269e-3;
  pm_math_Vector3_cross_ra(xx + 87, xx + 16, xx + 19);
  pm_math_Quaternion_xform_ra(xx + 37, xx + 19, xx + 16);
  xx[19] = - 8.971513124181443e-3;
  xx[20] = 2.664040756922957e-3;
  xx[21] = 8.826233627727805e-3;
  pm_math_Quaternion_xform_ra(xx + 48, xx + 19, xx + 36);
  xx[19] = - 4.078893315886636e-3;
  xx[20] = - 5.827739056241714e-3;
  xx[21] = - 8.784172772110375e-3;
  pm_math_Quaternion_xform_ra(xx + 52, xx + 19, xx + 39);
  xx[19] = - 6.076258262701207e-3;
  xx[20] = - 7.677368956859708e-3;
  xx[21] = 7.91492994686379e-3;
  pm_math_Quaternion_xform_ra(xx + 52, xx + 19, xx + 45);
  xx[19] = 7.869797269250363e-3 - xx[45];
  xx[20] = 0.01668052845419594 - xx[46];
  xx[21] = 2.96461486266998e-3 - xx[47];
  pm_math_Vector3_cross_ra(xx + 65, xx + 19, xx + 45);
  xx[19] = xx[39] * state[13] + xx[45];
  xx[20] = xx[40] * state[13] + xx[46];
  xx[21] = xx[41] * state[13] + xx[47];
  pm_math_Quaternion_xform_ra(xx + 48, xx + 19, xx + 39);
  xx[45] = fabs(pm_math_Vector3_dot_ra(xx + 5, xx + 62) + pm_math_Vector3_dot_ra
                (xx + 68, xx + 74));
  xx[46] = fabs(pm_math_Vector3_dot_ra(xx + 5, xx + 77) + pm_math_Vector3_dot_ra
                (xx + 58, xx + 74));
  xx[47] = fabs(xx[80] + xx[0] - (xx[1] + xx[30] * state[7] + xx[33]));
  xx[48] = fabs(xx[81] + xx[14] - (xx[2] + xx[31] * state[7] + xx[34]));
  xx[49] = fabs(xx[82] + xx[15] - (xx[3] + xx[32] * state[7] + xx[35]));
  xx[50] = fabs(pm_math_Vector3_dot_ra(xx + 22, xx + 42) +
                pm_math_Vector3_dot_ra(xx + 26, xx + 71));
  xx[51] = fabs(pm_math_Vector3_dot_ra(xx + 22, xx + 90) +
                pm_math_Vector3_dot_ra(xx + 8, xx + 71));
  xx[52] = fabs(xx[11] + xx[0] - (xx[16] + xx[36] * state[11] + xx[39]));
  xx[53] = fabs(xx[12] + xx[14] - (xx[17] + xx[37] * state[11] + xx[40]));
  xx[54] = fabs(xx[13] + xx[15] - (xx[18] + xx[38] * state[11] + xx[41]));
  ii[0] = 45;

  {
    int ll;
    for (ll = 46; ll < 55; ++ll)
      if (xx[ll] > xx[ii[0]])
        ii[0] = ll;
  }

  ii[0] -= 45;
  xx[0] = xx[45 + (ii[0])];
  return xx[0] > 1.0e-5;
}

PmfMessageId DynamicValidation_dfaed711_1_projectStateSim(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags, const int
  *modeVector, double *state, void *neDiagMgr0)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  int ii[10];
  double xx[370];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) modeVector;
  (void) neDiagMgr;
  xx[0] = 0.6190277160392075;
  xx[1] = - 0.2167287412203158;
  xx[2] = 0.7124684524173637;
  xx[3] = 0.2494434681733462;
  xx[4] = 0.5;
  xx[5] = xx[4] * state[0];
  xx[6] = 5.906528705024172e-8;
  xx[7] = sin(xx[5]);
  xx[8] = 0.7816132178294208;
  xx[9] = 0.6237633988254963;
  xx[10] = cos(xx[5]);
  xx[11] = xx[6] * xx[7];
  xx[12] = xx[8] * xx[7];
  xx[13] = xx[9] * xx[7];
  pm_math_Quaternion_compose_ra(xx + 0, xx + 10, xx + 14);
  xx[10] = - 0.9578985788971721;
  xx[11] = 0.01596084651607334;
  xx[12] = 0.2160322563859199;
  xx[13] = 0.188429371718097;
  xx[5] = xx[4] * state[2];
  xx[7] = 0.503898206037895;
  xx[18] = sin(xx[5]);
  xx[19] = 0.8089781762292558;
  xx[20] = 0.3027224939388848;
  xx[21] = cos(xx[5]);
  xx[22] = - (xx[7] * xx[18]);
  xx[23] = xx[19] * xx[18];
  xx[24] = - (xx[20] * xx[18]);
  pm_math_Quaternion_compose_ra(xx + 10, xx + 21, xx + 25);
  pm_math_Quaternion_compose_ra(xx + 14, xx + 25, xx + 21);
  xx[29] = - 0.7391371617538293;
  xx[30] = - 0.2713596951978788;
  xx[31] = - 0.5768318010060487;
  xx[32] = - 0.2174976902974528;
  xx[5] = xx[4] * state[4];
  xx[18] = 0.3618351326747742;
  xx[33] = sin(xx[5]);
  xx[34] = 0.342020353294726;
  xx[35] = 0.8672355012880755;
  xx[36] = cos(xx[5]);
  xx[37] = - (xx[18] * xx[33]);
  xx[38] = xx[34] * xx[33];
  xx[39] = - (xx[35] * xx[33]);
  pm_math_Quaternion_compose_ra(xx + 29, xx + 36, xx + 40);
  pm_math_Quaternion_compose_ra(xx + 21, xx + 40, xx + 36);
  xx[44] = - xx[7];
  xx[45] = xx[19];
  xx[46] = - xx[20];
  pm_math_Quaternion_inverseXform_ra(xx + 40, xx + 44, xx + 47);
  xx[5] = 0.5701273767950668;
  xx[33] = - 0.3420242615573683;
  xx[50] = xx[5];
  xx[51] = xx[33];
  xx[52] = - 0.7469766922300273;
  pm_math_Vector3_cross_ra(xx + 47, xx + 50, xx + 53);
  pm_math_Quaternion_xform_ra(xx + 36, xx + 53, xx + 56);
  xx[53] = 1.0;
  xx[59] = 0.324402019972231;
  xx[60] = 0.1793201049669474;
  xx[61] = 0.8745212078931046;
  xx[62] = 0.3127623480180516;
  xx[54] = xx[4] * state[6];
  xx[55] = 0.5165599445270539;
  xx[63] = sin(xx[54]);
  xx[64] = 0.7400480272337564;
  xx[65] = 0.4306910041986394;
  xx[66] = cos(xx[54]);
  xx[67] = xx[55] * xx[63];
  xx[68] = xx[64] * xx[63];
  xx[69] = xx[65] * xx[63];
  pm_math_Quaternion_compose_ra(xx + 59, xx + 66, xx + 70);
  xx[66] = - 0.8846589215343961;
  xx[67] = - 0.3902684207978444;
  xx[68] = 0.2533894697105629;
  xx[69] = - 0.02937565177755097;
  xx[54] = xx[4] * state[8];
  xx[63] = 0.5038982060378953;
  xx[74] = sin(xx[54]);
  xx[75] = 0.8089781762292564;
  xx[76] = 0.3027224939388842;
  xx[77] = cos(xx[54]);
  xx[78] = - (xx[63] * xx[74]);
  xx[79] = xx[75] * xx[74];
  xx[80] = - (xx[76] * xx[74]);
  pm_math_Quaternion_compose_ra(xx + 66, xx + 77, xx + 81);
  pm_math_Quaternion_compose_ra(xx + 70, xx + 81, xx + 77);
  xx[54] = 0.300048825040154;
  xx[74] = 0.7582880608547207;
  xx[85] = xx[54];
  xx[86] = xx[74];
  xx[87] = 0.2970285731203685;
  xx[88] = 0.4967332746125315;
  pm_math_Quaternion_compose_ra(xx + 77, xx + 85, xx + 89);
  xx[93] = xx[92] * xx[92];
  xx[94] = 2.0;
  xx[95] = xx[90] * xx[91];
  xx[96] = xx[89] * xx[92];
  xx[97] = xx[53] - (xx[91] * xx[91] + xx[93]) * xx[94];
  xx[98] = (xx[95] + xx[96]) * xx[94];
  xx[99] = xx[94] * (xx[90] * xx[92] - xx[89] * xx[91]);
  xx[100] = - 0.5520968141038278;
  xx[101] = - 0.7647171119509411;
  xx[102] = - 0.0712387727758421;
  pm_math_Quaternion_xform_ra(xx + 36, xx + 100, xx + 103);
  xx[106] = 0.6918377879007818;
  xx[107] = 0.1898690122702551;
  xx[108] = - 0.6442069447371817;
  pm_math_Quaternion_xform_ra(xx + 77, xx + 106, xx + 109);
  xx[112] = 0.1645059501899856;
  xx[113] = 0.932950220907751;
  xx[114] = - 0.05560783598731543;
  xx[115] = 0.3153560626293339;
  pm_math_Quaternion_compose_ra(xx + 36, xx + 112, xx + 116);
  xx[120] = (xx[117] * xx[119] + xx[116] * xx[118]) * xx[94];
  xx[121] = xx[94] * (xx[118] * xx[119] - xx[116] * xx[117]);
  xx[122] = xx[53] - (xx[117] * xx[117] + xx[118] * xx[118]) * xx[94];
  xx[116] = 0.0;
  xx[117] = xx[94] * (xx[95] - xx[96]);
  xx[118] = xx[53] - (xx[90] * xx[90] + xx[93]) * xx[94];
  xx[119] = (xx[91] * xx[92] + xx[89] * xx[90]) * xx[94];
  xx[89] = 0.4120439769573354;
  xx[90] = 0.3318633583279493;
  xx[91] = 0.2009827310428497;
  pm_math_Quaternion_xform_ra(xx + 77, xx + 89, xx + 123);
  xx[126] = 0.01027766876427663;
  xx[127] = 1.904031651085619e-3;
  xx[128] = - 0.01346562362772443;
  pm_math_Vector3_cross_ra(xx + 47, xx + 126, xx + 129);
  pm_math_Quaternion_xform_ra(xx + 36, xx + 129, xx + 132);
  xx[92] = 0.02695619245963533;
  xx[129] = - 3.265857732358156e-3;
  xx[130] = 0.01115673029435595;
  xx[131] = - 7.827670717550887e-3;
  pm_math_Quaternion_xform_ra(xx + 40, xx + 129, xx + 135);
  xx[93] = 0.01378863199166597;
  xx[95] = 3.369121440618853e-3;
  xx[138] = - (xx[92] + xx[135]);
  xx[139] = xx[93] - xx[136];
  xx[140] = xx[95] - xx[137];
  pm_math_Vector3_cross_ra(xx + 44, xx + 138, xx + 135);
  pm_math_Quaternion_xform_ra(xx + 25, xx + 135, xx + 141);
  xx[135] = - 4.078893315886357e-3;
  xx[136] = - 5.827739056241386e-3;
  xx[137] = - 8.784172772109595e-3;
  pm_math_Quaternion_xform_ra(xx + 25, xx + 135, xx + 144);
  xx[147] = xx[141] + xx[144];
  xx[148] = xx[142] + xx[145];
  xx[149] = xx[143] + xx[146];
  pm_math_Quaternion_xform_ra(xx + 14, xx + 147, xx + 141);
  xx[144] = - 2.954273507090521e-3;
  xx[145] = - 0.01378549493474648;
  xx[146] = - 4.204117446891596e-3;
  pm_math_Quaternion_xform_ra(xx + 36, xx + 144, xx + 147);
  xx[150] = - 6.9982898852701e-3;
  xx[151] = 5.850496230747418e-8;
  xx[152] = 2.919907170843508e-3;
  pm_math_Quaternion_xform_ra(xx + 40, xx + 150, xx + 153);
  pm_math_Quaternion_xform_ra(xx + 21, xx + 153, xx + 40);
  xx[153] = - 1.554801396352744e-3;
  xx[154] = - 2.22141934936573e-3;
  xx[155] = - 3.348341004773645e-3;
  pm_math_Quaternion_xform_ra(xx + 77, xx + 153, xx + 156);
  xx[159] = - 4.078893315886373e-3;
  xx[160] = - 5.827739056241395e-3;
  xx[161] = - 8.784172772109623e-3;
  pm_math_Quaternion_xform_ra(xx + 81, xx + 159, xx + 162);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 162, xx + 165);
  xx[162] = - 0.9319675119375604;
  xx[163] = - 0.3420158151110814;
  xx[164] = - 0.1202569703044931;
  pm_math_Vector3_cross_ra(xx + 47, xx + 162, xx + 168);
  pm_math_Quaternion_xform_ra(xx + 36, xx + 168, xx + 171);
  xx[174] = 0.4437459878328764;
  xx[175] = 0.1803099382577183;
  xx[176] = 0.7643348123670592;
  xx[177] = 0.4317060563062962;
  xx[43] = xx[4] * state[10];
  xx[96] = 0.6587699864704037;
  xx[168] = sin(xx[43]);
  xx[169] = 0.5622364142279257;
  xx[170] = 0.4999123117526796;
  xx[178] = cos(xx[43]);
  xx[179] = xx[96] * xx[168];
  xx[180] = xx[169] * xx[168];
  xx[181] = xx[170] * xx[168];
  pm_math_Quaternion_compose_ra(xx + 174, xx + 178, xx + 182);
  xx[178] = - 0.2207829197706712;
  xx[179] = - 0.6845661841871489;
  xx[180] = - 0.01617467817412936;
  xx[181] = 0.6945231613063365;
  xx[43] = xx[4] * state[12];
  xx[168] = 0.5038982060378495;
  xx[186] = sin(xx[43]);
  xx[187] = 0.8089781762292761;
  xx[188] = 0.3027224939389068;
  xx[189] = cos(xx[43]);
  xx[190] = - (xx[168] * xx[186]);
  xx[191] = xx[187] * xx[186];
  xx[192] = - (xx[188] * xx[186]);
  pm_math_Quaternion_compose_ra(xx + 178, xx + 189, xx + 193);
  pm_math_Quaternion_compose_ra(xx + 182, xx + 193, xx + 189);
  xx[197] = xx[54];
  xx[198] = xx[74];
  xx[199] = 0.2970285731203686;
  xx[200] = 0.4967332746125314;
  pm_math_Quaternion_compose_ra(xx + 189, xx + 197, xx + 201);
  xx[43] = xx[204] * xx[204];
  xx[54] = xx[202] * xx[203];
  xx[74] = xx[201] * xx[204];
  xx[205] = xx[53] - (xx[203] * xx[203] + xx[43]) * xx[94];
  xx[206] = (xx[54] + xx[74]) * xx[94];
  xx[207] = xx[94] * (xx[202] * xx[204] - xx[201] * xx[203]);
  xx[208] = - 0.3377385883360045;
  xx[209] = 0.7647221155941781;
  xx[210] = 0.4425051955296804;
  pm_math_Quaternion_xform_ra(xx + 36, xx + 208, xx + 211);
  xx[214] = 0.6918377879008099;
  xx[215] = 0.1898690122702212;
  xx[216] = - 0.6442069447371539;
  pm_math_Quaternion_xform_ra(xx + 189, xx + 214, xx + 217);
  xx[220] = 0.3676062374005039;
  xx[221] = - 0.4418868990508547;
  xx[222] = - 0.6040401092638356;
  xx[223] = 0.552030043631683;
  pm_math_Quaternion_compose_ra(xx + 36, xx + 220, xx + 224);
  xx[228] = (xx[225] * xx[227] + xx[224] * xx[226]) * xx[94];
  xx[229] = xx[94] * (xx[226] * xx[227] - xx[224] * xx[225]);
  xx[230] = xx[53] - (xx[225] * xx[225] + xx[226] * xx[226]) * xx[94];
  xx[224] = xx[94] * (xx[54] - xx[74]);
  xx[225] = xx[53] - (xx[202] * xx[202] + xx[43]) * xx[94];
  xx[226] = (xx[203] * xx[204] + xx[201] * xx[202]) * xx[94];
  xx[201] = 0.4120439769573357;
  xx[202] = 0.3318633583279115;
  xx[203] = 0.200982731042817;
  pm_math_Quaternion_xform_ra(xx + 189, xx + 201, xx + 231);
  xx[234] = - 0.01680036206308408;
  xx[235] = 1.904183946736232e-3;
  xx[236] = - 2.167844671666771e-3;
  pm_math_Vector3_cross_ra(xx + 47, xx + 234, xx + 237);
  pm_math_Quaternion_xform_ra(xx + 36, xx + 237, xx + 47);
  xx[237] = 9.099289191009439e-4;
  xx[238] = 0.01378546805120904;
  xx[239] = 5.057065117290848e-3;
  pm_math_Quaternion_xform_ra(xx + 36, xx + 237, xx + 240);
  xx[243] = - 1.55480139635266e-3;
  xx[244] = - 2.221419349365449e-3;
  xx[245] = - 3.348341004773165e-3;
  pm_math_Quaternion_xform_ra(xx + 189, xx + 243, xx + 246);
  xx[249] = - 4.078893315886636e-3;
  xx[250] = - 5.827739056241714e-3;
  xx[251] = - 8.784172772110375e-3;
  pm_math_Quaternion_xform_ra(xx + 193, xx + 249, xx + 252);
  pm_math_Quaternion_xform_ra(xx + 182, xx + 252, xx + 255);
  xx[258] = pm_math_Vector3_dot_ra(xx + 56, xx + 97);
  xx[259] = pm_math_Vector3_dot_ra(xx + 103, xx + 97);
  xx[260] = pm_math_Vector3_dot_ra(xx + 109, xx + 120);
  xx[261] = xx[116];
  xx[262] = pm_math_Vector3_dot_ra(xx + 56, xx + 117);
  xx[263] = pm_math_Vector3_dot_ra(xx + 103, xx + 117);
  xx[264] = pm_math_Vector3_dot_ra(xx + 123, xx + 120);
  xx[265] = xx[116];
  xx[266] = xx[132] + xx[141];
  xx[267] = xx[147] + xx[40];
  xx[268] = - (xx[156] + xx[165]);
  xx[269] = xx[116];
  xx[270] = xx[133] + xx[142];
  xx[271] = xx[148] + xx[41];
  xx[272] = - (xx[157] + xx[166]);
  xx[273] = xx[116];
  xx[274] = xx[134] + xx[143];
  xx[275] = xx[149] + xx[42];
  xx[276] = - (xx[158] + xx[167]);
  xx[277] = xx[116];
  xx[278] = pm_math_Vector3_dot_ra(xx + 171, xx + 205);
  xx[279] = pm_math_Vector3_dot_ra(xx + 211, xx + 205);
  xx[280] = xx[116];
  xx[281] = pm_math_Vector3_dot_ra(xx + 217, xx + 228);
  xx[282] = pm_math_Vector3_dot_ra(xx + 171, xx + 224);
  xx[283] = pm_math_Vector3_dot_ra(xx + 211, xx + 224);
  xx[284] = xx[116];
  xx[285] = pm_math_Vector3_dot_ra(xx + 231, xx + 228);
  xx[286] = xx[47] + xx[141];
  xx[287] = xx[240] + xx[40];
  xx[288] = xx[116];
  xx[289] = - (xx[246] + xx[255]);
  xx[290] = xx[48] + xx[142];
  xx[291] = xx[241] + xx[41];
  xx[292] = xx[116];
  xx[293] = - (xx[247] + xx[256]);
  xx[294] = xx[49] + xx[143];
  xx[295] = xx[242] + xx[42];
  xx[296] = xx[116];
  xx[297] = - (xx[248] + xx[257]);
  xx[40] = - 1.754291965803085e-3;
  xx[41] = 9.46128575715573e-3;
  xx[42] = - 5.462379018955248e-3;
  pm_math_Quaternion_xform_ra(xx + 77, xx + 40, xx + 47);
  xx[43] = 0.01659132440963332;
  xx[56] = - 6.076258598061595e-3;
  xx[57] = - 7.677368418456878e-3;
  xx[58] = 7.9149297453912e-3;
  pm_math_Quaternion_xform_ra(xx + 81, xx + 56, xx + 77);
  xx[54] = 8.500421875005637e-3;
  xx[74] = 8.386468547111092e-3;
  xx[80] = xx[43] - xx[77];
  xx[81] = xx[54] - xx[78];
  xx[82] = xx[74] - xx[79];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 80, xx + 77);
  xx[80] = - 0.01544153522065833;
  xx[81] = 1.74063061481753e-3;
  xx[82] = - 4.362197173551173e-3;
  pm_math_Quaternion_xform_ra(xx + 70, xx + 80, xx + 103);
  pm_math_Quaternion_xform_ra(xx + 36, xx + 126, xx + 80);
  pm_math_Quaternion_xform_ra(xx + 21, xx + 138, xx + 109);
  xx[21] = 1.706386300196451e-9;
  xx[22] = - 6.076258344851112e-3;
  xx[23] = - 7.677368824970993e-3;
  xx[24] = 7.914929897510211e-3;
  pm_math_Quaternion_xform_ra(xx + 25, xx + 22, xx + 123);
  xx[25] = 0.01104873622831247;
  xx[26] = 0.01832347446050686;
  xx[132] = xx[21] - xx[123];
  xx[133] = xx[25] - xx[124];
  xx[134] = xx[26] - xx[125];
  pm_math_Quaternion_xform_ra(xx + 14, xx + 132, xx + 123);
  xx[132] = - 2.009348425691778e-9;
  xx[133] = 3.882107956182032e-4;
  xx[134] = - 0.01888260476507436;
  pm_math_Quaternion_xform_ra(xx + 14, xx + 132, xx + 138);
  xx[27] = xx[109] + xx[123] - xx[138];
  xx[28] = 0.04200000000000426;
  xx[83] = xx[110] + xx[124] - xx[139] - xx[28];
  xx[84] = 0.03750000000000427;
  xx[109] = xx[111] + xx[125] - xx[140];
  xx[123] = - 1.75429196580309e-3;
  xx[124] = 9.461285757155713e-3;
  xx[125] = - 5.462379018955269e-3;
  pm_math_Quaternion_xform_ra(xx + 189, xx + 123, xx + 132);
  xx[110] = 7.869797269250363e-3;
  xx[141] = - 6.076258262701207e-3;
  xx[142] = - 7.677368956859708e-3;
  xx[143] = 7.91492994686379e-3;
  pm_math_Quaternion_xform_ra(xx + 193, xx + 141, xx + 147);
  xx[111] = 0.01668052845419594;
  xx[156] = 2.96461486266998e-3;
  xx[165] = xx[110] - xx[147];
  xx[166] = xx[111] - xx[148];
  xx[167] = xx[156] - xx[149];
  pm_math_Quaternion_xform_ra(xx + 182, xx + 165, xx + 147);
  xx[165] = - 6.483073904707459e-4;
  xx[166] = - 0.01395135758917342;
  xx[167] = 3.551989250699576e-3;
  pm_math_Quaternion_xform_ra(xx + 182, xx + 165, xx + 171);
  pm_math_Quaternion_xform_ra(xx + 36, xx + 234, xx + 165);
  xx[36] = 0.03300000000000426;
  xx[298] = - pm_math_Vector3_dot_ra(xx + 120, xx + 97);
  xx[299] = - pm_math_Vector3_dot_ra(xx + 120, xx + 117);
  xx[300] = xx[47] + xx[77] - xx[103] - (xx[80] + xx[27]);
  xx[301] = - (xx[81] + xx[83] - (xx[48] + xx[78] - xx[104]) + xx[84]);
  xx[302] = xx[49] + xx[79] - xx[105] - (xx[82] + xx[109]);
  xx[303] = - pm_math_Vector3_dot_ra(xx + 228, xx + 205);
  xx[304] = - pm_math_Vector3_dot_ra(xx + 228, xx + 224);
  xx[305] = xx[132] + xx[147] - xx[171] - (xx[165] + xx[27]);
  xx[306] = - (xx[166] + xx[83] - (xx[133] + xx[148] - xx[172]) + xx[36]);
  xx[307] = xx[134] + xx[149] - xx[173] - (xx[167] + xx[109]);
  xx[27] = 1.0e-8;
  memcpy(xx + 308, xx + 258, 40 * sizeof(double));
  factorAndSolveWide(10, 4, xx + 308, xx + 348, xx + 358, ii + 0, xx + 298, xx
                     [27], xx + 224);
  xx[37] = state[2] + xx[224];
  xx[38] = xx[37] * xx[4];
  xx[39] = sin(xx[38]);
  xx[77] = cos(xx[38]);
  xx[78] = - (xx[7] * xx[39]);
  xx[79] = xx[19] * xx[39];
  xx[80] = - (xx[20] * xx[39]);
  pm_math_Quaternion_compose_ra(xx + 10, xx + 77, xx + 117);
  pm_math_Quaternion_compose_ra(xx + 14, xx + 117, xx + 77);
  xx[38] = state[4] + xx[225];
  xx[39] = xx[38] * xx[4];
  xx[47] = sin(xx[39]);
  xx[189] = cos(xx[39]);
  xx[190] = - (xx[18] * xx[47]);
  xx[191] = xx[34] * xx[47];
  xx[192] = - (xx[35] * xx[47]);
  pm_math_Quaternion_compose_ra(xx + 29, xx + 189, xx + 193);
  pm_math_Quaternion_compose_ra(xx + 77, xx + 193, xx + 189);
  pm_math_Quaternion_inverseXform_ra(xx + 193, xx + 44, xx + 47);
  pm_math_Vector3_cross_ra(xx + 47, xx + 50, xx + 81);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 81, xx + 97);
  xx[39] = state[8] + xx[226];
  xx[81] = xx[39] * xx[4];
  xx[82] = sin(xx[81]);
  xx[204] = cos(xx[81]);
  xx[205] = - (xx[63] * xx[82]);
  xx[206] = xx[75] * xx[82];
  xx[207] = - (xx[76] * xx[82]);
  pm_math_Quaternion_compose_ra(xx + 66, xx + 204, xx + 252);
  pm_math_Quaternion_compose_ra(xx + 70, xx + 252, xx + 204);
  pm_math_Quaternion_compose_ra(xx + 204, xx + 85, xx + 256);
  xx[81] = xx[259] * xx[259];
  xx[82] = xx[257] * xx[258];
  xx[83] = xx[256] * xx[259];
  xx[132] = xx[53] - (xx[258] * xx[258] + xx[81]) * xx[94];
  xx[133] = (xx[82] + xx[83]) * xx[94];
  xx[134] = xx[94] * (xx[257] * xx[259] - xx[256] * xx[258]);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 100, xx + 147);
  pm_math_Quaternion_xform_ra(xx + 204, xx + 106, xx + 165);
  pm_math_Quaternion_compose_ra(xx + 189, xx + 112, xx + 260);
  xx[211] = (xx[261] * xx[263] + xx[260] * xx[262]) * xx[94];
  xx[212] = xx[94] * (xx[262] * xx[263] - xx[260] * xx[261]);
  xx[213] = xx[53] - (xx[261] * xx[261] + xx[262] * xx[262]) * xx[94];
  xx[217] = xx[94] * (xx[82] - xx[83]);
  xx[218] = xx[53] - (xx[257] * xx[257] + xx[81]) * xx[94];
  xx[219] = (xx[258] * xx[259] + xx[256] * xx[257]) * xx[94];
  pm_math_Quaternion_xform_ra(xx + 204, xx + 89, xx + 81);
  pm_math_Vector3_cross_ra(xx + 47, xx + 126, xx + 240);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 240, xx + 246);
  pm_math_Quaternion_xform_ra(xx + 193, xx + 129, xx + 240);
  xx[256] = - (xx[92] + xx[240]);
  xx[257] = xx[93] - xx[241];
  xx[258] = xx[95] - xx[242];
  pm_math_Vector3_cross_ra(xx + 44, xx + 256, xx + 240);
  pm_math_Quaternion_xform_ra(xx + 117, xx + 240, xx + 259);
  pm_math_Quaternion_xform_ra(xx + 117, xx + 135, xx + 240);
  xx[262] = xx[259] + xx[240];
  xx[263] = xx[260] + xx[241];
  xx[264] = xx[261] + xx[242];
  pm_math_Quaternion_xform_ra(xx + 14, xx + 262, xx + 240);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 144, xx + 259);
  pm_math_Quaternion_xform_ra(xx + 193, xx + 150, xx + 262);
  pm_math_Quaternion_xform_ra(xx + 77, xx + 262, xx + 193);
  pm_math_Quaternion_xform_ra(xx + 204, xx + 153, xx + 262);
  pm_math_Quaternion_xform_ra(xx + 252, xx + 159, xx + 265);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 265, xx + 268);
  pm_math_Vector3_cross_ra(xx + 47, xx + 162, xx + 265);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 265, xx + 271);
  xx[109] = state[12] + xx[227];
  xx[121] = xx[109] * xx[4];
  xx[122] = sin(xx[121]);
  xx[224] = cos(xx[121]);
  xx[225] = - (xx[168] * xx[122]);
  xx[226] = xx[187] * xx[122];
  xx[227] = - (xx[188] * xx[122]);
  pm_math_Quaternion_compose_ra(xx + 178, xx + 224, xx + 228);
  pm_math_Quaternion_compose_ra(xx + 182, xx + 228, xx + 224);
  pm_math_Quaternion_compose_ra(xx + 224, xx + 197, xx + 274);
  xx[121] = xx[277] * xx[277];
  xx[122] = xx[275] * xx[276];
  xx[157] = xx[274] * xx[277];
  xx[265] = xx[53] - (xx[276] * xx[276] + xx[121]) * xx[94];
  xx[266] = (xx[122] + xx[157]) * xx[94];
  xx[267] = xx[94] * (xx[275] * xx[277] - xx[274] * xx[276]);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 208, xx + 278);
  pm_math_Quaternion_xform_ra(xx + 224, xx + 214, xx + 281);
  pm_math_Quaternion_compose_ra(xx + 189, xx + 220, xx + 284);
  xx[288] = (xx[285] * xx[287] + xx[284] * xx[286]) * xx[94];
  xx[289] = xx[94] * (xx[286] * xx[287] - xx[284] * xx[285]);
  xx[290] = xx[53] - (xx[285] * xx[285] + xx[286] * xx[286]) * xx[94];
  xx[284] = xx[94] * (xx[122] - xx[157]);
  xx[285] = xx[53] - (xx[275] * xx[275] + xx[121]) * xx[94];
  xx[286] = (xx[276] * xx[277] + xx[274] * xx[275]) * xx[94];
  pm_math_Quaternion_xform_ra(xx + 224, xx + 201, xx + 274);
  pm_math_Vector3_cross_ra(xx + 47, xx + 234, xx + 291);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 291, xx + 47);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 237, xx + 291);
  pm_math_Quaternion_xform_ra(xx + 224, xx + 243, xx + 294);
  pm_math_Quaternion_xform_ra(xx + 228, xx + 249, xx + 297);
  pm_math_Quaternion_xform_ra(xx + 182, xx + 297, xx + 300);
  xx[303] = pm_math_Vector3_dot_ra(xx + 97, xx + 132);
  xx[304] = pm_math_Vector3_dot_ra(xx + 147, xx + 132);
  xx[305] = pm_math_Vector3_dot_ra(xx + 165, xx + 211);
  xx[306] = xx[116];
  xx[307] = pm_math_Vector3_dot_ra(xx + 97, xx + 217);
  xx[308] = pm_math_Vector3_dot_ra(xx + 147, xx + 217);
  xx[309] = pm_math_Vector3_dot_ra(xx + 81, xx + 211);
  xx[310] = xx[116];
  xx[311] = xx[246] + xx[240];
  xx[312] = xx[259] + xx[193];
  xx[313] = - (xx[262] + xx[268]);
  xx[314] = xx[116];
  xx[315] = xx[247] + xx[241];
  xx[316] = xx[260] + xx[194];
  xx[317] = - (xx[263] + xx[269]);
  xx[318] = xx[116];
  xx[319] = xx[248] + xx[242];
  xx[320] = xx[261] + xx[195];
  xx[321] = - (xx[264] + xx[270]);
  xx[322] = xx[116];
  xx[323] = pm_math_Vector3_dot_ra(xx + 271, xx + 265);
  xx[324] = pm_math_Vector3_dot_ra(xx + 278, xx + 265);
  xx[325] = xx[116];
  xx[326] = pm_math_Vector3_dot_ra(xx + 281, xx + 288);
  xx[327] = pm_math_Vector3_dot_ra(xx + 271, xx + 284);
  xx[328] = pm_math_Vector3_dot_ra(xx + 278, xx + 284);
  xx[329] = xx[116];
  xx[330] = pm_math_Vector3_dot_ra(xx + 274, xx + 288);
  xx[331] = xx[47] + xx[240];
  xx[332] = xx[291] + xx[193];
  xx[333] = xx[116];
  xx[334] = - (xx[294] + xx[300]);
  xx[335] = xx[48] + xx[241];
  xx[336] = xx[292] + xx[194];
  xx[337] = xx[116];
  xx[338] = - (xx[295] + xx[301]);
  xx[339] = xx[49] + xx[242];
  xx[340] = xx[293] + xx[195];
  xx[341] = xx[116];
  xx[342] = - (xx[296] + xx[302]);
  pm_math_Quaternion_xform_ra(xx + 204, xx + 40, xx + 47);
  pm_math_Quaternion_xform_ra(xx + 252, xx + 56, xx + 81);
  xx[97] = xx[43] - xx[81];
  xx[98] = xx[54] - xx[82];
  xx[99] = xx[74] - xx[83];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 97, xx + 81);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 126, xx + 97);
  pm_math_Quaternion_xform_ra(xx + 77, xx + 256, xx + 147);
  pm_math_Quaternion_xform_ra(xx + 117, xx + 22, xx + 77);
  xx[117] = xx[21] - xx[77];
  xx[118] = xx[25] - xx[78];
  xx[119] = xx[26] - xx[79];
  pm_math_Quaternion_xform_ra(xx + 14, xx + 117, xx + 77);
  xx[80] = xx[147] + xx[77] - xx[138];
  xx[117] = xx[148] + xx[78] - xx[139] - xx[28];
  xx[77] = xx[149] + xx[79] - xx[140];
  pm_math_Quaternion_xform_ra(xx + 224, xx + 123, xx + 118);
  pm_math_Quaternion_xform_ra(xx + 228, xx + 141, xx + 147);
  xx[165] = xx[110] - xx[147];
  xx[166] = xx[111] - xx[148];
  xx[167] = xx[156] - xx[149];
  pm_math_Quaternion_xform_ra(xx + 182, xx + 165, xx + 147);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 234, xx + 165);
  xx[224] = - pm_math_Vector3_dot_ra(xx + 211, xx + 132);
  xx[225] = - pm_math_Vector3_dot_ra(xx + 211, xx + 217);
  xx[226] = xx[47] + xx[81] - xx[103] - (xx[97] + xx[80]);
  xx[227] = - (xx[98] + xx[117] - (xx[48] + xx[82] - xx[104]) + xx[84]);
  xx[228] = xx[49] + xx[83] - xx[105] - (xx[99] + xx[77]);
  xx[229] = - pm_math_Vector3_dot_ra(xx + 288, xx + 265);
  xx[230] = - pm_math_Vector3_dot_ra(xx + 288, xx + 284);
  xx[231] = xx[118] + xx[147] - xx[171] - (xx[165] + xx[80]);
  xx[232] = - (xx[166] + xx[117] - (xx[119] + xx[148] - xx[172]) + xx[36]);
  xx[233] = xx[120] + xx[149] - xx[173] - (xx[167] + xx[77]);
  memcpy(xx + 262, xx + 303, 40 * sizeof(double));
  factorAndSolveWide(10, 4, xx + 262, xx + 343, xx + 353, ii + 0, xx + 224, xx
                     [27], xx + 252);
  xx[47] = xx[37] + xx[252];
  xx[37] = xx[38] + xx[253];
  xx[38] = xx[39] + xx[254];
  xx[39] = xx[109] + xx[255];
  xx[252] = state[0];
  xx[253] = state[1];
  xx[254] = xx[47];
  xx[255] = state[3];
  xx[256] = xx[37];
  xx[257] = state[5];
  xx[258] = state[6];
  xx[259] = state[7];
  xx[260] = xx[38];
  xx[261] = state[9];
  xx[262] = state[10];
  xx[263] = state[11];
  xx[264] = xx[39];
  xx[265] = state[13];
  xx[266] = state[14];
  xx[267] = state[15];
  xx[268] = state[16];
  xx[269] = state[17];
  xx[48] = xx[47] * xx[4];
  xx[47] = sin(xx[48]);
  xx[77] = cos(xx[48]);
  xx[78] = - (xx[7] * xx[47]);
  xx[79] = xx[19] * xx[47];
  xx[80] = - (xx[20] * xx[47]);
  pm_math_Quaternion_compose_ra(xx + 10, xx + 77, xx + 117);
  pm_math_Quaternion_compose_ra(xx + 14, xx + 117, xx + 77);
  xx[47] = xx[37] * xx[4];
  xx[37] = sin(xx[47]);
  xx[189] = cos(xx[47]);
  xx[190] = - (xx[18] * xx[37]);
  xx[191] = xx[34] * xx[37];
  xx[192] = - (xx[35] * xx[37]);
  pm_math_Quaternion_compose_ra(xx + 29, xx + 189, xx + 193);
  pm_math_Quaternion_compose_ra(xx + 77, xx + 193, xx + 189);
  pm_math_Quaternion_compose_ra(xx + 189, xx + 112, xx + 204);
  xx[47] = (xx[205] * xx[207] + xx[204] * xx[206]) * xx[94];
  xx[48] = xx[94] * (xx[206] * xx[207] - xx[204] * xx[205]);
  xx[49] = xx[53] - (xx[205] * xx[205] + xx[206] * xx[206]) * xx[94];
  xx[37] = xx[38] * xx[4];
  xx[38] = sin(xx[37]);
  xx[204] = cos(xx[37]);
  xx[205] = - (xx[63] * xx[38]);
  xx[206] = xx[75] * xx[38];
  xx[207] = - (xx[76] * xx[38]);
  pm_math_Quaternion_compose_ra(xx + 66, xx + 204, xx + 224);
  pm_math_Quaternion_compose_ra(xx + 70, xx + 224, xx + 204);
  pm_math_Quaternion_compose_ra(xx + 204, xx + 85, xx + 228);
  xx[37] = xx[231] * xx[231];
  xx[38] = xx[229] * xx[230];
  xx[81] = xx[228] * xx[231];
  xx[97] = xx[53] - (xx[230] * xx[230] + xx[37]) * xx[94];
  xx[98] = (xx[38] + xx[81]) * xx[94];
  xx[99] = xx[94] * (xx[229] * xx[231] - xx[228] * xx[230]);
  xx[132] = xx[94] * (xx[38] - xx[81]);
  xx[133] = xx[53] - (xx[229] * xx[229] + xx[37]) * xx[94];
  xx[134] = (xx[230] * xx[231] + xx[228] * xx[229]) * xx[94];
  pm_math_Quaternion_xform_ra(xx + 189, xx + 126, xx + 81);
  pm_math_Quaternion_xform_ra(xx + 193, xx + 129, xx + 147);
  xx[165] = - (xx[92] + xx[147]);
  xx[166] = xx[93] - xx[148];
  xx[167] = xx[95] - xx[149];
  pm_math_Quaternion_xform_ra(xx + 77, xx + 165, xx + 147);
  pm_math_Quaternion_xform_ra(xx + 117, xx + 22, xx + 77);
  xx[117] = xx[21] - xx[77];
  xx[118] = xx[25] - xx[78];
  xx[119] = xx[26] - xx[79];
  pm_math_Quaternion_xform_ra(xx + 14, xx + 117, xx + 77);
  xx[14] = xx[147] + xx[77] - xx[138];
  pm_math_Quaternion_xform_ra(xx + 204, xx + 40, xx + 15);
  pm_math_Quaternion_xform_ra(xx + 224, xx + 56, xx + 117);
  xx[120] = xx[43] - xx[117];
  xx[121] = xx[54] - xx[118];
  xx[122] = xx[74] - xx[119];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 120, xx + 117);
  xx[37] = xx[148] + xx[78] - xx[139] - xx[28];
  xx[28] = xx[149] + xx[79] - xx[140];
  pm_math_Quaternion_compose_ra(xx + 189, xx + 220, xx + 70);
  xx[77] = (xx[71] * xx[73] + xx[70] * xx[72]) * xx[94];
  xx[78] = xx[94] * (xx[72] * xx[73] - xx[70] * xx[71]);
  xx[79] = xx[53] - (xx[71] * xx[71] + xx[72] * xx[72]) * xx[94];
  xx[38] = xx[39] * xx[4];
  xx[39] = sin(xx[38]);
  xx[70] = cos(xx[38]);
  xx[71] = - (xx[168] * xx[39]);
  xx[72] = xx[187] * xx[39];
  xx[73] = - (xx[188] * xx[39]);
  pm_math_Quaternion_compose_ra(xx + 178, xx + 70, xx + 193);
  pm_math_Quaternion_compose_ra(xx + 182, xx + 193, xx + 70);
  pm_math_Quaternion_compose_ra(xx + 70, xx + 197, xx + 204);
  xx[38] = xx[207] * xx[207];
  xx[39] = xx[205] * xx[206];
  xx[80] = xx[204] * xx[207];
  xx[120] = xx[53] - (xx[206] * xx[206] + xx[38]) * xx[94];
  xx[121] = (xx[39] + xx[80]) * xx[94];
  xx[122] = xx[94] * (xx[205] * xx[207] - xx[204] * xx[206]);
  xx[138] = xx[94] * (xx[39] - xx[80]);
  xx[139] = xx[53] - (xx[205] * xx[205] + xx[38]) * xx[94];
  xx[140] = (xx[206] * xx[207] + xx[204] * xx[205]) * xx[94];
  pm_math_Quaternion_xform_ra(xx + 189, xx + 234, xx + 147);
  pm_math_Quaternion_xform_ra(xx + 70, xx + 123, xx + 165);
  pm_math_Quaternion_xform_ra(xx + 193, xx + 141, xx + 70);
  xx[189] = xx[110] - xx[70];
  xx[190] = xx[111] - xx[71];
  xx[191] = xx[156] - xx[72];
  pm_math_Quaternion_xform_ra(xx + 182, xx + 189, xx + 70);
  xx[224] = fabs(pm_math_Vector3_dot_ra(xx + 47, xx + 97));
  xx[225] = fabs(pm_math_Vector3_dot_ra(xx + 47, xx + 132));
  xx[226] = fabs(xx[81] + xx[14] - (xx[15] + xx[117] - xx[103]));
  xx[227] = fabs(xx[82] + xx[37] - (xx[16] + xx[118] - xx[104]) + xx[84]);
  xx[228] = fabs(xx[83] + xx[28] - (xx[17] + xx[119] - xx[105]));
  xx[229] = fabs(pm_math_Vector3_dot_ra(xx + 77, xx + 120));
  xx[230] = fabs(pm_math_Vector3_dot_ra(xx + 77, xx + 138));
  xx[231] = fabs(xx[147] + xx[14] - (xx[165] + xx[70] - xx[171]));
  xx[232] = fabs(xx[148] + xx[37] - (xx[166] + xx[71] - xx[172]) + xx[36]);
  xx[233] = fabs(xx[149] + xx[28] - (xx[167] + xx[72] - xx[173]));
  ii[0] = 224;

  {
    int ll;
    for (ll = 225; ll < 234; ++ll)
      if (xx[ll] > xx[ii[0]])
        ii[0] = ll;
  }

  ii[0] -= 224;
  xx[14] = xx[224 + (ii[0])];
  xx[15] = 1.0e-5;
  if (xx[14] > xx[15]) {
    switch (ii[0])
    {
     case 0:
     case 1:
     case 2:
     case 3:
     case 4:
      {
        return sm_ssci_recordRunTimeError(
          "physmod:sm:core:compiler:mechanism:mechanism:constraintViolation",
          "'DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 2/R23' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem.",
          neDiagMgr);
      }

     case 5:
     case 6:
     case 7:
     case 8:
     case 9:
      {
        return sm_ssci_recordRunTimeError(
          "physmod:sm:core:compiler:mechanism:mechanism:constraintViolation",
          "'DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 3/R33' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem.",
          neDiagMgr);
      }
    }
  }

  xx[14] = xx[4] * xx[252];
  xx[16] = sin(xx[14]);
  xx[36] = cos(xx[14]);
  xx[37] = xx[6] * xx[16];
  xx[38] = xx[8] * xx[16];
  xx[39] = xx[9] * xx[16];
  pm_math_Quaternion_compose_ra(xx + 0, xx + 36, xx + 70);
  xx[14] = xx[4] * xx[254];
  xx[16] = sin(xx[14]);
  xx[36] = cos(xx[14]);
  xx[37] = - (xx[7] * xx[16]);
  xx[38] = xx[19] * xx[16];
  xx[39] = - (xx[20] * xx[16]);
  pm_math_Quaternion_compose_ra(xx + 10, xx + 36, xx + 77);
  pm_math_Quaternion_compose_ra(xx + 70, xx + 77, xx + 36);
  xx[14] = xx[4] * xx[256];
  xx[16] = sin(xx[14]);
  xx[81] = cos(xx[14]);
  xx[82] = - (xx[18] * xx[16]);
  xx[83] = xx[34] * xx[16];
  xx[84] = - (xx[35] * xx[16]);
  pm_math_Quaternion_compose_ra(xx + 29, xx + 81, xx + 117);
  pm_math_Quaternion_compose_ra(xx + 36, xx + 117, xx + 81);
  pm_math_Quaternion_inverseXform_ra(xx + 117, xx + 44, xx + 47);
  pm_math_Vector3_cross_ra(xx + 47, xx + 50, xx + 97);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 97, xx + 103);
  xx[14] = xx[4] * xx[258];
  xx[16] = sin(xx[14]);
  xx[182] = cos(xx[14]);
  xx[183] = xx[55] * xx[16];
  xx[184] = xx[64] * xx[16];
  xx[185] = xx[65] * xx[16];
  pm_math_Quaternion_compose_ra(xx + 59, xx + 182, xx + 189);
  xx[14] = xx[4] * xx[260];
  xx[16] = sin(xx[14]);
  xx[182] = cos(xx[14]);
  xx[183] = - (xx[63] * xx[16]);
  xx[184] = xx[75] * xx[16];
  xx[185] = - (xx[76] * xx[16]);
  pm_math_Quaternion_compose_ra(xx + 66, xx + 182, xx + 193);
  pm_math_Quaternion_compose_ra(xx + 189, xx + 193, xx + 182);
  pm_math_Quaternion_compose_ra(xx + 182, xx + 85, xx + 204);
  xx[14] = xx[207] * xx[207];
  xx[16] = xx[205] * xx[206];
  xx[17] = xx[204] * xx[207];
  xx[85] = xx[53] - (xx[206] * xx[206] + xx[14]) * xx[94];
  xx[86] = (xx[16] + xx[17]) * xx[94];
  xx[87] = xx[94] * (xx[205] * xx[207] - xx[204] * xx[206]);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 100, xx + 97);
  pm_math_Quaternion_xform_ra(xx + 182, xx + 106, xx + 100);
  pm_math_Quaternion_compose_ra(xx + 81, xx + 112, xx + 106);
  xx[112] = (xx[107] * xx[109] + xx[106] * xx[108]) * xx[94];
  xx[113] = xx[94] * (xx[108] * xx[109] - xx[106] * xx[107]);
  xx[114] = xx[53] - (xx[107] * xx[107] + xx[108] * xx[108]) * xx[94];
  xx[106] = xx[94] * (xx[16] - xx[17]);
  xx[107] = xx[53] - (xx[205] * xx[205] + xx[14]) * xx[94];
  xx[108] = (xx[206] * xx[207] + xx[204] * xx[205]) * xx[94];
  pm_math_Quaternion_xform_ra(xx + 182, xx + 89, xx + 132);
  pm_math_Vector3_cross_ra(xx + 47, xx + 126, xx + 88);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 88, xx + 138);
  pm_math_Quaternion_xform_ra(xx + 117, xx + 129, xx + 88);
  xx[129] = - (xx[92] + xx[88]);
  xx[130] = xx[93] - xx[89];
  xx[131] = xx[95] - xx[90];
  pm_math_Vector3_cross_ra(xx + 44, xx + 129, xx + 88);
  pm_math_Quaternion_xform_ra(xx + 77, xx + 88, xx + 44);
  pm_math_Quaternion_xform_ra(xx + 77, xx + 135, xx + 88);
  xx[91] = xx[44] + xx[88];
  xx[92] = xx[45] + xx[89];
  xx[93] = xx[46] + xx[90];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 91, xx + 44);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 144, xx + 91);
  pm_math_Quaternion_xform_ra(xx + 117, xx + 150, xx + 135);
  pm_math_Quaternion_xform_ra(xx + 36, xx + 135, xx + 144);
  pm_math_Quaternion_xform_ra(xx + 182, xx + 153, xx + 147);
  pm_math_Quaternion_xform_ra(xx + 193, xx + 159, xx + 150);
  pm_math_Quaternion_xform_ra(xx + 189, xx + 150, xx + 153);
  pm_math_Vector3_cross_ra(xx + 47, xx + 162, xx + 157);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 157, xx + 165);
  xx[14] = xx[4] * xx[262];
  xx[16] = sin(xx[14]);
  xx[157] = cos(xx[14]);
  xx[158] = xx[96] * xx[16];
  xx[159] = xx[169] * xx[16];
  xx[160] = xx[170] * xx[16];
  pm_math_Quaternion_compose_ra(xx + 174, xx + 157, xx + 204);
  xx[14] = xx[4] * xx[264];
  xx[16] = sin(xx[14]);
  xx[157] = cos(xx[14]);
  xx[158] = - (xx[168] * xx[16]);
  xx[159] = xx[187] * xx[16];
  xx[160] = - (xx[188] * xx[16]);
  pm_math_Quaternion_compose_ra(xx + 178, xx + 157, xx + 224);
  pm_math_Quaternion_compose_ra(xx + 204, xx + 224, xx + 157);
  pm_math_Quaternion_compose_ra(xx + 157, xx + 197, xx + 228);
  xx[14] = xx[231] * xx[231];
  xx[16] = xx[229] * xx[230];
  xx[17] = xx[228] * xx[231];
  xx[171] = xx[53] - (xx[230] * xx[230] + xx[14]) * xx[94];
  xx[172] = (xx[16] + xx[17]) * xx[94];
  xx[173] = xx[94] * (xx[229] * xx[231] - xx[228] * xx[230]);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 208, xx + 197);
  pm_math_Quaternion_xform_ra(xx + 157, xx + 214, xx + 208);
  pm_math_Quaternion_compose_ra(xx + 81, xx + 220, xx + 211);
  xx[215] = (xx[212] * xx[214] + xx[211] * xx[213]) * xx[94];
  xx[216] = xx[94] * (xx[213] * xx[214] - xx[211] * xx[212]);
  xx[217] = xx[53] - (xx[212] * xx[212] + xx[213] * xx[213]) * xx[94];
  xx[211] = xx[94] * (xx[16] - xx[17]);
  xx[212] = xx[53] - (xx[229] * xx[229] + xx[14]) * xx[94];
  xx[213] = (xx[230] * xx[231] + xx[228] * xx[229]) * xx[94];
  pm_math_Quaternion_xform_ra(xx + 157, xx + 201, xx + 218);
  pm_math_Vector3_cross_ra(xx + 47, xx + 234, xx + 200);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 200, xx + 47);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 237, xx + 200);
  pm_math_Quaternion_xform_ra(xx + 157, xx + 243, xx + 221);
  pm_math_Quaternion_xform_ra(xx + 224, xx + 249, xx + 228);
  pm_math_Quaternion_xform_ra(xx + 204, xx + 228, xx + 231);
  xx[270] = pm_math_Vector3_dot_ra(xx + 103, xx + 85);
  xx[271] = pm_math_Vector3_dot_ra(xx + 97, xx + 85);
  xx[272] = pm_math_Vector3_dot_ra(xx + 100, xx + 112);
  xx[273] = xx[116];
  xx[274] = pm_math_Vector3_dot_ra(xx + 103, xx + 106);
  xx[275] = pm_math_Vector3_dot_ra(xx + 97, xx + 106);
  xx[276] = pm_math_Vector3_dot_ra(xx + 132, xx + 112);
  xx[277] = xx[116];
  xx[278] = xx[138] + xx[44];
  xx[279] = xx[91] + xx[144];
  xx[280] = - (xx[147] + xx[153]);
  xx[281] = xx[116];
  xx[282] = xx[139] + xx[45];
  xx[283] = xx[92] + xx[145];
  xx[284] = - (xx[148] + xx[154]);
  xx[285] = xx[116];
  xx[286] = xx[140] + xx[46];
  xx[287] = xx[93] + xx[146];
  xx[288] = - (xx[149] + xx[155]);
  xx[289] = xx[116];
  xx[290] = pm_math_Vector3_dot_ra(xx + 165, xx + 171);
  xx[291] = pm_math_Vector3_dot_ra(xx + 197, xx + 171);
  xx[292] = xx[116];
  xx[293] = pm_math_Vector3_dot_ra(xx + 208, xx + 215);
  xx[294] = pm_math_Vector3_dot_ra(xx + 165, xx + 211);
  xx[295] = pm_math_Vector3_dot_ra(xx + 197, xx + 211);
  xx[296] = xx[116];
  xx[297] = pm_math_Vector3_dot_ra(xx + 218, xx + 215);
  xx[298] = xx[47] + xx[44];
  xx[299] = xx[200] + xx[144];
  xx[300] = xx[116];
  xx[301] = - (xx[221] + xx[231]);
  xx[302] = xx[48] + xx[45];
  xx[303] = xx[201] + xx[145];
  xx[304] = xx[116];
  xx[305] = - (xx[222] + xx[232]);
  xx[306] = xx[49] + xx[46];
  xx[307] = xx[202] + xx[146];
  xx[308] = xx[116];
  xx[309] = - (xx[223] + xx[233]);
  xx[44] = xx[6] * xx[253];
  xx[45] = xx[8] * xx[253];
  xx[46] = xx[9] * xx[253];
  pm_math_Quaternion_inverseXform_ra(xx + 77, xx + 44, xx + 47);
  xx[91] = xx[47] - xx[7] * xx[255];
  xx[92] = xx[48] + xx[19] * xx[255];
  xx[93] = xx[49] - xx[20] * xx[255];
  pm_math_Quaternion_inverseXform_ra(xx + 117, xx + 91, xx + 97);
  xx[100] = xx[97] - xx[18] * xx[257];
  xx[101] = xx[98] + xx[34] * xx[257];
  xx[102] = xx[99] - xx[35] * xx[257];
  pm_math_Vector3_cross_ra(xx + 100, xx + 50, xx + 97);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 97, xx + 103);
  xx[97] = xx[55] * xx[259];
  xx[98] = xx[64] * xx[259];
  xx[99] = xx[65] * xx[259];
  pm_math_Quaternion_inverseXform_ra(xx + 193, xx + 97, xx + 132);
  xx[138] = xx[132] - xx[63] * xx[261];
  xx[139] = xx[133] + xx[75] * xx[261];
  xx[140] = xx[134] - xx[76] * xx[261];
  xx[14] = 0.3300601612855789;
  xx[16] = 0.7485549122714541;
  xx[144] = xx[14];
  xx[145] = xx[16];
  xx[146] = 0.5750876743996639;
  pm_math_Vector3_cross_ra(xx + 138, xx + 144, xx + 147);
  pm_math_Quaternion_xform_ra(xx + 182, xx + 147, xx + 153);
  xx[147] = 0.1523779706481012;
  xx[148] = - 0.643489458684202;
  xx[149] = 0.7501348349620082;
  pm_math_Vector3_cross_ra(xx + 138, xx + 147, xx + 165);
  pm_math_Quaternion_xform_ra(xx + 182, xx + 165, xx + 197);
  pm_math_Vector3_cross_ra(xx + 138, xx + 40, xx + 165);
  pm_math_Quaternion_xform_ra(xx + 182, xx + 165, xx + 138);
  xx[165] = 3.977909360125871e-3;
  xx[166] = 4.397193980568331e-3;
  xx[167] = - 0.012326617731341;
  pm_math_Quaternion_xform_ra(xx + 189, xx + 165, xx + 200);
  xx[17] = xx[200] * xx[259];
  pm_math_Quaternion_xform_ra(xx + 193, xx + 56, xx + 165);
  xx[56] = xx[43] - xx[165];
  xx[57] = xx[54] - xx[166];
  xx[58] = xx[74] - xx[167];
  pm_math_Vector3_cross_ra(xx + 97, xx + 56, xx + 165);
  xx[56] = xx[150] * xx[261] + xx[165];
  xx[57] = xx[151] * xx[261] + xx[166];
  xx[58] = xx[152] * xx[261] + xx[167];
  pm_math_Quaternion_xform_ra(xx + 189, xx + 56, xx + 97);
  pm_math_Vector3_cross_ra(xx + 100, xx + 126, xx + 56);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 56, xx + 193);
  xx[56] = 0.01500104515676648;
  xx[57] = 1.380515327287848e-10;
  xx[58] = - 1.593463070824624e-9;
  pm_math_Quaternion_xform_ra(xx + 70, xx + 56, xx + 208);
  xx[28] = xx[208] * xx[253];
  pm_math_Quaternion_xform_ra(xx + 77, xx + 22, xx + 56);
  xx[22] = xx[21] - xx[56];
  xx[23] = xx[25] - xx[57];
  xx[24] = xx[26] - xx[58];
  pm_math_Vector3_cross_ra(xx + 44, xx + 22, xx + 56);
  xx[21] = xx[88] * xx[255] + xx[56];
  xx[22] = xx[89] * xx[255] + xx[57];
  xx[23] = xx[90] * xx[255] + xx[58];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 21, xx + 24);
  pm_math_Vector3_cross_ra(xx + 91, xx + 129, xx + 21);
  xx[43] = xx[135] * xx[257] + xx[21];
  xx[44] = xx[136] * xx[257] + xx[22];
  xx[45] = xx[137] * xx[257] + xx[23];
  pm_math_Quaternion_xform_ra(xx + 36, xx + 43, xx + 21);
  xx[43] = xx[28] + xx[24] + xx[21];
  xx[44] = xx[201] * xx[259];
  xx[45] = xx[209] * xx[253];
  xx[46] = xx[45] + xx[25] + xx[22];
  xx[53] = xx[202] * xx[259];
  xx[54] = xx[253] * xx[210];
  xx[21] = xx[54] + xx[26] + xx[23];
  pm_math_Vector3_cross_ra(xx + 100, xx + 162, xx + 22);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 22, xx + 77);
  xx[22] = xx[96] * xx[263];
  xx[23] = xx[169] * xx[263];
  xx[24] = xx[170] * xx[263];
  pm_math_Quaternion_inverseXform_ra(xx + 224, xx + 22, xx + 91);
  xx[200] = xx[91] - xx[168] * xx[265];
  xx[201] = xx[92] + xx[187] * xx[265];
  xx[202] = xx[93] - xx[188] * xx[265];
  xx[208] = xx[14];
  xx[209] = xx[16];
  xx[210] = 0.5750876743996637;
  pm_math_Vector3_cross_ra(xx + 200, xx + 208, xx + 218);
  pm_math_Quaternion_xform_ra(xx + 157, xx + 218, xx + 221);
  xx[218] = 0.1523779706481014;
  xx[219] = - 0.6434894586842019;
  xx[220] = 0.7501348349620083;
  pm_math_Vector3_cross_ra(xx + 200, xx + 218, xx + 231);
  pm_math_Quaternion_xform_ra(xx + 157, xx + 231, xx + 237);
  pm_math_Vector3_cross_ra(xx + 200, xx + 123, xx + 231);
  pm_math_Quaternion_xform_ra(xx + 157, xx + 231, xx + 200);
  xx[231] = - 8.971513124181443e-3;
  xx[232] = 2.664040756922957e-3;
  xx[233] = 8.826233627727805e-3;
  pm_math_Quaternion_xform_ra(xx + 204, xx + 231, xx + 240);
  xx[14] = xx[240] * xx[263];
  pm_math_Quaternion_xform_ra(xx + 224, xx + 141, xx + 231);
  xx[141] = xx[110] - xx[231];
  xx[142] = xx[111] - xx[232];
  xx[143] = xx[156] - xx[233];
  pm_math_Vector3_cross_ra(xx + 22, xx + 141, xx + 109);
  xx[22] = xx[228] * xx[265] + xx[109];
  xx[23] = xx[229] * xx[265] + xx[110];
  xx[24] = xx[230] * xx[265] + xx[111];
  pm_math_Quaternion_xform_ra(xx + 204, xx + 22, xx + 141);
  pm_math_Vector3_cross_ra(xx + 100, xx + 234, xx + 22);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 22, xx + 100);
  xx[16] = xx[241] * xx[263];
  xx[22] = xx[242] * xx[263];
  xx[240] = - (pm_math_Vector3_dot_ra(xx + 103, xx + 85) +
               pm_math_Vector3_dot_ra(xx + 153, xx + 112));
  xx[241] = - (pm_math_Vector3_dot_ra(xx + 103, xx + 106) +
               pm_math_Vector3_dot_ra(xx + 197, xx + 112));
  xx[242] = xx[138] + xx[17] + xx[97] - (xx[193] + xx[43]);
  xx[243] = xx[139] + xx[44] + xx[98] - (xx[194] + xx[46]);
  xx[244] = xx[140] + xx[53] + xx[99] - (xx[195] + xx[21]);
  xx[245] = - (pm_math_Vector3_dot_ra(xx + 77, xx + 171) +
               pm_math_Vector3_dot_ra(xx + 221, xx + 215));
  xx[246] = - (pm_math_Vector3_dot_ra(xx + 77, xx + 211) +
               pm_math_Vector3_dot_ra(xx + 237, xx + 215));
  xx[247] = xx[200] + xx[14] + xx[141] - (xx[100] + xx[43]);
  xx[248] = xx[201] + xx[16] + xx[142] - (xx[101] + xx[46]);
  xx[249] = xx[202] + xx[22] + xx[143] - (xx[102] + xx[21]);
  memcpy(xx + 310, xx + 270, 40 * sizeof(double));
  factorAndSolveWide(10, 4, xx + 310, xx + 350, xx + 360, ii + 0, xx + 240, xx
                     [27], xx + 193);
  xx[21] = xx[255] + xx[193];
  xx[23] = xx[257] + xx[194];
  xx[24] = xx[261] + xx[195];
  xx[25] = xx[265] + xx[196];
  xx[270] = xx[252];
  xx[271] = xx[253];
  xx[272] = xx[254];
  xx[273] = xx[21];
  xx[274] = xx[256];
  xx[275] = xx[23];
  xx[276] = xx[258];
  xx[277] = xx[259];
  xx[278] = xx[260];
  xx[279] = xx[24];
  xx[280] = xx[262];
  xx[281] = xx[263];
  xx[282] = xx[264];
  xx[283] = xx[25];
  xx[284] = xx[266];
  xx[285] = xx[267];
  xx[286] = xx[268];
  xx[287] = xx[269];
  xx[77] = xx[47] - xx[21] * xx[7];
  xx[78] = xx[48] + xx[21] * xx[19];
  xx[79] = xx[49] - xx[21] * xx[20];
  pm_math_Quaternion_inverseXform_ra(xx + 117, xx + 77, xx + 46);
  xx[97] = xx[46] - xx[23] * xx[18];
  xx[98] = xx[47] + xx[23] * xx[34];
  xx[99] = xx[48] - xx[23] * xx[35];
  pm_math_Vector3_cross_ra(xx + 97, xx + 50, xx + 46);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 46, xx + 49);
  xx[46] = xx[132] - xx[24] * xx[63];
  xx[47] = xx[133] + xx[24] * xx[75];
  xx[48] = xx[134] - xx[24] * xx[76];
  pm_math_Vector3_cross_ra(xx + 46, xx + 144, xx + 100);
  pm_math_Quaternion_xform_ra(xx + 182, xx + 100, xx + 103);
  pm_math_Vector3_cross_ra(xx + 46, xx + 147, xx + 100);
  pm_math_Quaternion_xform_ra(xx + 182, xx + 100, xx + 115);
  pm_math_Vector3_cross_ra(xx + 97, xx + 126, xx + 100);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 100, xx + 118);
  xx[100] = xx[21] * xx[88] + xx[56];
  xx[101] = xx[21] * xx[89] + xx[57];
  xx[102] = xx[21] * xx[90] + xx[58];
  pm_math_Quaternion_xform_ra(xx + 70, xx + 100, xx + 56);
  pm_math_Vector3_cross_ra(xx + 77, xx + 129, xx + 70);
  xx[77] = xx[23] * xx[135] + xx[70];
  xx[78] = xx[23] * xx[136] + xx[71];
  xx[79] = xx[23] * xx[137] + xx[72];
  pm_math_Quaternion_xform_ra(xx + 36, xx + 77, xx + 70);
  xx[21] = xx[28] + xx[56] + xx[70];
  pm_math_Vector3_cross_ra(xx + 46, xx + 40, xx + 26);
  pm_math_Quaternion_xform_ra(xx + 182, xx + 26, xx + 36);
  xx[26] = xx[24] * xx[150] + xx[165];
  xx[27] = xx[24] * xx[151] + xx[166];
  xx[28] = xx[24] * xx[152] + xx[167];
  pm_math_Quaternion_xform_ra(xx + 189, xx + 26, xx + 39);
  xx[23] = xx[45] + xx[57] + xx[71];
  xx[24] = xx[54] + xx[58] + xx[72];
  pm_math_Vector3_cross_ra(xx + 97, xx + 162, xx + 26);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 26, xx + 45);
  xx[26] = xx[91] - xx[25] * xx[168];
  xx[27] = xx[92] + xx[25] * xx[187];
  xx[28] = xx[93] - xx[25] * xx[188];
  pm_math_Vector3_cross_ra(xx + 26, xx + 208, xx + 56);
  pm_math_Quaternion_xform_ra(xx + 157, xx + 56, xx + 70);
  pm_math_Vector3_cross_ra(xx + 26, xx + 218, xx + 56);
  pm_math_Quaternion_xform_ra(xx + 157, xx + 56, xx + 77);
  pm_math_Vector3_cross_ra(xx + 97, xx + 234, xx + 56);
  pm_math_Quaternion_xform_ra(xx + 81, xx + 56, xx + 88);
  pm_math_Vector3_cross_ra(xx + 26, xx + 123, xx + 56);
  pm_math_Quaternion_xform_ra(xx + 157, xx + 56, xx + 26);
  xx[56] = xx[25] * xx[228] + xx[109];
  xx[57] = xx[25] * xx[229] + xx[110];
  xx[58] = xx[25] * xx[230] + xx[111];
  pm_math_Quaternion_xform_ra(xx + 204, xx + 56, xx + 80);
  xx[121] = fabs(pm_math_Vector3_dot_ra(xx + 49, xx + 85) +
                 pm_math_Vector3_dot_ra(xx + 103, xx + 112));
  xx[122] = fabs(pm_math_Vector3_dot_ra(xx + 49, xx + 106) +
                 pm_math_Vector3_dot_ra(xx + 115, xx + 112));
  xx[123] = fabs(xx[118] + xx[21] - (xx[36] + xx[17] + xx[39]));
  xx[124] = fabs(xx[119] + xx[23] - (xx[37] + xx[44] + xx[40]));
  xx[125] = fabs(xx[120] + xx[24] - (xx[38] + xx[53] + xx[41]));
  xx[126] = fabs(pm_math_Vector3_dot_ra(xx + 45, xx + 171) +
                 pm_math_Vector3_dot_ra(xx + 70, xx + 215));
  xx[127] = fabs(pm_math_Vector3_dot_ra(xx + 45, xx + 211) +
                 pm_math_Vector3_dot_ra(xx + 77, xx + 215));
  xx[128] = fabs(xx[88] + xx[21] - (xx[26] + xx[14] + xx[80]));
  xx[129] = fabs(xx[89] + xx[23] - (xx[27] + xx[16] + xx[81]));
  xx[130] = fabs(xx[90] + xx[24] - (xx[28] + xx[22] + xx[82]));
  ii[0] = 121;

  {
    int ll;
    for (ll = 122; ll < 131; ++ll)
      if (xx[ll] > xx[ii[0]])
        ii[0] = ll;
  }

  ii[0] -= 121;
  xx[14] = xx[121 + (ii[0])];
  if (xx[14] > xx[15]) {
    switch (ii[0])
    {
     case 0:
     case 1:
     case 2:
     case 3:
     case 4:
      {
        return sm_ssci_recordRunTimeError(
          "physmod:sm:core:compiler:mechanism:mechanism:constraintViolation",
          "'DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 2/R23' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem.",
          neDiagMgr);
      }

     case 5:
     case 6:
     case 7:
     case 8:
     case 9:
      {
        return sm_ssci_recordRunTimeError(
          "physmod:sm:core:compiler:mechanism:mechanism:constraintViolation",
          "'DynamicValidation/Dynamic_Model_PID/SPM MODEL/LEG 3/R33' kinematic constraints cannot be maintained. Check solver type and consistency tolerance in the Simscape Solver Configuration block. Check Simulink solver type and tolerances in Model Configuration Parameters. A kinematic singularity might be the source of this problem.",
          neDiagMgr);
      }
    }
  }

  xx[14] = - 0.8969355644339009;
  xx[15] = 0.2764797434074396;
  xx[16] = 0.158748647566253;
  xx[17] = - 0.3063729942976073;
  xx[21] = xx[4] * xx[276];
  xx[22] = sin(xx[21]);
  xx[23] = cos(xx[21]);
  xx[24] = xx[55] * xx[22];
  xx[25] = xx[64] * xx[22];
  xx[26] = xx[65] * xx[22];
  pm_math_Quaternion_compose_ra(xx + 59, xx + 23, xx + 36);
  xx[21] = xx[4] * xx[278];
  xx[22] = sin(xx[21]);
  xx[23] = cos(xx[21]);
  xx[24] = - (xx[63] * xx[22]);
  xx[25] = xx[75] * xx[22];
  xx[26] = - (xx[76] * xx[22]);
  pm_math_Quaternion_compose_ra(xx + 66, xx + 23, xx + 40);
  pm_math_Quaternion_compose_ra(xx + 36, xx + 40, xx + 21);
  xx[25] = xx[4] * xx[270];
  xx[26] = sin(xx[25]);
  xx[36] = cos(xx[25]);
  xx[37] = xx[6] * xx[26];
  xx[38] = xx[8] * xx[26];
  xx[39] = xx[9] * xx[26];
  pm_math_Quaternion_compose_ra(xx + 0, xx + 36, xx + 25);
  xx[0] = xx[4] * xx[272];
  xx[1] = sin(xx[0]);
  xx[36] = cos(xx[0]);
  xx[37] = - (xx[7] * xx[1]);
  xx[38] = xx[19] * xx[1];
  xx[39] = - (xx[20] * xx[1]);
  pm_math_Quaternion_compose_ra(xx + 10, xx + 36, xx + 0);
  pm_math_Quaternion_compose_ra(xx + 25, xx + 0, xx + 10);
  xx[25] = xx[4] * xx[274];
  xx[26] = sin(xx[25]);
  xx[36] = cos(xx[25]);
  xx[37] = - (xx[18] * xx[26]);
  xx[38] = xx[34] * xx[26];
  xx[39] = - (xx[35] * xx[26]);
  pm_math_Quaternion_compose_ra(xx + 29, xx + 36, xx + 25);
  pm_math_Quaternion_compose_ra(xx + 10, xx + 25, xx + 29);
  pm_math_Quaternion_inverseCompose_ra(xx + 21, xx + 29, xx + 10);
  pm_math_Quaternion_inverseCompose_ra(xx + 14, xx + 10, xx + 21);
  xx[14] = xx[5];
  xx[15] = xx[33];
  xx[16] = - 0.7469766922300272;
  xx[36] = xx[6] * xx[271];
  xx[37] = xx[8] * xx[271];
  xx[38] = xx[9] * xx[271];
  pm_math_Quaternion_inverseXform_ra(xx + 0, xx + 36, xx + 44);
  xx[0] = xx[44] - xx[7] * xx[273];
  xx[1] = xx[45] + xx[19] * xx[273];
  xx[2] = xx[46] - xx[20] * xx[273];
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 0, xx + 5);
  xx[0] = xx[5] - xx[18] * xx[275];
  xx[1] = xx[55] * xx[277];
  xx[2] = xx[64] * xx[277];
  xx[3] = xx[65] * xx[277];
  pm_math_Quaternion_inverseXform_ra(xx + 40, xx + 1, xx + 17);
  xx[1] = xx[17] - xx[63] * xx[279];
  xx[2] = xx[18] + xx[75] * xx[279];
  xx[3] = xx[19] - xx[76] * xx[279];
  pm_math_Quaternion_inverseXform_ra(xx + 10, xx + 1, xx + 17);
  xx[1] = xx[6] + xx[34] * xx[275];
  xx[2] = xx[7] - xx[35] * xx[275];
  xx[5] = xx[0] - xx[17];
  xx[6] = xx[1] - xx[18];
  xx[7] = xx[2] - xx[19];
  xx[8] = 0.1299832206933693;
  xx[9] = 0.05267645187405559;
  xx[10] = - 0.9285287989198832;
  xx[11] = - 0.3437496521198803;
  xx[3] = xx[4] * xx[280];
  xx[12] = sin(xx[3]);
  xx[17] = cos(xx[3]);
  xx[18] = xx[96] * xx[12];
  xx[19] = xx[169] * xx[12];
  xx[20] = xx[170] * xx[12];
  pm_math_Quaternion_compose_ra(xx + 174, xx + 17, xx + 25);
  xx[3] = xx[4] * xx[282];
  xx[4] = sin(xx[3]);
  xx[17] = cos(xx[3]);
  xx[18] = - (xx[168] * xx[4]);
  xx[19] = xx[187] * xx[4];
  xx[20] = - (xx[188] * xx[4]);
  pm_math_Quaternion_compose_ra(xx + 178, xx + 17, xx + 33);
  pm_math_Quaternion_compose_ra(xx + 25, xx + 33, xx + 17);
  pm_math_Quaternion_inverseCompose_ra(xx + 17, xx + 29, xx + 25);
  pm_math_Quaternion_inverseCompose_ra(xx + 8, xx + 25, xx + 17);
  xx[8] = xx[96] * xx[281];
  xx[9] = xx[169] * xx[281];
  xx[10] = xx[170] * xx[281];
  pm_math_Quaternion_inverseXform_ra(xx + 33, xx + 8, xx + 11);
  xx[8] = xx[11] - xx[168] * xx[283];
  xx[9] = xx[12] + xx[187] * xx[283];
  xx[10] = xx[13] - xx[188] * xx[283];
  pm_math_Quaternion_inverseXform_ra(xx + 25, xx + 8, xx + 11);
  xx[8] = xx[0] - xx[11];
  xx[9] = xx[1] - xx[12];
  xx[10] = xx[2] - xx[13];
  state[0] = xx[270];
  state[1] = xx[271];
  state[2] = xx[272];
  state[3] = xx[273];
  state[4] = xx[274];
  state[5] = xx[275];
  state[6] = xx[276];
  state[7] = xx[277];
  state[8] = xx[278];
  state[9] = xx[279];
  state[10] = xx[280];
  state[11] = xx[281];
  state[12] = xx[282];
  state[13] = xx[283];
  state[14] = xx[284] + pm_math_canonicalAngle(xx[94] * atan2(sqrt(xx[22] * xx
    [22] + xx[23] * xx[23] + xx[24] * xx[24]), fabs(xx[21])) *
    ((pm_math_Vector3_dot_ra(xx + 22, xx + 14) * xx[21]) < 0.0 ? -1.0 : +1.0) -
    xx[284]);
  state[15] = pm_math_Vector3_dot_ra(xx + 5, xx + 14);
  state[16] = xx[286] + pm_math_canonicalAngle(xx[94] * atan2(sqrt(xx[18] * xx
    [18] + xx[19] * xx[19] + xx[20] * xx[20]), fabs(xx[17])) *
    ((pm_math_Vector3_dot_ra(xx + 18, xx + 162) * xx[17]) < 0.0 ? -1.0 : +1.0) -
    xx[286]);
  state[17] = pm_math_Vector3_dot_ra(xx + 8, xx + 162);
  return NULL;
}

void DynamicValidation_dfaed711_1_computeConstraintError(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const int *modeVector,
  double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[65];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 0.6190277160392075;
  xx[1] = - 0.2167287412203158;
  xx[2] = 0.7124684524173637;
  xx[3] = 0.2494434681733462;
  xx[4] = 0.5;
  xx[5] = xx[4] * state[0];
  xx[6] = sin(xx[5]);
  xx[7] = cos(xx[5]);
  xx[8] = 5.906528705024172e-8 * xx[6];
  xx[9] = 0.7816132178294208 * xx[6];
  xx[10] = 0.6237633988254963 * xx[6];
  pm_math_Quaternion_compose_ra(xx + 0, xx + 7, xx + 11);
  xx[0] = - 0.9578985788971721;
  xx[1] = 0.01596084651607334;
  xx[2] = 0.2160322563859199;
  xx[3] = 0.188429371718097;
  xx[5] = xx[4] * state[2];
  xx[6] = sin(xx[5]);
  xx[7] = cos(xx[5]);
  xx[8] = - (0.503898206037895 * xx[6]);
  xx[9] = 0.8089781762292558 * xx[6];
  xx[10] = - (0.3027224939388848 * xx[6]);
  pm_math_Quaternion_compose_ra(xx + 0, xx + 7, xx + 15);
  pm_math_Quaternion_compose_ra(xx + 11, xx + 15, xx + 0);
  xx[5] = - 0.7391371617538293;
  xx[6] = - 0.2713596951978788;
  xx[7] = - 0.5768318010060487;
  xx[8] = - 0.2174976902974528;
  xx[9] = xx[4] * state[4];
  xx[10] = sin(xx[9]);
  xx[19] = cos(xx[9]);
  xx[20] = - (0.3618351326747742 * xx[10]);
  xx[21] = 0.342020353294726 * xx[10];
  xx[22] = - (0.8672355012880755 * xx[10]);
  pm_math_Quaternion_compose_ra(xx + 5, xx + 19, xx + 23);
  pm_math_Quaternion_compose_ra(xx + 0, xx + 23, xx + 5);
  xx[19] = 0.1645059501899856;
  xx[20] = 0.932950220907751;
  xx[21] = - 0.05560783598731543;
  xx[22] = 0.3153560626293339;
  pm_math_Quaternion_compose_ra(xx + 5, xx + 19, xx + 27);
  xx[9] = 2.0;
  xx[10] = 1.0;
  xx[19] = (xx[28] * xx[30] + xx[27] * xx[29]) * xx[9];
  xx[20] = xx[9] * (xx[29] * xx[30] - xx[27] * xx[28]);
  xx[21] = xx[10] - (xx[28] * xx[28] + xx[29] * xx[29]) * xx[9];
  xx[27] = 0.324402019972231;
  xx[28] = 0.1793201049669474;
  xx[29] = 0.8745212078931046;
  xx[30] = 0.3127623480180516;
  xx[22] = xx[4] * state[6];
  xx[31] = sin(xx[22]);
  xx[32] = cos(xx[22]);
  xx[33] = 0.5165599445270539 * xx[31];
  xx[34] = 0.7400480272337564 * xx[31];
  xx[35] = 0.4306910041986394 * xx[31];
  pm_math_Quaternion_compose_ra(xx + 27, xx + 32, xx + 36);
  xx[27] = - 0.8846589215343961;
  xx[28] = - 0.3902684207978444;
  xx[29] = 0.2533894697105629;
  xx[30] = - 0.02937565177755097;
  xx[22] = xx[4] * state[8];
  xx[31] = sin(xx[22]);
  xx[32] = cos(xx[22]);
  xx[33] = - (0.5038982060378953 * xx[31]);
  xx[34] = 0.8089781762292564 * xx[31];
  xx[35] = - (0.3027224939388842 * xx[31]);
  pm_math_Quaternion_compose_ra(xx + 27, xx + 32, xx + 40);
  pm_math_Quaternion_compose_ra(xx + 36, xx + 40, xx + 27);
  xx[22] = 0.300048825040154;
  xx[31] = 0.7582880608547207;
  xx[32] = xx[22];
  xx[33] = xx[31];
  xx[34] = 0.2970285731203685;
  xx[35] = 0.4967332746125315;
  pm_math_Quaternion_compose_ra(xx + 27, xx + 32, xx + 44);
  xx[32] = xx[47] * xx[47];
  xx[33] = xx[45] * xx[46];
  xx[34] = xx[44] * xx[47];
  xx[48] = xx[10] - (xx[46] * xx[46] + xx[32]) * xx[9];
  xx[49] = (xx[33] + xx[34]) * xx[9];
  xx[50] = xx[9] * (xx[45] * xx[47] - xx[44] * xx[46]);
  xx[51] = xx[9] * (xx[33] - xx[34]);
  xx[52] = xx[10] - (xx[45] * xx[45] + xx[32]) * xx[9];
  xx[53] = (xx[46] * xx[47] + xx[44] * xx[45]) * xx[9];
  xx[32] = 0.01027766876427663;
  xx[33] = 1.904031651085619e-3;
  xx[34] = - 0.01346562362772443;
  pm_math_Quaternion_xform_ra(xx + 5, xx + 32, xx + 44);
  xx[32] = - 3.265857732358156e-3;
  xx[33] = 0.01115673029435595;
  xx[34] = - 7.827670717550887e-3;
  pm_math_Quaternion_xform_ra(xx + 23, xx + 32, xx + 54);
  xx[23] = - (0.02695619245963533 + xx[54]);
  xx[24] = 0.01378863199166597 - xx[55];
  xx[25] = 3.369121440618853e-3 - xx[56];
  pm_math_Quaternion_xform_ra(xx + 0, xx + 23, xx + 32);
  xx[0] = - 6.076258344851112e-3;
  xx[1] = - 7.677368824970993e-3;
  xx[2] = 7.914929897510211e-3;
  pm_math_Quaternion_xform_ra(xx + 15, xx + 0, xx + 23);
  xx[0] = 1.706386300196451e-9 - xx[23];
  xx[1] = 0.01104873622831247 - xx[24];
  xx[2] = 0.01832347446050686 - xx[25];
  pm_math_Quaternion_xform_ra(xx + 11, xx + 0, xx + 15);
  xx[0] = - 2.009348425691778e-9;
  xx[1] = 3.882107956182032e-4;
  xx[2] = - 0.01888260476507436;
  pm_math_Quaternion_xform_ra(xx + 11, xx + 0, xx + 23);
  xx[0] = xx[32] + xx[15] - xx[23];
  xx[1] = - 1.754291965803085e-3;
  xx[2] = 9.46128575715573e-3;
  xx[3] = - 5.462379018955248e-3;
  pm_math_Quaternion_xform_ra(xx + 27, xx + 1, xx + 11);
  xx[1] = - 6.076258598061595e-3;
  xx[2] = - 7.677368418456878e-3;
  xx[3] = 7.9149297453912e-3;
  pm_math_Quaternion_xform_ra(xx + 40, xx + 1, xx + 26);
  xx[1] = 0.01659132440963332 - xx[26];
  xx[2] = 8.500421875005637e-3 - xx[27];
  xx[3] = 8.386468547111092e-3 - xx[28];
  pm_math_Quaternion_xform_ra(xx + 36, xx + 1, xx + 26);
  xx[1] = - 0.01544153522065833;
  xx[2] = 1.74063061481753e-3;
  xx[3] = - 4.362197173551173e-3;
  pm_math_Quaternion_xform_ra(xx + 36, xx + 1, xx + 40);
  xx[1] = xx[33] + xx[16] - xx[24] - 0.04200000000000426;
  xx[2] = xx[34] + xx[17] - xx[25];
  xx[14] = 0.3676062374005039;
  xx[15] = - 0.4418868990508547;
  xx[16] = - 0.6040401092638356;
  xx[17] = 0.552030043631683;
  pm_math_Quaternion_compose_ra(xx + 5, xx + 14, xx + 32);
  xx[14] = (xx[33] * xx[35] + xx[32] * xx[34]) * xx[9];
  xx[15] = xx[9] * (xx[34] * xx[35] - xx[32] * xx[33]);
  xx[16] = xx[10] - (xx[33] * xx[33] + xx[34] * xx[34]) * xx[9];
  xx[32] = 0.4437459878328764;
  xx[33] = 0.1803099382577183;
  xx[34] = 0.7643348123670592;
  xx[35] = 0.4317060563062962;
  xx[3] = xx[4] * state[10];
  xx[17] = sin(xx[3]);
  xx[36] = cos(xx[3]);
  xx[37] = 0.6587699864704037 * xx[17];
  xx[38] = 0.5622364142279257 * xx[17];
  xx[39] = 0.4999123117526796 * xx[17];
  pm_math_Quaternion_compose_ra(xx + 32, xx + 36, xx + 54);
  xx[32] = - 0.2207829197706712;
  xx[33] = - 0.6845661841871489;
  xx[34] = - 0.01617467817412936;
  xx[35] = 0.6945231613063365;
  xx[3] = xx[4] * state[12];
  xx[4] = sin(xx[3]);
  xx[36] = cos(xx[3]);
  xx[37] = - (0.5038982060378495 * xx[4]);
  xx[38] = 0.8089781762292761 * xx[4];
  xx[39] = - (0.3027224939389068 * xx[4]);
  pm_math_Quaternion_compose_ra(xx + 32, xx + 36, xx + 58);
  pm_math_Quaternion_compose_ra(xx + 54, xx + 58, xx + 32);
  xx[36] = xx[22];
  xx[37] = xx[31];
  xx[38] = 0.2970285731203686;
  xx[39] = 0.4967332746125314;
  pm_math_Quaternion_compose_ra(xx + 32, xx + 36, xx + 22);
  xx[3] = xx[25] * xx[25];
  xx[4] = xx[23] * xx[24];
  xx[17] = xx[22] * xx[25];
  xx[29] = xx[10] - (xx[24] * xx[24] + xx[3]) * xx[9];
  xx[30] = (xx[4] + xx[17]) * xx[9];
  xx[31] = xx[9] * (xx[23] * xx[25] - xx[22] * xx[24]);
  xx[36] = xx[9] * (xx[4] - xx[17]);
  xx[37] = xx[10] - (xx[23] * xx[23] + xx[3]) * xx[9];
  xx[38] = (xx[24] * xx[25] + xx[22] * xx[23]) * xx[9];
  xx[22] = - 0.01680036206308408;
  xx[23] = 1.904183946736232e-3;
  xx[24] = - 2.167844671666771e-3;
  pm_math_Quaternion_xform_ra(xx + 5, xx + 22, xx + 62);
  xx[3] = - 1.75429196580309e-3;
  xx[4] = 9.461285757155713e-3;
  xx[5] = - 5.462379018955269e-3;
  pm_math_Quaternion_xform_ra(xx + 32, xx + 3, xx + 6);
  xx[3] = - 6.076258262701207e-3;
  xx[4] = - 7.677368956859708e-3;
  xx[5] = 7.91492994686379e-3;
  pm_math_Quaternion_xform_ra(xx + 58, xx + 3, xx + 22);
  xx[3] = 7.869797269250363e-3 - xx[22];
  xx[4] = 0.01668052845419594 - xx[23];
  xx[5] = 2.96461486266998e-3 - xx[24];
  pm_math_Quaternion_xform_ra(xx + 54, xx + 3, xx + 22);
  xx[3] = - 6.483073904707459e-4;
  xx[4] = - 0.01395135758917342;
  xx[5] = 3.551989250699576e-3;
  pm_math_Quaternion_xform_ra(xx + 54, xx + 3, xx + 32);
  error[0] = pm_math_Vector3_dot_ra(xx + 19, xx + 48);
  error[1] = pm_math_Vector3_dot_ra(xx + 19, xx + 51);
  error[2] = xx[44] + xx[0] - (xx[11] + xx[26] - xx[40]);
  error[3] = xx[45] + xx[1] - (xx[12] + xx[27] - xx[41]) + 0.03750000000000427;
  error[4] = xx[46] + xx[2] - (xx[13] + xx[28] - xx[42]);
  error[5] = pm_math_Vector3_dot_ra(xx + 14, xx + 29);
  error[6] = pm_math_Vector3_dot_ra(xx + 14, xx + 36);
  error[7] = xx[62] + xx[0] - (xx[6] + xx[22] - xx[32]);
  error[8] = xx[63] + xx[1] - (xx[7] + xx[23] - xx[33]) + 0.03300000000000426;
  error[9] = xx[64] + xx[2] - (xx[8] + xx[24] - xx[34]);
}

void DynamicValidation_dfaed711_1_resetModeVector(const void *mech, int
  *modeVector)
{
  (void) mech;
  (void) modeVector;
}

boolean_T DynamicValidation_dfaed711_1_hasJointDisToNormModeChange(const void
  *mech, const int *prevModeVector, const int *modeVector)
{
  (void) mech;
  (void) prevModeVector;
  (void) modeVector;
  return 0;
}

PmfMessageId DynamicValidation_dfaed711_1_performJointDisToNormModeChange(const
  void *mech, const RuntimeDerivedValuesBundle *rtdv, const int *eqnEnableFlags,
  const int *prevModeVector, const int *modeVector, const double *input, double *
  state, void *neDiagMgr0)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  NeuDiagnosticManager *neDiagMgr = (NeuDiagnosticManager *) neDiagMgr0;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) eqnEnableFlags;
  (void) prevModeVector;
  (void) modeVector;
  (void) input;
  (void) state;
  (void) neDiagMgr;
  return NULL;
}

void DynamicValidation_dfaed711_1_onModeChangedCutJoints(const void *mech, const
  int *prevModeVector, const int *modeVector, double *state)
{
  (void) mech;
  (void) prevModeVector;
  (void) modeVector;
  (void) state;
}
