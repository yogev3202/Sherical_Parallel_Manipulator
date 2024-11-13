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
#include "sm_CTarget.h"

void DynamicValidation_dfaed711_1_setTargets(const RuntimeDerivedValuesBundle
  *rtdv, CTarget *targets)
{
  (void) rtdv;
  (void) targets;
}

void DynamicValidation_dfaed711_1_resetAsmStateVector(const void *mech, double
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

void DynamicValidation_dfaed711_1_initializeTrackedAngleState(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const int *modeVector, const double
  *motionData, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[31];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 2.0;
  xx[1] = - 0.7391371617538293;
  xx[2] = - 0.2713596951978788;
  xx[3] = - 0.5768318010060487;
  xx[4] = - 0.2174976902974528;
  xx[5] = motionData[49];
  xx[6] = motionData[50];
  xx[7] = motionData[51];
  xx[8] = motionData[52];
  xx[9] = motionData[63];
  xx[10] = motionData[64];
  xx[11] = motionData[65];
  xx[12] = motionData[66];
  pm_math_Quaternion_inverseCompose_ra(xx + 5, xx + 9, xx + 13);
  pm_math_Quaternion_inverseCompose_ra(xx + 1, xx + 13, xx + 5);
  xx[1] = - 0.3618351326747742;
  xx[2] = 0.342020353294726;
  xx[3] = - 0.8672355012880755;
  xx[17] = motionData[90];
  xx[18] = motionData[91];
  xx[19] = motionData[92];
  pm_math_Quaternion_inverseXform_ra(xx + 13, xx + 17, xx + 20);
  xx[13] = motionData[108] - xx[20];
  xx[14] = motionData[109] - xx[21];
  xx[15] = motionData[110] - xx[22];
  xx[16] = 0.1299832206933693;
  xx[17] = 0.05267645187405559;
  xx[18] = - 0.9285287989198832;
  xx[19] = - 0.3437496521198803;
  xx[20] = motionData[77];
  xx[21] = motionData[78];
  xx[22] = motionData[79];
  xx[23] = motionData[80];
  pm_math_Quaternion_inverseCompose_ra(xx + 20, xx + 9, xx + 24);
  pm_math_Quaternion_inverseCompose_ra(xx + 16, xx + 24, xx + 9);
  xx[16] = - 0.9319675119375604;
  xx[17] = - 0.3420158151110814;
  xx[18] = - 0.1202569703044931;
  xx[19] = motionData[120];
  xx[20] = motionData[121];
  xx[21] = motionData[122];
  pm_math_Quaternion_inverseXform_ra(xx + 24, xx + 19, xx + 28);
  xx[19] = motionData[108] - xx[28];
  xx[20] = motionData[109] - xx[29];
  xx[21] = motionData[110] - xx[30];
  state[14] = pm_math_canonicalAngle(xx[0] * atan2(sqrt(xx[6] * xx[6] + xx[7] *
    xx[7] + xx[8] * xx[8]), fabs(xx[5])) * ((pm_math_Vector3_dot_ra(xx + 6, xx +
    1) * xx[5]) < 0.0 ? -1.0 : +1.0));
  state[15] = pm_math_Vector3_dot_ra(xx + 13, xx + 1);
  state[16] = pm_math_canonicalAngle(xx[0] * atan2(sqrt(xx[10] * xx[10] + xx[11]
    * xx[11] + xx[12] * xx[12]), fabs(xx[9])) * ((pm_math_Vector3_dot_ra(xx + 10,
    xx + 16) * xx[9]) < 0.0 ? -1.0 : +1.0));
  state[17] = pm_math_Vector3_dot_ra(xx + 19, xx + 16);
}

void DynamicValidation_dfaed711_1_computeDiscreteState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
}

void DynamicValidation_dfaed711_1_adjustPosition(const void *mech, const double *
  dofDeltas, double *state)
{
  (void) mech;
  state[0] = state[0] + dofDeltas[0];
  state[2] = state[2] + dofDeltas[1];
  state[4] = state[4] + dofDeltas[2];
  state[6] = state[6] + dofDeltas[3];
  state[8] = state[8] + dofDeltas[4];
  state[10] = state[10] + dofDeltas[5];
  state[12] = state[12] + dofDeltas[6];
}

static void perturbAsmJointPrimitiveState_0_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbAsmJointPrimitiveState_0_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[1] = state[1] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_1_0(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbAsmJointPrimitiveState_1_0v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[3] = state[3] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_2_0(double mag, double *state)
{
  state[4] = state[4] + mag;
}

static void perturbAsmJointPrimitiveState_2_0v(double mag, double *state)
{
  state[4] = state[4] + mag;
  state[5] = state[5] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_3_0(double mag, double *state)
{
  state[6] = state[6] + mag;
}

static void perturbAsmJointPrimitiveState_3_0v(double mag, double *state)
{
  state[6] = state[6] + mag;
  state[7] = state[7] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_4_0(double mag, double *state)
{
  state[8] = state[8] + mag;
}

static void perturbAsmJointPrimitiveState_4_0v(double mag, double *state)
{
  state[8] = state[8] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_5_0(double mag, double *state)
{
  state[10] = state[10] + mag;
}

static void perturbAsmJointPrimitiveState_5_0v(double mag, double *state)
{
  state[10] = state[10] + mag;
  state[11] = state[11] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_6_0(double mag, double *state)
{
  state[12] = state[12] + mag;
}

static void perturbAsmJointPrimitiveState_6_0v(double mag, double *state)
{
  state[12] = state[12] + mag;
  state[13] = state[13] - 0.875 * mag;
}

void DynamicValidation_dfaed711_1_perturbAsmJointPrimitiveState(const void *mech,
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
    perturbAsmJointPrimitiveState_0_0(mag, state);
    break;

   case 1:
    perturbAsmJointPrimitiveState_0_0v(mag, state);
    break;

   case 12:
    perturbAsmJointPrimitiveState_1_0(mag, state);
    break;

   case 13:
    perturbAsmJointPrimitiveState_1_0v(mag, state);
    break;

   case 24:
    perturbAsmJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbAsmJointPrimitiveState_2_0v(mag, state);
    break;

   case 36:
    perturbAsmJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbAsmJointPrimitiveState_3_0v(mag, state);
    break;

   case 48:
    perturbAsmJointPrimitiveState_4_0(mag, state);
    break;

   case 49:
    perturbAsmJointPrimitiveState_4_0v(mag, state);
    break;

   case 60:
    perturbAsmJointPrimitiveState_5_0(mag, state);
    break;

   case 61:
    perturbAsmJointPrimitiveState_5_0v(mag, state);
    break;

   case 72:
    perturbAsmJointPrimitiveState_6_0(mag, state);
    break;

   case 73:
    perturbAsmJointPrimitiveState_6_0v(mag, state);
    break;
  }
}

void DynamicValidation_dfaed711_1_computePosDofBlendMatrix(const void *mech,
  size_t stageIdx, size_t primIdx, const double *state, int partialType, double *
  matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void DynamicValidation_dfaed711_1_computeVelDofBlendMatrix(const void *mech,
  size_t stageIdx, size_t primIdx, const double *state, int partialType, double *
  matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void DynamicValidation_dfaed711_1_projectPartiallyTargetedPos(const void *mech,
  size_t stageIdx, size_t primIdx, const double *origState, int partialType,
  double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) origState;
  (void) partialType;
  (void) state;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void DynamicValidation_dfaed711_1_propagateMotion(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[150];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
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
  xx[0] = - 2.009348425691778e-9;
  xx[1] = 3.882107956182032e-4;
  xx[2] = - 0.01888260476507436;
  pm_math_Quaternion_xform_ra(xx + 14, xx + 0, xx + 10);
  xx[0] = 0.04200000000000426;
  xx[18] = - 0.9578985788971721;
  xx[19] = 0.01596084651607334;
  xx[20] = 0.2160322563859199;
  xx[21] = 0.188429371718097;
  xx[1] = xx[4] * state[2];
  xx[2] = 0.503898206037895;
  xx[3] = sin(xx[1]);
  xx[5] = 0.8089781762292558;
  xx[7] = 0.3027224939388848;
  xx[22] = cos(xx[1]);
  xx[23] = - (xx[2] * xx[3]);
  xx[24] = xx[5] * xx[3];
  xx[25] = - (xx[7] * xx[3]);
  pm_math_Quaternion_compose_ra(xx + 18, xx + 22, xx + 26);
  xx[18] = - 6.076258344851112e-3;
  xx[19] = - 7.677368824970993e-3;
  xx[20] = 7.914929897510211e-3;
  pm_math_Quaternion_xform_ra(xx + 26, xx + 18, xx + 21);
  xx[1] = 1.706386300196451e-9 - xx[21];
  xx[3] = 0.01104873622831247 - xx[22];
  xx[13] = 0.01832347446050686 - xx[23];
  xx[18] = 0.324402019972231;
  xx[19] = 0.1793201049669474;
  xx[20] = 0.8745212078931046;
  xx[21] = 0.3127623480180516;
  xx[22] = xx[4] * state[4];
  xx[23] = 0.5165599445270539;
  xx[24] = sin(xx[22]);
  xx[25] = 0.7400480272337564;
  xx[30] = 0.4306910041986394;
  xx[31] = cos(xx[22]);
  xx[32] = xx[23] * xx[24];
  xx[33] = xx[25] * xx[24];
  xx[34] = xx[30] * xx[24];
  pm_math_Quaternion_compose_ra(xx + 18, xx + 31, xx + 35);
  xx[18] = - 0.01544153522065833;
  xx[19] = 1.74063061481753e-3;
  xx[20] = - 4.362197173551173e-3;
  pm_math_Quaternion_xform_ra(xx + 35, xx + 18, xx + 31);
  xx[18] = 0.03750000000000427 + xx[32];
  xx[19] = - 0.8846589215343961;
  xx[20] = - 0.3902684207978444;
  xx[21] = 0.2533894697105629;
  xx[22] = - 0.02937565177755097;
  xx[24] = xx[4] * state[6];
  xx[32] = 0.5038982060378953;
  xx[34] = sin(xx[24]);
  xx[39] = 0.8089781762292564;
  xx[40] = 0.3027224939388842;
  xx[41] = cos(xx[24]);
  xx[42] = - (xx[32] * xx[34]);
  xx[43] = xx[39] * xx[34];
  xx[44] = - (xx[40] * xx[34]);
  pm_math_Quaternion_compose_ra(xx + 19, xx + 41, xx + 45);
  xx[19] = 0.01659132440963332;
  xx[20] = - 6.076258598061595e-3;
  xx[21] = - 7.677368418456878e-3;
  xx[22] = 7.9149297453912e-3;
  pm_math_Quaternion_xform_ra(xx + 45, xx + 20, xx + 41);
  xx[20] = xx[19] - xx[41];
  xx[21] = 8.500421875005637e-3;
  xx[22] = xx[21] - xx[42];
  xx[24] = 8.386468547111092e-3;
  xx[34] = xx[24] - xx[43];
  xx[49] = - 0.8969355644339009;
  xx[50] = 0.2764797434074396;
  xx[51] = 0.158748647566253;
  xx[52] = - 0.3063729942976073;
  xx[44] = xx[4] * state[8];
  xx[53] = 0.5701273767950668;
  xx[54] = sin(xx[44]);
  xx[55] = 0.3420242615573683;
  xx[56] = 0.7469766922300272;
  xx[57] = cos(xx[44]);
  xx[58] = xx[53] * xx[54];
  xx[59] = - (xx[55] * xx[54]);
  xx[60] = - (xx[56] * xx[54]);
  pm_math_Quaternion_compose_ra(xx + 49, xx + 57, xx + 61);
  xx[49] = 0.01027766876427663;
  xx[50] = 1.904031651085619e-3;
  xx[51] = - 0.01346562362772443;
  pm_math_Quaternion_xform_ra(xx + 61, xx + 49, xx + 57);
  xx[44] = - (1.754291965803085e-3 + xx[57]);
  xx[49] = 9.46128575715573e-3 - xx[58];
  xx[50] = - (5.462379018955248e-3 + xx[59]);
  xx[57] = 0.4437459878328764;
  xx[58] = 0.1803099382577183;
  xx[59] = 0.7643348123670592;
  xx[60] = 0.4317060563062962;
  xx[51] = xx[4] * state[10];
  xx[52] = 0.6587699864704037;
  xx[54] = sin(xx[51]);
  xx[65] = 0.5622364142279257;
  xx[66] = 0.4999123117526796;
  xx[67] = cos(xx[51]);
  xx[68] = xx[52] * xx[54];
  xx[69] = xx[65] * xx[54];
  xx[70] = xx[66] * xx[54];
  pm_math_Quaternion_compose_ra(xx + 57, xx + 67, xx + 71);
  xx[57] = - 6.483073904707459e-4;
  xx[58] = - 0.01395135758917342;
  xx[59] = 3.551989250699576e-3;
  pm_math_Quaternion_xform_ra(xx + 71, xx + 57, xx + 67);
  xx[51] = 0.03300000000000426;
  xx[57] = - 0.2207829197706712;
  xx[58] = - 0.6845661841871489;
  xx[59] = - 0.01617467817412936;
  xx[60] = 0.6945231613063365;
  xx[54] = xx[4] * state[12];
  xx[4] = 0.5038982060378495;
  xx[70] = sin(xx[54]);
  xx[75] = 0.8089781762292761;
  xx[76] = 0.3027224939389068;
  xx[77] = cos(xx[54]);
  xx[78] = - (xx[4] * xx[70]);
  xx[79] = xx[75] * xx[70];
  xx[80] = - (xx[76] * xx[70]);
  pm_math_Quaternion_compose_ra(xx + 57, xx + 77, xx + 81);
  xx[57] = - 6.076258262701207e-3;
  xx[58] = - 7.677368956859708e-3;
  xx[59] = 7.91492994686379e-3;
  pm_math_Quaternion_xform_ra(xx + 81, xx + 57, xx + 77);
  xx[54] = 7.869797269250363e-3 - xx[77];
  xx[57] = 0.01668052845419594 - xx[78];
  xx[58] = 2.96461486266998e-3 - xx[79];
  pm_math_Quaternion_compose_ra(xx + 14, xx + 26, xx + 77);
  xx[85] = xx[1];
  xx[86] = xx[3];
  xx[87] = xx[13];
  pm_math_Quaternion_xform_ra(xx + 14, xx + 85, xx + 88);
  pm_math_Quaternion_compose_ra(xx + 35, xx + 45, xx + 91);
  xx[95] = xx[20];
  xx[96] = xx[22];
  xx[97] = xx[34];
  pm_math_Quaternion_xform_ra(xx + 35, xx + 95, xx + 98);
  xx[59] = xx[98] - xx[31];
  xx[60] = xx[99] - xx[18];
  xx[70] = xx[100] - xx[33];
  pm_math_Quaternion_compose_ra(xx + 91, xx + 61, xx + 98);
  xx[102] = xx[44];
  xx[103] = xx[49];
  xx[104] = xx[50];
  pm_math_Quaternion_xform_ra(xx + 91, xx + 102, xx + 105);
  pm_math_Quaternion_compose_ra(xx + 45, xx + 61, xx + 108);
  pm_math_Quaternion_xform_ra(xx + 45, xx + 102, xx + 112);
  pm_math_Quaternion_compose_ra(xx + 71, xx + 81, xx + 115);
  xx[119] = xx[54];
  xx[120] = xx[57];
  xx[121] = xx[58];
  pm_math_Quaternion_xform_ra(xx + 71, xx + 119, xx + 122);
  xx[125] = xx[6] * state[1];
  xx[6] = xx[8] * state[1];
  xx[8] = xx[9] * state[1];
  xx[9] = 0.01500104515676648 * state[1];
  xx[126] = 1.380515327287848e-10 * state[1];
  xx[127] = 1.593463070824624e-9 * state[1];
  xx[128] = xx[125];
  xx[129] = xx[6];
  xx[130] = xx[8];
  pm_math_Quaternion_inverseXform_ra(xx + 26, xx + 128, xx + 131);
  pm_math_Vector3_cross_ra(xx + 128, xx + 85, xx + 134);
  xx[85] = xx[134] + xx[9];
  xx[86] = xx[135] + xx[126];
  xx[87] = xx[136] - xx[127];
  pm_math_Quaternion_inverseXform_ra(xx + 26, xx + 85, xx + 128);
  xx[85] = xx[23] * state[5];
  xx[23] = xx[25] * state[5];
  xx[25] = xx[30] * state[5];
  xx[30] = 3.977909360125871e-3 * state[5];
  xx[86] = 4.397193980568331e-3 * state[5];
  xx[87] = 0.012326617731341 * state[5];
  xx[134] = xx[85];
  xx[135] = xx[23];
  xx[136] = xx[25];
  pm_math_Quaternion_inverseXform_ra(xx + 45, xx + 134, xx + 137);
  xx[140] = xx[137] - xx[32] * state[7];
  xx[32] = xx[138] + xx[39] * state[7];
  xx[39] = xx[139] - xx[40] * state[7];
  pm_math_Vector3_cross_ra(xx + 134, xx + 95, xx + 137);
  xx[95] = xx[137] + xx[30];
  xx[96] = xx[138] + xx[86];
  xx[97] = xx[139] - xx[87];
  pm_math_Quaternion_inverseXform_ra(xx + 45, xx + 95, xx + 134);
  xx[40] = xx[134] - 4.078893315886373e-3 * state[7];
  xx[95] = xx[135] - 5.827739056241395e-3 * state[7];
  xx[96] = xx[136] - 8.784172772109623e-3 * state[7];
  xx[134] = xx[140];
  xx[135] = xx[32];
  xx[136] = xx[39];
  pm_math_Quaternion_inverseXform_ra(xx + 61, xx + 134, xx + 137);
  pm_math_Vector3_cross_ra(xx + 134, xx + 102, xx + 141);
  xx[102] = xx[141] + xx[40];
  xx[103] = xx[142] + xx[95];
  xx[104] = xx[143] + xx[96];
  pm_math_Quaternion_inverseXform_ra(xx + 61, xx + 102, xx + 134);
  xx[97] = xx[52] * state[11];
  xx[52] = xx[65] * state[11];
  xx[65] = xx[66] * state[11];
  xx[66] = 8.971513124181443e-3 * state[11];
  xx[102] = 2.664040756922957e-3 * state[11];
  xx[103] = 8.826233627727805e-3 * state[11];
  xx[141] = xx[97];
  xx[142] = xx[52];
  xx[143] = xx[65];
  pm_math_Quaternion_inverseXform_ra(xx + 81, xx + 141, xx + 144);
  pm_math_Vector3_cross_ra(xx + 141, xx + 119, xx + 147);
  xx[119] = xx[147] - xx[66];
  xx[120] = xx[148] + xx[102];
  xx[121] = xx[149] + xx[103];
  pm_math_Quaternion_inverseXform_ra(xx + 81, xx + 119, xx + 141);
  motionData[0] = xx[14];
  motionData[1] = xx[15];
  motionData[2] = xx[16];
  motionData[3] = xx[17];
  motionData[4] = - xx[10];
  motionData[5] = - (xx[0] + xx[11]);
  motionData[6] = - xx[12];
  motionData[7] = xx[26];
  motionData[8] = xx[27];
  motionData[9] = xx[28];
  motionData[10] = xx[29];
  motionData[11] = xx[1];
  motionData[12] = xx[3];
  motionData[13] = xx[13];
  motionData[14] = xx[35];
  motionData[15] = xx[36];
  motionData[16] = xx[37];
  motionData[17] = xx[38];
  motionData[18] = - xx[31];
  motionData[19] = - xx[18];
  motionData[20] = - xx[33];
  motionData[21] = xx[45];
  motionData[22] = xx[46];
  motionData[23] = xx[47];
  motionData[24] = xx[48];
  motionData[25] = xx[20];
  motionData[26] = xx[22];
  motionData[27] = xx[34];
  motionData[28] = xx[61];
  motionData[29] = xx[62];
  motionData[30] = xx[63];
  motionData[31] = xx[64];
  motionData[32] = xx[44];
  motionData[33] = xx[49];
  motionData[34] = xx[50];
  motionData[35] = xx[71];
  motionData[36] = xx[72];
  motionData[37] = xx[73];
  motionData[38] = xx[74];
  motionData[39] = - xx[67];
  motionData[40] = - (xx[51] + xx[68]);
  motionData[41] = - xx[69];
  motionData[42] = xx[81];
  motionData[43] = xx[82];
  motionData[44] = xx[83];
  motionData[45] = xx[84];
  motionData[46] = xx[54];
  motionData[47] = xx[57];
  motionData[48] = xx[58];
  motionData[49] = xx[77];
  motionData[50] = xx[78];
  motionData[51] = xx[79];
  motionData[52] = xx[80];
  motionData[53] = xx[88] - xx[10];
  motionData[54] = xx[89] - xx[11] - xx[0];
  motionData[55] = xx[90] - xx[12];
  motionData[56] = xx[91];
  motionData[57] = xx[92];
  motionData[58] = xx[93];
  motionData[59] = xx[94];
  motionData[60] = xx[59];
  motionData[61] = xx[60];
  motionData[62] = xx[70];
  motionData[63] = xx[98];
  motionData[64] = xx[99];
  motionData[65] = xx[100];
  motionData[66] = xx[101];
  motionData[67] = xx[105] + xx[59];
  motionData[68] = xx[106] + xx[60];
  motionData[69] = xx[107] + xx[70];
  motionData[70] = xx[108];
  motionData[71] = xx[109];
  motionData[72] = xx[110];
  motionData[73] = xx[111];
  motionData[74] = xx[112] - xx[41] + xx[19];
  motionData[75] = xx[113] - xx[42] + xx[21];
  motionData[76] = xx[114] - xx[43] + xx[24];
  motionData[77] = xx[115];
  motionData[78] = xx[116];
  motionData[79] = xx[117];
  motionData[80] = xx[118];
  motionData[81] = xx[122] - xx[67];
  motionData[82] = xx[123] - xx[68] - xx[51];
  motionData[83] = xx[124] - xx[69];
  motionData[84] = xx[125];
  motionData[85] = xx[6];
  motionData[86] = xx[8];
  motionData[87] = xx[9];
  motionData[88] = xx[126];
  motionData[89] = - xx[127];
  motionData[90] = xx[131] - xx[2] * state[3];
  motionData[91] = xx[132] + xx[5] * state[3];
  motionData[92] = xx[133] - xx[7] * state[3];
  motionData[93] = xx[128] - 4.078893315886357e-3 * state[3];
  motionData[94] = xx[129] - 5.827739056241386e-3 * state[3];
  motionData[95] = xx[130] - 8.784172772109595e-3 * state[3];
  motionData[96] = xx[85];
  motionData[97] = xx[23];
  motionData[98] = xx[25];
  motionData[99] = xx[30];
  motionData[100] = xx[86];
  motionData[101] = - xx[87];
  motionData[102] = xx[140];
  motionData[103] = xx[32];
  motionData[104] = xx[39];
  motionData[105] = xx[40];
  motionData[106] = xx[95];
  motionData[107] = xx[96];
  motionData[108] = xx[137] + xx[53] * state[9];
  motionData[109] = xx[138] - xx[55] * state[9];
  motionData[110] = xx[139] - xx[56] * state[9];
  motionData[111] = xx[134] - 6.027837242311112e-3 * state[9];
  motionData[112] = xx[135] + 5.834159102525993e-8 * state[9];
  motionData[113] = xx[136] - 4.600752640201167e-3 * state[9];
  motionData[114] = xx[97];
  motionData[115] = xx[52];
  motionData[116] = xx[65];
  motionData[117] = - xx[66];
  motionData[118] = xx[102];
  motionData[119] = xx[103];
  motionData[120] = xx[144] - xx[4] * state[13];
  motionData[121] = xx[145] + xx[75] * state[13];
  motionData[122] = xx[146] - xx[76] * state[13];
  motionData[123] = xx[141] - 4.078893315886636e-3 * state[13];
  motionData[124] = xx[142] - 5.827739056241714e-3 * state[13];
  motionData[125] = xx[143] - 8.784172772110375e-3 * state[13];
}

static size_t computeAssemblyError_0(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[20];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[49];
  xx[1] = motionData[50];
  xx[2] = motionData[51];
  xx[3] = motionData[52];
  xx[4] = 0.7582880608547204;
  xx[5] = - 0.3000488250401547;
  xx[6] = - 0.4967332746125314;
  xx[7] = 0.2970285731203689;
  pm_math_Quaternion_compose_ra(xx + 0, xx + 4, xx + 8);
  xx[4] = motionData[63];
  xx[5] = motionData[64];
  xx[6] = motionData[65];
  xx[7] = motionData[66];
  xx[12] = 0.2571292065762446;
  xx[13] = - 0.7069199206988735;
  xx[14] = - 0.658697181080302;
  xx[15] = 0.01633464053578271;
  pm_math_Quaternion_compose_ra(xx + 4, xx + 12, xx + 16);
  pm_math_Quaternion_inverseCompose_ra(xx + 8, xx + 16, xx + 12);
  xx[8] = - 3.265857732358156e-3;
  xx[9] = 0.01115673029435595;
  xx[10] = - 7.827670717550887e-3;
  pm_math_Quaternion_xform_ra(xx + 4, xx + 8, xx + 15);
  xx[4] = - 0.02695619245963533;
  xx[5] = 0.01378863199166597;
  xx[6] = 3.369121440618853e-3;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = xx[13];
  error[1] = xx[14];
  error[2] = xx[15] + motionData[67] - (xx[7] + motionData[53]);
  error[3] = xx[16] + motionData[68] - (xx[8] + motionData[54]);
  error[4] = xx[17] + motionData[69] - (xx[9] + motionData[55]);
  return 5;
}

static size_t computeAssemblyError_1(const RuntimeDerivedValuesBundle *rtdv,
  const int *modeVector, const double *motionData, double *error)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[20];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[77];
  xx[1] = motionData[78];
  xx[2] = motionData[79];
  xx[3] = motionData[80];
  xx[4] = 0.300048825040154;
  xx[5] = 0.7582880608547207;
  xx[6] = 0.2970285731203686;
  xx[7] = 0.4967332746125314;
  pm_math_Quaternion_compose_ra(xx + 0, xx + 4, xx + 8);
  xx[4] = motionData[63];
  xx[5] = motionData[64];
  xx[6] = motionData[65];
  xx[7] = motionData[66];
  xx[12] = 0.3676062374005039;
  xx[13] = - 0.4418868990508547;
  xx[14] = - 0.6040401092638356;
  xx[15] = 0.552030043631683;
  pm_math_Quaternion_compose_ra(xx + 4, xx + 12, xx + 16);
  pm_math_Quaternion_inverseCompose_ra(xx + 8, xx + 16, xx + 12);
  xx[8] = - 0.01680036206308408;
  xx[9] = 1.904183946736232e-3;
  xx[10] = - 2.167844671666771e-3;
  pm_math_Quaternion_xform_ra(xx + 4, xx + 8, xx + 15);
  xx[4] = - 1.75429196580309e-3;
  xx[5] = 9.461285757155713e-3;
  xx[6] = - 5.462379018955269e-3;
  pm_math_Quaternion_xform_ra(xx + 0, xx + 4, xx + 7);
  error[0] = xx[13];
  error[1] = xx[14];
  error[2] = xx[15] + motionData[67] - (xx[7] + motionData[81]);
  error[3] = xx[16] + motionData[68] - (xx[8] + motionData[82]);
  error[4] = xx[17] + motionData[69] - (xx[9] + motionData[83]);
  return 5;
}

size_t DynamicValidation_dfaed711_1_computeAssemblyError(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int *modeVector,
  const double *motionData, double *error)
{
  (void) mech;
  (void)rtdv;
  (void) modeVector;
  (void) motionData;
  (void) error;
  switch (constraintIdx)
  {
   case 0:
    return computeAssemblyError_0(rtdv, modeVector, motionData, error);

   case 1:
    return computeAssemblyError_1(rtdv, modeVector, motionData, error);
  }

  return 0;
}

static size_t computeAssemblyJacobian_0(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[73];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[49];
  xx[1] = motionData[50];
  xx[2] = motionData[51];
  xx[3] = motionData[52];
  xx[4] = 0.7582880608547204;
  xx[5] = - 0.3000488250401547;
  xx[6] = - 0.4967332746125314;
  xx[7] = 0.2970285731203689;
  pm_math_Quaternion_compose_ra(xx + 0, xx + 4, xx + 8);
  xx[4] = motionData[63];
  xx[5] = motionData[64];
  xx[6] = motionData[65];
  xx[7] = motionData[66];
  xx[12] = 0.2571292065762446;
  xx[13] = - 0.7069199206988735;
  xx[14] = - 0.658697181080302;
  xx[15] = 0.01633464053578271;
  pm_math_Quaternion_compose_ra(xx + 4, xx + 12, xx + 16);
  pm_math_Quaternion_inverseCompose_ra(xx + 8, xx + 16, xx + 4);
  xx[8] = motionData[7];
  xx[9] = motionData[8];
  xx[10] = motionData[9];
  xx[11] = motionData[10];
  xx[20] = 5.906528705024172e-8;
  xx[21] = 0.7816132178294208;
  xx[22] = 0.6237633988254963;
  pm_math_Quaternion_inverseXform_ra(xx + 8, xx + 20, xx + 23);
  pm_math_Quaternion_xform_ra(xx + 0, xx + 23, xx + 26);
  pm_math_Quaternion_inverseXform_ra(xx + 16, xx + 26, xx + 29);
  xx[26] = - xx[29];
  xx[27] = - xx[30];
  xx[28] = - xx[31];
  pm_math_Quaternion_compDeriv_ra(xx + 4, xx + 26, xx + 29);
  xx[26] = 0.503898206037895;
  xx[27] = 0.8089781762292558;
  xx[28] = 0.3027224939388848;
  xx[33] = - xx[26];
  xx[34] = xx[27];
  xx[35] = - xx[28];
  pm_math_Quaternion_xform_ra(xx + 0, xx + 33, xx + 36);
  pm_math_Quaternion_inverseXform_ra(xx + 16, xx + 36, xx + 0);
  xx[16] = - xx[0];
  xx[17] = - xx[1];
  xx[18] = - xx[2];
  pm_math_Quaternion_compDeriv_ra(xx + 4, xx + 16, xx + 0);
  xx[16] = motionData[70];
  xx[17] = motionData[71];
  xx[18] = motionData[72];
  xx[19] = motionData[73];
  xx[33] = 0.5165599445270539;
  xx[34] = 0.7400480272337564;
  xx[35] = 0.4306910041986394;
  pm_math_Quaternion_inverseXform_ra(xx + 16, xx + 33, xx + 36);
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 36, xx + 39);
  pm_math_Quaternion_compDeriv_ra(xx + 4, xx + 39, xx + 42);
  xx[46] = motionData[28];
  xx[47] = motionData[29];
  xx[48] = motionData[30];
  xx[49] = motionData[31];
  xx[39] = 0.5038982060378953;
  xx[40] = 0.8089781762292564;
  xx[41] = 0.3027224939388842;
  xx[50] = - xx[39];
  xx[51] = xx[40];
  xx[52] = - xx[41];
  pm_math_Quaternion_inverseXform_ra(xx + 46, xx + 50, xx + 53);
  pm_math_Quaternion_inverseXform_ra(xx + 12, xx + 53, xx + 56);
  pm_math_Quaternion_compDeriv_ra(xx + 4, xx + 56, xx + 12);
  xx[56] = - 0.4820907072649103;
  xx[57] = 0.8137976813493701;
  xx[58] = 0.3245333323392334;
  pm_math_Quaternion_compDeriv_ra(xx + 4, xx + 56, xx + 59);
  xx[4] = 9.87654321;
  xx[56] = 0.6190277160392075;
  xx[57] = - 0.2167287412203158;
  xx[58] = 0.7124684524173637;
  xx[59] = 0.2494434681733462;
  xx[0] = 0.5;
  xx[3] = xx[0] * state[0];
  xx[5] = sin(xx[3]);
  xx[62] = cos(xx[3]);
  xx[63] = xx[20] * xx[5];
  xx[64] = xx[21] * xx[5];
  xx[65] = xx[22] * xx[5];
  pm_math_Quaternion_compose_ra(xx + 56, xx + 62, xx + 66);
  pm_math_Quaternion_compose_ra(xx + 66, xx + 8, xx + 56);
  xx[5] = - 0.02695619245963533;
  xx[6] = 0.01378863199166597;
  xx[7] = 3.369121440618853e-3;
  pm_math_Vector3_cross_ra(xx + 23, xx + 5, xx + 8);
  pm_math_Quaternion_xform_ra(xx + 56, xx + 8, xx + 5);
  xx[8] = motionData[11];
  xx[9] = motionData[12];
  xx[10] = motionData[13];
  pm_math_Vector3_cross_ra(xx + 20, xx + 8, xx + 23);
  pm_math_Quaternion_xform_ra(xx + 66, xx + 23, xx + 8);
  xx[20] = 0.01500104515676648;
  xx[21] = 1.380515327287848e-10;
  xx[22] = - 1.593463070824624e-9;
  pm_math_Quaternion_xform_ra(xx + 66, xx + 20, xx + 23);
  xx[56] = motionData[0];
  xx[57] = motionData[1];
  xx[58] = motionData[2];
  xx[59] = motionData[3];
  xx[62] = - 0.9578985788971721;
  xx[63] = 0.01596084651607334;
  xx[64] = 0.2160322563859199;
  xx[65] = 0.188429371718097;
  xx[3] = xx[0] * state[2];
  xx[11] = sin(xx[3]);
  xx[66] = cos(xx[3]);
  xx[67] = - (xx[26] * xx[11]);
  xx[68] = xx[27] * xx[11];
  xx[69] = - (xx[28] * xx[11]);
  pm_math_Quaternion_compose_ra(xx + 62, xx + 66, xx + 26);
  pm_math_Quaternion_compose_ra(xx + 56, xx + 26, xx + 62);
  xx[20] = 6.89967478304934e-3;
  xx[21] = 9.857940058329018e-3;
  xx[22] = 0.01485890448976339;
  pm_math_Quaternion_xform_ra(xx + 62, xx + 20, xx + 66);
  xx[20] = - 4.078893315886357e-3;
  xx[21] = - 5.827739056241386e-3;
  xx[22] = - 8.784172772109595e-3;
  pm_math_Quaternion_xform_ra(xx + 26, xx + 20, xx + 62);
  pm_math_Quaternion_xform_ra(xx + 56, xx + 62, xx + 20);
  xx[26] = 0.324402019972231;
  xx[27] = 0.1793201049669474;
  xx[28] = 0.8745212078931046;
  xx[29] = 0.3127623480180516;
  xx[3] = xx[0] * state[4];
  xx[11] = sin(xx[3]);
  xx[56] = cos(xx[3]);
  xx[57] = xx[33] * xx[11];
  xx[58] = xx[34] * xx[11];
  xx[59] = xx[35] * xx[11];
  pm_math_Quaternion_compose_ra(xx + 26, xx + 56, xx + 62);
  pm_math_Quaternion_compose_ra(xx + 62, xx + 16, xx + 26);
  xx[15] = - 3.265857732358156e-3;
  xx[16] = 0.01115673029435595;
  xx[17] = - 7.827670717550887e-3;
  pm_math_Vector3_cross_ra(xx + 36, xx + 15, xx + 56);
  pm_math_Quaternion_xform_ra(xx + 26, xx + 56, xx + 36);
  xx[26] = motionData[74];
  xx[27] = motionData[75];
  xx[28] = motionData[76];
  pm_math_Vector3_cross_ra(xx + 33, xx + 26, xx + 56);
  pm_math_Quaternion_xform_ra(xx + 62, xx + 56, xx + 26);
  xx[32] = 3.977909360125871e-3;
  xx[33] = 4.397193980568331e-3;
  xx[34] = - 0.012326617731341;
  pm_math_Quaternion_xform_ra(xx + 62, xx + 32, xx + 56);
  xx[32] = motionData[14];
  xx[33] = motionData[15];
  xx[34] = motionData[16];
  xx[35] = motionData[17];
  xx[62] = - 0.8846589215343961;
  xx[63] = - 0.3902684207978444;
  xx[64] = 0.2533894697105629;
  xx[65] = - 0.02937565177755097;
  xx[3] = xx[0] * state[6];
  xx[11] = sin(xx[3]);
  xx[69] = cos(xx[3]);
  xx[70] = - (xx[39] * xx[11]);
  xx[71] = xx[40] * xx[11];
  xx[72] = - (xx[41] * xx[11]);
  pm_math_Quaternion_compose_ra(xx + 62, xx + 69, xx + 39);
  pm_math_Quaternion_compose_ra(xx + 32, xx + 39, xx + 62);
  pm_math_Quaternion_compose_ra(xx + 62, xx + 46, xx + 69);
  pm_math_Vector3_cross_ra(xx + 53, xx + 15, xx + 45);
  pm_math_Quaternion_xform_ra(xx + 69, xx + 45, xx + 15);
  xx[45] = motionData[32];
  xx[46] = motionData[33];
  xx[47] = motionData[34];
  pm_math_Vector3_cross_ra(xx + 50, xx + 45, xx + 53);
  pm_math_Quaternion_xform_ra(xx + 39, xx + 53, xx + 45);
  xx[48] = - 4.078893315886373e-3;
  xx[49] = - 5.827739056241395e-3;
  xx[50] = - 8.784172772109623e-3;
  pm_math_Quaternion_xform_ra(xx + 39, xx + 48, xx + 51);
  xx[39] = xx[45] + xx[51];
  xx[40] = xx[46] + xx[52];
  xx[41] = xx[47] + xx[53];
  pm_math_Quaternion_xform_ra(xx + 32, xx + 39, xx + 45);
  xx[32] = motionData[56];
  xx[33] = motionData[57];
  xx[34] = motionData[58];
  xx[35] = motionData[59];
  xx[39] = - 0.8969355644339009;
  xx[40] = 0.2764797434074396;
  xx[41] = 0.158748647566253;
  xx[42] = - 0.3063729942976073;
  xx[3] = xx[0] * state[8];
  xx[0] = sin(xx[3]);
  xx[48] = cos(xx[3]);
  xx[49] = 0.5701273767950668 * xx[0];
  xx[50] = - (0.3420242615573683 * xx[0]);
  xx[51] = - (0.7469766922300272 * xx[0]);
  pm_math_Quaternion_compose_ra(xx + 39, xx + 48, xx + 52);
  pm_math_Quaternion_compose_ra(xx + 32, xx + 52, xx + 39);
  xx[48] = 0.01101107078826512;
  xx[49] = 6.902288978823598e-3;
  xx[50] = 5.243754797069989e-3;
  pm_math_Quaternion_xform_ra(xx + 39, xx + 48, xx + 62);
  xx[39] = - 6.027837242311112e-3;
  xx[40] = 5.834159102525993e-8;
  xx[41] = - 4.600752640201167e-3;
  pm_math_Quaternion_xform_ra(xx + 52, xx + 39, xx + 48);
  pm_math_Quaternion_xform_ra(xx + 32, xx + 48, xx + 39);
  J[0] = xx[30];
  J[1] = xx[1];
  J[2] = xx[43];
  J[3] = xx[13];
  J[4] = xx[60];
  J[7] = xx[31];
  J[8] = xx[2];
  J[9] = xx[44];
  J[10] = xx[14];
  J[11] = xx[61];
  J[14] = - (xx[5] + xx[8] + xx[23]);
  J[15] = - (xx[66] + xx[20]);
  J[16] = xx[36] + xx[26] + xx[56];
  J[17] = xx[15] + xx[45];
  J[18] = xx[62] + xx[39];
  J[21] = - (xx[6] + xx[9] + xx[24]);
  J[22] = - (xx[67] + xx[21]);
  J[23] = xx[37] + xx[27] + xx[57];
  J[24] = xx[16] + xx[46];
  J[25] = xx[63] + xx[40];
  J[28] = - (xx[7] + xx[10] + xx[25]);
  J[29] = - (xx[68] + xx[22]);
  J[30] = xx[38] + xx[28] + xx[58];
  J[31] = xx[17] + xx[47];
  J[32] = xx[64] + xx[41];
  return 5;
}

static size_t computeAssemblyJacobian_1(const RuntimeDerivedValuesBundle *rtdv,
  const double *state, const int *modeVector, const double *motionData, double
  *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[72];
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = 9.87654321;
  xx[1] = motionData[77];
  xx[2] = motionData[78];
  xx[3] = motionData[79];
  xx[4] = motionData[80];
  xx[5] = 0.300048825040154;
  xx[6] = 0.7582880608547207;
  xx[7] = 0.2970285731203686;
  xx[8] = 0.4967332746125314;
  pm_math_Quaternion_compose_ra(xx + 1, xx + 5, xx + 9);
  xx[5] = motionData[63];
  xx[6] = motionData[64];
  xx[7] = motionData[65];
  xx[8] = motionData[66];
  xx[13] = 0.3676062374005039;
  xx[14] = - 0.4418868990508547;
  xx[15] = - 0.6040401092638356;
  xx[16] = 0.552030043631683;
  pm_math_Quaternion_compose_ra(xx + 5, xx + 13, xx + 17);
  pm_math_Quaternion_inverseCompose_ra(xx + 9, xx + 17, xx + 5);
  xx[9] = motionData[70];
  xx[10] = motionData[71];
  xx[11] = motionData[72];
  xx[12] = motionData[73];
  xx[21] = 0.5165599445270539;
  xx[22] = 0.7400480272337564;
  xx[23] = 0.4306910041986394;
  pm_math_Quaternion_inverseXform_ra(xx + 9, xx + 21, xx + 24);
  pm_math_Quaternion_inverseXform_ra(xx + 13, xx + 24, xx + 27);
  pm_math_Quaternion_compDeriv_ra(xx + 5, xx + 27, xx + 30);
  xx[34] = motionData[28];
  xx[35] = motionData[29];
  xx[36] = motionData[30];
  xx[37] = motionData[31];
  xx[27] = 0.5038982060378953;
  xx[28] = 0.8089781762292564;
  xx[29] = 0.3027224939388842;
  xx[38] = - xx[27];
  xx[39] = xx[28];
  xx[40] = - xx[29];
  pm_math_Quaternion_inverseXform_ra(xx + 34, xx + 38, xx + 41);
  pm_math_Quaternion_inverseXform_ra(xx + 13, xx + 41, xx + 44);
  pm_math_Quaternion_compDeriv_ra(xx + 5, xx + 44, xx + 13);
  xx[44] = - 0.4820907072649068;
  xx[45] = 0.8137976813493766;
  xx[46] = - 0.3245333323392225;
  pm_math_Quaternion_compDeriv_ra(xx + 5, xx + 44, xx + 47);
  xx[51] = motionData[42];
  xx[52] = motionData[43];
  xx[53] = motionData[44];
  xx[54] = motionData[45];
  xx[44] = 0.6587699864704037;
  xx[45] = 0.5622364142279257;
  xx[46] = 0.4999123117526796;
  pm_math_Quaternion_inverseXform_ra(xx + 51, xx + 44, xx + 55);
  pm_math_Quaternion_xform_ra(xx + 1, xx + 55, xx + 58);
  pm_math_Quaternion_inverseXform_ra(xx + 17, xx + 58, xx + 61);
  xx[58] = - xx[61];
  xx[59] = - xx[62];
  xx[60] = - xx[63];
  pm_math_Quaternion_compDeriv_ra(xx + 5, xx + 58, xx + 61);
  xx[58] = 0.5038982060378495;
  xx[59] = 0.8089781762292761;
  xx[60] = 0.3027224939389068;
  xx[65] = - xx[58];
  xx[66] = xx[59];
  xx[67] = - xx[60];
  pm_math_Quaternion_xform_ra(xx + 1, xx + 65, xx + 68);
  pm_math_Quaternion_inverseXform_ra(xx + 17, xx + 68, xx + 1);
  xx[17] = - xx[1];
  xx[18] = - xx[2];
  xx[19] = - xx[3];
  pm_math_Quaternion_compDeriv_ra(xx + 5, xx + 17, xx + 1);
  xx[4] = 0.324402019972231;
  xx[5] = 0.1793201049669474;
  xx[6] = 0.8745212078931046;
  xx[7] = 0.3127623480180516;
  xx[1] = 0.5;
  xx[8] = xx[1] * state[4];
  xx[13] = sin(xx[8]);
  xx[16] = cos(xx[8]);
  xx[17] = xx[21] * xx[13];
  xx[18] = xx[22] * xx[13];
  xx[19] = xx[23] * xx[13];
  pm_math_Quaternion_compose_ra(xx + 4, xx + 16, xx + 64);
  pm_math_Quaternion_compose_ra(xx + 64, xx + 9, xx + 4);
  xx[8] = - 0.01680036206308408;
  xx[9] = 1.904183946736232e-3;
  xx[10] = - 2.167844671666771e-3;
  pm_math_Vector3_cross_ra(xx + 24, xx + 8, xx + 11);
  pm_math_Quaternion_xform_ra(xx + 4, xx + 11, xx + 16);
  xx[4] = motionData[74];
  xx[5] = motionData[75];
  xx[6] = motionData[76];
  pm_math_Vector3_cross_ra(xx + 21, xx + 4, xx + 11);
  pm_math_Quaternion_xform_ra(xx + 64, xx + 11, xx + 4);
  xx[11] = 3.977909360125871e-3;
  xx[12] = 4.397193980568331e-3;
  xx[13] = - 0.012326617731341;
  pm_math_Quaternion_xform_ra(xx + 64, xx + 11, xx + 19);
  xx[22] = motionData[14];
  xx[23] = motionData[15];
  xx[24] = motionData[16];
  xx[25] = motionData[17];
  xx[64] = - 0.8846589215343961;
  xx[65] = - 0.3902684207978444;
  xx[66] = 0.2533894697105629;
  xx[67] = - 0.02937565177755097;
  xx[7] = xx[1] * state[6];
  xx[11] = sin(xx[7]);
  xx[68] = cos(xx[7]);
  xx[69] = - (xx[27] * xx[11]);
  xx[70] = xx[28] * xx[11];
  xx[71] = - (xx[29] * xx[11]);
  pm_math_Quaternion_compose_ra(xx + 64, xx + 68, xx + 26);
  pm_math_Quaternion_compose_ra(xx + 22, xx + 26, xx + 64);
  pm_math_Quaternion_compose_ra(xx + 64, xx + 34, xx + 68);
  pm_math_Vector3_cross_ra(xx + 41, xx + 8, xx + 11);
  pm_math_Quaternion_xform_ra(xx + 68, xx + 11, xx + 7);
  xx[10] = motionData[32];
  xx[11] = motionData[33];
  xx[12] = motionData[34];
  pm_math_Vector3_cross_ra(xx + 38, xx + 10, xx + 33);
  pm_math_Quaternion_xform_ra(xx + 26, xx + 33, xx + 10);
  xx[33] = - 4.078893315886373e-3;
  xx[34] = - 5.827739056241395e-3;
  xx[35] = - 8.784172772109623e-3;
  pm_math_Quaternion_xform_ra(xx + 26, xx + 33, xx + 36);
  xx[26] = xx[10] + xx[36];
  xx[27] = xx[11] + xx[37];
  xx[28] = xx[12] + xx[38];
  pm_math_Quaternion_xform_ra(xx + 22, xx + 26, xx + 10);
  xx[22] = motionData[56];
  xx[23] = motionData[57];
  xx[24] = motionData[58];
  xx[25] = motionData[59];
  xx[26] = - 0.8969355644339009;
  xx[27] = 0.2764797434074396;
  xx[28] = 0.158748647566253;
  xx[29] = - 0.3063729942976073;
  xx[13] = xx[1] * state[8];
  xx[30] = sin(xx[13]);
  xx[33] = cos(xx[13]);
  xx[34] = 0.5701273767950668 * xx[30];
  xx[35] = - (0.3420242615573683 * xx[30]);
  xx[36] = - (0.7469766922300272 * xx[30]);
  pm_math_Quaternion_compose_ra(xx + 26, xx + 33, xx + 37);
  pm_math_Quaternion_compose_ra(xx + 22, xx + 37, xx + 26);
  xx[33] = 2.163836498928452e-3;
  xx[34] = 0.01378542647810592;
  xx[35] = - 4.660504030034749e-3;
  pm_math_Quaternion_xform_ra(xx + 26, xx + 33, xx + 41);
  xx[26] = - 6.027837242311112e-3;
  xx[27] = 5.834159102525993e-8;
  xx[28] = - 4.600752640201167e-3;
  pm_math_Quaternion_xform_ra(xx + 37, xx + 26, xx + 33);
  pm_math_Quaternion_xform_ra(xx + 22, xx + 33, xx + 26);
  xx[22] = 0.4437459878328764;
  xx[23] = 0.1803099382577183;
  xx[24] = 0.7643348123670592;
  xx[25] = 0.4317060563062962;
  xx[13] = xx[1] * state[10];
  xx[29] = sin(xx[13]);
  xx[33] = cos(xx[13]);
  xx[34] = xx[44] * xx[29];
  xx[35] = xx[45] * xx[29];
  xx[36] = xx[46] * xx[29];
  pm_math_Quaternion_compose_ra(xx + 22, xx + 33, xx + 37);
  pm_math_Quaternion_compose_ra(xx + 37, xx + 51, xx + 22);
  xx[33] = - 1.75429196580309e-3;
  xx[34] = 9.461285757155713e-3;
  xx[35] = - 5.462379018955269e-3;
  pm_math_Vector3_cross_ra(xx + 55, xx + 33, xx + 50);
  pm_math_Quaternion_xform_ra(xx + 22, xx + 50, xx + 33);
  xx[22] = motionData[46];
  xx[23] = motionData[47];
  xx[24] = motionData[48];
  pm_math_Vector3_cross_ra(xx + 44, xx + 22, xx + 50);
  pm_math_Quaternion_xform_ra(xx + 37, xx + 50, xx + 22);
  xx[44] = - 8.971513124181443e-3;
  xx[45] = 2.664040756922957e-3;
  xx[46] = 8.826233627727805e-3;
  pm_math_Quaternion_xform_ra(xx + 37, xx + 44, xx + 50);
  xx[36] = motionData[35];
  xx[37] = motionData[36];
  xx[38] = motionData[37];
  xx[39] = motionData[38];
  xx[44] = - 0.2207829197706712;
  xx[45] = - 0.6845661841871489;
  xx[46] = - 0.01617467817412936;
  xx[47] = 0.6945231613063365;
  xx[13] = xx[1] * state[12];
  xx[1] = sin(xx[13]);
  xx[53] = cos(xx[13]);
  xx[54] = - (xx[58] * xx[1]);
  xx[55] = xx[59] * xx[1];
  xx[56] = - (xx[60] * xx[1]);
  pm_math_Quaternion_compose_ra(xx + 44, xx + 53, xx + 57);
  pm_math_Quaternion_compose_ra(xx + 36, xx + 57, xx + 44);
  xx[53] = - 1.55480139635266e-3;
  xx[54] = - 2.221419349365449e-3;
  xx[55] = - 3.348341004773165e-3;
  pm_math_Quaternion_xform_ra(xx + 44, xx + 53, xx + 64);
  xx[44] = - 4.078893315886636e-3;
  xx[45] = - 5.827739056241714e-3;
  xx[46] = - 8.784172772110375e-3;
  pm_math_Quaternion_xform_ra(xx + 57, xx + 44, xx + 53);
  pm_math_Quaternion_xform_ra(xx + 36, xx + 53, xx + 44);
  J[2] = xx[31];
  J[3] = xx[14];
  J[4] = xx[48];
  J[5] = xx[62];
  J[6] = xx[2];
  J[9] = xx[32];
  J[10] = xx[15];
  J[11] = xx[49];
  J[12] = xx[63];
  J[13] = xx[3];
  J[16] = xx[16] + xx[4] + xx[19];
  J[17] = xx[7] + xx[10];
  J[18] = xx[41] + xx[26];
  J[19] = - (xx[33] + xx[22] + xx[50]);
  J[20] = - (xx[64] + xx[44]);
  J[23] = xx[17] + xx[5] + xx[20];
  J[24] = xx[8] + xx[11];
  J[25] = xx[42] + xx[27];
  J[26] = - (xx[34] + xx[23] + xx[51]);
  J[27] = - (xx[65] + xx[45]);
  J[30] = xx[18] + xx[6] + xx[21];
  J[31] = xx[9] + xx[12];
  J[32] = xx[43] + xx[28];
  J[33] = - (xx[35] + xx[24] + xx[52]);
  J[34] = - (xx[66] + xx[46]);
  return 5;
}

size_t DynamicValidation_dfaed711_1_computeAssemblyJacobian(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, boolean_T
  forVelocitySatisfaction, const double *state, const int *modeVector, const
  double *motionData, double *J)
{
  (void) mech;
  (void) rtdv;
  (void) state;
  (void) modeVector;
  (void) forVelocitySatisfaction;
  (void) motionData;
  (void) J;
  switch (constraintIdx)
  {
   case 0:
    return computeAssemblyJacobian_0(rtdv, state, modeVector, motionData, J);

   case 1:
    return computeAssemblyJacobian_1(rtdv, state, modeVector, motionData, J);
  }

  return 0;
}

size_t DynamicValidation_dfaed711_1_computeFullAssemblyJacobian(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, const double *state, const int
  *modeVector, const double *motionData, double *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[115];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  xx[0] = motionData[49];
  xx[1] = motionData[50];
  xx[2] = motionData[51];
  xx[3] = motionData[52];
  xx[4] = 0.4967332746125314;
  xx[5] = 0.7582880608547204;
  xx[6] = - 0.3000488250401547;
  xx[7] = - xx[4];
  xx[8] = 0.2970285731203689;
  pm_math_Quaternion_compose_ra(xx + 0, xx + 5, xx + 9);
  xx[5] = motionData[63];
  xx[6] = motionData[64];
  xx[7] = motionData[65];
  xx[8] = motionData[66];
  xx[13] = 0.2571292065762446;
  xx[14] = - 0.7069199206988735;
  xx[15] = - 0.658697181080302;
  xx[16] = 0.01633464053578271;
  pm_math_Quaternion_compose_ra(xx + 5, xx + 13, xx + 17);
  pm_math_Quaternion_inverseCompose_ra(xx + 9, xx + 17, xx + 21);
  xx[9] = motionData[7];
  xx[10] = motionData[8];
  xx[11] = motionData[9];
  xx[12] = motionData[10];
  xx[25] = 5.906528705024172e-8;
  xx[26] = 0.7816132178294208;
  xx[27] = 0.6237633988254963;
  pm_math_Quaternion_inverseXform_ra(xx + 9, xx + 25, xx + 28);
  pm_math_Quaternion_xform_ra(xx + 0, xx + 28, xx + 31);
  pm_math_Quaternion_inverseXform_ra(xx + 17, xx + 31, xx + 34);
  xx[31] = - xx[34];
  xx[32] = - xx[35];
  xx[33] = - xx[36];
  pm_math_Quaternion_compDeriv_ra(xx + 21, xx + 31, xx + 34);
  xx[31] = 0.503898206037895;
  xx[32] = 0.8089781762292558;
  xx[33] = 0.3027224939388848;
  xx[38] = - xx[31];
  xx[39] = xx[32];
  xx[40] = - xx[33];
  pm_math_Quaternion_xform_ra(xx + 0, xx + 38, xx + 41);
  pm_math_Quaternion_inverseXform_ra(xx + 17, xx + 41, xx + 0);
  xx[17] = - xx[0];
  xx[18] = - xx[1];
  xx[19] = - xx[2];
  pm_math_Quaternion_compDeriv_ra(xx + 21, xx + 17, xx + 0);
  xx[17] = motionData[70];
  xx[18] = motionData[71];
  xx[19] = motionData[72];
  xx[20] = motionData[73];
  xx[38] = 0.5165599445270539;
  xx[39] = 0.7400480272337564;
  xx[40] = 0.4306910041986394;
  pm_math_Quaternion_inverseXform_ra(xx + 17, xx + 38, xx + 41);
  pm_math_Quaternion_inverseXform_ra(xx + 13, xx + 41, xx + 44);
  pm_math_Quaternion_compDeriv_ra(xx + 21, xx + 44, xx + 47);
  xx[51] = motionData[28];
  xx[52] = motionData[29];
  xx[53] = motionData[30];
  xx[54] = motionData[31];
  xx[44] = 0.5038982060378953;
  xx[45] = 0.8089781762292564;
  xx[46] = 0.3027224939388842;
  xx[55] = - xx[44];
  xx[56] = xx[45];
  xx[57] = - xx[46];
  pm_math_Quaternion_inverseXform_ra(xx + 51, xx + 55, xx + 58);
  pm_math_Quaternion_inverseXform_ra(xx + 13, xx + 58, xx + 61);
  pm_math_Quaternion_compDeriv_ra(xx + 21, xx + 61, xx + 13);
  xx[61] = - 0.4820907072649103;
  xx[62] = 0.8137976813493701;
  xx[63] = 0.3245333323392334;
  pm_math_Quaternion_compDeriv_ra(xx + 21, xx + 61, xx + 64);
  xx[21] = 9.87654321;
  xx[61] = 0.6190277160392075;
  xx[62] = - 0.2167287412203158;
  xx[63] = 0.7124684524173637;
  xx[64] = 0.2494434681733462;
  xx[0] = 0.5;
  xx[3] = xx[0] * state[0];
  xx[13] = sin(xx[3]);
  xx[67] = cos(xx[3]);
  xx[68] = xx[25] * xx[13];
  xx[69] = xx[26] * xx[13];
  xx[70] = xx[27] * xx[13];
  pm_math_Quaternion_compose_ra(xx + 61, xx + 67, xx + 71);
  pm_math_Quaternion_compose_ra(xx + 71, xx + 9, xx + 61);
  xx[9] = - 0.02695619245963533;
  xx[10] = 0.01378863199166597;
  xx[11] = 3.369121440618853e-3;
  pm_math_Vector3_cross_ra(xx + 28, xx + 9, xx + 22);
  pm_math_Quaternion_xform_ra(xx + 61, xx + 22, xx + 9);
  xx[22] = motionData[11];
  xx[23] = motionData[12];
  xx[24] = motionData[13];
  pm_math_Vector3_cross_ra(xx + 25, xx + 22, xx + 28);
  pm_math_Quaternion_xform_ra(xx + 71, xx + 28, xx + 22);
  xx[25] = 0.01500104515676648;
  xx[26] = 1.380515327287848e-10;
  xx[27] = - 1.593463070824624e-9;
  pm_math_Quaternion_xform_ra(xx + 71, xx + 25, xx + 28);
  xx[61] = motionData[0];
  xx[62] = motionData[1];
  xx[63] = motionData[2];
  xx[64] = motionData[3];
  xx[67] = - 0.9578985788971721;
  xx[68] = 0.01596084651607334;
  xx[69] = 0.2160322563859199;
  xx[70] = 0.188429371718097;
  xx[3] = xx[0] * state[2];
  xx[12] = sin(xx[3]);
  xx[71] = cos(xx[3]);
  xx[72] = - (xx[31] * xx[12]);
  xx[73] = xx[32] * xx[12];
  xx[74] = - (xx[33] * xx[12]);
  pm_math_Quaternion_compose_ra(xx + 67, xx + 71, xx + 31);
  pm_math_Quaternion_compose_ra(xx + 61, xx + 31, xx + 67);
  xx[25] = 6.89967478304934e-3;
  xx[26] = 9.857940058329018e-3;
  xx[27] = 0.01485890448976339;
  pm_math_Quaternion_xform_ra(xx + 67, xx + 25, xx + 71);
  xx[25] = - 4.078893315886357e-3;
  xx[26] = - 5.827739056241386e-3;
  xx[27] = - 8.784172772109595e-3;
  pm_math_Quaternion_xform_ra(xx + 31, xx + 25, xx + 67);
  pm_math_Quaternion_xform_ra(xx + 61, xx + 67, xx + 25);
  xx[31] = 0.324402019972231;
  xx[32] = 0.1793201049669474;
  xx[33] = 0.8745212078931046;
  xx[34] = 0.3127623480180516;
  xx[3] = xx[0] * state[4];
  xx[12] = sin(xx[3]);
  xx[61] = cos(xx[3]);
  xx[62] = xx[38] * xx[12];
  xx[63] = xx[39] * xx[12];
  xx[64] = xx[40] * xx[12];
  pm_math_Quaternion_compose_ra(xx + 31, xx + 61, xx + 67);
  pm_math_Quaternion_compose_ra(xx + 67, xx + 17, xx + 31);
  xx[16] = - 3.265857732358156e-3;
  xx[17] = 0.01115673029435595;
  xx[18] = - 7.827670717550887e-3;
  pm_math_Vector3_cross_ra(xx + 41, xx + 16, xx + 61);
  pm_math_Quaternion_xform_ra(xx + 31, xx + 61, xx + 74);
  xx[61] = motionData[74];
  xx[62] = motionData[75];
  xx[63] = motionData[76];
  pm_math_Vector3_cross_ra(xx + 38, xx + 61, xx + 77);
  pm_math_Quaternion_xform_ra(xx + 67, xx + 77, xx + 37);
  xx[61] = 3.977909360125871e-3;
  xx[62] = 4.397193980568331e-3;
  xx[63] = - 0.012326617731341;
  pm_math_Quaternion_xform_ra(xx + 67, xx + 61, xx + 77);
  xx[3] = xx[37] + xx[77];
  xx[61] = motionData[14];
  xx[62] = motionData[15];
  xx[63] = motionData[16];
  xx[64] = motionData[17];
  xx[67] = - 0.8846589215343961;
  xx[68] = - 0.3902684207978444;
  xx[69] = 0.2533894697105629;
  xx[70] = - 0.02937565177755097;
  xx[12] = xx[0] * state[6];
  xx[13] = sin(xx[12]);
  xx[80] = cos(xx[12]);
  xx[81] = - (xx[44] * xx[13]);
  xx[82] = xx[45] * xx[13];
  xx[83] = - (xx[46] * xx[13]);
  pm_math_Quaternion_compose_ra(xx + 67, xx + 80, xx + 44);
  pm_math_Quaternion_compose_ra(xx + 61, xx + 44, xx + 67);
  pm_math_Quaternion_compose_ra(xx + 67, xx + 51, xx + 80);
  pm_math_Vector3_cross_ra(xx + 58, xx + 16, xx + 50);
  pm_math_Quaternion_xform_ra(xx + 80, xx + 50, xx + 16);
  xx[50] = motionData[32];
  xx[51] = motionData[33];
  xx[52] = motionData[34];
  pm_math_Vector3_cross_ra(xx + 55, xx + 50, xx + 67);
  pm_math_Quaternion_xform_ra(xx + 44, xx + 67, xx + 50);
  xx[53] = - 4.078893315886373e-3;
  xx[54] = - 5.827739056241395e-3;
  xx[55] = - 8.784172772109623e-3;
  pm_math_Quaternion_xform_ra(xx + 44, xx + 53, xx + 67);
  xx[44] = xx[50] + xx[67];
  xx[45] = xx[51] + xx[68];
  xx[46] = xx[52] + xx[69];
  pm_math_Quaternion_xform_ra(xx + 61, xx + 44, xx + 50);
  xx[44] = motionData[56];
  xx[45] = motionData[57];
  xx[46] = motionData[58];
  xx[47] = motionData[59];
  xx[53] = - 0.8969355644339009;
  xx[54] = 0.2764797434074396;
  xx[55] = 0.158748647566253;
  xx[56] = - 0.3063729942976073;
  xx[12] = xx[0] * state[8];
  xx[13] = sin(xx[12]);
  xx[61] = cos(xx[12]);
  xx[62] = 0.5701273767950668 * xx[13];
  xx[63] = - (0.3420242615573683 * xx[13]);
  xx[64] = - (0.7469766922300272 * xx[13]);
  pm_math_Quaternion_compose_ra(xx + 53, xx + 61, xx + 67);
  pm_math_Quaternion_compose_ra(xx + 44, xx + 67, xx + 53);
  xx[61] = 0.01101107078826512;
  xx[62] = 6.902288978823598e-3;
  xx[63] = 5.243754797069989e-3;
  pm_math_Quaternion_xform_ra(xx + 53, xx + 61, xx + 84);
  xx[61] = - 6.027837242311112e-3;
  xx[62] = 5.834159102525993e-8;
  xx[63] = - 4.600752640201167e-3;
  pm_math_Quaternion_xform_ra(xx + 67, xx + 61, xx + 87);
  pm_math_Quaternion_xform_ra(xx + 44, xx + 87, xx + 61);
  xx[12] = xx[38] + xx[78];
  xx[13] = xx[39] + xx[79];
  xx[37] = motionData[77];
  xx[38] = motionData[78];
  xx[39] = motionData[79];
  xx[40] = motionData[80];
  xx[44] = 0.300048825040154;
  xx[45] = 0.7582880608547207;
  xx[46] = 0.2970285731203686;
  xx[47] = xx[4];
  pm_math_Quaternion_compose_ra(xx + 37, xx + 44, xx + 67);
  xx[44] = 0.3676062374005039;
  xx[45] = - 0.4418868990508547;
  xx[46] = - 0.6040401092638356;
  xx[47] = 0.552030043631683;
  pm_math_Quaternion_compose_ra(xx + 5, xx + 44, xx + 87);
  pm_math_Quaternion_inverseCompose_ra(xx + 67, xx + 87, xx + 4);
  pm_math_Quaternion_inverseXform_ra(xx + 44, xx + 41, xx + 67);
  pm_math_Quaternion_compDeriv_ra(xx + 4, xx + 67, xx + 91);
  pm_math_Quaternion_inverseXform_ra(xx + 44, xx + 58, xx + 67);
  pm_math_Quaternion_compDeriv_ra(xx + 4, xx + 67, xx + 44);
  xx[67] = - 0.4820907072649068;
  xx[68] = 0.8137976813493766;
  xx[69] = - 0.3245333323392225;
  pm_math_Quaternion_compDeriv_ra(xx + 4, xx + 67, xx + 95);
  xx[67] = motionData[42];
  xx[68] = motionData[43];
  xx[69] = motionData[44];
  xx[70] = motionData[45];
  xx[8] = 0.6587699864704037;
  xx[19] = 0.5622364142279257;
  xx[20] = 0.4999123117526796;
  xx[77] = xx[8];
  xx[78] = xx[19];
  xx[79] = xx[20];
  pm_math_Quaternion_inverseXform_ra(xx + 67, xx + 77, xx + 99);
  pm_math_Quaternion_xform_ra(xx + 37, xx + 99, xx + 102);
  pm_math_Quaternion_inverseXform_ra(xx + 87, xx + 102, xx + 105);
  xx[102] = - xx[105];
  xx[103] = - xx[106];
  xx[104] = - xx[107];
  pm_math_Quaternion_compDeriv_ra(xx + 4, xx + 102, xx + 105);
  xx[57] = 0.5038982060378495;
  xx[64] = 0.8089781762292761;
  xx[102] = 0.3027224939389068;
  xx[109] = - xx[57];
  xx[110] = xx[64];
  xx[111] = - xx[102];
  pm_math_Quaternion_xform_ra(xx + 37, xx + 109, xx + 112);
  pm_math_Quaternion_inverseXform_ra(xx + 87, xx + 112, xx + 37);
  xx[87] = - xx[37];
  xx[88] = - xx[38];
  xx[89] = - xx[39];
  pm_math_Quaternion_compDeriv_ra(xx + 4, xx + 87, xx + 37);
  xx[4] = - 0.01680036206308408;
  xx[5] = 1.904183946736232e-3;
  xx[6] = - 2.167844671666771e-3;
  pm_math_Vector3_cross_ra(xx + 41, xx + 4, xx + 87);
  pm_math_Quaternion_xform_ra(xx + 31, xx + 87, xx + 40);
  pm_math_Vector3_cross_ra(xx + 58, xx + 4, xx + 31);
  pm_math_Quaternion_xform_ra(xx + 80, xx + 31, xx + 4);
  xx[31] = 2.163836498928452e-3;
  xx[32] = 0.01378542647810592;
  xx[33] = - 4.660504030034749e-3;
  pm_math_Quaternion_xform_ra(xx + 53, xx + 31, xx + 58);
  xx[31] = 0.4437459878328764;
  xx[32] = 0.1803099382577183;
  xx[33] = 0.7643348123670592;
  xx[34] = 0.4317060563062962;
  xx[7] = xx[0] * state[10];
  xx[37] = sin(xx[7]);
  xx[53] = cos(xx[7]);
  xx[54] = xx[8] * xx[37];
  xx[55] = xx[19] * xx[37];
  xx[56] = xx[20] * xx[37];
  pm_math_Quaternion_compose_ra(xx + 31, xx + 53, xx + 80);
  pm_math_Quaternion_compose_ra(xx + 80, xx + 67, xx + 31);
  xx[53] = - 1.75429196580309e-3;
  xx[54] = 9.461285757155713e-3;
  xx[55] = - 5.462379018955269e-3;
  pm_math_Vector3_cross_ra(xx + 99, xx + 53, xx + 67);
  pm_math_Quaternion_xform_ra(xx + 31, xx + 67, xx + 53);
  xx[31] = motionData[46];
  xx[32] = motionData[47];
  xx[33] = motionData[48];
  pm_math_Vector3_cross_ra(xx + 77, xx + 31, xx + 67);
  pm_math_Quaternion_xform_ra(xx + 80, xx + 67, xx + 31);
  xx[67] = - 8.971513124181443e-3;
  xx[68] = 2.664040756922957e-3;
  xx[69] = 8.826233627727805e-3;
  pm_math_Quaternion_xform_ra(xx + 80, xx + 67, xx + 77);
  xx[67] = motionData[35];
  xx[68] = motionData[36];
  xx[69] = motionData[37];
  xx[70] = motionData[38];
  xx[80] = - 0.2207829197706712;
  xx[81] = - 0.6845661841871489;
  xx[82] = - 0.01617467817412936;
  xx[83] = 0.6945231613063365;
  xx[7] = xx[0] * state[12];
  xx[0] = sin(xx[7]);
  xx[87] = cos(xx[7]);
  xx[88] = - (xx[57] * xx[0]);
  xx[89] = xx[64] * xx[0];
  xx[90] = - (xx[102] * xx[0]);
  pm_math_Quaternion_compose_ra(xx + 80, xx + 87, xx + 98);
  pm_math_Quaternion_compose_ra(xx + 67, xx + 98, xx + 80);
  xx[87] = - 1.55480139635266e-3;
  xx[88] = - 2.221419349365449e-3;
  xx[89] = - 3.348341004773165e-3;
  pm_math_Quaternion_xform_ra(xx + 80, xx + 87, xx + 102);
  xx[80] = - 4.078893315886636e-3;
  xx[81] = - 5.827739056241714e-3;
  xx[82] = - 8.784172772110375e-3;
  pm_math_Quaternion_xform_ra(xx + 98, xx + 80, xx + 87);
  pm_math_Quaternion_xform_ra(xx + 67, xx + 87, xx + 80);
  J[0] = xx[35];
  J[1] = xx[1];
  J[2] = xx[48];
  J[3] = xx[14];
  J[4] = xx[65];
  J[7] = xx[36];
  J[8] = xx[2];
  J[9] = xx[49];
  J[10] = xx[15];
  J[11] = xx[66];
  J[14] = - (xx[9] + xx[22] + xx[28]);
  J[15] = - (xx[71] + xx[25]);
  J[16] = xx[74] + xx[3];
  J[17] = xx[16] + xx[50];
  J[18] = xx[84] + xx[61];
  J[21] = - (xx[10] + xx[23] + xx[29]);
  J[22] = - (xx[72] + xx[26]);
  J[23] = xx[75] + xx[12];
  J[24] = xx[17] + xx[51];
  J[25] = xx[85] + xx[62];
  J[28] = - (xx[11] + xx[24] + xx[30]);
  J[29] = - (xx[73] + xx[27]);
  J[30] = xx[76] + xx[13];
  J[31] = xx[18] + xx[52];
  J[32] = xx[86] + xx[63];
  J[37] = xx[92];
  J[38] = xx[45];
  J[39] = xx[96];
  J[40] = xx[106];
  J[41] = xx[38];
  J[44] = xx[93];
  J[45] = xx[46];
  J[46] = xx[97];
  J[47] = xx[107];
  J[48] = xx[39];
  J[51] = xx[40] + xx[3];
  J[52] = xx[4] + xx[50];
  J[53] = xx[58] + xx[61];
  J[54] = - (xx[53] + xx[31] + xx[77]);
  J[55] = - (xx[102] + xx[80]);
  J[58] = xx[41] + xx[12];
  J[59] = xx[5] + xx[51];
  J[60] = xx[59] + xx[62];
  J[61] = - (xx[54] + xx[32] + xx[78]);
  J[62] = - (xx[103] + xx[81]);
  J[65] = xx[42] + xx[13];
  J[66] = xx[6] + xx[52];
  J[67] = xx[60] + xx[63];
  J[68] = - (xx[55] + xx[33] + xx[79]);
  J[69] = - (xx[104] + xx[82]);
  return 10;
}

static boolean_T isInKinematicSingularity_0(const RuntimeDerivedValuesBundle
  *rtdv, const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

static boolean_T isInKinematicSingularity_1(const RuntimeDerivedValuesBundle
  *rtdv, const int *modeVector, const double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvd;
  (void) rtdvi;
  (void) modeVector;
  (void) motionData;
  return 0;
}

boolean_T DynamicValidation_dfaed711_1_isInKinematicSingularity(const void *mech,
  const RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int
  *modeVector, const double *motionData)
{
  (void) mech;
  (void) rtdv;
  (void) modeVector;
  (void) motionData;
  switch (constraintIdx)
  {
   case 0:
    return isInKinematicSingularity_0(rtdv, modeVector, motionData);

   case 1:
    return isInKinematicSingularity_1(rtdv, modeVector, motionData);
  }

  return 0;
}

void DynamicValidation_dfaed711_1_convertStateVector(const void *asmMech, const
  RuntimeDerivedValuesBundle *rtdv, const void *simMech, const double *asmState,
  const int *asmModeVector, const int *simModeVector, double *simState)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) asmMech;
  (void) rtdvd;
  (void) rtdvi;
  (void) simMech;
  (void) asmModeVector;
  (void) simModeVector;
  simState[0] = asmState[0];
  simState[1] = asmState[1];
  simState[2] = asmState[2];
  simState[3] = asmState[3];
  simState[4] = asmState[14];
  simState[5] = asmState[15];
  simState[6] = asmState[4];
  simState[7] = asmState[5];
  simState[8] = asmState[6];
  simState[9] = asmState[7];
  simState[10] = asmState[10];
  simState[11] = asmState[11];
  simState[12] = asmState[12];
  simState[13] = asmState[13];
  simState[14] = asmState[8];
  simState[15] = asmState[9];
  simState[16] = asmState[16];
  simState[17] = asmState[17];
}
