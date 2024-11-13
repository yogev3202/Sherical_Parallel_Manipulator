/* This file was automatically generated by CasADi 3.6.4.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) path_con_fun_ ## ID
#endif

#include <math.h>
#include <string.h>
#ifdef MATLAB_MEX_FILE
#include <mex.h>
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_from_mex CASADI_PREFIX(from_mex)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_sq CASADI_PREFIX(sq)
#define casadi_to_mex CASADI_PREFIX(to_mex)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

casadi_real casadi_sq(casadi_real x) { return x*x;}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

#ifdef MATLAB_MEX_FILE
casadi_real* casadi_from_mex(const mxArray* p, casadi_real* y, const casadi_int* sp, casadi_real* w) {
  casadi_int nrow, ncol, is_sparse, c, k, p_nrow, p_ncol;
  const casadi_int *colind, *row;
  mwIndex *Jc, *Ir;
  const double* p_data;
  if (!mxIsDouble(p) || mxGetNumberOfDimensions(p)!=2)
    mexErrMsgIdAndTxt("Casadi:RuntimeError",
      "\"from_mex\" failed: Not a two-dimensional matrix of double precision.");
  nrow = *sp++;
  ncol = *sp++;
  colind = sp;
  row = sp+ncol+1;
  p_nrow = mxGetM(p);
  p_ncol = mxGetN(p);
  is_sparse = mxIsSparse(p);
  Jc = 0;
  Ir = 0;
  if (is_sparse) {
    Jc = mxGetJc(p);
    Ir = mxGetIr(p);
  }
  p_data = (const double*)mxGetData(p);
  if (p_nrow==1 && p_ncol==1) {
    casadi_int nnz;
    double v = is_sparse && Jc[1]==0 ? 0 : *p_data;
    nnz = sp[ncol];
    casadi_fill(y, nnz, v);
  } else {
    casadi_int tr = 0;
    if (nrow!=p_nrow || ncol!=p_ncol) {
      tr = nrow==p_ncol && ncol==p_nrow && (nrow==1 || ncol==1);
      if (!tr) mexErrMsgIdAndTxt("Casadi:RuntimeError",
                                 "\"from_mex\" failed: Dimension mismatch. "
                                 "Expected %d-by-%d, got %d-by-%d instead.",
                                 nrow, ncol, p_nrow, p_ncol);
    }
    if (is_sparse) {
      if (tr) {
        for (c=0; c<ncol; ++c)
          for (k=colind[c]; k<colind[c+1]; ++k) w[row[k]+c*nrow]=0;
        for (c=0; c<p_ncol; ++c)
          for (k=Jc[c]; k<(casadi_int) Jc[c+1]; ++k) w[c+Ir[k]*p_ncol] = p_data[k];
        for (c=0; c<ncol; ++c)
          for (k=colind[c]; k<colind[c+1]; ++k) y[k] = w[row[k]+c*nrow];
      } else {
        for (c=0; c<ncol; ++c) {
          for (k=colind[c]; k<colind[c+1]; ++k) w[row[k]]=0;
          for (k=Jc[c]; k<(casadi_int) Jc[c+1]; ++k) w[Ir[k]]=p_data[k];
          for (k=colind[c]; k<colind[c+1]; ++k) y[k]=w[row[k]];
        }
      }
    } else {
      for (c=0; c<ncol; ++c) {
        for (k=colind[c]; k<colind[c+1]; ++k) {
          y[k] = p_data[row[k]+c*nrow];
        }
      }
    }
  }
  return y;
}

#endif

#define casadi_to_double(x) ((double) x)

#ifdef MATLAB_MEX_FILE
mxArray* casadi_to_mex(const casadi_int* sp, const casadi_real* x) {
  casadi_int nrow, ncol, c, k;
#ifndef CASADI_MEX_NO_SPARSE
  casadi_int nnz;
#endif
  const casadi_int *colind, *row;
  mxArray *p;
  double *d;
#ifndef CASADI_MEX_NO_SPARSE
  casadi_int i;
  mwIndex *j;
#endif /* CASADI_MEX_NO_SPARSE */
  nrow = *sp++;
  ncol = *sp++;
  colind = sp;
  row = sp+ncol+1;
#ifndef CASADI_MEX_NO_SPARSE
  nnz = sp[ncol];
  if (nnz!=nrow*ncol) {
    p = mxCreateSparse(nrow, ncol, nnz, mxREAL);
    for (i=0, j=mxGetJc(p); i<=ncol; ++i) *j++ = *colind++;
    for (i=0, j=mxGetIr(p); i<nnz; ++i) *j++ = *row++;
    if (x) {
      d = (double*)mxGetData(p);
      for (i=0; i<nnz; ++i) *d++ = casadi_to_double(*x++);
    }
    return p;
  }
#endif /* CASADI_MEX_NO_SPARSE */
  p = mxCreateDoubleMatrix(nrow, ncol, mxREAL);
  if (x) {
    d = (double*)mxGetData(p);
    for (c=0; c<ncol; ++c) {
      for (k=colind[c]; k<colind[c+1]; ++k) {
        d[row[k]+c*nrow] = casadi_to_double(*x++);
      }
    }
  }
  return p;
}

#endif

static const casadi_int casadi_s0[16] = {12, 1, 0, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[19] = {15, 1, 0, 15, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14};
static const casadi_int casadi_s3[11] = {7, 1, 0, 7, 0, 1, 2, 3, 4, 5, 6};

/* path_con_fun:(states[12],controls[3],params[15])->(general_con[7]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a46, a47, a48, a49, a5, a50, a51, a52, a53, a54, a55, a6, a7, a8, a9;
  a0=9.0630778703664994e-01;
  a1=9.3969262078590843e-01;
  a2=arg[0]? arg[0][0] : 0;
  a3=cos(a2);
  a4=arg[0]? arg[0][1] : 0;
  a5=sin(a4);
  a6=(a3*a5);
  a7=arg[0]? arg[0][2] : 0;
  a8=sin(a7);
  a9=(a6*a8);
  a10=sin(a2);
  a11=cos(a7);
  a12=(a10*a11);
  a9=(a9-a12);
  a12=(a1*a9);
  a13=-3.4202014332566871e-01;
  a14=sin(a7);
  a10=(a10*a14);
  a15=cos(a7);
  a6=(a6*a15);
  a10=(a10+a6);
  a6=(a13*a10);
  a12=(a12+a6);
  a12=(a0*a12);
  a12=casadi_sq(a12);
  a6=cos(a2);
  a11=(a6*a11);
  a16=sin(a2);
  a5=(a16*a5);
  a17=(a5*a8);
  a11=(a11+a17);
  a17=(a1*a11);
  a5=(a5*a15);
  a6=(a6*a14);
  a5=(a5-a6);
  a6=(a13*a5);
  a17=(a17+a6);
  a6=(a0*a17);
  a14=4.2261826174069944e-01;
  a18=cos(a4);
  a8=(a18*a8);
  a19=(a1*a8);
  a18=(a18*a15);
  a15=(a13*a18);
  a19=(a19+a15);
  a15=(a14*a19);
  a6=(a6+a15);
  a15=5.0000000000000011e-01;
  a6=(a6+a15);
  a20=-9.0630778703664994e-01;
  a17=(a20*a17);
  a19=(a14*a19);
  a17=(a17+a19);
  a17=(a17+a15);
  a6=(a6*a17);
  a12=(a12-a6);
  if (res[0]!=0) res[0][0]=a12;
  a12=-5.0000000000000044e-01;
  a6=-8.1379768134937358e-01;
  a17=cos(a4);
  a3=(a3*a17);
  a19=(a6*a3);
  a21=-4.6984631039295466e-01;
  a22=(a21*a9);
  a19=(a19+a22);
  a22=(a13*a10);
  a19=(a19+a22);
  a22=(a12*a19);
  a23=-8.6602540378443849e-01;
  a16=(a16*a17);
  a17=(a6*a16);
  a24=(a21*a11);
  a17=(a17+a24);
  a24=(a13*a5);
  a17=(a17+a24);
  a24=(a23*a17);
  a22=(a22-a24);
  a22=(a0*a22);
  a22=casadi_sq(a22);
  a24=(a23*a19);
  a25=(a12*a17);
  a24=(a24+a25);
  a24=(a0*a24);
  a25=(a21*a8);
  a26=sin(a4);
  a27=(a6*a26);
  a25=(a25-a27);
  a27=(a13*a18);
  a25=(a25+a27);
  a27=(a14*a25);
  a24=(a24+a27);
  a24=(a24+a15);
  a19=(a23*a19);
  a17=(a12*a17);
  a19=(a19+a17);
  a19=(a20*a19);
  a25=(a14*a25);
  a19=(a19+a25);
  a19=(a19+a15);
  a24=(a24*a19);
  a22=(a22-a24);
  if (res[0]!=0) res[0][1]=a22;
  a22=-4.9999999999999978e-01;
  a24=8.1379768134937380e-01;
  a3=(a24*a3);
  a19=-4.6984631039295399e-01;
  a9=(a19*a9);
  a3=(a3+a9);
  a10=(a13*a10);
  a3=(a3+a10);
  a10=(a22*a3);
  a9=8.6602540378443871e-01;
  a16=(a24*a16);
  a11=(a19*a11);
  a16=(a16+a11);
  a5=(a13*a5);
  a16=(a16+a5);
  a5=(a9*a16);
  a10=(a10-a5);
  a10=(a0*a10);
  a10=casadi_sq(a10);
  a5=(a9*a3);
  a11=(a22*a16);
  a5=(a5+a11);
  a5=(a0*a5);
  a8=(a19*a8);
  a26=(a24*a26);
  a8=(a8-a26);
  a18=(a13*a18);
  a8=(a8+a18);
  a18=(a14*a8);
  a5=(a5+a18);
  a5=(a5+a15);
  a3=(a9*a3);
  a16=(a22*a16);
  a3=(a3+a16);
  a3=(a20*a3);
  a8=(a14*a8);
  a3=(a3+a8);
  a3=(a3+a15);
  a5=(a5*a3);
  a10=(a10-a5);
  if (res[0]!=0) res[0][2]=a10;
  a10=3.3333333333333331e-01;
  a5=2.;
  a3=cos(a2);
  a8=sin(a4);
  a16=(a3*a8);
  a18=sin(a7);
  a26=(a16*a18);
  a11=sin(a2);
  a25=cos(a7);
  a17=(a11*a25);
  a26=(a26-a17);
  a17=(a1*a26);
  a27=sin(a7);
  a11=(a11*a27);
  a28=cos(a7);
  a16=(a16*a28);
  a11=(a11+a16);
  a16=(a13*a11);
  a17=(a17+a16);
  a17=(a0*a17);
  a16=casadi_sq(a17);
  a29=cos(a2);
  a25=(a29*a25);
  a30=sin(a2);
  a8=(a30*a8);
  a31=(a8*a18);
  a25=(a25+a31);
  a31=(a1*a25);
  a8=(a8*a28);
  a29=(a29*a27);
  a8=(a8-a29);
  a29=(a13*a8);
  a31=(a31+a29);
  a29=(a0*a31);
  a27=cos(a4);
  a18=(a27*a18);
  a32=(a1*a18);
  a27=(a27*a28);
  a28=(a13*a27);
  a32=(a32+a28);
  a28=(a14*a32);
  a29=(a29+a28);
  a29=(a29+a15);
  a31=(a20*a31);
  a32=(a14*a32);
  a31=(a31+a32);
  a31=(a31+a15);
  a31=(a29*a31);
  a16=(a16-a31);
  a16=sqrt(a16);
  a17=(a17+a16);
  a17=(a17/a29);
  a17=(-a17);
  a17=atan(a17);
  a17=(a5*a17);
  a29=cos(a17);
  a29=(a0*a29);
  a16=cos(a4);
  a31=sin(a7);
  a32=(a16*a31);
  a28=(a1*a32);
  a33=cos(a7);
  a16=(a16*a33);
  a34=(a13*a16);
  a28=(a28+a34);
  a34=(a29*a28);
  a35=-4.2261826174069944e-01;
  a36=cos(a2);
  a37=cos(a7);
  a38=(a36*a37);
  a39=sin(a2);
  a40=sin(a4);
  a41=(a39*a40);
  a42=(a41*a31);
  a38=(a38+a42);
  a42=(a1*a38);
  a41=(a41*a33);
  a43=sin(a7);
  a36=(a36*a43);
  a41=(a41-a36);
  a36=(a13*a41);
  a42=(a42+a36);
  a36=(a35*a42);
  a34=(a34-a36);
  a36=cos(a2);
  a40=(a36*a40);
  a31=(a40*a31);
  a44=sin(a2);
  a37=(a44*a37);
  a31=(a31-a37);
  a37=(a1*a31);
  a44=(a44*a43);
  a40=(a40*a33);
  a44=(a44+a40);
  a40=(a13*a44);
  a37=(a37+a40);
  a40=(a29*a37);
  a17=sin(a17);
  a17=(a0*a17);
  a33=(a17*a42);
  a40=(a40+a33);
  a34=(a34/a40);
  a33=(a10*a34);
  a33=(a33*a34);
  a43=cos(a4);
  a3=(a3*a43);
  a45=(a6*a3);
  a46=(a21*a26);
  a45=(a45+a46);
  a46=(a13*a11);
  a45=(a45+a46);
  a46=(a12*a45);
  a30=(a30*a43);
  a43=(a6*a30);
  a47=(a21*a25);
  a43=(a43+a47);
  a47=(a13*a8);
  a43=(a43+a47);
  a47=(a23*a43);
  a46=(a46-a47);
  a46=(a0*a46);
  a47=casadi_sq(a46);
  a48=(a23*a45);
  a49=(a12*a43);
  a48=(a48+a49);
  a48=(a0*a48);
  a49=(a21*a18);
  a50=sin(a4);
  a51=(a6*a50);
  a49=(a49-a51);
  a51=(a13*a27);
  a49=(a49+a51);
  a51=(a14*a49);
  a48=(a48+a51);
  a48=(a48+a15);
  a45=(a23*a45);
  a43=(a12*a43);
  a45=(a45+a43);
  a45=(a20*a45);
  a49=(a14*a49);
  a45=(a45+a49);
  a45=(a45+a15);
  a45=(a48*a45);
  a47=(a47-a45);
  a47=sqrt(a47);
  a46=(a46+a47);
  a46=(a46/a48);
  a46=(-a46);
  a46=atan(a46);
  a46=(a5*a46);
  a48=cos(a46);
  a48=(a0*a48);
  a48=(a12*a48);
  a47=-7.8488556722139569e-01;
  a45=sin(a46);
  a45=(a47*a45);
  a48=(a48+a45);
  a45=(a21*a32);
  a49=sin(a4);
  a43=(a6*a49);
  a45=(a45-a43);
  a43=(a13*a16);
  a45=(a45+a43);
  a43=(a48*a45);
  a51=cos(a4);
  a39=(a39*a51);
  a52=(a6*a39);
  a53=(a21*a38);
  a52=(a52+a53);
  a53=(a13*a41);
  a52=(a52+a53);
  a53=(a35*a52);
  a43=(a43-a53);
  a53=cos(a46);
  a53=(a0*a53);
  a53=(a23*a53);
  a54=-4.5315389351832536e-01;
  a46=sin(a46);
  a46=(a54*a46);
  a53=(a53-a46);
  a46=(a53*a52);
  a36=(a36*a51);
  a51=(a6*a36);
  a55=(a21*a31);
  a51=(a51+a55);
  a55=(a13*a44);
  a51=(a51+a55);
  a55=(a48*a51);
  a46=(a46-a55);
  a43=(a43/a46);
  a55=(a10*a43);
  a55=(a55*a43);
  a33=(a33+a55);
  a3=(a24*a3);
  a26=(a19*a26);
  a3=(a3+a26);
  a11=(a13*a11);
  a3=(a3+a11);
  a11=(a22*a3);
  a30=(a24*a30);
  a25=(a19*a25);
  a30=(a30+a25);
  a8=(a13*a8);
  a30=(a30+a8);
  a8=(a9*a30);
  a11=(a11-a8);
  a11=(a0*a11);
  a8=casadi_sq(a11);
  a25=(a9*a3);
  a26=(a22*a30);
  a25=(a25+a26);
  a25=(a0*a25);
  a18=(a19*a18);
  a50=(a24*a50);
  a18=(a18-a50);
  a27=(a13*a27);
  a18=(a18+a27);
  a27=(a14*a18);
  a25=(a25+a27);
  a25=(a25+a15);
  a3=(a9*a3);
  a30=(a22*a30);
  a3=(a3+a30);
  a3=(a20*a3);
  a18=(a14*a18);
  a3=(a3+a18);
  a3=(a3+a15);
  a3=(a25*a3);
  a8=(a8-a3);
  a8=sqrt(a8);
  a11=(a11+a8);
  a11=(a11/a25);
  a11=(-a11);
  a11=atan(a11);
  a11=(a5*a11);
  a25=cos(a11);
  a25=(a0*a25);
  a25=(a22*a25);
  a8=7.8488556722139580e-01;
  a3=sin(a11);
  a3=(a8*a3);
  a25=(a25+a3);
  a32=(a19*a32);
  a49=(a24*a49);
  a32=(a32-a49);
  a16=(a13*a16);
  a32=(a32+a16);
  a16=(a25*a32);
  a39=(a24*a39);
  a38=(a19*a38);
  a39=(a39+a38);
  a41=(a13*a41);
  a39=(a39+a41);
  a41=(a35*a39);
  a16=(a16-a41);
  a41=cos(a11);
  a41=(a0*a41);
  a41=(a9*a41);
  a38=-4.5315389351832475e-01;
  a11=sin(a11);
  a11=(a38*a11);
  a41=(a41-a11);
  a11=(a41*a39);
  a36=(a24*a36);
  a31=(a19*a31);
  a36=(a36+a31);
  a44=(a13*a44);
  a36=(a36+a44);
  a44=(a25*a36);
  a11=(a11-a44);
  a16=(a16/a11);
  a44=(a10*a16);
  a44=(a44*a16);
  a33=(a33+a44);
  a44=(a35*a37);
  a28=(a17*a28);
  a44=(a44+a28);
  a44=(a44/a40);
  a28=(a10*a44);
  a28=(a28*a44);
  a31=(a35*a51);
  a45=(a53*a45);
  a31=(a31-a45);
  a31=(a31/a46);
  a45=(a10*a31);
  a45=(a45*a31);
  a28=(a28+a45);
  a45=(a35*a36);
  a32=(a41*a32);
  a45=(a45-a32);
  a45=(a45/a11);
  a32=(a10*a45);
  a32=(a32*a45);
  a28=(a28+a32);
  a33=(a33+a28);
  a17=(a17*a42);
  a29=(a29*a37);
  a17=(a17+a29);
  a17=(a17/a40);
  a40=(a10*a17);
  a40=(a40*a17);
  a53=(a53*a52);
  a48=(a48*a51);
  a53=(a53-a48);
  a53=(a53/a46);
  a46=(a10*a53);
  a46=(a46*a53);
  a40=(a40+a46);
  a41=(a41*a39);
  a25=(a25*a36);
  a41=(a41-a25);
  a41=(a41/a11);
  a11=(a10*a41);
  a11=(a11*a41);
  a40=(a40+a11);
  a33=(a33+a40);
  a33=sqrt(a33);
  a40=(a31*a41);
  a11=(a45*a53);
  a40=(a40-a11);
  a11=(a31*a41);
  a25=(a45*a53);
  a11=(a11-a25);
  a11=(a34*a11);
  a25=(a44*a41);
  a36=(a45*a17);
  a25=(a25+a36);
  a25=(a43*a25);
  a11=(a11-a25);
  a25=(a44*a53);
  a36=(a31*a17);
  a25=(a25+a36);
  a25=(a16*a25);
  a11=(a11+a25);
  a40=(a40/a11);
  a40=(a10*a40);
  a25=(a31*a41);
  a36=(a53*a45);
  a25=(a25-a36);
  a36=(a31*a41);
  a39=(a53*a45);
  a36=(a36-a39);
  a36=(a34*a36);
  a39=(a43*a41);
  a46=(a53*a16);
  a39=(a39-a46);
  a39=(a44*a39);
  a36=(a36-a39);
  a39=(a43*a45);
  a46=(a31*a16);
  a39=(a39-a46);
  a39=(a17*a39);
  a36=(a36-a39);
  a25=(a25/a36);
  a40=(a40*a25);
  a25=(a43*a41);
  a39=(a16*a53);
  a25=(a25-a39);
  a25=(a25/a11);
  a25=(a10*a25);
  a39=(a43*a41);
  a46=(a53*a16);
  a39=(a39-a46);
  a39=(a39/a36);
  a25=(a25*a39);
  a40=(a40+a25);
  a25=(a43*a45);
  a39=(a16*a31);
  a25=(a25-a39);
  a25=(a25/a11);
  a25=(a10*a25);
  a39=(a43*a45);
  a46=(a31*a16);
  a39=(a39-a46);
  a39=(a39/a36);
  a25=(a25*a39);
  a40=(a40+a25);
  a25=(a44*a41);
  a39=(a45*a17);
  a25=(a25+a39);
  a25=(a25/a11);
  a25=(a10*a25);
  a39=(a44*a41);
  a46=(a17*a45);
  a39=(a39+a46);
  a39=(a39/a36);
  a25=(a25*a39);
  a39=(a34*a41);
  a46=(a16*a17);
  a39=(a39+a46);
  a39=(a39/a11);
  a39=(a10*a39);
  a41=(a34*a41);
  a46=(a17*a16);
  a41=(a41+a46);
  a41=(a41/a36);
  a39=(a39*a41);
  a25=(a25+a39);
  a39=(a16*a44);
  a41=(a34*a45);
  a39=(a39-a41);
  a39=(a39/a11);
  a39=(a10*a39);
  a16=(a44*a16);
  a45=(a34*a45);
  a16=(a16-a45);
  a16=(a16/a36);
  a39=(a39*a16);
  a25=(a25+a39);
  a40=(a40+a25);
  a25=(a44*a53);
  a39=(a31*a17);
  a25=(a25+a39);
  a25=(a25/a11);
  a25=(a10*a25);
  a39=(a44*a53);
  a16=(a17*a31);
  a39=(a39+a16);
  a39=(a39/a36);
  a25=(a25*a39);
  a39=(a34*a53);
  a16=(a43*a17);
  a39=(a39+a16);
  a39=(a39/a11);
  a39=(a10*a39);
  a53=(a34*a53);
  a17=(a17*a43);
  a53=(a53+a17);
  a53=(a53/a36);
  a39=(a39*a53);
  a25=(a25+a39);
  a39=(a43*a44);
  a53=(a34*a31);
  a39=(a39-a53);
  a39=(a39/a11);
  a10=(a10*a39);
  a44=(a44*a43);
  a34=(a34*a31);
  a44=(a44-a34);
  a44=(a44/a36);
  a10=(a10*a44);
  a25=(a25+a10);
  a40=(a40+a25);
  a40=sqrt(a40);
  a33=(a33*a40);
  a33=(1./a33);
  if (res[0]!=0) res[0][3]=a33;
  a33=cos(a2);
  a40=sin(a4);
  a25=(a33*a40);
  a10=sin(a7);
  a44=(a25*a10);
  a36=sin(a2);
  a34=cos(a7);
  a31=(a36*a34);
  a44=(a44-a31);
  a31=(a1*a44);
  a43=sin(a7);
  a36=(a36*a43);
  a7=cos(a7);
  a25=(a25*a7);
  a36=(a36+a25);
  a25=(a13*a36);
  a31=(a31+a25);
  a25=(a35*a31);
  a39=(a0*a31);
  a11=casadi_sq(a39);
  a53=cos(a2);
  a34=(a53*a34);
  a17=sin(a2);
  a40=(a17*a40);
  a16=(a40*a10);
  a34=(a34+a16);
  a16=(a1*a34);
  a40=(a40*a7);
  a53=(a53*a43);
  a40=(a40-a53);
  a53=(a13*a40);
  a16=(a16+a53);
  a53=(a0*a16);
  a43=cos(a4);
  a10=(a43*a10);
  a1=(a1*a10);
  a43=(a43*a7);
  a7=(a13*a43);
  a1=(a1+a7);
  a7=(a14*a1);
  a53=(a53+a7);
  a53=(a53+a15);
  a7=(a20*a16);
  a45=(a14*a1);
  a7=(a7+a45);
  a7=(a7+a15);
  a7=(a53*a7);
  a11=(a11-a7);
  a11=sqrt(a11);
  a39=(a39+a11);
  a39=(a39/a53);
  a39=(-a39);
  a39=atan(a39);
  a39=(a5*a39);
  a53=sin(a39);
  a53=(a0*a53);
  a11=(a53*a1);
  a25=(a25+a11);
  a11=cos(a2);
  a7=(a25*a11);
  a39=cos(a39);
  a39=(a0*a39);
  a1=(a39*a1);
  a45=(a35*a16);
  a1=(a1-a45);
  a2=sin(a2);
  a45=(a1*a2);
  a7=(a7-a45);
  a53=(a53*a16);
  a39=(a39*a31);
  a53=(a53+a39);
  a7=(a7/a53);
  a39=arg[0]? arg[0][4] : 0;
  a7=(a7*a39);
  a31=arg[0]? arg[0][3] : 0;
  a7=(a7-a31);
  a16=cos(a4);
  a45=(a11*a16);
  a1=(a1*a45);
  a16=(a2*a16);
  a25=(a25*a16);
  a1=(a1+a25);
  a25=sin(a4);
  a41=(a53*a25);
  a1=(a1+a41);
  a1=(a1/a53);
  a53=arg[0]? arg[0][5] : 0;
  a1=(a1*a53);
  a7=(a7+a1);
  if (res[0]!=0) res[0][4]=a7;
  a7=cos(a4);
  a33=(a33*a7);
  a1=(a6*a33);
  a41=(a21*a44);
  a1=(a1+a41);
  a41=(a13*a36);
  a1=(a1+a41);
  a41=(a35*a1);
  a46=(a12*a1);
  a17=(a17*a7);
  a7=(a6*a17);
  a48=(a21*a34);
  a7=(a7+a48);
  a48=(a13*a40);
  a7=(a7+a48);
  a48=(a23*a7);
  a46=(a46-a48);
  a46=(a0*a46);
  a48=casadi_sq(a46);
  a51=(a23*a1);
  a52=(a12*a7);
  a51=(a51+a52);
  a51=(a0*a51);
  a21=(a21*a10);
  a4=sin(a4);
  a6=(a6*a4);
  a21=(a21-a6);
  a6=(a13*a43);
  a21=(a21+a6);
  a6=(a14*a21);
  a51=(a51+a6);
  a51=(a51+a15);
  a6=(a23*a1);
  a52=(a12*a7);
  a6=(a6+a52);
  a6=(a20*a6);
  a52=(a14*a21);
  a6=(a6+a52);
  a6=(a6+a15);
  a6=(a51*a6);
  a48=(a48-a6);
  a48=sqrt(a48);
  a46=(a46+a48);
  a46=(a46/a51);
  a46=(-a46);
  a46=atan(a46);
  a46=(a5*a46);
  a51=cos(a46);
  a51=(a0*a51);
  a23=(a23*a51);
  a51=sin(a46);
  a54=(a54*a51);
  a23=(a23-a54);
  a54=(a23*a21);
  a41=(a41-a54);
  a54=(a41*a11);
  a51=cos(a46);
  a51=(a0*a51);
  a12=(a12*a51);
  a46=sin(a46);
  a47=(a47*a46);
  a12=(a12+a47);
  a21=(a12*a21);
  a47=(a35*a7);
  a21=(a21-a47);
  a47=(a21*a2);
  a54=(a54-a47);
  a23=(a23*a7);
  a12=(a12*a1);
  a23=(a23-a12);
  a54=(a54/a23);
  a54=(a54*a39);
  a54=(a31+a54);
  a21=(a21*a45);
  a41=(a41*a16);
  a21=(a21+a41);
  a41=(a23*a25);
  a21=(a21-a41);
  a21=(a21/a23);
  a21=(a21*a53);
  a54=(a54+a21);
  a54=(-a54);
  if (res[0]!=0) res[0][5]=a54;
  a33=(a24*a33);
  a44=(a19*a44);
  a33=(a33+a44);
  a36=(a13*a36);
  a33=(a33+a36);
  a36=(a35*a33);
  a44=(a22*a33);
  a17=(a24*a17);
  a34=(a19*a34);
  a17=(a17+a34);
  a40=(a13*a40);
  a17=(a17+a40);
  a40=(a9*a17);
  a44=(a44-a40);
  a44=(a0*a44);
  a40=casadi_sq(a44);
  a34=(a9*a33);
  a54=(a22*a17);
  a34=(a34+a54);
  a34=(a0*a34);
  a19=(a19*a10);
  a24=(a24*a4);
  a19=(a19-a24);
  a13=(a13*a43);
  a19=(a19+a13);
  a13=(a14*a19);
  a34=(a34+a13);
  a34=(a34+a15);
  a13=(a9*a33);
  a43=(a22*a17);
  a13=(a13+a43);
  a20=(a20*a13);
  a14=(a14*a19);
  a20=(a20+a14);
  a20=(a20+a15);
  a20=(a34*a20);
  a40=(a40-a20);
  a40=sqrt(a40);
  a44=(a44+a40);
  a44=(a44/a34);
  a44=(-a44);
  a44=atan(a44);
  a5=(a5*a44);
  a44=cos(a5);
  a44=(a0*a44);
  a9=(a9*a44);
  a44=sin(a5);
  a38=(a38*a44);
  a9=(a9-a38);
  a38=(a9*a19);
  a36=(a36-a38);
  a11=(a36*a11);
  a38=cos(a5);
  a0=(a0*a38);
  a22=(a22*a0);
  a5=sin(a5);
  a8=(a8*a5);
  a22=(a22+a8);
  a19=(a22*a19);
  a35=(a35*a17);
  a19=(a19-a35);
  a2=(a19*a2);
  a11=(a11-a2);
  a9=(a9*a17);
  a22=(a22*a33);
  a9=(a9-a22);
  a11=(a11/a9);
  a11=(a11*a39);
  a31=(a31+a11);
  a19=(a19*a45);
  a36=(a36*a16);
  a19=(a19+a36);
  a25=(a9*a25);
  a19=(a19-a25);
  a19=(a19/a9);
  a19=(a19*a53);
  a31=(a31+a19);
  a31=(-a31);
  if (res[0]!=0) res[0][6]=a31;
  return 0;
}

CASADI_SYMBOL_EXPORT int path_con_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int path_con_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int path_con_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void path_con_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int path_con_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void path_con_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void path_con_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void path_con_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int path_con_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int path_con_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real path_con_fun_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* path_con_fun_name_in(casadi_int i) {
  switch (i) {
    case 0: return "states";
    case 1: return "controls";
    case 2: return "params";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* path_con_fun_name_out(casadi_int i) {
  switch (i) {
    case 0: return "general_con";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* path_con_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* path_con_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int path_con_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

#ifdef MATLAB_MEX_FILE
void mex_path_con_fun(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  casadi_int i;
  int mem;
  casadi_real w[93];
  casadi_int *iw = 0;
  const casadi_real* arg[3] = {0};
  casadi_real* res[1] = {0};
  if (argc>3) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"path_con_fun\" failed. Too many input arguments (%d, max 3)", argc);
  if (resc>1) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"path_con_fun\" failed. Too many output arguments (%d, max 1)", resc);
  if (--argc>=0) arg[0] = casadi_from_mex(argv[0], w, casadi_s0, w+37);
  if (--argc>=0) arg[1] = casadi_from_mex(argv[1], w+12, casadi_s1, w+37);
  if (--argc>=0) arg[2] = casadi_from_mex(argv[2], w+15, casadi_s2, w+37);
  --resc;
  res[0] = w+30;
  path_con_fun_incref();
  mem = path_con_fun_checkout();
  i = path_con_fun(arg, res, iw, w+37, mem);
  if (i) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"path_con_fun\" failed.");
  path_con_fun_release(mem);
  path_con_fun_decref();
  if (res[0]) resv[0] = casadi_to_mex(casadi_s3, res[0]);
}
#endif


#ifdef MATLAB_MEX_FILE
void mexFunction(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  char buf[13];
  int buf_ok = argc > 0 && !mxGetString(*argv, buf, sizeof(buf));
  if (!buf_ok) {
    mex_path_con_fun(resc, resv, argc, argv);
    return;
  } else if (strcmp(buf, "path_con_fun")==0) {
    mex_path_con_fun(resc, resv, argc-1, argv+1);
    return;
  }
  mexErrMsgTxt("First input should be a command string. Possible values: 'path_con_fun'");
}
#endif
#ifdef __cplusplus
} /* extern "C" */
#endif