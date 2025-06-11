#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4633573009917725202) {
   out_4633573009917725202[0] = delta_x[0] + nom_x[0];
   out_4633573009917725202[1] = delta_x[1] + nom_x[1];
   out_4633573009917725202[2] = delta_x[2] + nom_x[2];
   out_4633573009917725202[3] = delta_x[3] + nom_x[3];
   out_4633573009917725202[4] = delta_x[4] + nom_x[4];
   out_4633573009917725202[5] = delta_x[5] + nom_x[5];
   out_4633573009917725202[6] = delta_x[6] + nom_x[6];
   out_4633573009917725202[7] = delta_x[7] + nom_x[7];
   out_4633573009917725202[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3575493581669951122) {
   out_3575493581669951122[0] = -nom_x[0] + true_x[0];
   out_3575493581669951122[1] = -nom_x[1] + true_x[1];
   out_3575493581669951122[2] = -nom_x[2] + true_x[2];
   out_3575493581669951122[3] = -nom_x[3] + true_x[3];
   out_3575493581669951122[4] = -nom_x[4] + true_x[4];
   out_3575493581669951122[5] = -nom_x[5] + true_x[5];
   out_3575493581669951122[6] = -nom_x[6] + true_x[6];
   out_3575493581669951122[7] = -nom_x[7] + true_x[7];
   out_3575493581669951122[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6051152977375382637) {
   out_6051152977375382637[0] = 1.0;
   out_6051152977375382637[1] = 0.0;
   out_6051152977375382637[2] = 0.0;
   out_6051152977375382637[3] = 0.0;
   out_6051152977375382637[4] = 0.0;
   out_6051152977375382637[5] = 0.0;
   out_6051152977375382637[6] = 0.0;
   out_6051152977375382637[7] = 0.0;
   out_6051152977375382637[8] = 0.0;
   out_6051152977375382637[9] = 0.0;
   out_6051152977375382637[10] = 1.0;
   out_6051152977375382637[11] = 0.0;
   out_6051152977375382637[12] = 0.0;
   out_6051152977375382637[13] = 0.0;
   out_6051152977375382637[14] = 0.0;
   out_6051152977375382637[15] = 0.0;
   out_6051152977375382637[16] = 0.0;
   out_6051152977375382637[17] = 0.0;
   out_6051152977375382637[18] = 0.0;
   out_6051152977375382637[19] = 0.0;
   out_6051152977375382637[20] = 1.0;
   out_6051152977375382637[21] = 0.0;
   out_6051152977375382637[22] = 0.0;
   out_6051152977375382637[23] = 0.0;
   out_6051152977375382637[24] = 0.0;
   out_6051152977375382637[25] = 0.0;
   out_6051152977375382637[26] = 0.0;
   out_6051152977375382637[27] = 0.0;
   out_6051152977375382637[28] = 0.0;
   out_6051152977375382637[29] = 0.0;
   out_6051152977375382637[30] = 1.0;
   out_6051152977375382637[31] = 0.0;
   out_6051152977375382637[32] = 0.0;
   out_6051152977375382637[33] = 0.0;
   out_6051152977375382637[34] = 0.0;
   out_6051152977375382637[35] = 0.0;
   out_6051152977375382637[36] = 0.0;
   out_6051152977375382637[37] = 0.0;
   out_6051152977375382637[38] = 0.0;
   out_6051152977375382637[39] = 0.0;
   out_6051152977375382637[40] = 1.0;
   out_6051152977375382637[41] = 0.0;
   out_6051152977375382637[42] = 0.0;
   out_6051152977375382637[43] = 0.0;
   out_6051152977375382637[44] = 0.0;
   out_6051152977375382637[45] = 0.0;
   out_6051152977375382637[46] = 0.0;
   out_6051152977375382637[47] = 0.0;
   out_6051152977375382637[48] = 0.0;
   out_6051152977375382637[49] = 0.0;
   out_6051152977375382637[50] = 1.0;
   out_6051152977375382637[51] = 0.0;
   out_6051152977375382637[52] = 0.0;
   out_6051152977375382637[53] = 0.0;
   out_6051152977375382637[54] = 0.0;
   out_6051152977375382637[55] = 0.0;
   out_6051152977375382637[56] = 0.0;
   out_6051152977375382637[57] = 0.0;
   out_6051152977375382637[58] = 0.0;
   out_6051152977375382637[59] = 0.0;
   out_6051152977375382637[60] = 1.0;
   out_6051152977375382637[61] = 0.0;
   out_6051152977375382637[62] = 0.0;
   out_6051152977375382637[63] = 0.0;
   out_6051152977375382637[64] = 0.0;
   out_6051152977375382637[65] = 0.0;
   out_6051152977375382637[66] = 0.0;
   out_6051152977375382637[67] = 0.0;
   out_6051152977375382637[68] = 0.0;
   out_6051152977375382637[69] = 0.0;
   out_6051152977375382637[70] = 1.0;
   out_6051152977375382637[71] = 0.0;
   out_6051152977375382637[72] = 0.0;
   out_6051152977375382637[73] = 0.0;
   out_6051152977375382637[74] = 0.0;
   out_6051152977375382637[75] = 0.0;
   out_6051152977375382637[76] = 0.0;
   out_6051152977375382637[77] = 0.0;
   out_6051152977375382637[78] = 0.0;
   out_6051152977375382637[79] = 0.0;
   out_6051152977375382637[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1721947030779493947) {
   out_1721947030779493947[0] = state[0];
   out_1721947030779493947[1] = state[1];
   out_1721947030779493947[2] = state[2];
   out_1721947030779493947[3] = state[3];
   out_1721947030779493947[4] = state[4];
   out_1721947030779493947[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1721947030779493947[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1721947030779493947[7] = state[7];
   out_1721947030779493947[8] = state[8];
}
void F_fun(double *state, double dt, double *out_169967702265322698) {
   out_169967702265322698[0] = 1;
   out_169967702265322698[1] = 0;
   out_169967702265322698[2] = 0;
   out_169967702265322698[3] = 0;
   out_169967702265322698[4] = 0;
   out_169967702265322698[5] = 0;
   out_169967702265322698[6] = 0;
   out_169967702265322698[7] = 0;
   out_169967702265322698[8] = 0;
   out_169967702265322698[9] = 0;
   out_169967702265322698[10] = 1;
   out_169967702265322698[11] = 0;
   out_169967702265322698[12] = 0;
   out_169967702265322698[13] = 0;
   out_169967702265322698[14] = 0;
   out_169967702265322698[15] = 0;
   out_169967702265322698[16] = 0;
   out_169967702265322698[17] = 0;
   out_169967702265322698[18] = 0;
   out_169967702265322698[19] = 0;
   out_169967702265322698[20] = 1;
   out_169967702265322698[21] = 0;
   out_169967702265322698[22] = 0;
   out_169967702265322698[23] = 0;
   out_169967702265322698[24] = 0;
   out_169967702265322698[25] = 0;
   out_169967702265322698[26] = 0;
   out_169967702265322698[27] = 0;
   out_169967702265322698[28] = 0;
   out_169967702265322698[29] = 0;
   out_169967702265322698[30] = 1;
   out_169967702265322698[31] = 0;
   out_169967702265322698[32] = 0;
   out_169967702265322698[33] = 0;
   out_169967702265322698[34] = 0;
   out_169967702265322698[35] = 0;
   out_169967702265322698[36] = 0;
   out_169967702265322698[37] = 0;
   out_169967702265322698[38] = 0;
   out_169967702265322698[39] = 0;
   out_169967702265322698[40] = 1;
   out_169967702265322698[41] = 0;
   out_169967702265322698[42] = 0;
   out_169967702265322698[43] = 0;
   out_169967702265322698[44] = 0;
   out_169967702265322698[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_169967702265322698[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_169967702265322698[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_169967702265322698[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_169967702265322698[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_169967702265322698[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_169967702265322698[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_169967702265322698[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_169967702265322698[53] = -9.8000000000000007*dt;
   out_169967702265322698[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_169967702265322698[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_169967702265322698[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_169967702265322698[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_169967702265322698[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_169967702265322698[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_169967702265322698[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_169967702265322698[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_169967702265322698[62] = 0;
   out_169967702265322698[63] = 0;
   out_169967702265322698[64] = 0;
   out_169967702265322698[65] = 0;
   out_169967702265322698[66] = 0;
   out_169967702265322698[67] = 0;
   out_169967702265322698[68] = 0;
   out_169967702265322698[69] = 0;
   out_169967702265322698[70] = 1;
   out_169967702265322698[71] = 0;
   out_169967702265322698[72] = 0;
   out_169967702265322698[73] = 0;
   out_169967702265322698[74] = 0;
   out_169967702265322698[75] = 0;
   out_169967702265322698[76] = 0;
   out_169967702265322698[77] = 0;
   out_169967702265322698[78] = 0;
   out_169967702265322698[79] = 0;
   out_169967702265322698[80] = 1;
}
void h_25(double *state, double *unused, double *out_6413001333912154685) {
   out_6413001333912154685[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5284705215293175388) {
   out_5284705215293175388[0] = 0;
   out_5284705215293175388[1] = 0;
   out_5284705215293175388[2] = 0;
   out_5284705215293175388[3] = 0;
   out_5284705215293175388[4] = 0;
   out_5284705215293175388[5] = 0;
   out_5284705215293175388[6] = 1;
   out_5284705215293175388[7] = 0;
   out_5284705215293175388[8] = 0;
}
void h_24(double *state, double *unused, double *out_6028044839323201886) {
   out_6028044839323201886[0] = state[4];
   out_6028044839323201886[1] = state[5];
}
void H_24(double *state, double *unused, double *out_916327464557986038) {
   out_916327464557986038[0] = 0;
   out_916327464557986038[1] = 0;
   out_916327464557986038[2] = 0;
   out_916327464557986038[3] = 0;
   out_916327464557986038[4] = 1;
   out_916327464557986038[5] = 0;
   out_916327464557986038[6] = 0;
   out_916327464557986038[7] = 0;
   out_916327464557986038[8] = 0;
   out_916327464557986038[9] = 0;
   out_916327464557986038[10] = 0;
   out_916327464557986038[11] = 0;
   out_916327464557986038[12] = 0;
   out_916327464557986038[13] = 0;
   out_916327464557986038[14] = 1;
   out_916327464557986038[15] = 0;
   out_916327464557986038[16] = 0;
   out_916327464557986038[17] = 0;
}
void h_30(double *state, double *unused, double *out_5878295543956736802) {
   out_5878295543956736802[0] = state[4];
}
void H_30(double *state, double *unused, double *out_757008885165567190) {
   out_757008885165567190[0] = 0;
   out_757008885165567190[1] = 0;
   out_757008885165567190[2] = 0;
   out_757008885165567190[3] = 0;
   out_757008885165567190[4] = 1;
   out_757008885165567190[5] = 0;
   out_757008885165567190[6] = 0;
   out_757008885165567190[7] = 0;
   out_757008885165567190[8] = 0;
}
void h_26(double *state, double *unused, double *out_536428308070706239) {
   out_536428308070706239[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1543201896419119164) {
   out_1543201896419119164[0] = 0;
   out_1543201896419119164[1] = 0;
   out_1543201896419119164[2] = 0;
   out_1543201896419119164[3] = 0;
   out_1543201896419119164[4] = 0;
   out_1543201896419119164[5] = 0;
   out_1543201896419119164[6] = 0;
   out_1543201896419119164[7] = 1;
   out_1543201896419119164[8] = 0;
}
void h_27(double *state, double *unused, double *out_1711865638415083517) {
   out_1711865638415083517[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2980602956349510407) {
   out_2980602956349510407[0] = 0;
   out_2980602956349510407[1] = 0;
   out_2980602956349510407[2] = 0;
   out_2980602956349510407[3] = 1;
   out_2980602956349510407[4] = 0;
   out_2980602956349510407[5] = 0;
   out_2980602956349510407[6] = 0;
   out_2980602956349510407[7] = 0;
   out_2980602956349510407[8] = 0;
}
void h_29(double *state, double *unused, double *out_6169407012338965668) {
   out_6169407012338965668[0] = state[1];
}
void H_29(double *state, double *unused, double *out_1267240229479959374) {
   out_1267240229479959374[0] = 0;
   out_1267240229479959374[1] = 1;
   out_1267240229479959374[2] = 0;
   out_1267240229479959374[3] = 0;
   out_1267240229479959374[4] = 0;
   out_1267240229479959374[5] = 0;
   out_1267240229479959374[6] = 0;
   out_1267240229479959374[7] = 0;
   out_1267240229479959374[8] = 0;
}
void h_28(double *state, double *unused, double *out_4203765115318778819) {
   out_4203765115318778819[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3230870501045285625) {
   out_3230870501045285625[0] = 1;
   out_3230870501045285625[1] = 0;
   out_3230870501045285625[2] = 0;
   out_3230870501045285625[3] = 0;
   out_3230870501045285625[4] = 0;
   out_3230870501045285625[5] = 0;
   out_3230870501045285625[6] = 0;
   out_3230870501045285625[7] = 0;
   out_3230870501045285625[8] = 0;
}
void h_31(double *state, double *unused, double *out_5262907513447045995) {
   out_5262907513447045995[0] = state[8];
}
void H_31(double *state, double *unused, double *out_916993794185767688) {
   out_916993794185767688[0] = 0;
   out_916993794185767688[1] = 0;
   out_916993794185767688[2] = 0;
   out_916993794185767688[3] = 0;
   out_916993794185767688[4] = 0;
   out_916993794185767688[5] = 0;
   out_916993794185767688[6] = 0;
   out_916993794185767688[7] = 0;
   out_916993794185767688[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_4633573009917725202) {
  err_fun(nom_x, delta_x, out_4633573009917725202);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3575493581669951122) {
  inv_err_fun(nom_x, true_x, out_3575493581669951122);
}
void car_H_mod_fun(double *state, double *out_6051152977375382637) {
  H_mod_fun(state, out_6051152977375382637);
}
void car_f_fun(double *state, double dt, double *out_1721947030779493947) {
  f_fun(state,  dt, out_1721947030779493947);
}
void car_F_fun(double *state, double dt, double *out_169967702265322698) {
  F_fun(state,  dt, out_169967702265322698);
}
void car_h_25(double *state, double *unused, double *out_6413001333912154685) {
  h_25(state, unused, out_6413001333912154685);
}
void car_H_25(double *state, double *unused, double *out_5284705215293175388) {
  H_25(state, unused, out_5284705215293175388);
}
void car_h_24(double *state, double *unused, double *out_6028044839323201886) {
  h_24(state, unused, out_6028044839323201886);
}
void car_H_24(double *state, double *unused, double *out_916327464557986038) {
  H_24(state, unused, out_916327464557986038);
}
void car_h_30(double *state, double *unused, double *out_5878295543956736802) {
  h_30(state, unused, out_5878295543956736802);
}
void car_H_30(double *state, double *unused, double *out_757008885165567190) {
  H_30(state, unused, out_757008885165567190);
}
void car_h_26(double *state, double *unused, double *out_536428308070706239) {
  h_26(state, unused, out_536428308070706239);
}
void car_H_26(double *state, double *unused, double *out_1543201896419119164) {
  H_26(state, unused, out_1543201896419119164);
}
void car_h_27(double *state, double *unused, double *out_1711865638415083517) {
  h_27(state, unused, out_1711865638415083517);
}
void car_H_27(double *state, double *unused, double *out_2980602956349510407) {
  H_27(state, unused, out_2980602956349510407);
}
void car_h_29(double *state, double *unused, double *out_6169407012338965668) {
  h_29(state, unused, out_6169407012338965668);
}
void car_H_29(double *state, double *unused, double *out_1267240229479959374) {
  H_29(state, unused, out_1267240229479959374);
}
void car_h_28(double *state, double *unused, double *out_4203765115318778819) {
  h_28(state, unused, out_4203765115318778819);
}
void car_H_28(double *state, double *unused, double *out_3230870501045285625) {
  H_28(state, unused, out_3230870501045285625);
}
void car_h_31(double *state, double *unused, double *out_5262907513447045995) {
  h_31(state, unused, out_5262907513447045995);
}
void car_H_31(double *state, double *unused, double *out_916993794185767688) {
  H_31(state, unused, out_916993794185767688);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
