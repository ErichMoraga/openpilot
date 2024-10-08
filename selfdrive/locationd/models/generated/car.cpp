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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5709880423575590152) {
   out_5709880423575590152[0] = delta_x[0] + nom_x[0];
   out_5709880423575590152[1] = delta_x[1] + nom_x[1];
   out_5709880423575590152[2] = delta_x[2] + nom_x[2];
   out_5709880423575590152[3] = delta_x[3] + nom_x[3];
   out_5709880423575590152[4] = delta_x[4] + nom_x[4];
   out_5709880423575590152[5] = delta_x[5] + nom_x[5];
   out_5709880423575590152[6] = delta_x[6] + nom_x[6];
   out_5709880423575590152[7] = delta_x[7] + nom_x[7];
   out_5709880423575590152[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_8634507341529600210) {
   out_8634507341529600210[0] = -nom_x[0] + true_x[0];
   out_8634507341529600210[1] = -nom_x[1] + true_x[1];
   out_8634507341529600210[2] = -nom_x[2] + true_x[2];
   out_8634507341529600210[3] = -nom_x[3] + true_x[3];
   out_8634507341529600210[4] = -nom_x[4] + true_x[4];
   out_8634507341529600210[5] = -nom_x[5] + true_x[5];
   out_8634507341529600210[6] = -nom_x[6] + true_x[6];
   out_8634507341529600210[7] = -nom_x[7] + true_x[7];
   out_8634507341529600210[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_3871090052338825407) {
   out_3871090052338825407[0] = 1.0;
   out_3871090052338825407[1] = 0;
   out_3871090052338825407[2] = 0;
   out_3871090052338825407[3] = 0;
   out_3871090052338825407[4] = 0;
   out_3871090052338825407[5] = 0;
   out_3871090052338825407[6] = 0;
   out_3871090052338825407[7] = 0;
   out_3871090052338825407[8] = 0;
   out_3871090052338825407[9] = 0;
   out_3871090052338825407[10] = 1.0;
   out_3871090052338825407[11] = 0;
   out_3871090052338825407[12] = 0;
   out_3871090052338825407[13] = 0;
   out_3871090052338825407[14] = 0;
   out_3871090052338825407[15] = 0;
   out_3871090052338825407[16] = 0;
   out_3871090052338825407[17] = 0;
   out_3871090052338825407[18] = 0;
   out_3871090052338825407[19] = 0;
   out_3871090052338825407[20] = 1.0;
   out_3871090052338825407[21] = 0;
   out_3871090052338825407[22] = 0;
   out_3871090052338825407[23] = 0;
   out_3871090052338825407[24] = 0;
   out_3871090052338825407[25] = 0;
   out_3871090052338825407[26] = 0;
   out_3871090052338825407[27] = 0;
   out_3871090052338825407[28] = 0;
   out_3871090052338825407[29] = 0;
   out_3871090052338825407[30] = 1.0;
   out_3871090052338825407[31] = 0;
   out_3871090052338825407[32] = 0;
   out_3871090052338825407[33] = 0;
   out_3871090052338825407[34] = 0;
   out_3871090052338825407[35] = 0;
   out_3871090052338825407[36] = 0;
   out_3871090052338825407[37] = 0;
   out_3871090052338825407[38] = 0;
   out_3871090052338825407[39] = 0;
   out_3871090052338825407[40] = 1.0;
   out_3871090052338825407[41] = 0;
   out_3871090052338825407[42] = 0;
   out_3871090052338825407[43] = 0;
   out_3871090052338825407[44] = 0;
   out_3871090052338825407[45] = 0;
   out_3871090052338825407[46] = 0;
   out_3871090052338825407[47] = 0;
   out_3871090052338825407[48] = 0;
   out_3871090052338825407[49] = 0;
   out_3871090052338825407[50] = 1.0;
   out_3871090052338825407[51] = 0;
   out_3871090052338825407[52] = 0;
   out_3871090052338825407[53] = 0;
   out_3871090052338825407[54] = 0;
   out_3871090052338825407[55] = 0;
   out_3871090052338825407[56] = 0;
   out_3871090052338825407[57] = 0;
   out_3871090052338825407[58] = 0;
   out_3871090052338825407[59] = 0;
   out_3871090052338825407[60] = 1.0;
   out_3871090052338825407[61] = 0;
   out_3871090052338825407[62] = 0;
   out_3871090052338825407[63] = 0;
   out_3871090052338825407[64] = 0;
   out_3871090052338825407[65] = 0;
   out_3871090052338825407[66] = 0;
   out_3871090052338825407[67] = 0;
   out_3871090052338825407[68] = 0;
   out_3871090052338825407[69] = 0;
   out_3871090052338825407[70] = 1.0;
   out_3871090052338825407[71] = 0;
   out_3871090052338825407[72] = 0;
   out_3871090052338825407[73] = 0;
   out_3871090052338825407[74] = 0;
   out_3871090052338825407[75] = 0;
   out_3871090052338825407[76] = 0;
   out_3871090052338825407[77] = 0;
   out_3871090052338825407[78] = 0;
   out_3871090052338825407[79] = 0;
   out_3871090052338825407[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6137814433410422852) {
   out_6137814433410422852[0] = state[0];
   out_6137814433410422852[1] = state[1];
   out_6137814433410422852[2] = state[2];
   out_6137814433410422852[3] = state[3];
   out_6137814433410422852[4] = state[4];
   out_6137814433410422852[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6137814433410422852[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6137814433410422852[7] = state[7];
   out_6137814433410422852[8] = state[8];
}
void F_fun(double *state, double dt, double *out_7621765943040258911) {
   out_7621765943040258911[0] = 1;
   out_7621765943040258911[1] = 0;
   out_7621765943040258911[2] = 0;
   out_7621765943040258911[3] = 0;
   out_7621765943040258911[4] = 0;
   out_7621765943040258911[5] = 0;
   out_7621765943040258911[6] = 0;
   out_7621765943040258911[7] = 0;
   out_7621765943040258911[8] = 0;
   out_7621765943040258911[9] = 0;
   out_7621765943040258911[10] = 1;
   out_7621765943040258911[11] = 0;
   out_7621765943040258911[12] = 0;
   out_7621765943040258911[13] = 0;
   out_7621765943040258911[14] = 0;
   out_7621765943040258911[15] = 0;
   out_7621765943040258911[16] = 0;
   out_7621765943040258911[17] = 0;
   out_7621765943040258911[18] = 0;
   out_7621765943040258911[19] = 0;
   out_7621765943040258911[20] = 1;
   out_7621765943040258911[21] = 0;
   out_7621765943040258911[22] = 0;
   out_7621765943040258911[23] = 0;
   out_7621765943040258911[24] = 0;
   out_7621765943040258911[25] = 0;
   out_7621765943040258911[26] = 0;
   out_7621765943040258911[27] = 0;
   out_7621765943040258911[28] = 0;
   out_7621765943040258911[29] = 0;
   out_7621765943040258911[30] = 1;
   out_7621765943040258911[31] = 0;
   out_7621765943040258911[32] = 0;
   out_7621765943040258911[33] = 0;
   out_7621765943040258911[34] = 0;
   out_7621765943040258911[35] = 0;
   out_7621765943040258911[36] = 0;
   out_7621765943040258911[37] = 0;
   out_7621765943040258911[38] = 0;
   out_7621765943040258911[39] = 0;
   out_7621765943040258911[40] = 1;
   out_7621765943040258911[41] = 0;
   out_7621765943040258911[42] = 0;
   out_7621765943040258911[43] = 0;
   out_7621765943040258911[44] = 0;
   out_7621765943040258911[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_7621765943040258911[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_7621765943040258911[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7621765943040258911[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_7621765943040258911[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_7621765943040258911[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_7621765943040258911[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_7621765943040258911[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_7621765943040258911[53] = -9.8000000000000007*dt;
   out_7621765943040258911[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_7621765943040258911[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_7621765943040258911[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7621765943040258911[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7621765943040258911[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_7621765943040258911[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_7621765943040258911[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_7621765943040258911[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_7621765943040258911[62] = 0;
   out_7621765943040258911[63] = 0;
   out_7621765943040258911[64] = 0;
   out_7621765943040258911[65] = 0;
   out_7621765943040258911[66] = 0;
   out_7621765943040258911[67] = 0;
   out_7621765943040258911[68] = 0;
   out_7621765943040258911[69] = 0;
   out_7621765943040258911[70] = 1;
   out_7621765943040258911[71] = 0;
   out_7621765943040258911[72] = 0;
   out_7621765943040258911[73] = 0;
   out_7621765943040258911[74] = 0;
   out_7621765943040258911[75] = 0;
   out_7621765943040258911[76] = 0;
   out_7621765943040258911[77] = 0;
   out_7621765943040258911[78] = 0;
   out_7621765943040258911[79] = 0;
   out_7621765943040258911[80] = 1;
}
void h_25(double *state, double *unused, double *out_4722807775135100883) {
   out_4722807775135100883[0] = state[6];
}
void H_25(double *state, double *unused, double *out_3374924203140226621) {
   out_3374924203140226621[0] = 0;
   out_3374924203140226621[1] = 0;
   out_3374924203140226621[2] = 0;
   out_3374924203140226621[3] = 0;
   out_3374924203140226621[4] = 0;
   out_3374924203140226621[5] = 0;
   out_3374924203140226621[6] = 1;
   out_3374924203140226621[7] = 0;
   out_3374924203140226621[8] = 0;
}
void h_24(double *state, double *unused, double *out_2835190905026714642) {
   out_2835190905026714642[0] = state[4];
   out_2835190905026714642[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1202274604134727055) {
   out_1202274604134727055[0] = 0;
   out_1202274604134727055[1] = 0;
   out_1202274604134727055[2] = 0;
   out_1202274604134727055[3] = 0;
   out_1202274604134727055[4] = 1;
   out_1202274604134727055[5] = 0;
   out_1202274604134727055[6] = 0;
   out_1202274604134727055[7] = 0;
   out_1202274604134727055[8] = 0;
   out_1202274604134727055[9] = 0;
   out_1202274604134727055[10] = 0;
   out_1202274604134727055[11] = 0;
   out_1202274604134727055[12] = 0;
   out_1202274604134727055[13] = 0;
   out_1202274604134727055[14] = 1;
   out_1202274604134727055[15] = 0;
   out_1202274604134727055[16] = 0;
   out_1202274604134727055[17] = 0;
}
void h_30(double *state, double *unused, double *out_1085358246881078533) {
   out_1085358246881078533[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5893257161647475248) {
   out_5893257161647475248[0] = 0;
   out_5893257161647475248[1] = 0;
   out_5893257161647475248[2] = 0;
   out_5893257161647475248[3] = 0;
   out_5893257161647475248[4] = 1;
   out_5893257161647475248[5] = 0;
   out_5893257161647475248[6] = 0;
   out_5893257161647475248[7] = 0;
   out_5893257161647475248[8] = 0;
}
void h_26(double *state, double *unused, double *out_6151775765599421256) {
   out_6151775765599421256[0] = state[7];
}
void H_26(double *state, double *unused, double *out_366579115733829603) {
   out_366579115733829603[0] = 0;
   out_366579115733829603[1] = 0;
   out_366579115733829603[2] = 0;
   out_366579115733829603[3] = 0;
   out_366579115733829603[4] = 0;
   out_366579115733829603[5] = 0;
   out_366579115733829603[6] = 0;
   out_366579115733829603[7] = 1;
   out_366579115733829603[8] = 0;
}
void h_27(double *state, double *unused, double *out_278865492525744762) {
   out_278865492525744762[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3718493849847050337) {
   out_3718493849847050337[0] = 0;
   out_3718493849847050337[1] = 0;
   out_3718493849847050337[2] = 0;
   out_3718493849847050337[3] = 1;
   out_3718493849847050337[4] = 0;
   out_3718493849847050337[5] = 0;
   out_3718493849847050337[6] = 0;
   out_3718493849847050337[7] = 0;
   out_3718493849847050337[8] = 0;
}
void h_29(double *state, double *unused, double *out_3916315020779767112) {
   out_3916315020779767112[0] = state[1];
}
void H_29(double *state, double *unused, double *out_6403488505961867432) {
   out_6403488505961867432[0] = 0;
   out_6403488505961867432[1] = 1;
   out_6403488505961867432[2] = 0;
   out_6403488505961867432[3] = 0;
   out_6403488505961867432[4] = 0;
   out_6403488505961867432[5] = 0;
   out_6403488505961867432[6] = 0;
   out_6403488505961867432[7] = 0;
   out_6403488505961867432[8] = 0;
}
void h_28(double *state, double *unused, double *out_1260949585861042874) {
   out_1260949585861042874[0] = state[0];
}
void H_28(double *state, double *unused, double *out_1321089488892336858) {
   out_1321089488892336858[0] = 1;
   out_1321089488892336858[1] = 0;
   out_1321089488892336858[2] = 0;
   out_1321089488892336858[3] = 0;
   out_1321089488892336858[4] = 0;
   out_1321089488892336858[5] = 0;
   out_1321089488892336858[6] = 0;
   out_1321089488892336858[7] = 0;
   out_1321089488892336858[8] = 0;
}
void h_31(double *state, double *unused, double *out_4998001837419606772) {
   out_4998001837419606772[0] = state[8];
}
void H_31(double *state, double *unused, double *out_992787217967181079) {
   out_992787217967181079[0] = 0;
   out_992787217967181079[1] = 0;
   out_992787217967181079[2] = 0;
   out_992787217967181079[3] = 0;
   out_992787217967181079[4] = 0;
   out_992787217967181079[5] = 0;
   out_992787217967181079[6] = 0;
   out_992787217967181079[7] = 0;
   out_992787217967181079[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_5709880423575590152) {
  err_fun(nom_x, delta_x, out_5709880423575590152);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8634507341529600210) {
  inv_err_fun(nom_x, true_x, out_8634507341529600210);
}
void car_H_mod_fun(double *state, double *out_3871090052338825407) {
  H_mod_fun(state, out_3871090052338825407);
}
void car_f_fun(double *state, double dt, double *out_6137814433410422852) {
  f_fun(state,  dt, out_6137814433410422852);
}
void car_F_fun(double *state, double dt, double *out_7621765943040258911) {
  F_fun(state,  dt, out_7621765943040258911);
}
void car_h_25(double *state, double *unused, double *out_4722807775135100883) {
  h_25(state, unused, out_4722807775135100883);
}
void car_H_25(double *state, double *unused, double *out_3374924203140226621) {
  H_25(state, unused, out_3374924203140226621);
}
void car_h_24(double *state, double *unused, double *out_2835190905026714642) {
  h_24(state, unused, out_2835190905026714642);
}
void car_H_24(double *state, double *unused, double *out_1202274604134727055) {
  H_24(state, unused, out_1202274604134727055);
}
void car_h_30(double *state, double *unused, double *out_1085358246881078533) {
  h_30(state, unused, out_1085358246881078533);
}
void car_H_30(double *state, double *unused, double *out_5893257161647475248) {
  H_30(state, unused, out_5893257161647475248);
}
void car_h_26(double *state, double *unused, double *out_6151775765599421256) {
  h_26(state, unused, out_6151775765599421256);
}
void car_H_26(double *state, double *unused, double *out_366579115733829603) {
  H_26(state, unused, out_366579115733829603);
}
void car_h_27(double *state, double *unused, double *out_278865492525744762) {
  h_27(state, unused, out_278865492525744762);
}
void car_H_27(double *state, double *unused, double *out_3718493849847050337) {
  H_27(state, unused, out_3718493849847050337);
}
void car_h_29(double *state, double *unused, double *out_3916315020779767112) {
  h_29(state, unused, out_3916315020779767112);
}
void car_H_29(double *state, double *unused, double *out_6403488505961867432) {
  H_29(state, unused, out_6403488505961867432);
}
void car_h_28(double *state, double *unused, double *out_1260949585861042874) {
  h_28(state, unused, out_1260949585861042874);
}
void car_H_28(double *state, double *unused, double *out_1321089488892336858) {
  H_28(state, unused, out_1321089488892336858);
}
void car_h_31(double *state, double *unused, double *out_4998001837419606772) {
  h_31(state, unused, out_4998001837419606772);
}
void car_H_31(double *state, double *unused, double *out_992787217967181079) {
  H_31(state, unused, out_992787217967181079);
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
