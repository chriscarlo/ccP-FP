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
void err_fun(double *nom_x, double *delta_x, double *out_384395408799656553) {
   out_384395408799656553[0] = delta_x[0] + nom_x[0];
   out_384395408799656553[1] = delta_x[1] + nom_x[1];
   out_384395408799656553[2] = delta_x[2] + nom_x[2];
   out_384395408799656553[3] = delta_x[3] + nom_x[3];
   out_384395408799656553[4] = delta_x[4] + nom_x[4];
   out_384395408799656553[5] = delta_x[5] + nom_x[5];
   out_384395408799656553[6] = delta_x[6] + nom_x[6];
   out_384395408799656553[7] = delta_x[7] + nom_x[7];
   out_384395408799656553[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_4317967038342513827) {
   out_4317967038342513827[0] = -nom_x[0] + true_x[0];
   out_4317967038342513827[1] = -nom_x[1] + true_x[1];
   out_4317967038342513827[2] = -nom_x[2] + true_x[2];
   out_4317967038342513827[3] = -nom_x[3] + true_x[3];
   out_4317967038342513827[4] = -nom_x[4] + true_x[4];
   out_4317967038342513827[5] = -nom_x[5] + true_x[5];
   out_4317967038342513827[6] = -nom_x[6] + true_x[6];
   out_4317967038342513827[7] = -nom_x[7] + true_x[7];
   out_4317967038342513827[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_4181576082676280227) {
   out_4181576082676280227[0] = 1.0;
   out_4181576082676280227[1] = 0;
   out_4181576082676280227[2] = 0;
   out_4181576082676280227[3] = 0;
   out_4181576082676280227[4] = 0;
   out_4181576082676280227[5] = 0;
   out_4181576082676280227[6] = 0;
   out_4181576082676280227[7] = 0;
   out_4181576082676280227[8] = 0;
   out_4181576082676280227[9] = 0;
   out_4181576082676280227[10] = 1.0;
   out_4181576082676280227[11] = 0;
   out_4181576082676280227[12] = 0;
   out_4181576082676280227[13] = 0;
   out_4181576082676280227[14] = 0;
   out_4181576082676280227[15] = 0;
   out_4181576082676280227[16] = 0;
   out_4181576082676280227[17] = 0;
   out_4181576082676280227[18] = 0;
   out_4181576082676280227[19] = 0;
   out_4181576082676280227[20] = 1.0;
   out_4181576082676280227[21] = 0;
   out_4181576082676280227[22] = 0;
   out_4181576082676280227[23] = 0;
   out_4181576082676280227[24] = 0;
   out_4181576082676280227[25] = 0;
   out_4181576082676280227[26] = 0;
   out_4181576082676280227[27] = 0;
   out_4181576082676280227[28] = 0;
   out_4181576082676280227[29] = 0;
   out_4181576082676280227[30] = 1.0;
   out_4181576082676280227[31] = 0;
   out_4181576082676280227[32] = 0;
   out_4181576082676280227[33] = 0;
   out_4181576082676280227[34] = 0;
   out_4181576082676280227[35] = 0;
   out_4181576082676280227[36] = 0;
   out_4181576082676280227[37] = 0;
   out_4181576082676280227[38] = 0;
   out_4181576082676280227[39] = 0;
   out_4181576082676280227[40] = 1.0;
   out_4181576082676280227[41] = 0;
   out_4181576082676280227[42] = 0;
   out_4181576082676280227[43] = 0;
   out_4181576082676280227[44] = 0;
   out_4181576082676280227[45] = 0;
   out_4181576082676280227[46] = 0;
   out_4181576082676280227[47] = 0;
   out_4181576082676280227[48] = 0;
   out_4181576082676280227[49] = 0;
   out_4181576082676280227[50] = 1.0;
   out_4181576082676280227[51] = 0;
   out_4181576082676280227[52] = 0;
   out_4181576082676280227[53] = 0;
   out_4181576082676280227[54] = 0;
   out_4181576082676280227[55] = 0;
   out_4181576082676280227[56] = 0;
   out_4181576082676280227[57] = 0;
   out_4181576082676280227[58] = 0;
   out_4181576082676280227[59] = 0;
   out_4181576082676280227[60] = 1.0;
   out_4181576082676280227[61] = 0;
   out_4181576082676280227[62] = 0;
   out_4181576082676280227[63] = 0;
   out_4181576082676280227[64] = 0;
   out_4181576082676280227[65] = 0;
   out_4181576082676280227[66] = 0;
   out_4181576082676280227[67] = 0;
   out_4181576082676280227[68] = 0;
   out_4181576082676280227[69] = 0;
   out_4181576082676280227[70] = 1.0;
   out_4181576082676280227[71] = 0;
   out_4181576082676280227[72] = 0;
   out_4181576082676280227[73] = 0;
   out_4181576082676280227[74] = 0;
   out_4181576082676280227[75] = 0;
   out_4181576082676280227[76] = 0;
   out_4181576082676280227[77] = 0;
   out_4181576082676280227[78] = 0;
   out_4181576082676280227[79] = 0;
   out_4181576082676280227[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_1674564019154246051) {
   out_1674564019154246051[0] = state[0];
   out_1674564019154246051[1] = state[1];
   out_1674564019154246051[2] = state[2];
   out_1674564019154246051[3] = state[3];
   out_1674564019154246051[4] = state[4];
   out_1674564019154246051[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_1674564019154246051[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_1674564019154246051[7] = state[7];
   out_1674564019154246051[8] = state[8];
}
void F_fun(double *state, double dt, double *out_6487155568395618351) {
   out_6487155568395618351[0] = 1;
   out_6487155568395618351[1] = 0;
   out_6487155568395618351[2] = 0;
   out_6487155568395618351[3] = 0;
   out_6487155568395618351[4] = 0;
   out_6487155568395618351[5] = 0;
   out_6487155568395618351[6] = 0;
   out_6487155568395618351[7] = 0;
   out_6487155568395618351[8] = 0;
   out_6487155568395618351[9] = 0;
   out_6487155568395618351[10] = 1;
   out_6487155568395618351[11] = 0;
   out_6487155568395618351[12] = 0;
   out_6487155568395618351[13] = 0;
   out_6487155568395618351[14] = 0;
   out_6487155568395618351[15] = 0;
   out_6487155568395618351[16] = 0;
   out_6487155568395618351[17] = 0;
   out_6487155568395618351[18] = 0;
   out_6487155568395618351[19] = 0;
   out_6487155568395618351[20] = 1;
   out_6487155568395618351[21] = 0;
   out_6487155568395618351[22] = 0;
   out_6487155568395618351[23] = 0;
   out_6487155568395618351[24] = 0;
   out_6487155568395618351[25] = 0;
   out_6487155568395618351[26] = 0;
   out_6487155568395618351[27] = 0;
   out_6487155568395618351[28] = 0;
   out_6487155568395618351[29] = 0;
   out_6487155568395618351[30] = 1;
   out_6487155568395618351[31] = 0;
   out_6487155568395618351[32] = 0;
   out_6487155568395618351[33] = 0;
   out_6487155568395618351[34] = 0;
   out_6487155568395618351[35] = 0;
   out_6487155568395618351[36] = 0;
   out_6487155568395618351[37] = 0;
   out_6487155568395618351[38] = 0;
   out_6487155568395618351[39] = 0;
   out_6487155568395618351[40] = 1;
   out_6487155568395618351[41] = 0;
   out_6487155568395618351[42] = 0;
   out_6487155568395618351[43] = 0;
   out_6487155568395618351[44] = 0;
   out_6487155568395618351[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_6487155568395618351[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_6487155568395618351[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6487155568395618351[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_6487155568395618351[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_6487155568395618351[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_6487155568395618351[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_6487155568395618351[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_6487155568395618351[53] = -9.8000000000000007*dt;
   out_6487155568395618351[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_6487155568395618351[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_6487155568395618351[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6487155568395618351[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6487155568395618351[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_6487155568395618351[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_6487155568395618351[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_6487155568395618351[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_6487155568395618351[62] = 0;
   out_6487155568395618351[63] = 0;
   out_6487155568395618351[64] = 0;
   out_6487155568395618351[65] = 0;
   out_6487155568395618351[66] = 0;
   out_6487155568395618351[67] = 0;
   out_6487155568395618351[68] = 0;
   out_6487155568395618351[69] = 0;
   out_6487155568395618351[70] = 1;
   out_6487155568395618351[71] = 0;
   out_6487155568395618351[72] = 0;
   out_6487155568395618351[73] = 0;
   out_6487155568395618351[74] = 0;
   out_6487155568395618351[75] = 0;
   out_6487155568395618351[76] = 0;
   out_6487155568395618351[77] = 0;
   out_6487155568395618351[78] = 0;
   out_6487155568395618351[79] = 0;
   out_6487155568395618351[80] = 1;
}
void h_25(double *state, double *unused, double *out_3011331904376095653) {
   out_3011331904376095653[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2229680463775396768) {
   out_2229680463775396768[0] = 0;
   out_2229680463775396768[1] = 0;
   out_2229680463775396768[2] = 0;
   out_2229680463775396768[3] = 0;
   out_2229680463775396768[4] = 0;
   out_2229680463775396768[5] = 0;
   out_2229680463775396768[6] = 1;
   out_2229680463775396768[7] = 0;
   out_2229680463775396768[8] = 0;
}
void h_24(double *state, double *unused, double *out_181593291700838695) {
   out_181593291700838695[0] = state[4];
   out_181593291700838695[1] = state[5];
}
void H_24(double *state, double *unused, double *out_4406894887382546741) {
   out_4406894887382546741[0] = 0;
   out_4406894887382546741[1] = 0;
   out_4406894887382546741[2] = 0;
   out_4406894887382546741[3] = 0;
   out_4406894887382546741[4] = 1;
   out_4406894887382546741[5] = 0;
   out_4406894887382546741[6] = 0;
   out_4406894887382546741[7] = 0;
   out_4406894887382546741[8] = 0;
   out_4406894887382546741[9] = 0;
   out_4406894887382546741[10] = 0;
   out_4406894887382546741[11] = 0;
   out_4406894887382546741[12] = 0;
   out_4406894887382546741[13] = 0;
   out_4406894887382546741[14] = 1;
   out_4406894887382546741[15] = 0;
   out_4406894887382546741[16] = 0;
   out_4406894887382546741[17] = 0;
}
void h_30(double *state, double *unused, double *out_7109635202655467517) {
   out_7109635202655467517[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4748013422282645395) {
   out_4748013422282645395[0] = 0;
   out_4748013422282645395[1] = 0;
   out_4748013422282645395[2] = 0;
   out_4748013422282645395[3] = 0;
   out_4748013422282645395[4] = 1;
   out_4748013422282645395[5] = 0;
   out_4748013422282645395[6] = 0;
   out_4748013422282645395[7] = 0;
   out_4748013422282645395[8] = 0;
}
void h_26(double *state, double *unused, double *out_4072849151211134832) {
   out_4072849151211134832[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1511822855098659456) {
   out_1511822855098659456[0] = 0;
   out_1511822855098659456[1] = 0;
   out_1511822855098659456[2] = 0;
   out_1511822855098659456[3] = 0;
   out_1511822855098659456[4] = 0;
   out_1511822855098659456[5] = 0;
   out_1511822855098659456[6] = 0;
   out_1511822855098659456[7] = 1;
   out_1511822855098659456[8] = 0;
}
void h_27(double *state, double *unused, double *out_3954841757277758088) {
   out_3954841757277758088[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2573250110482220484) {
   out_2573250110482220484[0] = 0;
   out_2573250110482220484[1] = 0;
   out_2573250110482220484[2] = 0;
   out_2573250110482220484[3] = 1;
   out_2573250110482220484[4] = 0;
   out_2573250110482220484[5] = 0;
   out_2573250110482220484[6] = 0;
   out_2573250110482220484[7] = 0;
   out_2573250110482220484[8] = 0;
}
void h_29(double *state, double *unused, double *out_4938067353640234953) {
   out_4938067353640234953[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5258244766597037579) {
   out_5258244766597037579[0] = 0;
   out_5258244766597037579[1] = 1;
   out_5258244766597037579[2] = 0;
   out_5258244766597037579[3] = 0;
   out_5258244766597037579[4] = 0;
   out_5258244766597037579[5] = 0;
   out_5258244766597037579[6] = 0;
   out_5258244766597037579[7] = 0;
   out_5258244766597037579[8] = 0;
}
void h_28(double *state, double *unused, double *out_3196991612116939278) {
   out_3196991612116939278[0] = state[0];
}
void H_28(double *state, double *unused, double *out_175845749527507005) {
   out_175845749527507005[0] = 1;
   out_175845749527507005[1] = 0;
   out_175845749527507005[2] = 0;
   out_175845749527507005[3] = 0;
   out_175845749527507005[4] = 0;
   out_175845749527507005[5] = 0;
   out_175845749527507005[6] = 0;
   out_175845749527507005[7] = 0;
   out_175845749527507005[8] = 0;
}
void h_31(double *state, double *unused, double *out_7554139518579406778) {
   out_7554139518579406778[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2138030957332010932) {
   out_2138030957332010932[0] = 0;
   out_2138030957332010932[1] = 0;
   out_2138030957332010932[2] = 0;
   out_2138030957332010932[3] = 0;
   out_2138030957332010932[4] = 0;
   out_2138030957332010932[5] = 0;
   out_2138030957332010932[6] = 0;
   out_2138030957332010932[7] = 0;
   out_2138030957332010932[8] = 1;
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
void car_err_fun(double *nom_x, double *delta_x, double *out_384395408799656553) {
  err_fun(nom_x, delta_x, out_384395408799656553);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4317967038342513827) {
  inv_err_fun(nom_x, true_x, out_4317967038342513827);
}
void car_H_mod_fun(double *state, double *out_4181576082676280227) {
  H_mod_fun(state, out_4181576082676280227);
}
void car_f_fun(double *state, double dt, double *out_1674564019154246051) {
  f_fun(state,  dt, out_1674564019154246051);
}
void car_F_fun(double *state, double dt, double *out_6487155568395618351) {
  F_fun(state,  dt, out_6487155568395618351);
}
void car_h_25(double *state, double *unused, double *out_3011331904376095653) {
  h_25(state, unused, out_3011331904376095653);
}
void car_H_25(double *state, double *unused, double *out_2229680463775396768) {
  H_25(state, unused, out_2229680463775396768);
}
void car_h_24(double *state, double *unused, double *out_181593291700838695) {
  h_24(state, unused, out_181593291700838695);
}
void car_H_24(double *state, double *unused, double *out_4406894887382546741) {
  H_24(state, unused, out_4406894887382546741);
}
void car_h_30(double *state, double *unused, double *out_7109635202655467517) {
  h_30(state, unused, out_7109635202655467517);
}
void car_H_30(double *state, double *unused, double *out_4748013422282645395) {
  H_30(state, unused, out_4748013422282645395);
}
void car_h_26(double *state, double *unused, double *out_4072849151211134832) {
  h_26(state, unused, out_4072849151211134832);
}
void car_H_26(double *state, double *unused, double *out_1511822855098659456) {
  H_26(state, unused, out_1511822855098659456);
}
void car_h_27(double *state, double *unused, double *out_3954841757277758088) {
  h_27(state, unused, out_3954841757277758088);
}
void car_H_27(double *state, double *unused, double *out_2573250110482220484) {
  H_27(state, unused, out_2573250110482220484);
}
void car_h_29(double *state, double *unused, double *out_4938067353640234953) {
  h_29(state, unused, out_4938067353640234953);
}
void car_H_29(double *state, double *unused, double *out_5258244766597037579) {
  H_29(state, unused, out_5258244766597037579);
}
void car_h_28(double *state, double *unused, double *out_3196991612116939278) {
  h_28(state, unused, out_3196991612116939278);
}
void car_H_28(double *state, double *unused, double *out_175845749527507005) {
  H_28(state, unused, out_175845749527507005);
}
void car_h_31(double *state, double *unused, double *out_7554139518579406778) {
  h_31(state, unused, out_7554139518579406778);
}
void car_H_31(double *state, double *unused, double *out_2138030957332010932) {
  H_31(state, unused, out_2138030957332010932);
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
