#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_384395408799656553);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4317967038342513827);
void car_H_mod_fun(double *state, double *out_4181576082676280227);
void car_f_fun(double *state, double dt, double *out_1674564019154246051);
void car_F_fun(double *state, double dt, double *out_6487155568395618351);
void car_h_25(double *state, double *unused, double *out_3011331904376095653);
void car_H_25(double *state, double *unused, double *out_2229680463775396768);
void car_h_24(double *state, double *unused, double *out_181593291700838695);
void car_H_24(double *state, double *unused, double *out_4406894887382546741);
void car_h_30(double *state, double *unused, double *out_7109635202655467517);
void car_H_30(double *state, double *unused, double *out_4748013422282645395);
void car_h_26(double *state, double *unused, double *out_4072849151211134832);
void car_H_26(double *state, double *unused, double *out_1511822855098659456);
void car_h_27(double *state, double *unused, double *out_3954841757277758088);
void car_H_27(double *state, double *unused, double *out_2573250110482220484);
void car_h_29(double *state, double *unused, double *out_4938067353640234953);
void car_H_29(double *state, double *unused, double *out_5258244766597037579);
void car_h_28(double *state, double *unused, double *out_3196991612116939278);
void car_H_28(double *state, double *unused, double *out_175845749527507005);
void car_h_31(double *state, double *unused, double *out_7554139518579406778);
void car_H_31(double *state, double *unused, double *out_2138030957332010932);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}