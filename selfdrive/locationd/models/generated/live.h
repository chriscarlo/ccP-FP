#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1273177161694806375);
void live_err_fun(double *nom_x, double *delta_x, double *out_1430031797466046139);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_4212555668691755734);
void live_H_mod_fun(double *state, double *out_5495397011748731228);
void live_f_fun(double *state, double dt, double *out_7830183483826484002);
void live_F_fun(double *state, double dt, double *out_9021355042514805664);
void live_h_4(double *state, double *unused, double *out_1356980630295645182);
void live_H_4(double *state, double *unused, double *out_8979306753825182228);
void live_h_9(double *state, double *unused, double *out_5605808899268505391);
void live_H_9(double *state, double *unused, double *out_2180218384619921918);
void live_h_10(double *state, double *unused, double *out_1990170345022901519);
void live_H_10(double *state, double *unused, double *out_3601392890603494076);
void live_h_12(double *state, double *unused, double *out_2423697160155285196);
void live_H_12(double *state, double *unused, double *out_1800309006201918896);
void live_h_35(double *state, double *unused, double *out_8299251158518133531);
void live_H_35(double *state, double *unused, double *out_6100775262511762012);
void live_h_32(double *state, double *unused, double *out_3487178591502051074);
void live_H_32(double *state, double *unused, double *out_6746558398330707371);
void live_h_13(double *state, double *unused, double *out_255412017322975092);
void live_H_13(double *state, double *unused, double *out_2514018880265918871);
void live_h_14(double *state, double *unused, double *out_5605808899268505391);
void live_H_14(double *state, double *unused, double *out_2180218384619921918);
void live_h_33(double *state, double *unused, double *out_8254374040319119309);
void live_H_33(double *state, double *unused, double *out_4095811030761952417);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}