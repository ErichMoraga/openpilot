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
void car_err_fun(double *nom_x, double *delta_x, double *out_5709880423575590152);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_8634507341529600210);
void car_H_mod_fun(double *state, double *out_3871090052338825407);
void car_f_fun(double *state, double dt, double *out_6137814433410422852);
void car_F_fun(double *state, double dt, double *out_7621765943040258911);
void car_h_25(double *state, double *unused, double *out_4722807775135100883);
void car_H_25(double *state, double *unused, double *out_3374924203140226621);
void car_h_24(double *state, double *unused, double *out_2835190905026714642);
void car_H_24(double *state, double *unused, double *out_1202274604134727055);
void car_h_30(double *state, double *unused, double *out_1085358246881078533);
void car_H_30(double *state, double *unused, double *out_5893257161647475248);
void car_h_26(double *state, double *unused, double *out_6151775765599421256);
void car_H_26(double *state, double *unused, double *out_366579115733829603);
void car_h_27(double *state, double *unused, double *out_278865492525744762);
void car_H_27(double *state, double *unused, double *out_3718493849847050337);
void car_h_29(double *state, double *unused, double *out_3916315020779767112);
void car_H_29(double *state, double *unused, double *out_6403488505961867432);
void car_h_28(double *state, double *unused, double *out_1260949585861042874);
void car_H_28(double *state, double *unused, double *out_1321089488892336858);
void car_h_31(double *state, double *unused, double *out_4998001837419606772);
void car_H_31(double *state, double *unused, double *out_992787217967181079);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}