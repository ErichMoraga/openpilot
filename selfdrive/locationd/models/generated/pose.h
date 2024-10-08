#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_7983792447504184197);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5020775487918234326);
void pose_H_mod_fun(double *state, double *out_2023537857534306321);
void pose_f_fun(double *state, double dt, double *out_2921235662366984185);
void pose_F_fun(double *state, double dt, double *out_4798703416850893340);
void pose_h_4(double *state, double *unused, double *out_5260931664586000243);
void pose_H_4(double *state, double *unused, double *out_3707798395825917046);
void pose_h_10(double *state, double *unused, double *out_6664006634176473998);
void pose_H_10(double *state, double *unused, double *out_5640695491572100361);
void pose_h_13(double *state, double *unused, double *out_2018719708536053041);
void pose_H_13(double *state, double *unused, double *out_6920072221158249847);
void pose_h_14(double *state, double *unused, double *out_88167374712919576);
void pose_H_14(double *state, double *unused, double *out_7671039252165401575);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}