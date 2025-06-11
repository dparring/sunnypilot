#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3180036843001874960);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7632041844693148483);
void pose_H_mod_fun(double *state, double *out_6446961619798279423);
void pose_f_fun(double *state, double dt, double *out_2638906143691493058);
void pose_F_fun(double *state, double dt, double *out_7572280307579006844);
void pose_h_4(double *state, double *unused, double *out_8190519028499974415);
void pose_H_4(double *state, double *unused, double *out_6847264068872191458);
void pose_h_10(double *state, double *unused, double *out_5274599112103372538);
void pose_H_10(double *state, double *unused, double *out_2523400390457892172);
void pose_h_13(double *state, double *unused, double *out_8352704181929443466);
void pose_H_13(double *state, double *unused, double *out_7765724541534836134);
void pose_h_14(double *state, double *unused, double *out_8386492674217883514);
void pose_H_14(double *state, double *unused, double *out_8516691572541987862);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}