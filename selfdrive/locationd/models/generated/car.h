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
void car_err_fun(double *nom_x, double *delta_x, double *out_4633573009917725202);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3575493581669951122);
void car_H_mod_fun(double *state, double *out_6051152977375382637);
void car_f_fun(double *state, double dt, double *out_1721947030779493947);
void car_F_fun(double *state, double dt, double *out_169967702265322698);
void car_h_25(double *state, double *unused, double *out_6413001333912154685);
void car_H_25(double *state, double *unused, double *out_5284705215293175388);
void car_h_24(double *state, double *unused, double *out_6028044839323201886);
void car_H_24(double *state, double *unused, double *out_916327464557986038);
void car_h_30(double *state, double *unused, double *out_5878295543956736802);
void car_H_30(double *state, double *unused, double *out_757008885165567190);
void car_h_26(double *state, double *unused, double *out_536428308070706239);
void car_H_26(double *state, double *unused, double *out_1543201896419119164);
void car_h_27(double *state, double *unused, double *out_1711865638415083517);
void car_H_27(double *state, double *unused, double *out_2980602956349510407);
void car_h_29(double *state, double *unused, double *out_6169407012338965668);
void car_H_29(double *state, double *unused, double *out_1267240229479959374);
void car_h_28(double *state, double *unused, double *out_4203765115318778819);
void car_H_28(double *state, double *unused, double *out_3230870501045285625);
void car_h_31(double *state, double *unused, double *out_5262907513447045995);
void car_H_31(double *state, double *unused, double *out_916993794185767688);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}