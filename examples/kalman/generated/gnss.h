/******************************************************************************
 *                      Code generated with sympy 1.7.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5006000321884084041);
void inv_err_fun(double *nom_x, double *true_x, double *out_3981933015473888526);
void H_mod_fun(double *state, double *out_3273191651415114259);
void f_fun(double *state, double dt, double *out_7258853033567657718);
void F_fun(double *state, double dt, double *out_443719917416495848);
void h_6(double *state, double *sat_pos, double *out_8500560742027598119);
void H_6(double *state, double *sat_pos, double *out_6644436718895073636);
void h_20(double *state, double *sat_pos, double *out_6578852598750211571);
void H_20(double *state, double *sat_pos, double *out_2923708671879078839);
void h_7(double *state, double *sat_pos_vel, double *out_8926697339168992080);
void H_7(double *state, double *sat_pos_vel, double *out_77648526793984282);
void h_21(double *state, double *sat_pos_vel, double *out_8926697339168992080);
void H_21(double *state, double *sat_pos_vel, double *out_77648526793984282);
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);

void predict(double *x, double *P, double *Q, double dt);
const static double MAHA_THRESH_6 = 3.841459;
void update_6(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_20 = 3.841459;
void update_20(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_7 = 3.841459;
void update_7(double *, double *, double *, double *, double *);
const static double MAHA_THRESH_21 = 3.841459;
void update_21(double *, double *, double *, double *, double *);