extern "C" {
#include <math.h>
/******************************************************************************
 *                      Code generated with sympy 1.7.1                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5006000321884084041) {
   out_5006000321884084041[0] = delta_x[0] + nom_x[0];
   out_5006000321884084041[1] = delta_x[1] + nom_x[1];
   out_5006000321884084041[2] = delta_x[2] + nom_x[2];
   out_5006000321884084041[3] = delta_x[3] + nom_x[3];
   out_5006000321884084041[4] = delta_x[4] + nom_x[4];
   out_5006000321884084041[5] = delta_x[5] + nom_x[5];
   out_5006000321884084041[6] = delta_x[6] + nom_x[6];
   out_5006000321884084041[7] = delta_x[7] + nom_x[7];
   out_5006000321884084041[8] = delta_x[8] + nom_x[8];
   out_5006000321884084041[9] = delta_x[9] + nom_x[9];
   out_5006000321884084041[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_3981933015473888526) {
   out_3981933015473888526[0] = -nom_x[0] + true_x[0];
   out_3981933015473888526[1] = -nom_x[1] + true_x[1];
   out_3981933015473888526[2] = -nom_x[2] + true_x[2];
   out_3981933015473888526[3] = -nom_x[3] + true_x[3];
   out_3981933015473888526[4] = -nom_x[4] + true_x[4];
   out_3981933015473888526[5] = -nom_x[5] + true_x[5];
   out_3981933015473888526[6] = -nom_x[6] + true_x[6];
   out_3981933015473888526[7] = -nom_x[7] + true_x[7];
   out_3981933015473888526[8] = -nom_x[8] + true_x[8];
   out_3981933015473888526[9] = -nom_x[9] + true_x[9];
   out_3981933015473888526[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_3273191651415114259) {
   out_3273191651415114259[0] = 1.0;
   out_3273191651415114259[1] = 0.0;
   out_3273191651415114259[2] = 0.0;
   out_3273191651415114259[3] = 0.0;
   out_3273191651415114259[4] = 0.0;
   out_3273191651415114259[5] = 0.0;
   out_3273191651415114259[6] = 0.0;
   out_3273191651415114259[7] = 0.0;
   out_3273191651415114259[8] = 0.0;
   out_3273191651415114259[9] = 0.0;
   out_3273191651415114259[10] = 0.0;
   out_3273191651415114259[11] = 0.0;
   out_3273191651415114259[12] = 1.0;
   out_3273191651415114259[13] = 0.0;
   out_3273191651415114259[14] = 0.0;
   out_3273191651415114259[15] = 0.0;
   out_3273191651415114259[16] = 0.0;
   out_3273191651415114259[17] = 0.0;
   out_3273191651415114259[18] = 0.0;
   out_3273191651415114259[19] = 0.0;
   out_3273191651415114259[20] = 0.0;
   out_3273191651415114259[21] = 0.0;
   out_3273191651415114259[22] = 0.0;
   out_3273191651415114259[23] = 0.0;
   out_3273191651415114259[24] = 1.0;
   out_3273191651415114259[25] = 0.0;
   out_3273191651415114259[26] = 0.0;
   out_3273191651415114259[27] = 0.0;
   out_3273191651415114259[28] = 0.0;
   out_3273191651415114259[29] = 0.0;
   out_3273191651415114259[30] = 0.0;
   out_3273191651415114259[31] = 0.0;
   out_3273191651415114259[32] = 0.0;
   out_3273191651415114259[33] = 0.0;
   out_3273191651415114259[34] = 0.0;
   out_3273191651415114259[35] = 0.0;
   out_3273191651415114259[36] = 1.0;
   out_3273191651415114259[37] = 0.0;
   out_3273191651415114259[38] = 0.0;
   out_3273191651415114259[39] = 0.0;
   out_3273191651415114259[40] = 0.0;
   out_3273191651415114259[41] = 0.0;
   out_3273191651415114259[42] = 0.0;
   out_3273191651415114259[43] = 0.0;
   out_3273191651415114259[44] = 0.0;
   out_3273191651415114259[45] = 0.0;
   out_3273191651415114259[46] = 0.0;
   out_3273191651415114259[47] = 0.0;
   out_3273191651415114259[48] = 1.0;
   out_3273191651415114259[49] = 0.0;
   out_3273191651415114259[50] = 0.0;
   out_3273191651415114259[51] = 0.0;
   out_3273191651415114259[52] = 0.0;
   out_3273191651415114259[53] = 0.0;
   out_3273191651415114259[54] = 0.0;
   out_3273191651415114259[55] = 0.0;
   out_3273191651415114259[56] = 0.0;
   out_3273191651415114259[57] = 0.0;
   out_3273191651415114259[58] = 0.0;
   out_3273191651415114259[59] = 0.0;
   out_3273191651415114259[60] = 1.0;
   out_3273191651415114259[61] = 0.0;
   out_3273191651415114259[62] = 0.0;
   out_3273191651415114259[63] = 0.0;
   out_3273191651415114259[64] = 0.0;
   out_3273191651415114259[65] = 0.0;
   out_3273191651415114259[66] = 0.0;
   out_3273191651415114259[67] = 0.0;
   out_3273191651415114259[68] = 0.0;
   out_3273191651415114259[69] = 0.0;
   out_3273191651415114259[70] = 0.0;
   out_3273191651415114259[71] = 0.0;
   out_3273191651415114259[72] = 1.0;
   out_3273191651415114259[73] = 0.0;
   out_3273191651415114259[74] = 0.0;
   out_3273191651415114259[75] = 0.0;
   out_3273191651415114259[76] = 0.0;
   out_3273191651415114259[77] = 0.0;
   out_3273191651415114259[78] = 0.0;
   out_3273191651415114259[79] = 0.0;
   out_3273191651415114259[80] = 0.0;
   out_3273191651415114259[81] = 0.0;
   out_3273191651415114259[82] = 0.0;
   out_3273191651415114259[83] = 0.0;
   out_3273191651415114259[84] = 1.0;
   out_3273191651415114259[85] = 0.0;
   out_3273191651415114259[86] = 0.0;
   out_3273191651415114259[87] = 0.0;
   out_3273191651415114259[88] = 0.0;
   out_3273191651415114259[89] = 0.0;
   out_3273191651415114259[90] = 0.0;
   out_3273191651415114259[91] = 0.0;
   out_3273191651415114259[92] = 0.0;
   out_3273191651415114259[93] = 0.0;
   out_3273191651415114259[94] = 0.0;
   out_3273191651415114259[95] = 0.0;
   out_3273191651415114259[96] = 1.0;
   out_3273191651415114259[97] = 0.0;
   out_3273191651415114259[98] = 0.0;
   out_3273191651415114259[99] = 0.0;
   out_3273191651415114259[100] = 0.0;
   out_3273191651415114259[101] = 0.0;
   out_3273191651415114259[102] = 0.0;
   out_3273191651415114259[103] = 0.0;
   out_3273191651415114259[104] = 0.0;
   out_3273191651415114259[105] = 0.0;
   out_3273191651415114259[106] = 0.0;
   out_3273191651415114259[107] = 0.0;
   out_3273191651415114259[108] = 1.0;
   out_3273191651415114259[109] = 0.0;
   out_3273191651415114259[110] = 0.0;
   out_3273191651415114259[111] = 0.0;
   out_3273191651415114259[112] = 0.0;
   out_3273191651415114259[113] = 0.0;
   out_3273191651415114259[114] = 0.0;
   out_3273191651415114259[115] = 0.0;
   out_3273191651415114259[116] = 0.0;
   out_3273191651415114259[117] = 0.0;
   out_3273191651415114259[118] = 0.0;
   out_3273191651415114259[119] = 0.0;
   out_3273191651415114259[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_7258853033567657718) {
   out_7258853033567657718[0] = dt*state[3] + state[0];
   out_7258853033567657718[1] = dt*state[4] + state[1];
   out_7258853033567657718[2] = dt*state[5] + state[2];
   out_7258853033567657718[3] = state[3];
   out_7258853033567657718[4] = state[4];
   out_7258853033567657718[5] = state[5];
   out_7258853033567657718[6] = dt*state[7] + state[6];
   out_7258853033567657718[7] = dt*state[8] + state[7];
   out_7258853033567657718[8] = state[8];
   out_7258853033567657718[9] = state[9];
   out_7258853033567657718[10] = state[10];
}
void F_fun(double *state, double dt, double *out_443719917416495848) {
   out_443719917416495848[0] = 1;
   out_443719917416495848[1] = 0;
   out_443719917416495848[2] = 0;
   out_443719917416495848[3] = dt;
   out_443719917416495848[4] = 0;
   out_443719917416495848[5] = 0;
   out_443719917416495848[6] = 0;
   out_443719917416495848[7] = 0;
   out_443719917416495848[8] = 0;
   out_443719917416495848[9] = 0;
   out_443719917416495848[10] = 0;
   out_443719917416495848[11] = 0;
   out_443719917416495848[12] = 1;
   out_443719917416495848[13] = 0;
   out_443719917416495848[14] = 0;
   out_443719917416495848[15] = dt;
   out_443719917416495848[16] = 0;
   out_443719917416495848[17] = 0;
   out_443719917416495848[18] = 0;
   out_443719917416495848[19] = 0;
   out_443719917416495848[20] = 0;
   out_443719917416495848[21] = 0;
   out_443719917416495848[22] = 0;
   out_443719917416495848[23] = 0;
   out_443719917416495848[24] = 1;
   out_443719917416495848[25] = 0;
   out_443719917416495848[26] = 0;
   out_443719917416495848[27] = dt;
   out_443719917416495848[28] = 0;
   out_443719917416495848[29] = 0;
   out_443719917416495848[30] = 0;
   out_443719917416495848[31] = 0;
   out_443719917416495848[32] = 0;
   out_443719917416495848[33] = 0;
   out_443719917416495848[34] = 0;
   out_443719917416495848[35] = 0;
   out_443719917416495848[36] = 1;
   out_443719917416495848[37] = 0;
   out_443719917416495848[38] = 0;
   out_443719917416495848[39] = 0;
   out_443719917416495848[40] = 0;
   out_443719917416495848[41] = 0;
   out_443719917416495848[42] = 0;
   out_443719917416495848[43] = 0;
   out_443719917416495848[44] = 0;
   out_443719917416495848[45] = 0;
   out_443719917416495848[46] = 0;
   out_443719917416495848[47] = 0;
   out_443719917416495848[48] = 1;
   out_443719917416495848[49] = 0;
   out_443719917416495848[50] = 0;
   out_443719917416495848[51] = 0;
   out_443719917416495848[52] = 0;
   out_443719917416495848[53] = 0;
   out_443719917416495848[54] = 0;
   out_443719917416495848[55] = 0;
   out_443719917416495848[56] = 0;
   out_443719917416495848[57] = 0;
   out_443719917416495848[58] = 0;
   out_443719917416495848[59] = 0;
   out_443719917416495848[60] = 1;
   out_443719917416495848[61] = 0;
   out_443719917416495848[62] = 0;
   out_443719917416495848[63] = 0;
   out_443719917416495848[64] = 0;
   out_443719917416495848[65] = 0;
   out_443719917416495848[66] = 0;
   out_443719917416495848[67] = 0;
   out_443719917416495848[68] = 0;
   out_443719917416495848[69] = 0;
   out_443719917416495848[70] = 0;
   out_443719917416495848[71] = 0;
   out_443719917416495848[72] = 1;
   out_443719917416495848[73] = dt;
   out_443719917416495848[74] = 0;
   out_443719917416495848[75] = 0;
   out_443719917416495848[76] = 0;
   out_443719917416495848[77] = 0;
   out_443719917416495848[78] = 0;
   out_443719917416495848[79] = 0;
   out_443719917416495848[80] = 0;
   out_443719917416495848[81] = 0;
   out_443719917416495848[82] = 0;
   out_443719917416495848[83] = 0;
   out_443719917416495848[84] = 1;
   out_443719917416495848[85] = dt;
   out_443719917416495848[86] = 0;
   out_443719917416495848[87] = 0;
   out_443719917416495848[88] = 0;
   out_443719917416495848[89] = 0;
   out_443719917416495848[90] = 0;
   out_443719917416495848[91] = 0;
   out_443719917416495848[92] = 0;
   out_443719917416495848[93] = 0;
   out_443719917416495848[94] = 0;
   out_443719917416495848[95] = 0;
   out_443719917416495848[96] = 1;
   out_443719917416495848[97] = 0;
   out_443719917416495848[98] = 0;
   out_443719917416495848[99] = 0;
   out_443719917416495848[100] = 0;
   out_443719917416495848[101] = 0;
   out_443719917416495848[102] = 0;
   out_443719917416495848[103] = 0;
   out_443719917416495848[104] = 0;
   out_443719917416495848[105] = 0;
   out_443719917416495848[106] = 0;
   out_443719917416495848[107] = 0;
   out_443719917416495848[108] = 1;
   out_443719917416495848[109] = 0;
   out_443719917416495848[110] = 0;
   out_443719917416495848[111] = 0;
   out_443719917416495848[112] = 0;
   out_443719917416495848[113] = 0;
   out_443719917416495848[114] = 0;
   out_443719917416495848[115] = 0;
   out_443719917416495848[116] = 0;
   out_443719917416495848[117] = 0;
   out_443719917416495848[118] = 0;
   out_443719917416495848[119] = 0;
   out_443719917416495848[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_8500560742027598119) {
   out_8500560742027598119[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_6644436718895073636) {
   out_6644436718895073636[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6644436718895073636[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6644436718895073636[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_6644436718895073636[3] = 0;
   out_6644436718895073636[4] = 0;
   out_6644436718895073636[5] = 0;
   out_6644436718895073636[6] = 1;
   out_6644436718895073636[7] = 0;
   out_6644436718895073636[8] = 0;
   out_6644436718895073636[9] = 0;
   out_6644436718895073636[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_6578852598750211571) {
   out_6578852598750211571[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_2923708671879078839) {
   out_2923708671879078839[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2923708671879078839[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2923708671879078839[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2923708671879078839[3] = 0;
   out_2923708671879078839[4] = 0;
   out_2923708671879078839[5] = 0;
   out_2923708671879078839[6] = 1;
   out_2923708671879078839[7] = 0;
   out_2923708671879078839[8] = 0;
   out_2923708671879078839[9] = 1;
   out_2923708671879078839[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_8926697339168992080) {
   out_8926697339168992080[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_77648526793984282) {
   out_77648526793984282[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_77648526793984282[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_77648526793984282[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_77648526793984282[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_77648526793984282[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_77648526793984282[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_77648526793984282[6] = 0;
   out_77648526793984282[7] = 1;
   out_77648526793984282[8] = 0;
   out_77648526793984282[9] = 0;
   out_77648526793984282[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_8926697339168992080) {
   out_8926697339168992080[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_77648526793984282) {
   out_77648526793984282[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_77648526793984282[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_77648526793984282[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_77648526793984282[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_77648526793984282[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_77648526793984282[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_77648526793984282[6] = 0;
   out_77648526793984282[7] = 1;
   out_77648526793984282[8] = 0;
   out_77648526793984282[9] = 0;
   out_77648526793984282[10] = 0;
}
}

extern "C"{
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



extern "C"{

      void update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
      }
    
      void update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
      }
    
      void update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
      }
    
      void update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
        update<1,3,0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
      }
    
}
