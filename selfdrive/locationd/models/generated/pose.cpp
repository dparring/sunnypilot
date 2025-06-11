#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3180036843001874960) {
   out_3180036843001874960[0] = delta_x[0] + nom_x[0];
   out_3180036843001874960[1] = delta_x[1] + nom_x[1];
   out_3180036843001874960[2] = delta_x[2] + nom_x[2];
   out_3180036843001874960[3] = delta_x[3] + nom_x[3];
   out_3180036843001874960[4] = delta_x[4] + nom_x[4];
   out_3180036843001874960[5] = delta_x[5] + nom_x[5];
   out_3180036843001874960[6] = delta_x[6] + nom_x[6];
   out_3180036843001874960[7] = delta_x[7] + nom_x[7];
   out_3180036843001874960[8] = delta_x[8] + nom_x[8];
   out_3180036843001874960[9] = delta_x[9] + nom_x[9];
   out_3180036843001874960[10] = delta_x[10] + nom_x[10];
   out_3180036843001874960[11] = delta_x[11] + nom_x[11];
   out_3180036843001874960[12] = delta_x[12] + nom_x[12];
   out_3180036843001874960[13] = delta_x[13] + nom_x[13];
   out_3180036843001874960[14] = delta_x[14] + nom_x[14];
   out_3180036843001874960[15] = delta_x[15] + nom_x[15];
   out_3180036843001874960[16] = delta_x[16] + nom_x[16];
   out_3180036843001874960[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7632041844693148483) {
   out_7632041844693148483[0] = -nom_x[0] + true_x[0];
   out_7632041844693148483[1] = -nom_x[1] + true_x[1];
   out_7632041844693148483[2] = -nom_x[2] + true_x[2];
   out_7632041844693148483[3] = -nom_x[3] + true_x[3];
   out_7632041844693148483[4] = -nom_x[4] + true_x[4];
   out_7632041844693148483[5] = -nom_x[5] + true_x[5];
   out_7632041844693148483[6] = -nom_x[6] + true_x[6];
   out_7632041844693148483[7] = -nom_x[7] + true_x[7];
   out_7632041844693148483[8] = -nom_x[8] + true_x[8];
   out_7632041844693148483[9] = -nom_x[9] + true_x[9];
   out_7632041844693148483[10] = -nom_x[10] + true_x[10];
   out_7632041844693148483[11] = -nom_x[11] + true_x[11];
   out_7632041844693148483[12] = -nom_x[12] + true_x[12];
   out_7632041844693148483[13] = -nom_x[13] + true_x[13];
   out_7632041844693148483[14] = -nom_x[14] + true_x[14];
   out_7632041844693148483[15] = -nom_x[15] + true_x[15];
   out_7632041844693148483[16] = -nom_x[16] + true_x[16];
   out_7632041844693148483[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_6446961619798279423) {
   out_6446961619798279423[0] = 1.0;
   out_6446961619798279423[1] = 0.0;
   out_6446961619798279423[2] = 0.0;
   out_6446961619798279423[3] = 0.0;
   out_6446961619798279423[4] = 0.0;
   out_6446961619798279423[5] = 0.0;
   out_6446961619798279423[6] = 0.0;
   out_6446961619798279423[7] = 0.0;
   out_6446961619798279423[8] = 0.0;
   out_6446961619798279423[9] = 0.0;
   out_6446961619798279423[10] = 0.0;
   out_6446961619798279423[11] = 0.0;
   out_6446961619798279423[12] = 0.0;
   out_6446961619798279423[13] = 0.0;
   out_6446961619798279423[14] = 0.0;
   out_6446961619798279423[15] = 0.0;
   out_6446961619798279423[16] = 0.0;
   out_6446961619798279423[17] = 0.0;
   out_6446961619798279423[18] = 0.0;
   out_6446961619798279423[19] = 1.0;
   out_6446961619798279423[20] = 0.0;
   out_6446961619798279423[21] = 0.0;
   out_6446961619798279423[22] = 0.0;
   out_6446961619798279423[23] = 0.0;
   out_6446961619798279423[24] = 0.0;
   out_6446961619798279423[25] = 0.0;
   out_6446961619798279423[26] = 0.0;
   out_6446961619798279423[27] = 0.0;
   out_6446961619798279423[28] = 0.0;
   out_6446961619798279423[29] = 0.0;
   out_6446961619798279423[30] = 0.0;
   out_6446961619798279423[31] = 0.0;
   out_6446961619798279423[32] = 0.0;
   out_6446961619798279423[33] = 0.0;
   out_6446961619798279423[34] = 0.0;
   out_6446961619798279423[35] = 0.0;
   out_6446961619798279423[36] = 0.0;
   out_6446961619798279423[37] = 0.0;
   out_6446961619798279423[38] = 1.0;
   out_6446961619798279423[39] = 0.0;
   out_6446961619798279423[40] = 0.0;
   out_6446961619798279423[41] = 0.0;
   out_6446961619798279423[42] = 0.0;
   out_6446961619798279423[43] = 0.0;
   out_6446961619798279423[44] = 0.0;
   out_6446961619798279423[45] = 0.0;
   out_6446961619798279423[46] = 0.0;
   out_6446961619798279423[47] = 0.0;
   out_6446961619798279423[48] = 0.0;
   out_6446961619798279423[49] = 0.0;
   out_6446961619798279423[50] = 0.0;
   out_6446961619798279423[51] = 0.0;
   out_6446961619798279423[52] = 0.0;
   out_6446961619798279423[53] = 0.0;
   out_6446961619798279423[54] = 0.0;
   out_6446961619798279423[55] = 0.0;
   out_6446961619798279423[56] = 0.0;
   out_6446961619798279423[57] = 1.0;
   out_6446961619798279423[58] = 0.0;
   out_6446961619798279423[59] = 0.0;
   out_6446961619798279423[60] = 0.0;
   out_6446961619798279423[61] = 0.0;
   out_6446961619798279423[62] = 0.0;
   out_6446961619798279423[63] = 0.0;
   out_6446961619798279423[64] = 0.0;
   out_6446961619798279423[65] = 0.0;
   out_6446961619798279423[66] = 0.0;
   out_6446961619798279423[67] = 0.0;
   out_6446961619798279423[68] = 0.0;
   out_6446961619798279423[69] = 0.0;
   out_6446961619798279423[70] = 0.0;
   out_6446961619798279423[71] = 0.0;
   out_6446961619798279423[72] = 0.0;
   out_6446961619798279423[73] = 0.0;
   out_6446961619798279423[74] = 0.0;
   out_6446961619798279423[75] = 0.0;
   out_6446961619798279423[76] = 1.0;
   out_6446961619798279423[77] = 0.0;
   out_6446961619798279423[78] = 0.0;
   out_6446961619798279423[79] = 0.0;
   out_6446961619798279423[80] = 0.0;
   out_6446961619798279423[81] = 0.0;
   out_6446961619798279423[82] = 0.0;
   out_6446961619798279423[83] = 0.0;
   out_6446961619798279423[84] = 0.0;
   out_6446961619798279423[85] = 0.0;
   out_6446961619798279423[86] = 0.0;
   out_6446961619798279423[87] = 0.0;
   out_6446961619798279423[88] = 0.0;
   out_6446961619798279423[89] = 0.0;
   out_6446961619798279423[90] = 0.0;
   out_6446961619798279423[91] = 0.0;
   out_6446961619798279423[92] = 0.0;
   out_6446961619798279423[93] = 0.0;
   out_6446961619798279423[94] = 0.0;
   out_6446961619798279423[95] = 1.0;
   out_6446961619798279423[96] = 0.0;
   out_6446961619798279423[97] = 0.0;
   out_6446961619798279423[98] = 0.0;
   out_6446961619798279423[99] = 0.0;
   out_6446961619798279423[100] = 0.0;
   out_6446961619798279423[101] = 0.0;
   out_6446961619798279423[102] = 0.0;
   out_6446961619798279423[103] = 0.0;
   out_6446961619798279423[104] = 0.0;
   out_6446961619798279423[105] = 0.0;
   out_6446961619798279423[106] = 0.0;
   out_6446961619798279423[107] = 0.0;
   out_6446961619798279423[108] = 0.0;
   out_6446961619798279423[109] = 0.0;
   out_6446961619798279423[110] = 0.0;
   out_6446961619798279423[111] = 0.0;
   out_6446961619798279423[112] = 0.0;
   out_6446961619798279423[113] = 0.0;
   out_6446961619798279423[114] = 1.0;
   out_6446961619798279423[115] = 0.0;
   out_6446961619798279423[116] = 0.0;
   out_6446961619798279423[117] = 0.0;
   out_6446961619798279423[118] = 0.0;
   out_6446961619798279423[119] = 0.0;
   out_6446961619798279423[120] = 0.0;
   out_6446961619798279423[121] = 0.0;
   out_6446961619798279423[122] = 0.0;
   out_6446961619798279423[123] = 0.0;
   out_6446961619798279423[124] = 0.0;
   out_6446961619798279423[125] = 0.0;
   out_6446961619798279423[126] = 0.0;
   out_6446961619798279423[127] = 0.0;
   out_6446961619798279423[128] = 0.0;
   out_6446961619798279423[129] = 0.0;
   out_6446961619798279423[130] = 0.0;
   out_6446961619798279423[131] = 0.0;
   out_6446961619798279423[132] = 0.0;
   out_6446961619798279423[133] = 1.0;
   out_6446961619798279423[134] = 0.0;
   out_6446961619798279423[135] = 0.0;
   out_6446961619798279423[136] = 0.0;
   out_6446961619798279423[137] = 0.0;
   out_6446961619798279423[138] = 0.0;
   out_6446961619798279423[139] = 0.0;
   out_6446961619798279423[140] = 0.0;
   out_6446961619798279423[141] = 0.0;
   out_6446961619798279423[142] = 0.0;
   out_6446961619798279423[143] = 0.0;
   out_6446961619798279423[144] = 0.0;
   out_6446961619798279423[145] = 0.0;
   out_6446961619798279423[146] = 0.0;
   out_6446961619798279423[147] = 0.0;
   out_6446961619798279423[148] = 0.0;
   out_6446961619798279423[149] = 0.0;
   out_6446961619798279423[150] = 0.0;
   out_6446961619798279423[151] = 0.0;
   out_6446961619798279423[152] = 1.0;
   out_6446961619798279423[153] = 0.0;
   out_6446961619798279423[154] = 0.0;
   out_6446961619798279423[155] = 0.0;
   out_6446961619798279423[156] = 0.0;
   out_6446961619798279423[157] = 0.0;
   out_6446961619798279423[158] = 0.0;
   out_6446961619798279423[159] = 0.0;
   out_6446961619798279423[160] = 0.0;
   out_6446961619798279423[161] = 0.0;
   out_6446961619798279423[162] = 0.0;
   out_6446961619798279423[163] = 0.0;
   out_6446961619798279423[164] = 0.0;
   out_6446961619798279423[165] = 0.0;
   out_6446961619798279423[166] = 0.0;
   out_6446961619798279423[167] = 0.0;
   out_6446961619798279423[168] = 0.0;
   out_6446961619798279423[169] = 0.0;
   out_6446961619798279423[170] = 0.0;
   out_6446961619798279423[171] = 1.0;
   out_6446961619798279423[172] = 0.0;
   out_6446961619798279423[173] = 0.0;
   out_6446961619798279423[174] = 0.0;
   out_6446961619798279423[175] = 0.0;
   out_6446961619798279423[176] = 0.0;
   out_6446961619798279423[177] = 0.0;
   out_6446961619798279423[178] = 0.0;
   out_6446961619798279423[179] = 0.0;
   out_6446961619798279423[180] = 0.0;
   out_6446961619798279423[181] = 0.0;
   out_6446961619798279423[182] = 0.0;
   out_6446961619798279423[183] = 0.0;
   out_6446961619798279423[184] = 0.0;
   out_6446961619798279423[185] = 0.0;
   out_6446961619798279423[186] = 0.0;
   out_6446961619798279423[187] = 0.0;
   out_6446961619798279423[188] = 0.0;
   out_6446961619798279423[189] = 0.0;
   out_6446961619798279423[190] = 1.0;
   out_6446961619798279423[191] = 0.0;
   out_6446961619798279423[192] = 0.0;
   out_6446961619798279423[193] = 0.0;
   out_6446961619798279423[194] = 0.0;
   out_6446961619798279423[195] = 0.0;
   out_6446961619798279423[196] = 0.0;
   out_6446961619798279423[197] = 0.0;
   out_6446961619798279423[198] = 0.0;
   out_6446961619798279423[199] = 0.0;
   out_6446961619798279423[200] = 0.0;
   out_6446961619798279423[201] = 0.0;
   out_6446961619798279423[202] = 0.0;
   out_6446961619798279423[203] = 0.0;
   out_6446961619798279423[204] = 0.0;
   out_6446961619798279423[205] = 0.0;
   out_6446961619798279423[206] = 0.0;
   out_6446961619798279423[207] = 0.0;
   out_6446961619798279423[208] = 0.0;
   out_6446961619798279423[209] = 1.0;
   out_6446961619798279423[210] = 0.0;
   out_6446961619798279423[211] = 0.0;
   out_6446961619798279423[212] = 0.0;
   out_6446961619798279423[213] = 0.0;
   out_6446961619798279423[214] = 0.0;
   out_6446961619798279423[215] = 0.0;
   out_6446961619798279423[216] = 0.0;
   out_6446961619798279423[217] = 0.0;
   out_6446961619798279423[218] = 0.0;
   out_6446961619798279423[219] = 0.0;
   out_6446961619798279423[220] = 0.0;
   out_6446961619798279423[221] = 0.0;
   out_6446961619798279423[222] = 0.0;
   out_6446961619798279423[223] = 0.0;
   out_6446961619798279423[224] = 0.0;
   out_6446961619798279423[225] = 0.0;
   out_6446961619798279423[226] = 0.0;
   out_6446961619798279423[227] = 0.0;
   out_6446961619798279423[228] = 1.0;
   out_6446961619798279423[229] = 0.0;
   out_6446961619798279423[230] = 0.0;
   out_6446961619798279423[231] = 0.0;
   out_6446961619798279423[232] = 0.0;
   out_6446961619798279423[233] = 0.0;
   out_6446961619798279423[234] = 0.0;
   out_6446961619798279423[235] = 0.0;
   out_6446961619798279423[236] = 0.0;
   out_6446961619798279423[237] = 0.0;
   out_6446961619798279423[238] = 0.0;
   out_6446961619798279423[239] = 0.0;
   out_6446961619798279423[240] = 0.0;
   out_6446961619798279423[241] = 0.0;
   out_6446961619798279423[242] = 0.0;
   out_6446961619798279423[243] = 0.0;
   out_6446961619798279423[244] = 0.0;
   out_6446961619798279423[245] = 0.0;
   out_6446961619798279423[246] = 0.0;
   out_6446961619798279423[247] = 1.0;
   out_6446961619798279423[248] = 0.0;
   out_6446961619798279423[249] = 0.0;
   out_6446961619798279423[250] = 0.0;
   out_6446961619798279423[251] = 0.0;
   out_6446961619798279423[252] = 0.0;
   out_6446961619798279423[253] = 0.0;
   out_6446961619798279423[254] = 0.0;
   out_6446961619798279423[255] = 0.0;
   out_6446961619798279423[256] = 0.0;
   out_6446961619798279423[257] = 0.0;
   out_6446961619798279423[258] = 0.0;
   out_6446961619798279423[259] = 0.0;
   out_6446961619798279423[260] = 0.0;
   out_6446961619798279423[261] = 0.0;
   out_6446961619798279423[262] = 0.0;
   out_6446961619798279423[263] = 0.0;
   out_6446961619798279423[264] = 0.0;
   out_6446961619798279423[265] = 0.0;
   out_6446961619798279423[266] = 1.0;
   out_6446961619798279423[267] = 0.0;
   out_6446961619798279423[268] = 0.0;
   out_6446961619798279423[269] = 0.0;
   out_6446961619798279423[270] = 0.0;
   out_6446961619798279423[271] = 0.0;
   out_6446961619798279423[272] = 0.0;
   out_6446961619798279423[273] = 0.0;
   out_6446961619798279423[274] = 0.0;
   out_6446961619798279423[275] = 0.0;
   out_6446961619798279423[276] = 0.0;
   out_6446961619798279423[277] = 0.0;
   out_6446961619798279423[278] = 0.0;
   out_6446961619798279423[279] = 0.0;
   out_6446961619798279423[280] = 0.0;
   out_6446961619798279423[281] = 0.0;
   out_6446961619798279423[282] = 0.0;
   out_6446961619798279423[283] = 0.0;
   out_6446961619798279423[284] = 0.0;
   out_6446961619798279423[285] = 1.0;
   out_6446961619798279423[286] = 0.0;
   out_6446961619798279423[287] = 0.0;
   out_6446961619798279423[288] = 0.0;
   out_6446961619798279423[289] = 0.0;
   out_6446961619798279423[290] = 0.0;
   out_6446961619798279423[291] = 0.0;
   out_6446961619798279423[292] = 0.0;
   out_6446961619798279423[293] = 0.0;
   out_6446961619798279423[294] = 0.0;
   out_6446961619798279423[295] = 0.0;
   out_6446961619798279423[296] = 0.0;
   out_6446961619798279423[297] = 0.0;
   out_6446961619798279423[298] = 0.0;
   out_6446961619798279423[299] = 0.0;
   out_6446961619798279423[300] = 0.0;
   out_6446961619798279423[301] = 0.0;
   out_6446961619798279423[302] = 0.0;
   out_6446961619798279423[303] = 0.0;
   out_6446961619798279423[304] = 1.0;
   out_6446961619798279423[305] = 0.0;
   out_6446961619798279423[306] = 0.0;
   out_6446961619798279423[307] = 0.0;
   out_6446961619798279423[308] = 0.0;
   out_6446961619798279423[309] = 0.0;
   out_6446961619798279423[310] = 0.0;
   out_6446961619798279423[311] = 0.0;
   out_6446961619798279423[312] = 0.0;
   out_6446961619798279423[313] = 0.0;
   out_6446961619798279423[314] = 0.0;
   out_6446961619798279423[315] = 0.0;
   out_6446961619798279423[316] = 0.0;
   out_6446961619798279423[317] = 0.0;
   out_6446961619798279423[318] = 0.0;
   out_6446961619798279423[319] = 0.0;
   out_6446961619798279423[320] = 0.0;
   out_6446961619798279423[321] = 0.0;
   out_6446961619798279423[322] = 0.0;
   out_6446961619798279423[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_2638906143691493058) {
   out_2638906143691493058[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_2638906143691493058[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_2638906143691493058[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_2638906143691493058[3] = dt*state[12] + state[3];
   out_2638906143691493058[4] = dt*state[13] + state[4];
   out_2638906143691493058[5] = dt*state[14] + state[5];
   out_2638906143691493058[6] = state[6];
   out_2638906143691493058[7] = state[7];
   out_2638906143691493058[8] = state[8];
   out_2638906143691493058[9] = state[9];
   out_2638906143691493058[10] = state[10];
   out_2638906143691493058[11] = state[11];
   out_2638906143691493058[12] = state[12];
   out_2638906143691493058[13] = state[13];
   out_2638906143691493058[14] = state[14];
   out_2638906143691493058[15] = state[15];
   out_2638906143691493058[16] = state[16];
   out_2638906143691493058[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7572280307579006844) {
   out_7572280307579006844[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7572280307579006844[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7572280307579006844[2] = 0;
   out_7572280307579006844[3] = 0;
   out_7572280307579006844[4] = 0;
   out_7572280307579006844[5] = 0;
   out_7572280307579006844[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7572280307579006844[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7572280307579006844[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7572280307579006844[9] = 0;
   out_7572280307579006844[10] = 0;
   out_7572280307579006844[11] = 0;
   out_7572280307579006844[12] = 0;
   out_7572280307579006844[13] = 0;
   out_7572280307579006844[14] = 0;
   out_7572280307579006844[15] = 0;
   out_7572280307579006844[16] = 0;
   out_7572280307579006844[17] = 0;
   out_7572280307579006844[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7572280307579006844[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7572280307579006844[20] = 0;
   out_7572280307579006844[21] = 0;
   out_7572280307579006844[22] = 0;
   out_7572280307579006844[23] = 0;
   out_7572280307579006844[24] = 0;
   out_7572280307579006844[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7572280307579006844[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7572280307579006844[27] = 0;
   out_7572280307579006844[28] = 0;
   out_7572280307579006844[29] = 0;
   out_7572280307579006844[30] = 0;
   out_7572280307579006844[31] = 0;
   out_7572280307579006844[32] = 0;
   out_7572280307579006844[33] = 0;
   out_7572280307579006844[34] = 0;
   out_7572280307579006844[35] = 0;
   out_7572280307579006844[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7572280307579006844[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7572280307579006844[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7572280307579006844[39] = 0;
   out_7572280307579006844[40] = 0;
   out_7572280307579006844[41] = 0;
   out_7572280307579006844[42] = 0;
   out_7572280307579006844[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7572280307579006844[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7572280307579006844[45] = 0;
   out_7572280307579006844[46] = 0;
   out_7572280307579006844[47] = 0;
   out_7572280307579006844[48] = 0;
   out_7572280307579006844[49] = 0;
   out_7572280307579006844[50] = 0;
   out_7572280307579006844[51] = 0;
   out_7572280307579006844[52] = 0;
   out_7572280307579006844[53] = 0;
   out_7572280307579006844[54] = 0;
   out_7572280307579006844[55] = 0;
   out_7572280307579006844[56] = 0;
   out_7572280307579006844[57] = 1;
   out_7572280307579006844[58] = 0;
   out_7572280307579006844[59] = 0;
   out_7572280307579006844[60] = 0;
   out_7572280307579006844[61] = 0;
   out_7572280307579006844[62] = 0;
   out_7572280307579006844[63] = 0;
   out_7572280307579006844[64] = 0;
   out_7572280307579006844[65] = 0;
   out_7572280307579006844[66] = dt;
   out_7572280307579006844[67] = 0;
   out_7572280307579006844[68] = 0;
   out_7572280307579006844[69] = 0;
   out_7572280307579006844[70] = 0;
   out_7572280307579006844[71] = 0;
   out_7572280307579006844[72] = 0;
   out_7572280307579006844[73] = 0;
   out_7572280307579006844[74] = 0;
   out_7572280307579006844[75] = 0;
   out_7572280307579006844[76] = 1;
   out_7572280307579006844[77] = 0;
   out_7572280307579006844[78] = 0;
   out_7572280307579006844[79] = 0;
   out_7572280307579006844[80] = 0;
   out_7572280307579006844[81] = 0;
   out_7572280307579006844[82] = 0;
   out_7572280307579006844[83] = 0;
   out_7572280307579006844[84] = 0;
   out_7572280307579006844[85] = dt;
   out_7572280307579006844[86] = 0;
   out_7572280307579006844[87] = 0;
   out_7572280307579006844[88] = 0;
   out_7572280307579006844[89] = 0;
   out_7572280307579006844[90] = 0;
   out_7572280307579006844[91] = 0;
   out_7572280307579006844[92] = 0;
   out_7572280307579006844[93] = 0;
   out_7572280307579006844[94] = 0;
   out_7572280307579006844[95] = 1;
   out_7572280307579006844[96] = 0;
   out_7572280307579006844[97] = 0;
   out_7572280307579006844[98] = 0;
   out_7572280307579006844[99] = 0;
   out_7572280307579006844[100] = 0;
   out_7572280307579006844[101] = 0;
   out_7572280307579006844[102] = 0;
   out_7572280307579006844[103] = 0;
   out_7572280307579006844[104] = dt;
   out_7572280307579006844[105] = 0;
   out_7572280307579006844[106] = 0;
   out_7572280307579006844[107] = 0;
   out_7572280307579006844[108] = 0;
   out_7572280307579006844[109] = 0;
   out_7572280307579006844[110] = 0;
   out_7572280307579006844[111] = 0;
   out_7572280307579006844[112] = 0;
   out_7572280307579006844[113] = 0;
   out_7572280307579006844[114] = 1;
   out_7572280307579006844[115] = 0;
   out_7572280307579006844[116] = 0;
   out_7572280307579006844[117] = 0;
   out_7572280307579006844[118] = 0;
   out_7572280307579006844[119] = 0;
   out_7572280307579006844[120] = 0;
   out_7572280307579006844[121] = 0;
   out_7572280307579006844[122] = 0;
   out_7572280307579006844[123] = 0;
   out_7572280307579006844[124] = 0;
   out_7572280307579006844[125] = 0;
   out_7572280307579006844[126] = 0;
   out_7572280307579006844[127] = 0;
   out_7572280307579006844[128] = 0;
   out_7572280307579006844[129] = 0;
   out_7572280307579006844[130] = 0;
   out_7572280307579006844[131] = 0;
   out_7572280307579006844[132] = 0;
   out_7572280307579006844[133] = 1;
   out_7572280307579006844[134] = 0;
   out_7572280307579006844[135] = 0;
   out_7572280307579006844[136] = 0;
   out_7572280307579006844[137] = 0;
   out_7572280307579006844[138] = 0;
   out_7572280307579006844[139] = 0;
   out_7572280307579006844[140] = 0;
   out_7572280307579006844[141] = 0;
   out_7572280307579006844[142] = 0;
   out_7572280307579006844[143] = 0;
   out_7572280307579006844[144] = 0;
   out_7572280307579006844[145] = 0;
   out_7572280307579006844[146] = 0;
   out_7572280307579006844[147] = 0;
   out_7572280307579006844[148] = 0;
   out_7572280307579006844[149] = 0;
   out_7572280307579006844[150] = 0;
   out_7572280307579006844[151] = 0;
   out_7572280307579006844[152] = 1;
   out_7572280307579006844[153] = 0;
   out_7572280307579006844[154] = 0;
   out_7572280307579006844[155] = 0;
   out_7572280307579006844[156] = 0;
   out_7572280307579006844[157] = 0;
   out_7572280307579006844[158] = 0;
   out_7572280307579006844[159] = 0;
   out_7572280307579006844[160] = 0;
   out_7572280307579006844[161] = 0;
   out_7572280307579006844[162] = 0;
   out_7572280307579006844[163] = 0;
   out_7572280307579006844[164] = 0;
   out_7572280307579006844[165] = 0;
   out_7572280307579006844[166] = 0;
   out_7572280307579006844[167] = 0;
   out_7572280307579006844[168] = 0;
   out_7572280307579006844[169] = 0;
   out_7572280307579006844[170] = 0;
   out_7572280307579006844[171] = 1;
   out_7572280307579006844[172] = 0;
   out_7572280307579006844[173] = 0;
   out_7572280307579006844[174] = 0;
   out_7572280307579006844[175] = 0;
   out_7572280307579006844[176] = 0;
   out_7572280307579006844[177] = 0;
   out_7572280307579006844[178] = 0;
   out_7572280307579006844[179] = 0;
   out_7572280307579006844[180] = 0;
   out_7572280307579006844[181] = 0;
   out_7572280307579006844[182] = 0;
   out_7572280307579006844[183] = 0;
   out_7572280307579006844[184] = 0;
   out_7572280307579006844[185] = 0;
   out_7572280307579006844[186] = 0;
   out_7572280307579006844[187] = 0;
   out_7572280307579006844[188] = 0;
   out_7572280307579006844[189] = 0;
   out_7572280307579006844[190] = 1;
   out_7572280307579006844[191] = 0;
   out_7572280307579006844[192] = 0;
   out_7572280307579006844[193] = 0;
   out_7572280307579006844[194] = 0;
   out_7572280307579006844[195] = 0;
   out_7572280307579006844[196] = 0;
   out_7572280307579006844[197] = 0;
   out_7572280307579006844[198] = 0;
   out_7572280307579006844[199] = 0;
   out_7572280307579006844[200] = 0;
   out_7572280307579006844[201] = 0;
   out_7572280307579006844[202] = 0;
   out_7572280307579006844[203] = 0;
   out_7572280307579006844[204] = 0;
   out_7572280307579006844[205] = 0;
   out_7572280307579006844[206] = 0;
   out_7572280307579006844[207] = 0;
   out_7572280307579006844[208] = 0;
   out_7572280307579006844[209] = 1;
   out_7572280307579006844[210] = 0;
   out_7572280307579006844[211] = 0;
   out_7572280307579006844[212] = 0;
   out_7572280307579006844[213] = 0;
   out_7572280307579006844[214] = 0;
   out_7572280307579006844[215] = 0;
   out_7572280307579006844[216] = 0;
   out_7572280307579006844[217] = 0;
   out_7572280307579006844[218] = 0;
   out_7572280307579006844[219] = 0;
   out_7572280307579006844[220] = 0;
   out_7572280307579006844[221] = 0;
   out_7572280307579006844[222] = 0;
   out_7572280307579006844[223] = 0;
   out_7572280307579006844[224] = 0;
   out_7572280307579006844[225] = 0;
   out_7572280307579006844[226] = 0;
   out_7572280307579006844[227] = 0;
   out_7572280307579006844[228] = 1;
   out_7572280307579006844[229] = 0;
   out_7572280307579006844[230] = 0;
   out_7572280307579006844[231] = 0;
   out_7572280307579006844[232] = 0;
   out_7572280307579006844[233] = 0;
   out_7572280307579006844[234] = 0;
   out_7572280307579006844[235] = 0;
   out_7572280307579006844[236] = 0;
   out_7572280307579006844[237] = 0;
   out_7572280307579006844[238] = 0;
   out_7572280307579006844[239] = 0;
   out_7572280307579006844[240] = 0;
   out_7572280307579006844[241] = 0;
   out_7572280307579006844[242] = 0;
   out_7572280307579006844[243] = 0;
   out_7572280307579006844[244] = 0;
   out_7572280307579006844[245] = 0;
   out_7572280307579006844[246] = 0;
   out_7572280307579006844[247] = 1;
   out_7572280307579006844[248] = 0;
   out_7572280307579006844[249] = 0;
   out_7572280307579006844[250] = 0;
   out_7572280307579006844[251] = 0;
   out_7572280307579006844[252] = 0;
   out_7572280307579006844[253] = 0;
   out_7572280307579006844[254] = 0;
   out_7572280307579006844[255] = 0;
   out_7572280307579006844[256] = 0;
   out_7572280307579006844[257] = 0;
   out_7572280307579006844[258] = 0;
   out_7572280307579006844[259] = 0;
   out_7572280307579006844[260] = 0;
   out_7572280307579006844[261] = 0;
   out_7572280307579006844[262] = 0;
   out_7572280307579006844[263] = 0;
   out_7572280307579006844[264] = 0;
   out_7572280307579006844[265] = 0;
   out_7572280307579006844[266] = 1;
   out_7572280307579006844[267] = 0;
   out_7572280307579006844[268] = 0;
   out_7572280307579006844[269] = 0;
   out_7572280307579006844[270] = 0;
   out_7572280307579006844[271] = 0;
   out_7572280307579006844[272] = 0;
   out_7572280307579006844[273] = 0;
   out_7572280307579006844[274] = 0;
   out_7572280307579006844[275] = 0;
   out_7572280307579006844[276] = 0;
   out_7572280307579006844[277] = 0;
   out_7572280307579006844[278] = 0;
   out_7572280307579006844[279] = 0;
   out_7572280307579006844[280] = 0;
   out_7572280307579006844[281] = 0;
   out_7572280307579006844[282] = 0;
   out_7572280307579006844[283] = 0;
   out_7572280307579006844[284] = 0;
   out_7572280307579006844[285] = 1;
   out_7572280307579006844[286] = 0;
   out_7572280307579006844[287] = 0;
   out_7572280307579006844[288] = 0;
   out_7572280307579006844[289] = 0;
   out_7572280307579006844[290] = 0;
   out_7572280307579006844[291] = 0;
   out_7572280307579006844[292] = 0;
   out_7572280307579006844[293] = 0;
   out_7572280307579006844[294] = 0;
   out_7572280307579006844[295] = 0;
   out_7572280307579006844[296] = 0;
   out_7572280307579006844[297] = 0;
   out_7572280307579006844[298] = 0;
   out_7572280307579006844[299] = 0;
   out_7572280307579006844[300] = 0;
   out_7572280307579006844[301] = 0;
   out_7572280307579006844[302] = 0;
   out_7572280307579006844[303] = 0;
   out_7572280307579006844[304] = 1;
   out_7572280307579006844[305] = 0;
   out_7572280307579006844[306] = 0;
   out_7572280307579006844[307] = 0;
   out_7572280307579006844[308] = 0;
   out_7572280307579006844[309] = 0;
   out_7572280307579006844[310] = 0;
   out_7572280307579006844[311] = 0;
   out_7572280307579006844[312] = 0;
   out_7572280307579006844[313] = 0;
   out_7572280307579006844[314] = 0;
   out_7572280307579006844[315] = 0;
   out_7572280307579006844[316] = 0;
   out_7572280307579006844[317] = 0;
   out_7572280307579006844[318] = 0;
   out_7572280307579006844[319] = 0;
   out_7572280307579006844[320] = 0;
   out_7572280307579006844[321] = 0;
   out_7572280307579006844[322] = 0;
   out_7572280307579006844[323] = 1;
}
void h_4(double *state, double *unused, double *out_8190519028499974415) {
   out_8190519028499974415[0] = state[6] + state[9];
   out_8190519028499974415[1] = state[7] + state[10];
   out_8190519028499974415[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_6847264068872191458) {
   out_6847264068872191458[0] = 0;
   out_6847264068872191458[1] = 0;
   out_6847264068872191458[2] = 0;
   out_6847264068872191458[3] = 0;
   out_6847264068872191458[4] = 0;
   out_6847264068872191458[5] = 0;
   out_6847264068872191458[6] = 1;
   out_6847264068872191458[7] = 0;
   out_6847264068872191458[8] = 0;
   out_6847264068872191458[9] = 1;
   out_6847264068872191458[10] = 0;
   out_6847264068872191458[11] = 0;
   out_6847264068872191458[12] = 0;
   out_6847264068872191458[13] = 0;
   out_6847264068872191458[14] = 0;
   out_6847264068872191458[15] = 0;
   out_6847264068872191458[16] = 0;
   out_6847264068872191458[17] = 0;
   out_6847264068872191458[18] = 0;
   out_6847264068872191458[19] = 0;
   out_6847264068872191458[20] = 0;
   out_6847264068872191458[21] = 0;
   out_6847264068872191458[22] = 0;
   out_6847264068872191458[23] = 0;
   out_6847264068872191458[24] = 0;
   out_6847264068872191458[25] = 1;
   out_6847264068872191458[26] = 0;
   out_6847264068872191458[27] = 0;
   out_6847264068872191458[28] = 1;
   out_6847264068872191458[29] = 0;
   out_6847264068872191458[30] = 0;
   out_6847264068872191458[31] = 0;
   out_6847264068872191458[32] = 0;
   out_6847264068872191458[33] = 0;
   out_6847264068872191458[34] = 0;
   out_6847264068872191458[35] = 0;
   out_6847264068872191458[36] = 0;
   out_6847264068872191458[37] = 0;
   out_6847264068872191458[38] = 0;
   out_6847264068872191458[39] = 0;
   out_6847264068872191458[40] = 0;
   out_6847264068872191458[41] = 0;
   out_6847264068872191458[42] = 0;
   out_6847264068872191458[43] = 0;
   out_6847264068872191458[44] = 1;
   out_6847264068872191458[45] = 0;
   out_6847264068872191458[46] = 0;
   out_6847264068872191458[47] = 1;
   out_6847264068872191458[48] = 0;
   out_6847264068872191458[49] = 0;
   out_6847264068872191458[50] = 0;
   out_6847264068872191458[51] = 0;
   out_6847264068872191458[52] = 0;
   out_6847264068872191458[53] = 0;
}
void h_10(double *state, double *unused, double *out_5274599112103372538) {
   out_5274599112103372538[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5274599112103372538[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5274599112103372538[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_2523400390457892172) {
   out_2523400390457892172[0] = 0;
   out_2523400390457892172[1] = 9.8100000000000005*cos(state[1]);
   out_2523400390457892172[2] = 0;
   out_2523400390457892172[3] = 0;
   out_2523400390457892172[4] = -state[8];
   out_2523400390457892172[5] = state[7];
   out_2523400390457892172[6] = 0;
   out_2523400390457892172[7] = state[5];
   out_2523400390457892172[8] = -state[4];
   out_2523400390457892172[9] = 0;
   out_2523400390457892172[10] = 0;
   out_2523400390457892172[11] = 0;
   out_2523400390457892172[12] = 1;
   out_2523400390457892172[13] = 0;
   out_2523400390457892172[14] = 0;
   out_2523400390457892172[15] = 1;
   out_2523400390457892172[16] = 0;
   out_2523400390457892172[17] = 0;
   out_2523400390457892172[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_2523400390457892172[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_2523400390457892172[20] = 0;
   out_2523400390457892172[21] = state[8];
   out_2523400390457892172[22] = 0;
   out_2523400390457892172[23] = -state[6];
   out_2523400390457892172[24] = -state[5];
   out_2523400390457892172[25] = 0;
   out_2523400390457892172[26] = state[3];
   out_2523400390457892172[27] = 0;
   out_2523400390457892172[28] = 0;
   out_2523400390457892172[29] = 0;
   out_2523400390457892172[30] = 0;
   out_2523400390457892172[31] = 1;
   out_2523400390457892172[32] = 0;
   out_2523400390457892172[33] = 0;
   out_2523400390457892172[34] = 1;
   out_2523400390457892172[35] = 0;
   out_2523400390457892172[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_2523400390457892172[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_2523400390457892172[38] = 0;
   out_2523400390457892172[39] = -state[7];
   out_2523400390457892172[40] = state[6];
   out_2523400390457892172[41] = 0;
   out_2523400390457892172[42] = state[4];
   out_2523400390457892172[43] = -state[3];
   out_2523400390457892172[44] = 0;
   out_2523400390457892172[45] = 0;
   out_2523400390457892172[46] = 0;
   out_2523400390457892172[47] = 0;
   out_2523400390457892172[48] = 0;
   out_2523400390457892172[49] = 0;
   out_2523400390457892172[50] = 1;
   out_2523400390457892172[51] = 0;
   out_2523400390457892172[52] = 0;
   out_2523400390457892172[53] = 1;
}
void h_13(double *state, double *unused, double *out_8352704181929443466) {
   out_8352704181929443466[0] = state[3];
   out_8352704181929443466[1] = state[4];
   out_8352704181929443466[2] = state[5];
}
void H_13(double *state, double *unused, double *out_7765724541534836134) {
   out_7765724541534836134[0] = 0;
   out_7765724541534836134[1] = 0;
   out_7765724541534836134[2] = 0;
   out_7765724541534836134[3] = 1;
   out_7765724541534836134[4] = 0;
   out_7765724541534836134[5] = 0;
   out_7765724541534836134[6] = 0;
   out_7765724541534836134[7] = 0;
   out_7765724541534836134[8] = 0;
   out_7765724541534836134[9] = 0;
   out_7765724541534836134[10] = 0;
   out_7765724541534836134[11] = 0;
   out_7765724541534836134[12] = 0;
   out_7765724541534836134[13] = 0;
   out_7765724541534836134[14] = 0;
   out_7765724541534836134[15] = 0;
   out_7765724541534836134[16] = 0;
   out_7765724541534836134[17] = 0;
   out_7765724541534836134[18] = 0;
   out_7765724541534836134[19] = 0;
   out_7765724541534836134[20] = 0;
   out_7765724541534836134[21] = 0;
   out_7765724541534836134[22] = 1;
   out_7765724541534836134[23] = 0;
   out_7765724541534836134[24] = 0;
   out_7765724541534836134[25] = 0;
   out_7765724541534836134[26] = 0;
   out_7765724541534836134[27] = 0;
   out_7765724541534836134[28] = 0;
   out_7765724541534836134[29] = 0;
   out_7765724541534836134[30] = 0;
   out_7765724541534836134[31] = 0;
   out_7765724541534836134[32] = 0;
   out_7765724541534836134[33] = 0;
   out_7765724541534836134[34] = 0;
   out_7765724541534836134[35] = 0;
   out_7765724541534836134[36] = 0;
   out_7765724541534836134[37] = 0;
   out_7765724541534836134[38] = 0;
   out_7765724541534836134[39] = 0;
   out_7765724541534836134[40] = 0;
   out_7765724541534836134[41] = 1;
   out_7765724541534836134[42] = 0;
   out_7765724541534836134[43] = 0;
   out_7765724541534836134[44] = 0;
   out_7765724541534836134[45] = 0;
   out_7765724541534836134[46] = 0;
   out_7765724541534836134[47] = 0;
   out_7765724541534836134[48] = 0;
   out_7765724541534836134[49] = 0;
   out_7765724541534836134[50] = 0;
   out_7765724541534836134[51] = 0;
   out_7765724541534836134[52] = 0;
   out_7765724541534836134[53] = 0;
}
void h_14(double *state, double *unused, double *out_8386492674217883514) {
   out_8386492674217883514[0] = state[6];
   out_8386492674217883514[1] = state[7];
   out_8386492674217883514[2] = state[8];
}
void H_14(double *state, double *unused, double *out_8516691572541987862) {
   out_8516691572541987862[0] = 0;
   out_8516691572541987862[1] = 0;
   out_8516691572541987862[2] = 0;
   out_8516691572541987862[3] = 0;
   out_8516691572541987862[4] = 0;
   out_8516691572541987862[5] = 0;
   out_8516691572541987862[6] = 1;
   out_8516691572541987862[7] = 0;
   out_8516691572541987862[8] = 0;
   out_8516691572541987862[9] = 0;
   out_8516691572541987862[10] = 0;
   out_8516691572541987862[11] = 0;
   out_8516691572541987862[12] = 0;
   out_8516691572541987862[13] = 0;
   out_8516691572541987862[14] = 0;
   out_8516691572541987862[15] = 0;
   out_8516691572541987862[16] = 0;
   out_8516691572541987862[17] = 0;
   out_8516691572541987862[18] = 0;
   out_8516691572541987862[19] = 0;
   out_8516691572541987862[20] = 0;
   out_8516691572541987862[21] = 0;
   out_8516691572541987862[22] = 0;
   out_8516691572541987862[23] = 0;
   out_8516691572541987862[24] = 0;
   out_8516691572541987862[25] = 1;
   out_8516691572541987862[26] = 0;
   out_8516691572541987862[27] = 0;
   out_8516691572541987862[28] = 0;
   out_8516691572541987862[29] = 0;
   out_8516691572541987862[30] = 0;
   out_8516691572541987862[31] = 0;
   out_8516691572541987862[32] = 0;
   out_8516691572541987862[33] = 0;
   out_8516691572541987862[34] = 0;
   out_8516691572541987862[35] = 0;
   out_8516691572541987862[36] = 0;
   out_8516691572541987862[37] = 0;
   out_8516691572541987862[38] = 0;
   out_8516691572541987862[39] = 0;
   out_8516691572541987862[40] = 0;
   out_8516691572541987862[41] = 0;
   out_8516691572541987862[42] = 0;
   out_8516691572541987862[43] = 0;
   out_8516691572541987862[44] = 1;
   out_8516691572541987862[45] = 0;
   out_8516691572541987862[46] = 0;
   out_8516691572541987862[47] = 0;
   out_8516691572541987862[48] = 0;
   out_8516691572541987862[49] = 0;
   out_8516691572541987862[50] = 0;
   out_8516691572541987862[51] = 0;
   out_8516691572541987862[52] = 0;
   out_8516691572541987862[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_3180036843001874960) {
  err_fun(nom_x, delta_x, out_3180036843001874960);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7632041844693148483) {
  inv_err_fun(nom_x, true_x, out_7632041844693148483);
}
void pose_H_mod_fun(double *state, double *out_6446961619798279423) {
  H_mod_fun(state, out_6446961619798279423);
}
void pose_f_fun(double *state, double dt, double *out_2638906143691493058) {
  f_fun(state,  dt, out_2638906143691493058);
}
void pose_F_fun(double *state, double dt, double *out_7572280307579006844) {
  F_fun(state,  dt, out_7572280307579006844);
}
void pose_h_4(double *state, double *unused, double *out_8190519028499974415) {
  h_4(state, unused, out_8190519028499974415);
}
void pose_H_4(double *state, double *unused, double *out_6847264068872191458) {
  H_4(state, unused, out_6847264068872191458);
}
void pose_h_10(double *state, double *unused, double *out_5274599112103372538) {
  h_10(state, unused, out_5274599112103372538);
}
void pose_H_10(double *state, double *unused, double *out_2523400390457892172) {
  H_10(state, unused, out_2523400390457892172);
}
void pose_h_13(double *state, double *unused, double *out_8352704181929443466) {
  h_13(state, unused, out_8352704181929443466);
}
void pose_H_13(double *state, double *unused, double *out_7765724541534836134) {
  H_13(state, unused, out_7765724541534836134);
}
void pose_h_14(double *state, double *unused, double *out_8386492674217883514) {
  h_14(state, unused, out_8386492674217883514);
}
void pose_H_14(double *state, double *unused, double *out_8516691572541987862) {
  H_14(state, unused, out_8516691572541987862);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
