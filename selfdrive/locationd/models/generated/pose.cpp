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
 *                       Code generated with SymPy 1.12                       *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7983792447504184197) {
   out_7983792447504184197[0] = delta_x[0] + nom_x[0];
   out_7983792447504184197[1] = delta_x[1] + nom_x[1];
   out_7983792447504184197[2] = delta_x[2] + nom_x[2];
   out_7983792447504184197[3] = delta_x[3] + nom_x[3];
   out_7983792447504184197[4] = delta_x[4] + nom_x[4];
   out_7983792447504184197[5] = delta_x[5] + nom_x[5];
   out_7983792447504184197[6] = delta_x[6] + nom_x[6];
   out_7983792447504184197[7] = delta_x[7] + nom_x[7];
   out_7983792447504184197[8] = delta_x[8] + nom_x[8];
   out_7983792447504184197[9] = delta_x[9] + nom_x[9];
   out_7983792447504184197[10] = delta_x[10] + nom_x[10];
   out_7983792447504184197[11] = delta_x[11] + nom_x[11];
   out_7983792447504184197[12] = delta_x[12] + nom_x[12];
   out_7983792447504184197[13] = delta_x[13] + nom_x[13];
   out_7983792447504184197[14] = delta_x[14] + nom_x[14];
   out_7983792447504184197[15] = delta_x[15] + nom_x[15];
   out_7983792447504184197[16] = delta_x[16] + nom_x[16];
   out_7983792447504184197[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5020775487918234326) {
   out_5020775487918234326[0] = -nom_x[0] + true_x[0];
   out_5020775487918234326[1] = -nom_x[1] + true_x[1];
   out_5020775487918234326[2] = -nom_x[2] + true_x[2];
   out_5020775487918234326[3] = -nom_x[3] + true_x[3];
   out_5020775487918234326[4] = -nom_x[4] + true_x[4];
   out_5020775487918234326[5] = -nom_x[5] + true_x[5];
   out_5020775487918234326[6] = -nom_x[6] + true_x[6];
   out_5020775487918234326[7] = -nom_x[7] + true_x[7];
   out_5020775487918234326[8] = -nom_x[8] + true_x[8];
   out_5020775487918234326[9] = -nom_x[9] + true_x[9];
   out_5020775487918234326[10] = -nom_x[10] + true_x[10];
   out_5020775487918234326[11] = -nom_x[11] + true_x[11];
   out_5020775487918234326[12] = -nom_x[12] + true_x[12];
   out_5020775487918234326[13] = -nom_x[13] + true_x[13];
   out_5020775487918234326[14] = -nom_x[14] + true_x[14];
   out_5020775487918234326[15] = -nom_x[15] + true_x[15];
   out_5020775487918234326[16] = -nom_x[16] + true_x[16];
   out_5020775487918234326[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_2023537857534306321) {
   out_2023537857534306321[0] = 1.0;
   out_2023537857534306321[1] = 0;
   out_2023537857534306321[2] = 0;
   out_2023537857534306321[3] = 0;
   out_2023537857534306321[4] = 0;
   out_2023537857534306321[5] = 0;
   out_2023537857534306321[6] = 0;
   out_2023537857534306321[7] = 0;
   out_2023537857534306321[8] = 0;
   out_2023537857534306321[9] = 0;
   out_2023537857534306321[10] = 0;
   out_2023537857534306321[11] = 0;
   out_2023537857534306321[12] = 0;
   out_2023537857534306321[13] = 0;
   out_2023537857534306321[14] = 0;
   out_2023537857534306321[15] = 0;
   out_2023537857534306321[16] = 0;
   out_2023537857534306321[17] = 0;
   out_2023537857534306321[18] = 0;
   out_2023537857534306321[19] = 1.0;
   out_2023537857534306321[20] = 0;
   out_2023537857534306321[21] = 0;
   out_2023537857534306321[22] = 0;
   out_2023537857534306321[23] = 0;
   out_2023537857534306321[24] = 0;
   out_2023537857534306321[25] = 0;
   out_2023537857534306321[26] = 0;
   out_2023537857534306321[27] = 0;
   out_2023537857534306321[28] = 0;
   out_2023537857534306321[29] = 0;
   out_2023537857534306321[30] = 0;
   out_2023537857534306321[31] = 0;
   out_2023537857534306321[32] = 0;
   out_2023537857534306321[33] = 0;
   out_2023537857534306321[34] = 0;
   out_2023537857534306321[35] = 0;
   out_2023537857534306321[36] = 0;
   out_2023537857534306321[37] = 0;
   out_2023537857534306321[38] = 1.0;
   out_2023537857534306321[39] = 0;
   out_2023537857534306321[40] = 0;
   out_2023537857534306321[41] = 0;
   out_2023537857534306321[42] = 0;
   out_2023537857534306321[43] = 0;
   out_2023537857534306321[44] = 0;
   out_2023537857534306321[45] = 0;
   out_2023537857534306321[46] = 0;
   out_2023537857534306321[47] = 0;
   out_2023537857534306321[48] = 0;
   out_2023537857534306321[49] = 0;
   out_2023537857534306321[50] = 0;
   out_2023537857534306321[51] = 0;
   out_2023537857534306321[52] = 0;
   out_2023537857534306321[53] = 0;
   out_2023537857534306321[54] = 0;
   out_2023537857534306321[55] = 0;
   out_2023537857534306321[56] = 0;
   out_2023537857534306321[57] = 1.0;
   out_2023537857534306321[58] = 0;
   out_2023537857534306321[59] = 0;
   out_2023537857534306321[60] = 0;
   out_2023537857534306321[61] = 0;
   out_2023537857534306321[62] = 0;
   out_2023537857534306321[63] = 0;
   out_2023537857534306321[64] = 0;
   out_2023537857534306321[65] = 0;
   out_2023537857534306321[66] = 0;
   out_2023537857534306321[67] = 0;
   out_2023537857534306321[68] = 0;
   out_2023537857534306321[69] = 0;
   out_2023537857534306321[70] = 0;
   out_2023537857534306321[71] = 0;
   out_2023537857534306321[72] = 0;
   out_2023537857534306321[73] = 0;
   out_2023537857534306321[74] = 0;
   out_2023537857534306321[75] = 0;
   out_2023537857534306321[76] = 1.0;
   out_2023537857534306321[77] = 0;
   out_2023537857534306321[78] = 0;
   out_2023537857534306321[79] = 0;
   out_2023537857534306321[80] = 0;
   out_2023537857534306321[81] = 0;
   out_2023537857534306321[82] = 0;
   out_2023537857534306321[83] = 0;
   out_2023537857534306321[84] = 0;
   out_2023537857534306321[85] = 0;
   out_2023537857534306321[86] = 0;
   out_2023537857534306321[87] = 0;
   out_2023537857534306321[88] = 0;
   out_2023537857534306321[89] = 0;
   out_2023537857534306321[90] = 0;
   out_2023537857534306321[91] = 0;
   out_2023537857534306321[92] = 0;
   out_2023537857534306321[93] = 0;
   out_2023537857534306321[94] = 0;
   out_2023537857534306321[95] = 1.0;
   out_2023537857534306321[96] = 0;
   out_2023537857534306321[97] = 0;
   out_2023537857534306321[98] = 0;
   out_2023537857534306321[99] = 0;
   out_2023537857534306321[100] = 0;
   out_2023537857534306321[101] = 0;
   out_2023537857534306321[102] = 0;
   out_2023537857534306321[103] = 0;
   out_2023537857534306321[104] = 0;
   out_2023537857534306321[105] = 0;
   out_2023537857534306321[106] = 0;
   out_2023537857534306321[107] = 0;
   out_2023537857534306321[108] = 0;
   out_2023537857534306321[109] = 0;
   out_2023537857534306321[110] = 0;
   out_2023537857534306321[111] = 0;
   out_2023537857534306321[112] = 0;
   out_2023537857534306321[113] = 0;
   out_2023537857534306321[114] = 1.0;
   out_2023537857534306321[115] = 0;
   out_2023537857534306321[116] = 0;
   out_2023537857534306321[117] = 0;
   out_2023537857534306321[118] = 0;
   out_2023537857534306321[119] = 0;
   out_2023537857534306321[120] = 0;
   out_2023537857534306321[121] = 0;
   out_2023537857534306321[122] = 0;
   out_2023537857534306321[123] = 0;
   out_2023537857534306321[124] = 0;
   out_2023537857534306321[125] = 0;
   out_2023537857534306321[126] = 0;
   out_2023537857534306321[127] = 0;
   out_2023537857534306321[128] = 0;
   out_2023537857534306321[129] = 0;
   out_2023537857534306321[130] = 0;
   out_2023537857534306321[131] = 0;
   out_2023537857534306321[132] = 0;
   out_2023537857534306321[133] = 1.0;
   out_2023537857534306321[134] = 0;
   out_2023537857534306321[135] = 0;
   out_2023537857534306321[136] = 0;
   out_2023537857534306321[137] = 0;
   out_2023537857534306321[138] = 0;
   out_2023537857534306321[139] = 0;
   out_2023537857534306321[140] = 0;
   out_2023537857534306321[141] = 0;
   out_2023537857534306321[142] = 0;
   out_2023537857534306321[143] = 0;
   out_2023537857534306321[144] = 0;
   out_2023537857534306321[145] = 0;
   out_2023537857534306321[146] = 0;
   out_2023537857534306321[147] = 0;
   out_2023537857534306321[148] = 0;
   out_2023537857534306321[149] = 0;
   out_2023537857534306321[150] = 0;
   out_2023537857534306321[151] = 0;
   out_2023537857534306321[152] = 1.0;
   out_2023537857534306321[153] = 0;
   out_2023537857534306321[154] = 0;
   out_2023537857534306321[155] = 0;
   out_2023537857534306321[156] = 0;
   out_2023537857534306321[157] = 0;
   out_2023537857534306321[158] = 0;
   out_2023537857534306321[159] = 0;
   out_2023537857534306321[160] = 0;
   out_2023537857534306321[161] = 0;
   out_2023537857534306321[162] = 0;
   out_2023537857534306321[163] = 0;
   out_2023537857534306321[164] = 0;
   out_2023537857534306321[165] = 0;
   out_2023537857534306321[166] = 0;
   out_2023537857534306321[167] = 0;
   out_2023537857534306321[168] = 0;
   out_2023537857534306321[169] = 0;
   out_2023537857534306321[170] = 0;
   out_2023537857534306321[171] = 1.0;
   out_2023537857534306321[172] = 0;
   out_2023537857534306321[173] = 0;
   out_2023537857534306321[174] = 0;
   out_2023537857534306321[175] = 0;
   out_2023537857534306321[176] = 0;
   out_2023537857534306321[177] = 0;
   out_2023537857534306321[178] = 0;
   out_2023537857534306321[179] = 0;
   out_2023537857534306321[180] = 0;
   out_2023537857534306321[181] = 0;
   out_2023537857534306321[182] = 0;
   out_2023537857534306321[183] = 0;
   out_2023537857534306321[184] = 0;
   out_2023537857534306321[185] = 0;
   out_2023537857534306321[186] = 0;
   out_2023537857534306321[187] = 0;
   out_2023537857534306321[188] = 0;
   out_2023537857534306321[189] = 0;
   out_2023537857534306321[190] = 1.0;
   out_2023537857534306321[191] = 0;
   out_2023537857534306321[192] = 0;
   out_2023537857534306321[193] = 0;
   out_2023537857534306321[194] = 0;
   out_2023537857534306321[195] = 0;
   out_2023537857534306321[196] = 0;
   out_2023537857534306321[197] = 0;
   out_2023537857534306321[198] = 0;
   out_2023537857534306321[199] = 0;
   out_2023537857534306321[200] = 0;
   out_2023537857534306321[201] = 0;
   out_2023537857534306321[202] = 0;
   out_2023537857534306321[203] = 0;
   out_2023537857534306321[204] = 0;
   out_2023537857534306321[205] = 0;
   out_2023537857534306321[206] = 0;
   out_2023537857534306321[207] = 0;
   out_2023537857534306321[208] = 0;
   out_2023537857534306321[209] = 1.0;
   out_2023537857534306321[210] = 0;
   out_2023537857534306321[211] = 0;
   out_2023537857534306321[212] = 0;
   out_2023537857534306321[213] = 0;
   out_2023537857534306321[214] = 0;
   out_2023537857534306321[215] = 0;
   out_2023537857534306321[216] = 0;
   out_2023537857534306321[217] = 0;
   out_2023537857534306321[218] = 0;
   out_2023537857534306321[219] = 0;
   out_2023537857534306321[220] = 0;
   out_2023537857534306321[221] = 0;
   out_2023537857534306321[222] = 0;
   out_2023537857534306321[223] = 0;
   out_2023537857534306321[224] = 0;
   out_2023537857534306321[225] = 0;
   out_2023537857534306321[226] = 0;
   out_2023537857534306321[227] = 0;
   out_2023537857534306321[228] = 1.0;
   out_2023537857534306321[229] = 0;
   out_2023537857534306321[230] = 0;
   out_2023537857534306321[231] = 0;
   out_2023537857534306321[232] = 0;
   out_2023537857534306321[233] = 0;
   out_2023537857534306321[234] = 0;
   out_2023537857534306321[235] = 0;
   out_2023537857534306321[236] = 0;
   out_2023537857534306321[237] = 0;
   out_2023537857534306321[238] = 0;
   out_2023537857534306321[239] = 0;
   out_2023537857534306321[240] = 0;
   out_2023537857534306321[241] = 0;
   out_2023537857534306321[242] = 0;
   out_2023537857534306321[243] = 0;
   out_2023537857534306321[244] = 0;
   out_2023537857534306321[245] = 0;
   out_2023537857534306321[246] = 0;
   out_2023537857534306321[247] = 1.0;
   out_2023537857534306321[248] = 0;
   out_2023537857534306321[249] = 0;
   out_2023537857534306321[250] = 0;
   out_2023537857534306321[251] = 0;
   out_2023537857534306321[252] = 0;
   out_2023537857534306321[253] = 0;
   out_2023537857534306321[254] = 0;
   out_2023537857534306321[255] = 0;
   out_2023537857534306321[256] = 0;
   out_2023537857534306321[257] = 0;
   out_2023537857534306321[258] = 0;
   out_2023537857534306321[259] = 0;
   out_2023537857534306321[260] = 0;
   out_2023537857534306321[261] = 0;
   out_2023537857534306321[262] = 0;
   out_2023537857534306321[263] = 0;
   out_2023537857534306321[264] = 0;
   out_2023537857534306321[265] = 0;
   out_2023537857534306321[266] = 1.0;
   out_2023537857534306321[267] = 0;
   out_2023537857534306321[268] = 0;
   out_2023537857534306321[269] = 0;
   out_2023537857534306321[270] = 0;
   out_2023537857534306321[271] = 0;
   out_2023537857534306321[272] = 0;
   out_2023537857534306321[273] = 0;
   out_2023537857534306321[274] = 0;
   out_2023537857534306321[275] = 0;
   out_2023537857534306321[276] = 0;
   out_2023537857534306321[277] = 0;
   out_2023537857534306321[278] = 0;
   out_2023537857534306321[279] = 0;
   out_2023537857534306321[280] = 0;
   out_2023537857534306321[281] = 0;
   out_2023537857534306321[282] = 0;
   out_2023537857534306321[283] = 0;
   out_2023537857534306321[284] = 0;
   out_2023537857534306321[285] = 1.0;
   out_2023537857534306321[286] = 0;
   out_2023537857534306321[287] = 0;
   out_2023537857534306321[288] = 0;
   out_2023537857534306321[289] = 0;
   out_2023537857534306321[290] = 0;
   out_2023537857534306321[291] = 0;
   out_2023537857534306321[292] = 0;
   out_2023537857534306321[293] = 0;
   out_2023537857534306321[294] = 0;
   out_2023537857534306321[295] = 0;
   out_2023537857534306321[296] = 0;
   out_2023537857534306321[297] = 0;
   out_2023537857534306321[298] = 0;
   out_2023537857534306321[299] = 0;
   out_2023537857534306321[300] = 0;
   out_2023537857534306321[301] = 0;
   out_2023537857534306321[302] = 0;
   out_2023537857534306321[303] = 0;
   out_2023537857534306321[304] = 1.0;
   out_2023537857534306321[305] = 0;
   out_2023537857534306321[306] = 0;
   out_2023537857534306321[307] = 0;
   out_2023537857534306321[308] = 0;
   out_2023537857534306321[309] = 0;
   out_2023537857534306321[310] = 0;
   out_2023537857534306321[311] = 0;
   out_2023537857534306321[312] = 0;
   out_2023537857534306321[313] = 0;
   out_2023537857534306321[314] = 0;
   out_2023537857534306321[315] = 0;
   out_2023537857534306321[316] = 0;
   out_2023537857534306321[317] = 0;
   out_2023537857534306321[318] = 0;
   out_2023537857534306321[319] = 0;
   out_2023537857534306321[320] = 0;
   out_2023537857534306321[321] = 0;
   out_2023537857534306321[322] = 0;
   out_2023537857534306321[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_2921235662366984185) {
   out_2921235662366984185[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_2921235662366984185[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_2921235662366984185[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_2921235662366984185[3] = dt*state[12] + state[3];
   out_2921235662366984185[4] = dt*state[13] + state[4];
   out_2921235662366984185[5] = dt*state[14] + state[5];
   out_2921235662366984185[6] = state[6];
   out_2921235662366984185[7] = state[7];
   out_2921235662366984185[8] = state[8];
   out_2921235662366984185[9] = state[9];
   out_2921235662366984185[10] = state[10];
   out_2921235662366984185[11] = state[11];
   out_2921235662366984185[12] = state[12];
   out_2921235662366984185[13] = state[13];
   out_2921235662366984185[14] = state[14];
   out_2921235662366984185[15] = state[15];
   out_2921235662366984185[16] = state[16];
   out_2921235662366984185[17] = state[17];
}
void F_fun(double *state, double dt, double *out_4798703416850893340) {
   out_4798703416850893340[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4798703416850893340[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4798703416850893340[2] = 0;
   out_4798703416850893340[3] = 0;
   out_4798703416850893340[4] = 0;
   out_4798703416850893340[5] = 0;
   out_4798703416850893340[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4798703416850893340[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4798703416850893340[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_4798703416850893340[9] = 0;
   out_4798703416850893340[10] = 0;
   out_4798703416850893340[11] = 0;
   out_4798703416850893340[12] = 0;
   out_4798703416850893340[13] = 0;
   out_4798703416850893340[14] = 0;
   out_4798703416850893340[15] = 0;
   out_4798703416850893340[16] = 0;
   out_4798703416850893340[17] = 0;
   out_4798703416850893340[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4798703416850893340[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4798703416850893340[20] = 0;
   out_4798703416850893340[21] = 0;
   out_4798703416850893340[22] = 0;
   out_4798703416850893340[23] = 0;
   out_4798703416850893340[24] = 0;
   out_4798703416850893340[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4798703416850893340[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_4798703416850893340[27] = 0;
   out_4798703416850893340[28] = 0;
   out_4798703416850893340[29] = 0;
   out_4798703416850893340[30] = 0;
   out_4798703416850893340[31] = 0;
   out_4798703416850893340[32] = 0;
   out_4798703416850893340[33] = 0;
   out_4798703416850893340[34] = 0;
   out_4798703416850893340[35] = 0;
   out_4798703416850893340[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4798703416850893340[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4798703416850893340[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4798703416850893340[39] = 0;
   out_4798703416850893340[40] = 0;
   out_4798703416850893340[41] = 0;
   out_4798703416850893340[42] = 0;
   out_4798703416850893340[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4798703416850893340[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_4798703416850893340[45] = 0;
   out_4798703416850893340[46] = 0;
   out_4798703416850893340[47] = 0;
   out_4798703416850893340[48] = 0;
   out_4798703416850893340[49] = 0;
   out_4798703416850893340[50] = 0;
   out_4798703416850893340[51] = 0;
   out_4798703416850893340[52] = 0;
   out_4798703416850893340[53] = 0;
   out_4798703416850893340[54] = 0;
   out_4798703416850893340[55] = 0;
   out_4798703416850893340[56] = 0;
   out_4798703416850893340[57] = 1;
   out_4798703416850893340[58] = 0;
   out_4798703416850893340[59] = 0;
   out_4798703416850893340[60] = 0;
   out_4798703416850893340[61] = 0;
   out_4798703416850893340[62] = 0;
   out_4798703416850893340[63] = 0;
   out_4798703416850893340[64] = 0;
   out_4798703416850893340[65] = 0;
   out_4798703416850893340[66] = dt;
   out_4798703416850893340[67] = 0;
   out_4798703416850893340[68] = 0;
   out_4798703416850893340[69] = 0;
   out_4798703416850893340[70] = 0;
   out_4798703416850893340[71] = 0;
   out_4798703416850893340[72] = 0;
   out_4798703416850893340[73] = 0;
   out_4798703416850893340[74] = 0;
   out_4798703416850893340[75] = 0;
   out_4798703416850893340[76] = 1;
   out_4798703416850893340[77] = 0;
   out_4798703416850893340[78] = 0;
   out_4798703416850893340[79] = 0;
   out_4798703416850893340[80] = 0;
   out_4798703416850893340[81] = 0;
   out_4798703416850893340[82] = 0;
   out_4798703416850893340[83] = 0;
   out_4798703416850893340[84] = 0;
   out_4798703416850893340[85] = dt;
   out_4798703416850893340[86] = 0;
   out_4798703416850893340[87] = 0;
   out_4798703416850893340[88] = 0;
   out_4798703416850893340[89] = 0;
   out_4798703416850893340[90] = 0;
   out_4798703416850893340[91] = 0;
   out_4798703416850893340[92] = 0;
   out_4798703416850893340[93] = 0;
   out_4798703416850893340[94] = 0;
   out_4798703416850893340[95] = 1;
   out_4798703416850893340[96] = 0;
   out_4798703416850893340[97] = 0;
   out_4798703416850893340[98] = 0;
   out_4798703416850893340[99] = 0;
   out_4798703416850893340[100] = 0;
   out_4798703416850893340[101] = 0;
   out_4798703416850893340[102] = 0;
   out_4798703416850893340[103] = 0;
   out_4798703416850893340[104] = dt;
   out_4798703416850893340[105] = 0;
   out_4798703416850893340[106] = 0;
   out_4798703416850893340[107] = 0;
   out_4798703416850893340[108] = 0;
   out_4798703416850893340[109] = 0;
   out_4798703416850893340[110] = 0;
   out_4798703416850893340[111] = 0;
   out_4798703416850893340[112] = 0;
   out_4798703416850893340[113] = 0;
   out_4798703416850893340[114] = 1;
   out_4798703416850893340[115] = 0;
   out_4798703416850893340[116] = 0;
   out_4798703416850893340[117] = 0;
   out_4798703416850893340[118] = 0;
   out_4798703416850893340[119] = 0;
   out_4798703416850893340[120] = 0;
   out_4798703416850893340[121] = 0;
   out_4798703416850893340[122] = 0;
   out_4798703416850893340[123] = 0;
   out_4798703416850893340[124] = 0;
   out_4798703416850893340[125] = 0;
   out_4798703416850893340[126] = 0;
   out_4798703416850893340[127] = 0;
   out_4798703416850893340[128] = 0;
   out_4798703416850893340[129] = 0;
   out_4798703416850893340[130] = 0;
   out_4798703416850893340[131] = 0;
   out_4798703416850893340[132] = 0;
   out_4798703416850893340[133] = 1;
   out_4798703416850893340[134] = 0;
   out_4798703416850893340[135] = 0;
   out_4798703416850893340[136] = 0;
   out_4798703416850893340[137] = 0;
   out_4798703416850893340[138] = 0;
   out_4798703416850893340[139] = 0;
   out_4798703416850893340[140] = 0;
   out_4798703416850893340[141] = 0;
   out_4798703416850893340[142] = 0;
   out_4798703416850893340[143] = 0;
   out_4798703416850893340[144] = 0;
   out_4798703416850893340[145] = 0;
   out_4798703416850893340[146] = 0;
   out_4798703416850893340[147] = 0;
   out_4798703416850893340[148] = 0;
   out_4798703416850893340[149] = 0;
   out_4798703416850893340[150] = 0;
   out_4798703416850893340[151] = 0;
   out_4798703416850893340[152] = 1;
   out_4798703416850893340[153] = 0;
   out_4798703416850893340[154] = 0;
   out_4798703416850893340[155] = 0;
   out_4798703416850893340[156] = 0;
   out_4798703416850893340[157] = 0;
   out_4798703416850893340[158] = 0;
   out_4798703416850893340[159] = 0;
   out_4798703416850893340[160] = 0;
   out_4798703416850893340[161] = 0;
   out_4798703416850893340[162] = 0;
   out_4798703416850893340[163] = 0;
   out_4798703416850893340[164] = 0;
   out_4798703416850893340[165] = 0;
   out_4798703416850893340[166] = 0;
   out_4798703416850893340[167] = 0;
   out_4798703416850893340[168] = 0;
   out_4798703416850893340[169] = 0;
   out_4798703416850893340[170] = 0;
   out_4798703416850893340[171] = 1;
   out_4798703416850893340[172] = 0;
   out_4798703416850893340[173] = 0;
   out_4798703416850893340[174] = 0;
   out_4798703416850893340[175] = 0;
   out_4798703416850893340[176] = 0;
   out_4798703416850893340[177] = 0;
   out_4798703416850893340[178] = 0;
   out_4798703416850893340[179] = 0;
   out_4798703416850893340[180] = 0;
   out_4798703416850893340[181] = 0;
   out_4798703416850893340[182] = 0;
   out_4798703416850893340[183] = 0;
   out_4798703416850893340[184] = 0;
   out_4798703416850893340[185] = 0;
   out_4798703416850893340[186] = 0;
   out_4798703416850893340[187] = 0;
   out_4798703416850893340[188] = 0;
   out_4798703416850893340[189] = 0;
   out_4798703416850893340[190] = 1;
   out_4798703416850893340[191] = 0;
   out_4798703416850893340[192] = 0;
   out_4798703416850893340[193] = 0;
   out_4798703416850893340[194] = 0;
   out_4798703416850893340[195] = 0;
   out_4798703416850893340[196] = 0;
   out_4798703416850893340[197] = 0;
   out_4798703416850893340[198] = 0;
   out_4798703416850893340[199] = 0;
   out_4798703416850893340[200] = 0;
   out_4798703416850893340[201] = 0;
   out_4798703416850893340[202] = 0;
   out_4798703416850893340[203] = 0;
   out_4798703416850893340[204] = 0;
   out_4798703416850893340[205] = 0;
   out_4798703416850893340[206] = 0;
   out_4798703416850893340[207] = 0;
   out_4798703416850893340[208] = 0;
   out_4798703416850893340[209] = 1;
   out_4798703416850893340[210] = 0;
   out_4798703416850893340[211] = 0;
   out_4798703416850893340[212] = 0;
   out_4798703416850893340[213] = 0;
   out_4798703416850893340[214] = 0;
   out_4798703416850893340[215] = 0;
   out_4798703416850893340[216] = 0;
   out_4798703416850893340[217] = 0;
   out_4798703416850893340[218] = 0;
   out_4798703416850893340[219] = 0;
   out_4798703416850893340[220] = 0;
   out_4798703416850893340[221] = 0;
   out_4798703416850893340[222] = 0;
   out_4798703416850893340[223] = 0;
   out_4798703416850893340[224] = 0;
   out_4798703416850893340[225] = 0;
   out_4798703416850893340[226] = 0;
   out_4798703416850893340[227] = 0;
   out_4798703416850893340[228] = 1;
   out_4798703416850893340[229] = 0;
   out_4798703416850893340[230] = 0;
   out_4798703416850893340[231] = 0;
   out_4798703416850893340[232] = 0;
   out_4798703416850893340[233] = 0;
   out_4798703416850893340[234] = 0;
   out_4798703416850893340[235] = 0;
   out_4798703416850893340[236] = 0;
   out_4798703416850893340[237] = 0;
   out_4798703416850893340[238] = 0;
   out_4798703416850893340[239] = 0;
   out_4798703416850893340[240] = 0;
   out_4798703416850893340[241] = 0;
   out_4798703416850893340[242] = 0;
   out_4798703416850893340[243] = 0;
   out_4798703416850893340[244] = 0;
   out_4798703416850893340[245] = 0;
   out_4798703416850893340[246] = 0;
   out_4798703416850893340[247] = 1;
   out_4798703416850893340[248] = 0;
   out_4798703416850893340[249] = 0;
   out_4798703416850893340[250] = 0;
   out_4798703416850893340[251] = 0;
   out_4798703416850893340[252] = 0;
   out_4798703416850893340[253] = 0;
   out_4798703416850893340[254] = 0;
   out_4798703416850893340[255] = 0;
   out_4798703416850893340[256] = 0;
   out_4798703416850893340[257] = 0;
   out_4798703416850893340[258] = 0;
   out_4798703416850893340[259] = 0;
   out_4798703416850893340[260] = 0;
   out_4798703416850893340[261] = 0;
   out_4798703416850893340[262] = 0;
   out_4798703416850893340[263] = 0;
   out_4798703416850893340[264] = 0;
   out_4798703416850893340[265] = 0;
   out_4798703416850893340[266] = 1;
   out_4798703416850893340[267] = 0;
   out_4798703416850893340[268] = 0;
   out_4798703416850893340[269] = 0;
   out_4798703416850893340[270] = 0;
   out_4798703416850893340[271] = 0;
   out_4798703416850893340[272] = 0;
   out_4798703416850893340[273] = 0;
   out_4798703416850893340[274] = 0;
   out_4798703416850893340[275] = 0;
   out_4798703416850893340[276] = 0;
   out_4798703416850893340[277] = 0;
   out_4798703416850893340[278] = 0;
   out_4798703416850893340[279] = 0;
   out_4798703416850893340[280] = 0;
   out_4798703416850893340[281] = 0;
   out_4798703416850893340[282] = 0;
   out_4798703416850893340[283] = 0;
   out_4798703416850893340[284] = 0;
   out_4798703416850893340[285] = 1;
   out_4798703416850893340[286] = 0;
   out_4798703416850893340[287] = 0;
   out_4798703416850893340[288] = 0;
   out_4798703416850893340[289] = 0;
   out_4798703416850893340[290] = 0;
   out_4798703416850893340[291] = 0;
   out_4798703416850893340[292] = 0;
   out_4798703416850893340[293] = 0;
   out_4798703416850893340[294] = 0;
   out_4798703416850893340[295] = 0;
   out_4798703416850893340[296] = 0;
   out_4798703416850893340[297] = 0;
   out_4798703416850893340[298] = 0;
   out_4798703416850893340[299] = 0;
   out_4798703416850893340[300] = 0;
   out_4798703416850893340[301] = 0;
   out_4798703416850893340[302] = 0;
   out_4798703416850893340[303] = 0;
   out_4798703416850893340[304] = 1;
   out_4798703416850893340[305] = 0;
   out_4798703416850893340[306] = 0;
   out_4798703416850893340[307] = 0;
   out_4798703416850893340[308] = 0;
   out_4798703416850893340[309] = 0;
   out_4798703416850893340[310] = 0;
   out_4798703416850893340[311] = 0;
   out_4798703416850893340[312] = 0;
   out_4798703416850893340[313] = 0;
   out_4798703416850893340[314] = 0;
   out_4798703416850893340[315] = 0;
   out_4798703416850893340[316] = 0;
   out_4798703416850893340[317] = 0;
   out_4798703416850893340[318] = 0;
   out_4798703416850893340[319] = 0;
   out_4798703416850893340[320] = 0;
   out_4798703416850893340[321] = 0;
   out_4798703416850893340[322] = 0;
   out_4798703416850893340[323] = 1;
}
void h_4(double *state, double *unused, double *out_5260931664586000243) {
   out_5260931664586000243[0] = state[6] + state[9];
   out_5260931664586000243[1] = state[7] + state[10];
   out_5260931664586000243[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_3707798395825917046) {
   out_3707798395825917046[0] = 0;
   out_3707798395825917046[1] = 0;
   out_3707798395825917046[2] = 0;
   out_3707798395825917046[3] = 0;
   out_3707798395825917046[4] = 0;
   out_3707798395825917046[5] = 0;
   out_3707798395825917046[6] = 1;
   out_3707798395825917046[7] = 0;
   out_3707798395825917046[8] = 0;
   out_3707798395825917046[9] = 1;
   out_3707798395825917046[10] = 0;
   out_3707798395825917046[11] = 0;
   out_3707798395825917046[12] = 0;
   out_3707798395825917046[13] = 0;
   out_3707798395825917046[14] = 0;
   out_3707798395825917046[15] = 0;
   out_3707798395825917046[16] = 0;
   out_3707798395825917046[17] = 0;
   out_3707798395825917046[18] = 0;
   out_3707798395825917046[19] = 0;
   out_3707798395825917046[20] = 0;
   out_3707798395825917046[21] = 0;
   out_3707798395825917046[22] = 0;
   out_3707798395825917046[23] = 0;
   out_3707798395825917046[24] = 0;
   out_3707798395825917046[25] = 1;
   out_3707798395825917046[26] = 0;
   out_3707798395825917046[27] = 0;
   out_3707798395825917046[28] = 1;
   out_3707798395825917046[29] = 0;
   out_3707798395825917046[30] = 0;
   out_3707798395825917046[31] = 0;
   out_3707798395825917046[32] = 0;
   out_3707798395825917046[33] = 0;
   out_3707798395825917046[34] = 0;
   out_3707798395825917046[35] = 0;
   out_3707798395825917046[36] = 0;
   out_3707798395825917046[37] = 0;
   out_3707798395825917046[38] = 0;
   out_3707798395825917046[39] = 0;
   out_3707798395825917046[40] = 0;
   out_3707798395825917046[41] = 0;
   out_3707798395825917046[42] = 0;
   out_3707798395825917046[43] = 0;
   out_3707798395825917046[44] = 1;
   out_3707798395825917046[45] = 0;
   out_3707798395825917046[46] = 0;
   out_3707798395825917046[47] = 1;
   out_3707798395825917046[48] = 0;
   out_3707798395825917046[49] = 0;
   out_3707798395825917046[50] = 0;
   out_3707798395825917046[51] = 0;
   out_3707798395825917046[52] = 0;
   out_3707798395825917046[53] = 0;
}
void h_10(double *state, double *unused, double *out_6664006634176473998) {
   out_6664006634176473998[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_6664006634176473998[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_6664006634176473998[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5640695491572100361) {
   out_5640695491572100361[0] = 0;
   out_5640695491572100361[1] = 9.8100000000000005*cos(state[1]);
   out_5640695491572100361[2] = 0;
   out_5640695491572100361[3] = 0;
   out_5640695491572100361[4] = -state[8];
   out_5640695491572100361[5] = state[7];
   out_5640695491572100361[6] = 0;
   out_5640695491572100361[7] = state[5];
   out_5640695491572100361[8] = -state[4];
   out_5640695491572100361[9] = 0;
   out_5640695491572100361[10] = 0;
   out_5640695491572100361[11] = 0;
   out_5640695491572100361[12] = 1;
   out_5640695491572100361[13] = 0;
   out_5640695491572100361[14] = 0;
   out_5640695491572100361[15] = 1;
   out_5640695491572100361[16] = 0;
   out_5640695491572100361[17] = 0;
   out_5640695491572100361[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5640695491572100361[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5640695491572100361[20] = 0;
   out_5640695491572100361[21] = state[8];
   out_5640695491572100361[22] = 0;
   out_5640695491572100361[23] = -state[6];
   out_5640695491572100361[24] = -state[5];
   out_5640695491572100361[25] = 0;
   out_5640695491572100361[26] = state[3];
   out_5640695491572100361[27] = 0;
   out_5640695491572100361[28] = 0;
   out_5640695491572100361[29] = 0;
   out_5640695491572100361[30] = 0;
   out_5640695491572100361[31] = 1;
   out_5640695491572100361[32] = 0;
   out_5640695491572100361[33] = 0;
   out_5640695491572100361[34] = 1;
   out_5640695491572100361[35] = 0;
   out_5640695491572100361[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5640695491572100361[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5640695491572100361[38] = 0;
   out_5640695491572100361[39] = -state[7];
   out_5640695491572100361[40] = state[6];
   out_5640695491572100361[41] = 0;
   out_5640695491572100361[42] = state[4];
   out_5640695491572100361[43] = -state[3];
   out_5640695491572100361[44] = 0;
   out_5640695491572100361[45] = 0;
   out_5640695491572100361[46] = 0;
   out_5640695491572100361[47] = 0;
   out_5640695491572100361[48] = 0;
   out_5640695491572100361[49] = 0;
   out_5640695491572100361[50] = 1;
   out_5640695491572100361[51] = 0;
   out_5640695491572100361[52] = 0;
   out_5640695491572100361[53] = 1;
}
void h_13(double *state, double *unused, double *out_2018719708536053041) {
   out_2018719708536053041[0] = state[3];
   out_2018719708536053041[1] = state[4];
   out_2018719708536053041[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6920072221158249847) {
   out_6920072221158249847[0] = 0;
   out_6920072221158249847[1] = 0;
   out_6920072221158249847[2] = 0;
   out_6920072221158249847[3] = 1;
   out_6920072221158249847[4] = 0;
   out_6920072221158249847[5] = 0;
   out_6920072221158249847[6] = 0;
   out_6920072221158249847[7] = 0;
   out_6920072221158249847[8] = 0;
   out_6920072221158249847[9] = 0;
   out_6920072221158249847[10] = 0;
   out_6920072221158249847[11] = 0;
   out_6920072221158249847[12] = 0;
   out_6920072221158249847[13] = 0;
   out_6920072221158249847[14] = 0;
   out_6920072221158249847[15] = 0;
   out_6920072221158249847[16] = 0;
   out_6920072221158249847[17] = 0;
   out_6920072221158249847[18] = 0;
   out_6920072221158249847[19] = 0;
   out_6920072221158249847[20] = 0;
   out_6920072221158249847[21] = 0;
   out_6920072221158249847[22] = 1;
   out_6920072221158249847[23] = 0;
   out_6920072221158249847[24] = 0;
   out_6920072221158249847[25] = 0;
   out_6920072221158249847[26] = 0;
   out_6920072221158249847[27] = 0;
   out_6920072221158249847[28] = 0;
   out_6920072221158249847[29] = 0;
   out_6920072221158249847[30] = 0;
   out_6920072221158249847[31] = 0;
   out_6920072221158249847[32] = 0;
   out_6920072221158249847[33] = 0;
   out_6920072221158249847[34] = 0;
   out_6920072221158249847[35] = 0;
   out_6920072221158249847[36] = 0;
   out_6920072221158249847[37] = 0;
   out_6920072221158249847[38] = 0;
   out_6920072221158249847[39] = 0;
   out_6920072221158249847[40] = 0;
   out_6920072221158249847[41] = 1;
   out_6920072221158249847[42] = 0;
   out_6920072221158249847[43] = 0;
   out_6920072221158249847[44] = 0;
   out_6920072221158249847[45] = 0;
   out_6920072221158249847[46] = 0;
   out_6920072221158249847[47] = 0;
   out_6920072221158249847[48] = 0;
   out_6920072221158249847[49] = 0;
   out_6920072221158249847[50] = 0;
   out_6920072221158249847[51] = 0;
   out_6920072221158249847[52] = 0;
   out_6920072221158249847[53] = 0;
}
void h_14(double *state, double *unused, double *out_88167374712919576) {
   out_88167374712919576[0] = state[6];
   out_88167374712919576[1] = state[7];
   out_88167374712919576[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7671039252165401575) {
   out_7671039252165401575[0] = 0;
   out_7671039252165401575[1] = 0;
   out_7671039252165401575[2] = 0;
   out_7671039252165401575[3] = 0;
   out_7671039252165401575[4] = 0;
   out_7671039252165401575[5] = 0;
   out_7671039252165401575[6] = 1;
   out_7671039252165401575[7] = 0;
   out_7671039252165401575[8] = 0;
   out_7671039252165401575[9] = 0;
   out_7671039252165401575[10] = 0;
   out_7671039252165401575[11] = 0;
   out_7671039252165401575[12] = 0;
   out_7671039252165401575[13] = 0;
   out_7671039252165401575[14] = 0;
   out_7671039252165401575[15] = 0;
   out_7671039252165401575[16] = 0;
   out_7671039252165401575[17] = 0;
   out_7671039252165401575[18] = 0;
   out_7671039252165401575[19] = 0;
   out_7671039252165401575[20] = 0;
   out_7671039252165401575[21] = 0;
   out_7671039252165401575[22] = 0;
   out_7671039252165401575[23] = 0;
   out_7671039252165401575[24] = 0;
   out_7671039252165401575[25] = 1;
   out_7671039252165401575[26] = 0;
   out_7671039252165401575[27] = 0;
   out_7671039252165401575[28] = 0;
   out_7671039252165401575[29] = 0;
   out_7671039252165401575[30] = 0;
   out_7671039252165401575[31] = 0;
   out_7671039252165401575[32] = 0;
   out_7671039252165401575[33] = 0;
   out_7671039252165401575[34] = 0;
   out_7671039252165401575[35] = 0;
   out_7671039252165401575[36] = 0;
   out_7671039252165401575[37] = 0;
   out_7671039252165401575[38] = 0;
   out_7671039252165401575[39] = 0;
   out_7671039252165401575[40] = 0;
   out_7671039252165401575[41] = 0;
   out_7671039252165401575[42] = 0;
   out_7671039252165401575[43] = 0;
   out_7671039252165401575[44] = 1;
   out_7671039252165401575[45] = 0;
   out_7671039252165401575[46] = 0;
   out_7671039252165401575[47] = 0;
   out_7671039252165401575[48] = 0;
   out_7671039252165401575[49] = 0;
   out_7671039252165401575[50] = 0;
   out_7671039252165401575[51] = 0;
   out_7671039252165401575[52] = 0;
   out_7671039252165401575[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_7983792447504184197) {
  err_fun(nom_x, delta_x, out_7983792447504184197);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5020775487918234326) {
  inv_err_fun(nom_x, true_x, out_5020775487918234326);
}
void pose_H_mod_fun(double *state, double *out_2023537857534306321) {
  H_mod_fun(state, out_2023537857534306321);
}
void pose_f_fun(double *state, double dt, double *out_2921235662366984185) {
  f_fun(state,  dt, out_2921235662366984185);
}
void pose_F_fun(double *state, double dt, double *out_4798703416850893340) {
  F_fun(state,  dt, out_4798703416850893340);
}
void pose_h_4(double *state, double *unused, double *out_5260931664586000243) {
  h_4(state, unused, out_5260931664586000243);
}
void pose_H_4(double *state, double *unused, double *out_3707798395825917046) {
  H_4(state, unused, out_3707798395825917046);
}
void pose_h_10(double *state, double *unused, double *out_6664006634176473998) {
  h_10(state, unused, out_6664006634176473998);
}
void pose_H_10(double *state, double *unused, double *out_5640695491572100361) {
  H_10(state, unused, out_5640695491572100361);
}
void pose_h_13(double *state, double *unused, double *out_2018719708536053041) {
  h_13(state, unused, out_2018719708536053041);
}
void pose_H_13(double *state, double *unused, double *out_6920072221158249847) {
  H_13(state, unused, out_6920072221158249847);
}
void pose_h_14(double *state, double *unused, double *out_88167374712919576) {
  h_14(state, unused, out_88167374712919576);
}
void pose_H_14(double *state, double *unused, double *out_7671039252165401575) {
  H_14(state, unused, out_7671039252165401575);
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
