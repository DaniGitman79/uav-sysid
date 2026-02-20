#ifndef Copter_acc_h_
#define Copter_acc_h_
#ifndef Copter_acc_COMMON_INCLUDES_
#define Copter_acc_COMMON_INCLUDES_
#include <stdlib.h>
#define S_FUNCTION_NAME simulink_only_sfcn
#define S_FUNCTION_LEVEL 2
#ifndef RTW_GENERATED_S_FUNCTION
#define RTW_GENERATED_S_FUNCTION
#endif
#include "rtwtypes.h"
#include "simstruc.h"
#include "fixedpoint.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif
#include "Copter_acc_types.h"
#include <stddef.h>
#include <float.h>
#include "mwmathutil.h"
#include "rt_defines.h"
typedef struct { real_T B_15_0_0 [ 3 ] ; real_T B_15_3_24 [ 3 ] ; real_T
B_15_6_48 [ 16 ] ; real_T B_15_22_176 [ 3 ] ; real_T B_15_25_200 [ 3 ] ;
real_T B_15_28_224 [ 3 ] ; real_T B_15_31_248 [ 3 ] ; real_T B_15_34_272 [ 3
] ; real_T B_15_37_296 ; real_T B_15_38_304 [ 3 ] ; real_T B_15_41_328 [ 3 ]
; real_T B_15_44_352 [ 3 ] ; real_T B_15_47_376 [ 3 ] ; real_T B_15_50_400 [
3 ] ; real_T B_15_53_424 [ 3 ] ; real_T B_15_56_448 [ 3 ] ; real_T
B_15_59_472 [ 3 ] ; real_T B_15_62_496 [ 3 ] ; real_T B_15_65_520 [ 3 ] ;
real_T B_15_68_544 [ 3 ] ; real_T B_15_71_568 [ 3 ] ; real_T B_15_74_592 [ 3
] ; real_T B_15_77_616 [ 3 ] ; real_T B_15_80_640 [ 3 ] ; real_T B_15_83_664
[ 3 ] ; real_T B_15_86_688 [ 3 ] ; real_T B_15_89_712 [ 3 ] ; real_T
B_15_92_736 [ 3 ] ; real_T B_15_95_760 [ 3 ] ; real_T B_15_98_784 [ 3 ] ;
real_T B_15_101_808 ; real_T B_15_102_816 [ 3 ] ; real_T B_15_105_840 ;
real_T B_15_106_848 ; real_T B_15_107_856 ; real_T B_15_108_864 ; real_T
B_15_109_872 ; real_T B_15_110_880 ; real_T B_15_111_888 ; real_T
B_15_112_896 ; real_T B_15_113_904 ; real_T B_15_114_912 ; real_T
B_15_115_920 ; real_T B_15_116_928 ; real_T B_15_117_936 ; real_T
B_15_118_944 ; real_T B_15_119_952 ; real_T B_15_120_960 ; real_T
B_15_121_968 ; real_T B_15_122_976 ; real_T B_15_123_984 [ 3 ] ; real_T
B_15_126_1008 [ 3 ] ; real_T B_15_129_1032 [ 3 ] ; real_T B_15_132_1056 [ 3 ]
; real_T B_15_135_1080 [ 3 ] ; real_T B_15_138_1104 [ 3 ] ; real_T
B_15_141_1128 ; real_T B_15_142_1136 [ 3 ] ; real_T B_15_145_1160 [ 3 ] ;
real_T B_15_148_1184 [ 3 ] ; real_T B_15_151_1208 [ 3 ] ; real_T
B_15_154_1232 ; real_T B_15_155_1240 ; real_T B_15_156_1248 ; real_T
B_15_157_1256 ; real_T B_15_158_1264 ; real_T B_15_159_1272 [ 3 ] ; real_T
B_15_162_1296 [ 3 ] ; real_T B_15_165_1320 ; real_T B_15_166_1328 ; real_T
B_15_167_1336 ; real_T B_15_168_1344 ; real_T B_15_169_1352 ; real_T
B_15_170_1360 ; real_T B_15_171_1368 ; real_T B_15_172_1376 ; real_T
B_15_173_1384 ; real_T B_15_174_1392 ; real_T B_15_175_1400 ; real_T
B_15_176_1408 ; real_T B_15_177_1416 ; real_T B_15_178_1424 ; real_T
B_15_179_1432 ; real_T B_15_180_1440 ; real_T B_15_181_1448 ; real_T
B_15_182_1456 ; real_T B_15_183_1464 ; real_T B_15_184_1472 ; real_T
B_15_185_1480 ; real_T B_15_186_1488 ; real_T B_15_187_1496 ; real_T
B_15_188_1504 ; real_T B_15_189_1512 ; real_T B_15_190_1520 ; real_T
B_15_191_1528 ; real_T B_15_192_1536 ; real_T B_15_193_1544 ; real_T
B_15_194_1552 ; real_T B_15_195_1560 ; real_T B_15_196_1568 ; real_T
B_15_197_1576 ; real_T B_15_198_1584 ; real_T B_15_199_1592 ; real_T
B_15_200_1600 ; real_T B_15_201_1608 ; real_T B_15_202_1616 ; real_T
B_15_203_1624 ; real_T B_15_204_1632 ; real_T B_15_205_1640 ; real_T
B_15_206_1648 ; real_T B_15_207_1656 ; real_T B_15_208_1664 ; real_T
B_15_209_1672 ; real_T B_15_210_1680 ; real_T B_15_211_1688 ; real_T
B_15_212_1696 ; real_T B_15_213_1704 ; real_T B_15_214_1712 ; real_T
B_15_215_1720 ; real_T B_15_216_1728 ; real_T B_15_217_1736 ; real_T
B_15_218_1744 ; real_T B_15_219_1752 ; real_T B_15_220_1760 ; real_T
B_15_221_1768 ; real_T B_15_222_1776 ; real_T B_15_223_1784 ; real_T
B_15_224_1792 ; real_T B_15_225_1800 ; real_T B_15_226_1808 ; real_T
B_15_227_1816 ; real_T B_15_228_1824 ; real_T B_15_229_1832 ; real_T
B_15_230_1840 ; real_T B_14_231_1848 ; real_T B_13_232_1856 ; real_T
B_12_233_1864 ; real_T B_11_234_1872 ; real_T B_10_235_1880 ; real_T
B_9_236_1888 ; real_T B_8_237_1896 ; real_T B_7_238_1904 ; real_T
B_6_239_1912 ; real_T B_5_240_1920 ; real_T B_4_241_1928 ; real_T
B_3_242_1936 ; real_T B_1_243_1944 [ 16 ] ; real_T B_15_259_2072 [ 6 ] ;
real_T B_15_265_2120 [ 6 ] ; boolean_T B_1_271_2168 ; char_T pad_B_1_271_2168
[ 7 ] ; } B_Copter_T ; typedef struct { real_T RateTransition_Buffer0 [ 16 ]
; real_T RateTransition_Buffer [ 3 ] ; real_T RateTransition1_Buffer [ 3 ] ;
real_T RateTransition2_Buffer [ 3 ] ; real_T RateTransition3_Buffer [ 3 ] ;
real_T RateTransition4_Buffer [ 3 ] ; real_T VariableTimeDelay_RWORK ; real_T
VariableTimeDelay_RWORK_p ; real_T VariableTimeDelay_RWORK_f ; real_T
VariableTimeDelay_RWORK_h ; real_T VariableTimeDelay_RWORK_j ; real_T
VariableTimeDelay_RWORK_o ; void * VariableTimeDelay_PWORK [ 2 ] ; void *
VariableTimeDelay_PWORK_d [ 2 ] ; void * VariableTimeDelay_PWORK_b [ 2 ] ;
void * VariableTimeDelay_PWORK_p [ 2 ] ; void * VariableTimeDelay_PWORK_g [ 2
] ; void * VariableTimeDelay_PWORK_bg [ 2 ] ; void *
CheckScopeGear_moments_PWORK ; void * CheckScopemomentsofeachmotor_PWORK [ 6
] ; void * CheckScopetildingmomentvector_PWORK ; void *
CheckScopetotalMomentvector_PWORK ; void * CheckScopetotalThrustvector_PWORK
; void * Scope_PWORK ; void * Scope1_PWORK ; void * Scope2_PWORK ; void *
Scope_PWORK_p ; void * Scope1_PWORK_o ; void * Scope2_PWORK_g ; void *
Scope_PWORK_m ; void * Scope1_PWORK_j ; void * Scope2_PWORK_k ; void *
Scope_PWORK_o ; void * Scope1_PWORK_jn ; void * Scope2_PWORK_n ; void *
Scope_PWORK_n ; void * Scope1_PWORK_c ; void * Scope2_PWORK_ki ; void *
Scope_PWORK_f ; void * Scope1_PWORK_n ; void * Scope2_PWORK_l ; int32_T
MATLABFunction_sysIdxToRun ; int32_T MATLABFunction_sysIdxToRun_o ; int32_T
MATLABFunction_sysIdxToRun_b ; int32_T MATLABFunction_sysIdxToRun_h ; int32_T
MATLABFunction_sysIdxToRun_d ; int32_T MATLABFunction_sysIdxToRun_dn ;
int32_T MATLABFunction_sysIdxToRun_i ; int32_T MATLABFunction_sysIdxToRun_n ;
int32_T MATLABFunction_sysIdxToRun_oc ; int32_T MATLABFunction_sysIdxToRun_dz
; int32_T MATLABFunction_sysIdxToRun_dv ; int32_T
MATLABFunction_sysIdxToRun_hj ; int32_T MATLABFunction1_sysIdxToRun ; int32_T
MATLABFunction_sysIdxToRun_d2 ; int32_T InitializeFunction_sysIdxToRun ;
int_T VariableTimeDelay_IWORK [ 5 ] ; int_T VariableTimeDelay_IWORK_g [ 5 ] ;
int_T VariableTimeDelay_IWORK_j [ 5 ] ; int_T VariableTimeDelay_IWORK_l [ 5 ]
; int_T VariableTimeDelay_IWORK_k [ 5 ] ; int_T VariableTimeDelay_IWORK_h [ 5
] ; char_T pad_VariableTimeDelay_IWORK_h [ 4 ] ; } DW_Copter_T ; typedef
struct { real_T uDOFEulerAngles_CSTATE [ 12 ] ; real_T TransferFcn_CSTATE ;
real_T TransferFcn_CSTATE_i ; real_T TransferFcn_CSTATE_h ; real_T
TransferFcn_CSTATE_i0 ; real_T TransferFcn_CSTATE_o ; real_T
TransferFcn_CSTATE_p ; } X_Copter_T ; typedef int_T PeriodicIndX_Copter_T [ 3
] ; typedef real_T PeriodicRngX_Copter_T [ 6 ] ; typedef struct { real_T
uDOFEulerAngles_CSTATE [ 12 ] ; real_T TransferFcn_CSTATE ; real_T
TransferFcn_CSTATE_i ; real_T TransferFcn_CSTATE_h ; real_T
TransferFcn_CSTATE_i0 ; real_T TransferFcn_CSTATE_o ; real_T
TransferFcn_CSTATE_p ; } XDot_Copter_T ; typedef struct { boolean_T
uDOFEulerAngles_CSTATE [ 12 ] ; boolean_T TransferFcn_CSTATE ; boolean_T
TransferFcn_CSTATE_i ; boolean_T TransferFcn_CSTATE_h ; boolean_T
TransferFcn_CSTATE_i0 ; boolean_T TransferFcn_CSTATE_o ; boolean_T
TransferFcn_CSTATE_p ; } XDis_Copter_T ; struct P_Copter_T_ { real_T P_0 [ 3
] ; real_T P_1 [ 3 ] ; real_T P_2 [ 3 ] ; real_T P_3 [ 3 ] ; real_T P_4 ;
real_T P_5 [ 9 ] ; real_T P_6 ; real_T P_7 ; real_T P_8 ; real_T P_9 ; real_T
P_10 ; real_T P_11 [ 8 ] ; real_T P_12 [ 8 ] ; real_T P_13 ; real_T P_14 ;
real_T P_15 ; real_T P_16 ; real_T P_17 [ 8 ] ; real_T P_18 [ 8 ] ; real_T
P_19 ; real_T P_20 ; real_T P_21 ; real_T P_22 ; real_T P_23 [ 8 ] ; real_T
P_24 [ 8 ] ; real_T P_25 ; real_T P_26 ; real_T P_27 ; real_T P_28 ; real_T
P_29 [ 8 ] ; real_T P_30 [ 8 ] ; real_T P_31 ; real_T P_32 ; real_T P_33 ;
real_T P_34 ; real_T P_35 [ 8 ] ; real_T P_36 [ 8 ] ; real_T P_37 ; real_T
P_38 ; real_T P_39 ; real_T P_40 ; real_T P_41 [ 8 ] ; real_T P_42 [ 8 ] ;
real_T P_43 ; real_T P_44 ; real_T P_45 ; real_T P_46 ; real_T P_47 ; real_T
P_48 ; real_T P_49 ; real_T P_50 ; real_T P_51 ; real_T P_52 ; real_T P_53 ;
real_T P_54 ; real_T P_55 ; real_T P_56 ; real_T P_57 ; real_T P_58 ; real_T
P_59 ; real_T P_60 ; real_T P_61 ; real_T P_62 ; real_T P_63 ; real_T P_64 ;
real_T P_65 ; real_T P_66 ; real_T P_67 [ 8 ] ; real_T P_68 [ 8 ] ; real_T
P_69 ; real_T P_70 ; real_T P_71 ; real_T P_72 ; real_T P_73 [ 8 ] ; real_T
P_74 [ 8 ] ; real_T P_75 ; real_T P_76 ; real_T P_77 ; real_T P_78 ; real_T
P_79 [ 8 ] ; real_T P_80 [ 8 ] ; real_T P_81 ; real_T P_82 ; real_T P_83 ;
real_T P_84 ; real_T P_85 [ 8 ] ; real_T P_86 [ 8 ] ; real_T P_87 ; real_T
P_88 ; real_T P_89 ; real_T P_90 ; real_T P_91 [ 8 ] ; real_T P_92 [ 8 ] ;
real_T P_93 ; real_T P_94 ; real_T P_95 ; real_T P_96 ; real_T P_97 [ 8 ] ;
real_T P_98 [ 8 ] ; real_T P_99 [ 3 ] ; real_T P_100 [ 3 ] ; real_T P_101 [ 3
] ; real_T P_102 [ 3 ] ; real_T P_103 [ 3 ] ; real_T P_104 [ 3 ] ; real_T
P_105 ; real_T P_106 [ 3 ] ; real_T P_107 [ 3 ] ; real_T P_108 [ 3 ] ; real_T
P_109 [ 3 ] ; real_T P_110 ; real_T P_111 ; real_T P_112 ; real_T P_113 ;
real_T P_114 ; real_T P_115 [ 3 ] ; real_T P_116 [ 3 ] ; real_T P_117 ;
real_T P_118 ; real_T P_119 ; real_T P_120 ; real_T P_121 ; real_T P_122 ;
real_T P_123 ; real_T P_124 ; real_T P_125 ; real_T P_126 ; real_T P_127 ;
real_T P_128 ; real_T P_129 ; real_T P_130 ; real_T P_131 ; real_T P_132 ;
real_T P_133 ; real_T P_134 ; real_T P_135 ; real_T P_136 ; real_T P_137 ;
real_T P_138 ; real_T P_139 ; real_T P_140 ; real_T P_141 ; real_T P_142 ;
real_T P_143 ; real_T P_144 ; real_T P_145 ; real_T P_146 ; real_T P_147 ;
real_T P_148 ; real_T P_149 ; real_T P_150 ; real_T P_151 ; real_T P_152 ;
real_T P_153 ; real_T P_154 ; real_T P_155 ; real_T P_156 ; real_T P_157 ;
real_T P_158 ; real_T P_159 ; real_T P_160 ; real_T P_161 ; real_T P_162 ;
real_T P_163 ; real_T P_164 ; real_T P_165 ; real_T P_166 ; real_T P_167 ;
real_T P_168 ; real_T P_169 ; real_T P_170 ; real_T P_171 ; real_T P_172 ;
real_T P_173 ; real_T P_174 ; real_T P_175 ; real_T P_176 ; real_T P_177 ;
real_T P_178 ; real_T P_179 ; real_T P_180 ; real_T P_181 ; real_T P_182 ; }
; extern P_Copter_T Copter_rtDefaultP ;
#endif
