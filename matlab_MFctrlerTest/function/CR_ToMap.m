% filepath: f:\MyProjectFlies\STM32H7\Frame_T\matlab_MFctrlerTest\CR_ToMap.m
function map = CR_ToMap(c, r)
C = mf_consts();
map = (r - 1) * C.MAP_COLS + c; % 用 double 返回，避免 int8 索引问题
end