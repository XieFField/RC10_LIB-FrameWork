% function [c, r] = Map_ToCR(map)
% C = mf_consts();
% r = floor((map - 1) / C.MAP_COLS) + 1;
% c = mod(map - 1, C.MAP_COLS) + 1;
% end

function [c,r] = Map_ToCR(map)
if map<1||map>30
    c=0; r=0; return;
end
c = mod(double(map)-1,5)+1;
r = floor((double(map)-1)/5)+1;
end