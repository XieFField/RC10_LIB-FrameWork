function p = MapCenterWorld(map)
C = mf_consts();
if map < 1 || map > 30
    p = [0 0 0];
else
    p = C.MapNum_RealPos(map, :);
end
end