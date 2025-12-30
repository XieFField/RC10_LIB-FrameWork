function MFNum = MapNum_TransforMFNum(mapNum)
MFNum_ = mapNum - 6 - 2 * floor((mapNum - 7) / 3);
if MFNum_ < 1 || MFNum_ > 12
    MFNum = int8(-1);
else
    MFNum = int8(MFNum_);
end
end