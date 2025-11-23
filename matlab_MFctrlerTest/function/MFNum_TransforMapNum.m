function mapNum = MFNum_TransforMapNum(MFNum)
if MFNum < 1 || MFNum > 12
    mapNum = int8(-1); return;
end
mapNum = int8(MFNum + 6 + 2 * floor((MFNum - 1) / 3));
end