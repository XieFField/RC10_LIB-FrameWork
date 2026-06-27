function res = MFNum_ToCatchRoadResult(MFNum)
res = struct('result1',int8(0),'result2',int8(0),'result3',int8(0));
if MFNum < 1 || MFNum > 12, return; end

mapNum = double(MFNum_TransforMapNum(MFNum));
cand = [mapNum-5, mapNum-1, mapNum+1, mapNum+5];

valid = [];
for i=1:4
    v = cand(i);
    if v >= 1 && v <= 30 && IsWalkable(v)
        valid(end+1) = v; %#ok<AGROW>
    end
end
valid = sort(valid);
if ~isempty(valid), res.result1 = int8(valid(1)); end
if numel(valid) >= 2, res.result2 = int8(valid(2)); end
if numel(valid) >= 3, res.result3 = int8(valid(3)); end
end