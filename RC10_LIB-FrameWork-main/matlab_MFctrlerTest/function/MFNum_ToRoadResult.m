function R = MFNum_ToRoadResult(MFNum)
% 前一通道候选（最多3）
R = struct('result1', int8(0), 'result2', int8(0), 'result3', int8(0));
if MFNum < 1 || MFNum > 12, return; end

mapNum = MFNum_TransforMapNum(MFNum);
cand = int8(zeros(1,4));
cand(1) = mapNum - 6;
cand(2) = mapNum - 4;
cand(3) = mapNum + 4;
cand(4) = mapNum + 6;

for i = 1:4
    if cand(i) < 1 || cand(i) > 30
        cand(i) = 0; continue;
    end
    if ~IsWalkable(cand(i))
        cand(i) = 0; continue;
    end
end

valid = cand(cand ~= 0);
valid = sort(valid, 'ascend');
% 最多取3个
n = min(numel(valid), 3);
if n >= 1, R.result1 = valid(1); end
if n >= 2, R.result2 = valid(2); end
if n >= 3, R.result3 = valid(3); end
end