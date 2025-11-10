function fullPath = BuildFullPath(seq)
% seq 为必须经过点的地图号序列（已过滤 0）
fullPath = int8([]);
if isempty(seq), return; end
last = seq(1);
fullPath(end+1) = last; 
for i = 2:numel(seq)
    target = seq(i);
    seg = BFS_Path(last, target);
    if isempty(seg)
        % 无法到达，跳过
        last = target;
        continue;
    end
    % 去重首格
    if seg(1) == fullPath(end) && numel(seg) > 1
        seg = seg(2:end);
    end
    fullPath = [fullPath seg]; 
    last = target;
end
end