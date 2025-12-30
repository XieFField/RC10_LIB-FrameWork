function pathCells = BFS_Path(startMap, goalMap)
C = mf_consts();
pathCells = int8([]);

if startMap == goalMap
    pathCells = int8(startMap); 
    return;
end
if ~IsWalkable(startMap) || ~IsWalkable(goalMap)
    return;
end

dist  = ones(1,30) * C.BFS_INF;
vis   = false(1,30);
parent = zeros(1,30,'int8');

q = zeros(1,64); h = 1; t = 1;
q(t) = double(startMap); t = t + 1;
idxStart = round(startMap);
vis(idxStart) = true;
dist(idxStart) = 0;

while h ~= t
    u = q(h); h = h + 1;
    if h > numel(q), h = 1; end
    if u == goalMap, break; end

    [c,r] = Map_ToCR(u);
    dc = [0 0 -1 1];
    dr = [-1 1 0 0];
    for k = 1:4
        cc = c + dc(k); rr = r + dr(k);
        if cc < 1 || cc > C.MAP_COLS || rr < 1 || rr > C.MAP_ROWS
            continue;
        end
        v = CR_ToMap(cc, rr);
        iv = round(v);
        if ~IsWalkable(iv) || vis(iv), continue; end
        vis(iv) = true;
        dist(iv) = dist(round(u)) + 1;
        parent(iv) = int8(u);
        q(t) = iv; t = t + 1; if t > numel(q), t = 1; end
    end
end

if ~vis(round(goalMap))
    return;
end

% »ØËİ
cur = int8(goalMap);
rev = int8([]);
while cur ~= 0 && cur ~= int8(startMap)
    rev(end+1) = cur; %#ok<AGROW>
    cur = parent(round(cur));
end
rev(end+1) = int8(startMap);

pathCells = int8(fliplr(rev));
end