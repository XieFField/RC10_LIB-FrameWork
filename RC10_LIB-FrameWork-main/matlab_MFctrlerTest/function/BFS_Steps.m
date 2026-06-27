function steps = BFS_Steps(startMap, goalMap)
C = mf_consts();
MAP_COLS = C.MAP_COLS;
MAP_ROWS = C.MAP_ROWS;
BFS_INF  = C.BFS_INF;

if startMap == goalMap
    steps = 0; return;
end
if ~IsWalkable(startMap) || ~IsWalkable(goalMap)
    steps = BFS_INF; return;
end

dist = int16(ones(1,31) * BFS_INF);
vis  = uint8(zeros(1,31));
for i = 1:30
    dist(i) = int16(BFS_INF);
    vis(i)  = uint8(0);
end

% ????(??32)
q = int8(zeros(1,32)); h = int32(0); t = int32(0);
    function qpush(v)
        q(bitand(t,31)+1) = int8(v); t = t + 1;
    end
    function v = qpop()
        v = int32(q(bitand(h,31)+1)); h = h + 1;
    end
    function tf = qempty()
        tf = (h == t);
    end

dist(startMap) = int16(0);
vis(startMap)  = uint8(1);
qpush(startMap);

while ~qempty()
    u = qpop();
    if u == goalMap, break; end

    [c,r] = Map_ToCR(u);
    dc = int32([0 0 -1 1]); dr = int32([-1 1 0 0]);
    for k = 1:4
        cc = int32(c) + dc(k); rr = int32(r) + dr(k);
        if cc < 1 || cc > MAP_COLS || rr < 1 || rr > MAP_ROWS, continue; end
        v = int32(CR_ToMap(double(cc), double(rr)));
        if ~IsWalkable(v) || vis(v), continue; end
        vis(v)  = uint8(1);
        dist(v) = int16(int32(dist(u)) + 1);
        qpush(v);
    end
end

steps = double(dist(goalMap));
end