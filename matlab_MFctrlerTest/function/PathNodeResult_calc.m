function out = PathNodeResult_calc(robotPos, MF1, MF2)
C = mf_consts();
out = struct('entranceMap',int8(0),'bestB1',int8(0),'bestBMF1',int8(0), ...
             'bestB2',int8(0),'bestBMF2',int8(0),'exitMap',int8(30));

found = false;

% 候选 B1
B1_can = MFNum_ToRoadResult(MF1);
B1set = int8([B1_can.result1, B1_can.result2, B1_can.result3]);
nB1 = uint8(0);
for i = 1:3
    if B1set(i) ~= 0
        nB1 = nB1 + 1;
    end
end
if nB1 == 0
    return;
end

% 候选 B2
B2_can = MFNum_ToRoadResult(MF2);
B2set = int8([B2_can.result1, B2_can.result2, B2_can.result3]);
nB2 = uint8(0);
for i = 1:3
    if B2set(i) ~= 0
        nB2 = nB2 + 1;
    end
end

% 候选 bestBMF1（最多两解）
bestBMF1_can = MFNum_ToCatchRoadResult(MF1);
bestMF1set = int8([bestBMF1_can.result1, bestBMF1_can.result2]);
fprintf("bestMF1set:%d,%d\n", bestMF1set(1), bestMF1set(2));
nbestBMF1 = uint8(0);
for i = 1:2
    if bestMF1set(i) ~= 0
        nbestBMF1 = nbestBMF1 + 1;
    end
end
if nbestBMF1 == 0
    return;
end

% 候选 bestBMF2（最多两解）
bestBMF2_can = MFNum_ToCatchRoadResult(MF2);
bestBMF2set = int8([bestBMF2_can.result1, bestBMF2_can.result2]);
fprintf("bestBMF2set:%d,%d\n", bestBMF2set(1), bestBMF2set(2));
nbestBMF2 = uint8(0);
for i = 1:2
    if bestBMF2set(i) ~= 0
        nbestBMF2 = nbestBMF2 + 1;
    end
end

% 可选入口集合（外圈通道格）
entrances = int8(zeros(1,30));
eCount = uint8(0);

isBelow  = (robotPos(2) < C.MapNum_RealPos(1,2));
isAbove  = (robotPos(2) > C.MapNum_RealPos(30,2));
isInside = (~isBelow && ~isAbove);

if isBelow
    for m = 1:5
        if IsWalkable(m)
            eCount = eCount + 1;
            entrances(eCount) = int8(m);
        end
    end
elseif isAbove
    for m = 26:30
        if IsWalkable(m)
            eCount = eCount + 1;
            entrances(eCount) = int8(m);
        end
    end
else
    eCount = 1; % 林内无需入口，以B1为起点
    entrances(1) = int8(0);
end

bestCost = 1.0e9;
bestE = int8(0); bestB1 = int8(0); bestB2 = int8(0);
bestBMF1 = int8(0); bestBMF2 = int8(0);
fprintf("eCout:%d\n", eCount);
% 全组合搜索全局最优
for ie = 1:double(eCount)
    fprintf("尝试入口:%d\n", entrances(ie));
    E = entrances(ie);
    d_out = 0;

    if isInside
        E = int8(0); % 林内无需入口，以B1为起点
        d_out = 0.0;
    end

    for i1 = 1:double(nB1)
        B1 = B1set(i1);
        if B1 == 0
            continue;
        end

        % sE1 = BFS_Steps(E, B1);

        if isInside
            d_out = euclid(robotPos, MapCenterWorld(B1)); % robot→B1 欧氏
            sE1 = 0; % 林内无需入口，入口到B1步数为0

        else
            d_out = euclid(robotPos, MapCenterWorld(E)); % robot→入口 欧氏
            sE1 = BFS_Steps(E, B1);
        end



        if sE1 >= C.BFS_INF
            continue;
        end

        % BMF1 必须与 B1 4-邻接
        for m1 = 1:double(nbestBMF1)
            BMF1 = bestMF1set(m1);
            if BMF1 == 0
                fprintf("BMF1 is zero at index %d\n", m1);
                continue;
            end
            if ~IsAdjacent4(B1, BMF1)
                fprintf("B1 :%d not adj to BMF1:%d\n", B1, BMF1);
                continue;
            end

            s1m1 = BFS_Steps(B1, BMF1);
            if s1m1 >= C.BFS_INF
                continue;
            end
            if (nB2 == 0) || (nbestBMF2 == 0)
                % 无第二段：E→B1→BMF1→Exit
                s_m1_X = BFS_Steps(BMF1, out.exitMap);
                if s_m1_X >= C.BFS_INF
                    continue;
                end

                J = d_out + C.CELL_M * double(sE1 + s1m1 + s_m1_X);
               
                if J < bestCost
                    bestCost = J;
                    bestE = E;
                    bestB1 = B1;
                    bestBMF1 = BMF1;
                    bestB2 = int8(0);
                    bestBMF2 = int8(0);
                    found = true;
                end
                continue;
            end

            % 有第二段：E→B1→BMF1→B2→BMF2→Exit
            for i2 = 1:double(nB2)
                B2 = B2set(i2);
                if B2 == 0
                    continue;
                end

                s_m1_2 = BFS_Steps(BMF1, B2);
                if s_m1_2 >= C.BFS_INF
                    continue;
                end

                % BMF2 必须与 B2 4-邻接
                for m2 = 1:double(nbestBMF2)
                    BMF2 = bestBMF2set(m2);
                    if BMF2 == 0
                        fprintf("BMF2 is zero at index %d\n", m2);
                        continue;
                    end
                    if ~IsAdjacent4(B2, BMF2)
                        fprintf("B2 :%d not adj to BMF2:%d\n", B2, BMF2);
                        continue;
                    end

                    s_2_m2 = BFS_Steps(B2, BMF2);
                    if s_2_m2 >= C.BFS_INF
                        continue;
                    end

                    s_m2_X = BFS_Steps(BMF2, out.exitMap);
                    if s_m2_X >= C.BFS_INF
                        continue;
                    end

                    J = d_out + C.CELL_M * double(sE1 + s1m1 + s_m1_2 + s_2_m2 + s_m2_X);
                    fprintf("J:%f\n", J);
                    if J < bestCost
                        bestCost = J;
                        bestE = E;
                        bestB1 = B1;
                        bestBMF1 = BMF1;
                        bestB2 = B2;
                        bestBMF2 = BMF2;
                        found = true;
                    end
                end
            end
        end
    end
end

% 回退策略：若没有任何可达链路
if ~found
    % 简单回退：选离机器人最近的入口；再选入口→B1 步数最小；再选 B1→B2 最小
    if eCount == 0
        out.entranceMap = int8(0);
        out.bestB1 = int8(0);
        out.bestB2 = int8(0);
        out.bestBMF1 = int8(0);
        out.bestBMF2 = int8(0);
        return;
    end

    bestD = 1.0e9;
    bestE = entrances(1);
    for ie = 1:double(eCount)
        d = euclid(robotPos, MapCenterWorld(entrances(ie)));
        if d < bestD
            bestD = d;
            bestE = entrances(ie);
        end
    end

    bestS1 = C.BFS_INF;
    for i1 = 1:double(nB1)
        B1 = B1set(i1);
        s = BFS_Steps(bestE, B1);
        if s < bestS1
            bestS1 = s;
            bestB1 = B1;
        end
    end

    if nB2 > 0
        bestS2 = C.BFS_INF;
        for i2 = 1:double(nB2)
            B2 = B2set(i2);
            s = BFS_Steps(bestB1, B2);
            if s < bestS2
                bestS2 = s;
                bestB2 = B2;
            end
        end
    end

    % 为回退分支补充 BMF1/BMF2（各自需与 B1/B2 四邻接）
    % 选择使剩余代价最小的相邻通道
    % 1) BMF1
    bestCost_m1 = C.BFS_INF;
    for m1 = 1:double(nbestBMF1)
        cand = bestMF1set(m1);
        if cand == 0
            continue;
        end
        if ~IsAdjacent4(bestB1, cand)
            continue;
        end
        s = BFS_Steps(bestB1, cand) + BFS_Steps(cand, out.exitMap);
        if s < bestCost_m1
            bestCost_m1 = s;
            bestBMF1 = cand;
        end
    end
    % 2) BMF2（若存在第二段）
    if (nB2 > 0) && (bestB2 ~= 0)
        bestCost_m2 = C.BFS_INF;
        for m2 = 1:double(nbestBMF2)
            cand = bestBMF2set(m2);
            if cand == 0
                continue;
            end
            if ~IsAdjacent4(bestB2, cand)
                continue;
            end
            s = BFS_Steps(bestB2, cand) + BFS_Steps(cand, out.exitMap);
            if s < bestCost_m2
                bestCost_m2 = s;
                bestBMF2 = cand;
            end
        end
    end
    fprintf('进入回退策略！\n');
end

out.entranceMap = bestE;
out.bestB1 = bestB1;
out.bestB2 = bestB2;
out.bestBMF1 = bestBMF1;
out.bestBMF2 = bestBMF2;
end