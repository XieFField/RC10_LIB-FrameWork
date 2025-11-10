clear; clc; close all;

% 把已有 create_forest.m 加到路径

addpath(genpath('F:\MyProjectFlies\STM32H7\Frame_T\matlab_MFctrlerTest\function'));
addpath('F:\MyProjectFlies\STM32H7\Frame_T\matlab\function');


C = mf_consts(); % 常量与坐标
CELL = C.CELL_M;

% 生成 3×4 梅花林
center = [3.0, 5.6];
nx = 3; ny = 4;
[pillars, forest_rect] = create_forest(center, nx, ny, CELL); 

% 绘制通道网格
hold on; axis([-1.2 6 -1.2 12]); grid on; view(2);
title('梅花林通道与最优路径');
xlabel('X (m)'); ylabel('Y (m)');
set(gca,'DataAspectRatio',[1 1 1]);   % 等比例
% 画出 5×6 所有格中心点与编号
P = C.MapNum_RealPos; % 30×3
for k = 1:30
    p = P(k, :);

    if IsWalkable(k)
        col = [0.85 0.95 1.0];
    else
        col = [0.85 0.85 0.85];
    end

    rectangle('Position', [p(1)-CELL/2, p(2)-CELL/2, CELL, CELL], ...
              'FaceColor', col, 'EdgeColor', [0.5 0.5 0.5]);
    text(p(1), p(2), num2str(k), 'HorizontalAlignment','center', ...
         'Color', [0 0 0], 'FontSize', 8);
end

% 输入：机器人位置与目标桩
robotPos = [0.6, 1, 0]; 
MF1 = 1;
MF2 = 9;

% 计算最优点位
R = PathNodeResult_calc(robotPos, MF1, MF2);

% 打印结果
fprintf('Entrance Map: %d\n', R.entranceMap);
fprintf('Best B1: %d\n', R.bestB1);
fprintf('Best BMF1: %d\n', R.bestBMF1);
fprintf('Best B2: %d\n', R.bestB2);
fprintf('Best BMF2: %d\n', R.bestBMF2);
fprintf('Exit Map: %d\n', R.exitMap);

% 链接起点与入口
if R.entranceMap ~= 0
    entryIdx = R.entranceMap;
else
    entryIdx = R.bestB1; % 林内：入口与 B1 相同
end
if entryIdx ~= 0
    epos = C.MapNum_RealPos(entryIdx, :);
    plot([robotPos(1) epos(1)], [robotPos(2) epos(2)], 'k--', 'LineWidth', 1.5);
end
% 标注机器人起点
plot3(robotPos(1), robotPos(2), 0.01, '^', 'MarkerFaceColor', [0 0 0], ...
      'MarkerEdgeColor', 'w', 'MarkerSize', 9);

% 构建必须经过的序列（过滤 0）
seq = [R.entranceMap, R.bestB1, R.bestBMF1, R.bestB2, R.bestBMF2, R.exitMap];
seq = seq(seq~=0);

% 若入口为 0（林内情况），确保从 B1 开始
if R.entranceMap == 0 && R.bestB1 ~= 0
    seq = [R.bestB1, R.bestBMF1, R.bestB2, R.bestBMF2, R.exitMap];
    seq = seq(seq~=0);
end

% 生成路径
gridPath = BuildFullPath(seq);

% 绘制路径
DrawGridPath(gridPath);

% 标注关键点（保持不变）
mark = @(idx, str, c) plot3(C.MapNum_RealPos(idx,1), C.MapNum_RealPos(idx,2), 0.01, ...
    'o', 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k', 'MarkerSize', 8);
if R.entranceMap~=0, mark(R.entranceMap,'E',[0.2 0.6 1.0]); end
if R.bestB1~=0,      mark(R.bestB1,'B1',[1.0 0.7 0.2]); end
if R.bestBMF1~=0,    mark(R.bestBMF1,'M1',[0.9 0.1 0.1]); end
if R.bestB2~=0,      mark(R.bestB2,'B2',[0.4 0.8 0.4]); end
if R.bestBMF2~=0,    mark(R.bestBMF2,'M2',[0.1 0.5 0.1]); end
mark(R.exitMap,'X',[0.5 0 0.9]);

hold off;