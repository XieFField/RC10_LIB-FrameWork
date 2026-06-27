%% 1. 清空环境 & 设置场地尺寸、网格划分
clear; clc; close all;
L = 2400; % 场地总长度（x方向，单位可自定义，如 mm）
W = 2400; % 场地总宽度（y方向）
Nx = 48; % x方向分成的段数 → 每段长度 dx = L/Nx = 50
Ny = 48; % y方向分成的段数 → 每段长度 dy = W/Ny = 50
dx = L / Nx;
dy = W / Ny;
% 生成网格坐标矩阵 X, Y
[X, Y] = meshgrid(0:dx:L, 0:dy:W);
Z = zeros(size(X)); % 初始化高度矩阵，后续按区域赋值
%% 2. 根据图纸尺寸，手动分析每个矩形块的 (x范围,y范围)，并赋高度
% 下面的“区间计算”需结合图纸中各块的宽度/高度，逐一对应到网格索引。
% 为方便演示，这里给不同区块赋不同的“示例高度”（1,2,3,...），您可按需修改。
% ------------------------------
% 【示例】左侧黄色小方块：宽度36，高度36 → x∈[0,36], y∈[0,36]
% 计算在网格中占据的列范围(ix_start:ix_end)、行范围(iy_start:iy_end)
ix_yellow = 1 : floor(36/dx); % 第 1 列 ~ 第 floor(36/50)=1 列（因为36<50）
iy_yellow = 1 : floor(36/dy); % 第 1 行 ~ 第 floor(36/50)=1 行
Z(iy_yellow, ix_yellow) = 1; % 给该区域赋高度 1
% ------------------------------
% 【示例】中间横向灰色长条带（假设在 y∈[500,1900] 附近，x∈[36,36+500]）
% 先算 x 范围对应的列：36 ≤ x ≤ 36+500 → 列索引 floor(36/50)+1 : floor((36+500)/50)
ix_gray1 = floor(36/dx)+1 : floor((36+500)/dx);
% 再算 y 范围对应的行：500 ≤ y ≤ 1900 → 行索引 floor(500/50)+1 : floor(1900/50)
iy_gray1 = floor(500/dy)+1 : floor(1900/dy);
Z(iy_gray1, ix_gray1) = 2; % 给该区域赋高度 2
% ------------------------------
% 【示例】右侧上方红色小方块：宽度500，高度400 → x∈[L-500,L], y∈[H-400,H]
ix_red_top = floor((L-500)/dx)+1 : Nx; % 右端 x: L-500 ≤ x ≤ L → 列索引 floor((2400-500)/50)+1 : 48
iy_red_top = floor((W-400)/dy)+1 : Ny; % 上端 y: W-400 ≤ y ≤ W → 行索引 floor((2400-400)/50)+1 : 48
Z(iy_red_top, ix_red_top) = 3; % 给该区域赋高度 3
% ------------------------------
% 【示例】右侧下方红色小方块：宽度500，高度400 → x∈[L-500,L], y∈[0,W-400]
ix_red_bottom = floor((L-500)/dx)+1 : Nx; % 右端 x 范围同上
iy_red_bottom = 1 : floor((W-400)/dy); % 下端 y: 0 ≤ y ≤ W-400 → 行索引 1 : floor((2400-400)/50)
Z(iy_red_bottom, ix_red_bottom) = 4; % 给该区域赋高度 4
% ------------------------------
% 【示例】中间紫色小方块：宽度500，高度30 → x∈[L/2-250,L/2+250], y∈[W/2-15,W/2+15]
ix_purple = floor((L/2 - 250)/dx)+1 : floor((L/2 + 250)/dx); % 中间 x: L/2-250 ~ L/2+250 → 列索引计算
iy_purple = floor((W/2 - 15)/dy)+1 : floor((W/2 + 15)/dy); % 中间 y: W/2-15 ~ W/2+15 → 行索引计算
Z(iy_purple, ix_purple) = 5; % 给该区域赋高度 5
% ------------------------------
% 【示例】右侧灰色小竖块：宽度150，高度≈全场 → x∈[L-150,L], y∈[0,W]
ix_smallGray = floor((L-150)/dx)+1 : Nx; % 右端 x: L-150 ≤ x ≤ L → 列索引计算
iy_smallGray = 1 : Ny; % 全 y 范围 → 行索引 1:Ny
Z(iy_smallGray, ix_smallGray) = 6; % 给该区域赋高度 6
% ------------------------------
% 【示例】中间灰色小竖块：宽度100，高度≈全场 → x∈[L/2-50,L/2+50], y∈[0,W]
ix_midGray = floor((L/2 - 50)/dx)+1 : floor((L/2 + 50)/dx); % 中间 x: L/2-50 ~ L/2+50 → 列索引计算
iy_midGray = 1 : Ny; % 全 y 范围 → 行索引 1:Ny
Z(iy_midGray, ix_midGray) = 7; % 给该区域赋高度 7
% ------------------------------
% （可选）若想让同一区块内有随机起伏，可叠加噪声：
% Z(iy_yellow, ix_yellow) = 1 + 0.2*randn(size(Z(iy_yellow, ix_yellow)));
%% 3. 三维地形可视化
figure('Color','w');
% 子图1：surf 曲面图
subplot(1,2,1);
surf(X, Y, Z, 'EdgeColor','none');
shading interp; % 让曲面更平滑
title('3D 地形曲面');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('高度');
view(30,30); % 设置视角
grid on; axis equal; % 显示网格、等轴比
% 子图2：imagesc 伪彩色平面图 + colorbar
subplot(1,2,2);
imagesc(Z');
axis equal tight; % 等轴比、紧凑布局
colormap(jet); % 使用 jet 色图（也可换为 hot, parula 等）
colorbar; % 显示颜色条
title('伪彩色平面示意图');
xlabel('X 网格索引'); ylabel('Y 网格索引');
set(gca,'YDir','normal'); % 让 y 轴正方向朝上（默认 imagesc 是翻转的）
