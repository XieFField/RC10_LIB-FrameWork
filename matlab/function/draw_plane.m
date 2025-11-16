function h = draw_plane(origin, normal, planeSize, colorRGB, alpha, edgeColor)
% draw_plane  函数功能：在三维空间中绘制一个平面并设置颜色
% 
% 输入参数说明：
%   origin    - 1×3 向量，平面中心点坐标 [x, y, z]
%   normal    - 1×3 向量，平面的法向量 [nx, ny, nz]
%   planeSize - 1×2 向量，平面的尺寸 [长度, 宽度]
%   colorRGB  - 1×3 向量，平面的 RGB 颜色
%   alpha     - 标量，平面的透明度（0=完全透明，1=完全不透明）
%   edgeColor - 1×3 向量，边框颜色（可选，默认为深灰色）
% 
% 输出参数说明：
%   h         - 返回 patch 对象的句柄

% 设置默认边框颜色
if nargin < 6
    edgeColor = [0.2, 0.2, 0.2];
end

% 解析参数
x0 = origin(1); y0 = origin(2); z0 = origin(3);
L = planeSize(1); W = planeSize(2);
nx = normal(1); ny = normal(2); nz = normal(3);

% 归一化法向量
normal_vec = [nx, ny, nz];
normal_vec = normal_vec / norm(normal_vec);

% 计算平面的局部坐标系
% 如果法向量接近z轴，使用x轴作为参考
if abs(normal_vec(3)) > 0.9
    ref_vec = [1, 0, 0];
else
    ref_vec = [0, 0, 1];
end

% 计算局部坐标系的x方向和y方向
local_x = cross(ref_vec, normal_vec);
local_x = local_x / norm(local_x);
local_y = cross(normal_vec, local_x);
local_y = local_y / norm(local_y);

% 计算平面的四个角点
hL = L/2; hW = W/2;

% 在局部坐标系中计算角点，然后转换到世界坐标系
corners_local = [ hL,  hW, 0;   % 右上
                 -hL,  hW, 0;   % 左上
                 -hL, -hW, 0;   % 左下
                  hL, -hW, 0];  % 右下

% 转换到世界坐标系
V = zeros(4, 3);
for i = 1:4
    V(i, :) = origin + corners_local(i, 1) * local_x + ...
                         corners_local(i, 2) * local_y;
end

% 定义面的顶点索引
F = [1, 2, 3, 4];  % 单个四边形面

% 绘制平面
h = patch('Faces', F, 'Vertices', V, ...
          'FaceColor', colorRGB, ...
          'FaceAlpha', alpha, ...
          'EdgeColor', edgeColor, ...
          'LineWidth', 1.5);
end