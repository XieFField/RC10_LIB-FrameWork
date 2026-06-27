function h = draw_ramp(center, baseSize, height, slopeDirection, colorRGB, alpha)
% draw_ramp  函数功能：在三维空间中绘制一个底部为长方形的斜坡（梯形棱柱体）
% 输入参数说明：
%   center        - 1×3 向量，代表斜坡底面中心坐标 [x, y, z]，这里为0.65,9.3,0
%   baseSize      - 1×2 向量，代表斜坡底面尺寸 [长度, 宽度]，这里为两个1.5
%   height        - 标量，代表斜坡的最大高度（斜坡低侧高度为0），这里为0.4
%   slopeDirection - 字符串，代表斜坡方向：'x+'、'x-'、'y+'、'y-'，这里为
%   colorRGB      - 1×3 向量，代表斜坡的 RGB 颜色，先设为
%   alpha         - 标量，代表斜坡的透明度
% 
% 输出参数说明：
%   h             - 返回 patch 对象的句柄

% 解析参数
x0 = center(1);  % 底面中心 x 坐标
y0 = center(2);  % 底面中心 y 坐标
z0 = center(3);  % 底面高度

L = baseSize(1); % 底面长度
W = baseSize(2); % 底面宽度
H = height;      % 斜坡高度

% 计算底面半尺寸
hL = L/2;  % 底面半长
hW = W/2;  % 底面半宽

% 根据斜坡方向确定顶点坐标
% 8个顶点：底面4个顶点 + 顶面4个顶点
switch slopeDirection
    case 'x+'  % 沿x轴正方向斜坡（低侧在-x，高侧在+x）
        % 底面4个顶点（z=0）
        V_bottom = [x0 - hL, y0 - hW, z0;   % 顶点1: 底面左下
                    x0 + hL, y0 - hW, z0;   % 顶点2: 底面右下  
                    x0 + hL, y0 + hW, z0;   % 顶点3: 底面右上
                    x0 - hL, y0 + hW, z0];  % 顶点4: 底面左上
        
        % 顶面4个顶点（斜坡：左侧低，右侧高）
        V_top = [x0 - hL, y0 - hW, z0;       % 顶点5: 顶面左下（低侧）
                 x0 + hL, y0 - hW, z0 + H;   % 顶点6: 顶面右下（高侧）
                 x0 + hL, y0 + hW, z0 + H;   % 顶点7: 顶面右上（高侧）
                 x0 - hL, y0 + hW, z0];      % 顶点8: 顶面左上（低侧）
             
    case 'x-'  % 沿x轴负方向斜坡（高侧在-x，低侧在+x）
        V_bottom = [x0 - hL, y0 - hW, z0;   % 顶点1: 底面左下
                    x0 + hL, y0 - hW, z0;   % 顶点2: 底面右下  
                    x0 + hL, y0 + hW, z0;   % 顶点3: 底面右上
                    x0 - hL, y0 + hW, z0];  % 顶点4: 底面左上
        
        V_top = [x0 - hL, y0 - hW, z0 + H;   % 顶点5: 顶面左下（高侧）
                 x0 + hL, y0 - hW, z0;       % 顶点6: 顶面右下（低侧）
                 x0 + hL, y0 + hW, z0;       % 顶点7: 顶面右上（低侧）
                 x0 - hL, y0 + hW, z0 + H];  % 顶点8: 顶面左上（高侧）
             
    case 'y+'  % 沿y轴正方向斜坡（低侧在-y，高侧在+y）
        V_bottom = [x0 - hL, y0 - hW, z0;   % 顶点1: 底面左下
                    x0 + hL, y0 - hW, z0;   % 顶点2: 底面右下  
                    x0 + hL, y0 + hW, z0;   % 顶点3: 底面右上
                    x0 - hL, y0 + hW, z0];  % 顶点4: 底面左上
        
        V_top = [x0 - hL, y0 - hW, z0;       % 顶点5: 顶面左下（低侧）
                 x0 + hL, y0 - hW, z0;       % 顶点6: 顶面右下（低侧）
                 x0 + hL, y0 + hW, z0 + H;   % 顶点7: 顶面右上（高侧）
                 x0 - hL, y0 + hW, z0 + H];  % 顶点8: 顶面左上（高侧）
             
    case 'y-'  % 沿y轴负方向斜坡（高侧在-y，低侧在+y）
        V_bottom = [x0 - hL, y0 - hW, z0;   % 顶点1: 底面左下
                    x0 + hL, y0 - hW, z0;   % 顶点2: 底面右下  
                    x0 + hL, y0 + hW, z0;   % 顶点3: 底面右上
                    x0 - hL, y0 + hW, z0];  % 顶点4: 底面左上
        
        V_top = [x0 - hL, y0 - hW, z0 + H;   % 顶点5: 顶面左下（高侧）
                 x0 + hL, y0 - hW, z0 + H;   % 顶点6: 顶面右下（高侧）
                 x0 + hL, y0 + hW, z0;       % 顶点7: 顶面右上（低侧）
                 x0 - hL, y0 + hW, z0];      % 顶点8: 顶面左上（低侧）
             
    otherwise
        error('方向参数必须是: ''x+'', ''x-'', ''y+'', ''y-''');
end

% 合并所有顶点（底面4个 + 顶面4个 = 总共8个顶点）
V = [V_bottom; V_top];

% 定义斜坡的面（Faces）索引 - 总共6个面
F = [1, 2, 3, 4;       % 底面（四边形）
     5, 6, 7, 8;       % 顶面（四边形，实际是斜面）
     1, 2, 6, 5;       % 前面（四边形）
     2, 3, 7, 6;       % 右面（四边形）
     3, 4, 8, 7;       % 后面（四边形）
     4, 1, 5, 8];      % 左面（四边形）

% 调用 patch 函数绘制斜坡
h = patch('Faces', F, 'Vertices', V, ...
          'FaceColor', colorRGB, ...
          'FaceAlpha', alpha, ...
          'EdgeColor', [0.2, 0.2, 0.2]);

% 添加斜坡方向标注
text(x0, y0, z0 + H/2, slopeDirection, ...
     'HorizontalAlignment', 'center', ...
     'Color', [0, 0, 0], ...
     'FontSize', 8, ...
     'FontWeight', 'bold');
end