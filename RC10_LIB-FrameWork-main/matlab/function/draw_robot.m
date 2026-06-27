function [patchBase, patchTurret, lineFix, lineExt] = draw_robot(x, y, yaw, theta, L, h, base_size, base_thick, turret_r)
% 底盘：扁平盒
patchBase = draw_box([x,y,base_thick/2], [base_size, base_size, base_thick], [0.2 0.4 1.0], 0.4);

% 云台立柱（放在底盘上沿中点, 高度随 h）
[xt, yt] = turret_mount_xy(x, y, yaw, base_size);
patchTurret = draw_cylinder([xt, yt, 0], turret_r, h, [0.6 0.6 0.6], 1.0);

% 机械臂：水平线段，固定段红色，伸长段蓝色
L_fix = min(L, 0.67);                 % 固定段（最短长度）
L_ext = max(L - 0.67, 0.0);           % 伸长段（蓝色）
phi = yaw + theta;
x1 = xt; y1 = yt;
x2 = x1 + L_fix*cos(phi); y2 = y1 + L_fix*sin(phi);
x3 = x2 + L_ext*cos(phi); y3 = y2 + L_ext*sin(phi);

lineFix = plot3([x1 x2],[y1 y2],[h h],'r-','LineWidth',3);
lineExt = plot3([x2 x3],[y2 y3],[h h],'b-','LineWidth',3);
end

function h = draw_cylinder(baseCenter, radius, height, colorRGB, alpha)
% 简化为 12 边柱
n = 24;
ang = linspace(0,2*pi,n+1); ang(end)=[];
xb = baseCenter(1) + radius*cos(ang);
yb = baseCenter(2) + radius*sin(ang);
zb = zeros(1,n) + baseCenter(3);
xt = xb; yt = yb; zt = zb + height;

% 侧面
F = [];
V = [xb', yb', zb'; xt', yt', zt'];
for i=1:n
    i2 = mod(i,n)+1;
    F = [F; i, i2, n+i2, n+i];
end
h = patch('Faces',F,'Vertices',V,'FaceColor',colorRGB,'FaceAlpha',alpha,'EdgeColor','none');
end