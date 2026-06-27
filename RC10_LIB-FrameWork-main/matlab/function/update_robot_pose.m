function update_robot_pose(pBase, pTurret, lFix, lExt, x, y, yaw, theta, L, h, base_size, base_thick, turret_r, L_min, L_draw_override)
% 更新底盘
set_box_pose(pBase, [x,y,base_thick/2], [base_size, base_size, base_thick], yaw);

% 更新云台
[xt, yt] = turret_mount_xy(x, y, yaw, base_size);
set_cylinder_pose(pTurret, [xt, yt, 0], turret_r, h);

% 可视长度：默认用 L；若提供了 L_draw_override，则用覆写值
if nargin >= 15 && ~isempty(L_draw_override)
    L_vis = L_draw_override;
else
    L_vis = L;
end

% 直臂分段（固定段/伸出段）基于 L_vis 计算
L_fix = min(L_vis, L_min);
L_ext = max(L_vis - L_min, 0.0);
phi = yaw + theta;

x1 = xt; y1 = yt;
x2 = x1 + L_fix*cos(phi); y2 = y1 + L_fix*sin(phi);
x3 = x2 + L_ext*cos(phi); y3 = y2 + L_ext*sin(phi);

set(lFix, 'XData',[x1 x2], 'YData',[y1 y2], 'ZData',[h h]);
set(lExt, 'XData',[x2 x3], 'YData',[y2 y3], 'ZData',[h h]);
end

function set_box_pose(p, center, sizeXYZ, yaw)
% 重新计算 8 顶点（仅 yaw 绕 Z 旋转）
x = center(1); y = center(2); z = center(3);
w = sizeXYZ(1); d = sizeXYZ(2); hgt = sizeXYZ(3);
hx = w/2; hy = d/2; hz = hgt/2;

Vloc = [ -hx -hy -hz;
          -hx  hy -hz;
           hx  hy -hz;
           hx -hy -hz;
          -hx -hy  hz;
          -hx  hy  hz;
           hx  hy  hz;
           hx -hy  hz ];
R = [cos(yaw) -sin(yaw) 0;
     sin(yaw)  cos(yaw) 0;
     0         0        1];
V = (R*Vloc')' + [x y z];
set(p, 'Vertices', V);
end

function set_cylinder_pose(pCyl, baseCenter, r, h)
% 将圆柱整体平移到 baseCenter，并更新高度为 h
V = get(pCyl,'Vertices');
n = size(V,1)/2;             % 下环顶点数
% 当前底面中心（XY 取下环顶点平均）
cx = mean(V(1:n,1)); cy = mean(V(1:n,2));
dx = baseCenter(1) - cx; dy = baseCenter(2) - cy;
% 平移 XY
V(:,1) = V(:,1) + dx;
V(:,2) = V(:,2) + dy;
% 更新 Z：底面 = baseCenter(3)，顶面 = 底面 + h
V(1:n,3)     = baseCenter(3);
V(n+1:end,3) = baseCenter(3) + h;
set(pCyl,'Vertices',V);
end