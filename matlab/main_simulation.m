clc; clear; close all;

% 路径
thisdir = fileparts(mfilename('fullpath'));
addpath(thisdir);
addpath(fullfile(thisdir,'function'));

% ====== 参数 ======
map_size = 12.0;
cell_size = 1.2; nx = 3; ny = 4;
allowed_cube_ids = [1,2,3,4,6,7,9,10,11,12];
cube_ids = [2,7,11];
cube_size = 0.35;

% 机器人参数
base_size = 0.8; base_thick = 0.08;
turret_radius = 0.06;
h_min = 0.375; h_max = 0.775;
L_min = 0.67;  L_max = 0.80;     % 伸长量=0.13m
t_lift = 0.40;
t_extend = 0.15;                 % 0.13m/0.15s
t_spin   = 0.50;                 % 0.5s/圈
v_base = 0.50;                   % 0.5m/s
suction_offset = 0.02;           % 吸盘侧面间隙
safety_margin = 0.01;            % 与柱安全裕度
bind_xy_tol   = 0.030;           % 绑定水平容差(米)
bind_h_tol    = 0.020;           % 绑定高度容差(米)
% 预测采样
probe_dt_rot  = 0.01;            % 旋转预测步距(s)
probe_dt_ext  = 0.01;            % 伸展预测步距(s)

% 衍生速率
dt = 0.02; T_total = 15.0;
omega_max = 2*pi / t_spin;                       % rad/s
vL_max    = (L_max - L_min) / max(t_extend,1e-6);
vh_max    = (h_max - h_min) / max(t_lift,  1e-6);

% ====== 场景 ======
figure('Color','w'); hold on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z'); view(45,20);
xlim([0 map_size]); ylim([0 map_size]); zlim([0 1.5]); grid on;

map_center = [map_size/2, map_size/2];
[pillars, forest_rect] = create_forest(map_center, nx, ny, cell_size);
cubes = place_cubes(pillars, allowed_cube_ids, cube_ids, cube_size);

% ====== 初始位姿 ======
forest_ymin = forest_rect.y - forest_rect.h/2;
x_b = forest_rect.x - forest_rect.w/2 - 2.0;
y_b = forest_ymin - 0.40 - base_size/2;
yaw = 0;
theta = 0; L = L_min; h = h_min;

[patchBase, patchTurret, lineArmFix, lineArmExt] = draw_robot(x_b, y_b, yaw, theta, L, h, base_size, base_thick, turret_radius);
hTurretDot = plot3(NaN,NaN,NaN,'ko','MarkerFaceColor','k');   % 云台支点
hTipDot    = plot3(NaN,NaN,NaN,'ro','MarkerFaceColor','r');   % 臂端

% 任务：取第一个立方体
target_idx = 1;
target_cube = cubes(target_idx);
cz = target_cube.zbase + target_cube.h/2;

STATE_ALIGN   = 1;   % 旋转对准立方体所在平面法向
STATE_AIM_EXT = 2;   % 预测触发伸展（只伸长/升降）
STATE_CARRY   = 3;   % 吸附后搬运回底盘
STATE_DONE    = 4;
state = STATE_ALIGN;
grabbed = false;

title('运动规划仿真');

% ====== 主循环 ======
for t = 0:dt:T_total
    % 底盘直行
    x_b = x_b + v_base * dt;
    [xt, yt] = turret_mount_xy(x_b, y_b, yaw, base_size);
    set(hTurretDot,'XData',xt,'YData',yt,'ZData',h);

    % 默认保持
    theta_cmd = theta; L_cmd = L; h_cmd = h;

    % 云台目标朝向：立方体所在平面法向（从支点指向立方体）
    cx = target_cube.x; cy = target_cube.y;
    theta_goal = wrapTo2Pi(atan2(cy - yt, cx - xt) - yaw);

    switch state
        case STATE_ALIGN
            % 目标朝向：支点→方块中心的法向
            dth_des = sign(atan2(sin(theta_goal-theta), cos(theta_goal-theta))) ...
                      * min(omega_max*dt, abs(atan2(sin(theta_goal-theta), cos(theta_goal-theta))));
            % 先抬到方块中心高度（更快评估为可旋转）
            h_cmd = min(max(cz, h_min), h_max);
            L_cmd = L_min; % 旋转时收臂最安全

            % 整段预测：若从现在起全速旋到 theta_goal 全程不撞桩 → 允许本帧全速旋转
            if can_rotate_now(xt, yt, yaw, theta, theta_goal, h, vh_max, h_cmd, v_base, L_min, pillars, safety_margin, omega_max, probe_dt_rot)
                theta_cmd = theta + dth_des;   % 放行：全速转
            else
                theta_cmd = theta;             % 禁转：等待更安全的时机
            end

            % 对齐判据
            if abs(atan2(sin(theta_goal-theta), cos(theta_goal-theta))) < deg2rad(2)
                state = STATE_AIM_EXT;
            end

        case STATE_AIM_EXT
            % 保持对齐（微调时也用整段预测，允许则按满速微调）
            ang_rem   = abs(atan2(sin(theta_goal-theta), cos(theta_goal-theta)));
            dth_des   = sign(atan2(sin(theta_goal-theta), cos(theta_goal-theta))) * min(omega_max*dt, ang_rem);
            if can_rotate_now(xt, yt, yaw, theta, theta_goal, h, vh_max, cz, v_base, L_min, pillars, safety_margin, omega_max, probe_dt_rot)
                theta_cmd = theta + dth_des;
            else
                theta_cmd = theta;
            end
            h_cmd = min(max(cz, h_min), h_max);

            % 预测对齐时刻的支点位置和接触点
            t_rot_rem = abs(atan2(sin(theta_goal-theta), cos(theta_goal-theta))) / max(omega_max,1e-6);
            xtf = xt + v_base * t_rot_rem; ytf = yt;
            dirf = [xtf - cx, ytf - cy]; nrm = hypot(dirf(1),dirf(2)) + 1e-9;
            u = dirf / nrm;
            pick_xy = [cx, cy] - (cube_size/2 + suction_offset) * u;
            phi_goal = yaw + theta_goal;
            vhat = [cos(phi_goal), sin(phi_goal)];
            L_goal_f = dot(pick_xy - [xtf, ytf], vhat);
            L_goal_f = min(max(L_goal_f, L_min), L_max);

            % 整段预测伸展：若从现在起按最大伸速伸到 L_goal_f，全程不撞桩 → 允许本帧全速伸展
            if can_extend_now(xt, yt, yaw, theta, L, L_goal_f, h, vh_max, cz, v_base, pillars, safety_margin, vL_max, probe_dt_ext)
                L_cmd = L_goal_f;   % 放行：全速伸
            else
                L_cmd = L_min;      % 禁伸：继续等
            end

            % 接触即绑定（允许接触瞬间穿模并立即绑定）
            phi_now = yaw + theta;
            dir_now = [xt - cx, yt - cy]; unow = dir_now / (hypot(dir_now(1),dir_now(2)) + 1e-9);
            pick_xy_now = [cx, cy] - (cube_size/2 + suction_offset) * unow;
            x_tip = xt + L*cos(phi_now); y_tip = yt + L*sin(phi_now);
            if hypot(x_tip - pick_xy_now(1), y_tip - pick_xy_now(2)) < bind_xy_tol && abs(h - cz) < bind_h_tol
                grabbed = true;
                state = STATE_CARRY;
            end

        case STATE_CARRY
            % 3) 搬运回底盘中心上方并放置
            goal_xy = [x_b, y_b];
            deck_h  = base_thick + target_cube.h/2;

            % 允许云台朝底盘中心旋回（每步防柱）
            theta_goal2 = wrapTo2Pi(atan2(goal_xy(2)-yt, goal_xy(1)-xt) - yaw);
            dth = sign(atan2(sin(theta_goal2-theta), cos(theta_goal2-theta))) * min(omega_max*dt, abs(atan2(sin(theta_goal2-theta), cos(theta_goal2-theta))));
            theta_try = theta + dth; phi_try = yaw + theta_try;
            if arm_hits_pillars(xt, yt, xt + L_min*cos(phi_try), yt + L_min*sin(phi_try), h, pillars, safety_margin)
                theta_cmd = theta;
            else
                theta_cmd = theta_try;
            end

            vec = goal_xy - [xt, yt];
            L_cmd = min(max(norm(vec), L_min), L_max);
            h_cmd = min(max(deck_h, h_min), h_max);

            % 立方体绑定随动
            if grabbed
                phi_c = yaw + theta;
                x_tip = xt + L*cos(phi_c);
                y_tip = yt + L*sin(phi_c);
                move_patch_center(target_cube.patch, [x_tip, y_tip, h]);
            end

            if abs(atan2(sin(theta_goal2-theta), cos(theta_goal2-theta))) < deg2rad(3) && (abs(L-L_cmd)+abs(h-h_cmd) < 0.02)
                grabbed = false; state = STATE_DONE;
            end

        case STATE_DONE
            theta_cmd = theta; L_cmd = L_min; h_cmd = h_min;
    end

    % ====== 速率限制 + 强约束（绝不碰柱）======
    % 角度：直接应用命令（允许时已按满速放行）
    theta_next = theta_cmd;
    % 高度
    h_next = h + sign(h_cmd - h) * min(vh_max*dt, abs(h_cmd - h));
    % 伸长：速率后再裁剪为“当前朝向的最大安全长度”
    dL_max = min(vL_max*dt, abs(L_cmd - L));
    L_try  = L + sign(L_cmd - L) * dL_max;
    phi_for_len = yaw + theta_next;
    L_maxSafe = max_safe_length_along_ray(xt, yt, phi_for_len, h_next, pillars, safety_margin, L_try);
    L_next = min(L_try, L_maxSafe);

    % 应用
    theta = theta_next; L = L_next; h = h_next;

    % 绘制
    update_robot_pose(patchBase, patchTurret, lineArmFix, lineArmExt, ...
        x_b, y_b, yaw, theta, L, h, base_size, base_thick, turret_radius, L_min);

    % 臂端标记
    set(hTipDot,'XData',xt + L*cos(yaw+theta), 'YData', yt + L*sin(yaw+theta), 'ZData', h);

    drawnow;
end

% 辅助：角度差（-pi..pi）
function d = angdiff(a,b)
d = atan2(sin(b-a), cos(b-a));
end

% ====== 本文件末尾：添加安全缩步函数 ======
function dth_safe = shrink_dth_until_safe(dth_des, xt, yt, yaw, theta, L_min, h, pillars, safety_margin)
    % 逐步缩小 dth，直到“最短臂长 L_min 的旋转扫描线”不与任一柱相交
    % 若完全不安全，则返回0（冻结）。这样比“一刀切冻结”更容易“慢速解卡住”
    dth_safe = dth_des;
    phi_try = yaw + (theta + dth_safe);
    if ~arm_hits_pillars(xt, yt, xt + L_min*cos(phi_try), yt + L_min*sin(phi_try), h, pillars, safety_margin)
        return; % 原始步长可用
    end
    % 二分/几何缩小
    max_iter = 12;
    low = 0; high = dth_des; % 在 [-|d|,0] 或 [0,|d|] 内搜索
    for k=1:max_iter
        mid = (low + high)/2;
        phi_mid = yaw + (theta + mid);
        hit = arm_hits_pillars(xt, yt, xt + L_min*cos(phi_mid), yt + L_min*sin(phi_mid), h, pillars, safety_margin);
        if hit
            high = mid; % 缩小步长
        else
            low = mid;  % 能走到这里，尝试再大一点
        end
    end
    dth_safe = low;
end

% ====== 新增：整段预测“是否可从现在开始全速旋转” ======
function ok = can_rotate_now(xt, yt, yaw, theta, theta_goal, h_now, vh_max, h_target, v_base, L_check, pillars, margin, omega_max, probe_dt)
    ok = true;
    d = atan2(sin(theta_goal-theta), cos(theta_goal-theta));
    s = sign(d); ang = abs(d);
    T = ang / max(omega_max,1e-6);
    t = 0.0;
    while t <= T
        th = theta + s*omega_max*min(t, T);
        phi = yaw + th;
        xti = xt + v_base*t; yti = yt;
        % 高度沿途快速抬向 h_target
        hi  = h_now + sign(h_target - h_now) * min(vh_max*t, abs(h_target - h_now));
        if arm_hits_pillars(xti, yti, xti + L_check*cos(phi), yti + L_check*sin(phi), hi, pillars, margin)
            ok = false; return;
        end
        t = t + probe_dt;
    end
end

% ====== 新增：整段预测“是否可从现在开始全速伸展到 L_goal” ======
function ok = can_extend_now(xt, yt, yaw, theta, L_now, L_goal, h_now, vh_max, h_target, v_base, pillars, margin, vL_max, probe_dt)
    ok = true;
    phi = yaw + theta;
    dL  = L_goal - L_now; s = sign(dL); T = abs(dL) / max(vL_max,1e-6);
    t = 0.0;
    while t <= T
        Lti = L_now + s * min(vL_max*t, abs(dL));         % 线性伸长
        xti = xt + v_base*t; yti = yt;                   % 底盘前进
        hi  = h_now + sign(h_target - h_now) * min(vh_max*t, abs(h_target - h_now)); % 抬高
        if arm_hits_pillars(xti, yti, xti + Lti*cos(phi), yti + Lti*sin(phi), hi, pillars, margin)
            ok = false; return;
        end
        t = t + probe_dt;
    end
end