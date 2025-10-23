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
v_base = 1.0;                   % 0.5m/s
suction_offset = 0.02;           % 吸盘侧面间隙
safety_margin = 0.01;            % 与柱安全裕度
bind_xy_tol   = 0.050;           % 绑定水平容差(米) ← 放宽
bind_h_tol    = 0.030;           % 绑定高度容差(米) ← 放宽
rotate_start_dist = 1.00;        % 接近后才允许旋转
% 预测采样
probe_dt_rot  = 0.01;            % 旋转预测步距(s)
probe_dt_ext  = 0.01;            % 伸展预测步距(s)

% 衍生速率
dt = 0.02; T_total = 15.0;
omega_max = 2*pi / t_spin;                       % rad/s
vL_max    = (L_max - L_min) / max(t_extend,1e-6);
vh_max    = (h_max - h_min) / max(t_lift,  1e-6);

% 旋转触发距离：只有靠近目标桩后才允许开始旋转（云台一开始不要转）
rotate_start_dist = 1.00;        % 可根据林带密度调 0.8~1.2m
% 新增：以“底盘前沿”作为触发判据（沿 +X 行驶，前沿 = x_b + base_size/2）
rotate_trigger_margin_front = 0.20;   % 前沿进入到桩前 0.2m 即触发

% ====== 场景 ======
figure('Color','w'); hold on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z'); view(45,20);
xlim([0 map_size]); ylim([0 map_size]); zlim([0 1.5]); grid on;

map_center = [map_size/2, map_size/2];
[pillars, forest_rect] = create_forest(map_center, nx, ny, cell_size);
cubes = place_cubes(pillars, allowed_cube_ids, cube_ids, cube_size);

% ====== 初始位姿 ======
forest_ymin = forest_rect.y - forest_rect.h/2;
% x_b = forest_rect.x - forest_rect.w/2 - 2.0;
% y_b = forest_ymin - 0.40 - base_size/2;
% 平行林带行进：把“与林带前缘的间隙”从 0.40 改为 0.30（0.30~0.35 皆可）
lane_clearance = 0.30;      % 与林带前缘的间隙（米）
x_b = forest_rect.x - forest_rect.w/2 - 2.0;
y_b = forest_ymin - lane_clearance - base_size/2;
yaw = 0;
theta = 0; L = L_min; h = h_min;

[patchBase, patchTurret, lineArmFix, lineArmExt] = draw_robot(x_b, y_b, yaw, theta, L, h, base_size, base_thick, turret_radius);
hTurretDot = plot3(NaN,NaN,NaN,'ko','MarkerFaceColor','k');   % 云台支点
hTipDot    = plot3(NaN,NaN,NaN,'ro','MarkerFaceColor','r');   % 臂端

% 任务：取第一个立方体
target_idx = 1;
target_cube = cubes(target_idx);
cz = target_cube.zbase + target_cube.h/2;

% 一开始就将高度抬到目标的中心高度
h_target = min(max(cz, h_min), h_max);
h = h_target;
% 同步刷新初始姿态（确保画面也抬高了）
update_robot_pose(patchBase, patchTurret, lineArmFix, lineArmExt, ...
    x_b, y_b, yaw, theta, L, h, base_size, base_thick, turret_radius, L_min);

STATE_ALIGN   = 1;   % 旋转对准立方体所在平面法向
STATE_AIM_EXT = 2;   % 预测触发伸展（只伸长/升降）
STATE_CARRY   = 3;   % 吸附后搬运回底盘
STATE_RETURN = 4;   % 放下后回位（抬高5cm并回初始姿态）
STATE_DONE    = 5;
state = STATE_ALIGN;
grabbed = false;
placed  = false;     % 已放置在底盘上并与底盘绑定
retract_phase = false;      % 吸附后立即回收阶段
theta_home    = 0;          % 待机云台角（初始角）
drop_offset_xy = [0, 0];    % 放置相对底盘中心的XY偏移
place_h        = NaN;       % 放置甲板高度（Z）

% 记录上一帧臂端位置（用于“线段与方块相交”绑定判定）
[xt0, yt0] = turret_mount_xy(x_b, y_b, yaw, base_size);
x_tip_prev = xt0 + L*cos(yaw + theta);
y_tip_prev = yt0 + L*sin(yaw + theta);

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
    r_to_cube = hypot(cx - xt, cy - yt);  % 与目标立方体的当前距离

    switch state
        case STATE_ALIGN
            % 一直保持到目标高度 & 收臂（旋转更安全）
            h_cmd = h_target;
            L_cmd = L_min;

            % 以“底盘前沿进入目标桩前区域”为唯一触发条件，不做任何干涉判断
            x_front = x_b + base_size/2;
            if x_front >= (cx - rotate_trigger_margin_front)
                % 触发后：无条件按最大角速度朝 theta_goal 全速旋转
                dth_des = sign(atan2(sin(theta_goal-theta), cos(theta_goal-theta))) ...
                          * min(omega_max*dt, abs(atan2(sin(theta_goal-theta), cos(theta_goal-theta))));
                theta_cmd = theta + dth_des;      % ≈14.4°/帧（720°/s）
            else
                theta_cmd = theta; % 未触发前不转
            end

            % 对齐判据
            if abs(atan2(sin(theta_goal-theta), cos(theta_goal-theta))) < deg2rad(2)
                state = STATE_AIM_EXT;
            end

        case STATE_AIM_EXT
            % 微调阶段也不做任何碰撞预测，直接全速旋转到位
            ang_rem   = abs(atan2(sin(theta_goal-theta), cos(theta_goal-theta)));
            dth_des   = sign(atan2(sin(theta_goal-theta), cos(theta_goal-theta))) * min(omega_max*dt, ang_rem);
            theta_cmd = theta + dth_des;

            % 高度保持到目标平面
            h_cmd = h_target;

            % 预测对齐时刻的支点位置和接触点（用于决定是否伸展）
            t_rot_rem = abs(atan2(sin(theta_goal-theta), cos(theta_goal-theta))) / max(omega_max,1e-6);
            xtf = xt + v_base * t_rot_rem; ytf = yt;
            dirf = [xtf - cx, ytf - cy]; nrm = hypot(dirf(1),dirf(2)) + 1e-9;
            u = dirf / nrm;
            pick_xy = [cx, cy] - (cube_size/2 + suction_offset) * u;
            phi_goal = yaw + theta_goal;
            vhat = [cos(phi_goal), sin(phi_goal)];
            L_goal_f = dot(pick_xy - [xtf, ytf], vhat);
            L_goal_f = min(max(L_goal_f, L_min), L_max);

            % 调试：显示是否可达（超出 L_max 则本轮必然无法触碰）
            % fprintf('L_goal=%.3f L_max=%.3f reach=%d\n', L_goal_f, L_max, L_goal_f<=L_max+1e-6);

            % 伸展仍保持原策略：预测可行才全速伸，否则保持 L_min
            % 注意：用 theta_goal（对齐后的朝向）做整段预测，避免当前θ未对齐导致“误判不可伸”
            if can_extend_now(xt, yt, yaw, theta_goal, L, L_goal_f, h, vh_max, h_target, v_base, pillars, safety_margin, vL_max, probe_dt_ext)
                L_cmd = L_goal_f;
            else
                L_cmd = L_min;
            end

            % 注意：原先这里的“接触即绑定”提前判定删除
            % 绑定逻辑已移到“速率更新之后”，避免错过接触瞬间
        case STATE_CARRY
            % 3) 搬运回底盘并放置
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

           % 触碰后优先“立即回收”：先把臂收至 L_min，再进行放置流程
           if retract_phase
               L_cmd = L_min;
               if abs(L - L_min) < 1e-3
                   retract_phase = false; % 回收到位，开始靠近并放置
               end
           else
               % 回收到位后，靠近底盘中心上方准备放置
               vec = goal_xy - [xt, yt];
               L_cmd = min(max(norm(vec), L_min), L_max);
           end
            h_cmd = min(max(deck_h, h_min), h_max);

            % 立方体绑定随动（仍在被吸附阶段：跟随臂端）
            if grabbed
                phi_c = yaw + theta;
                x_tip = xt + L*cos(phi_c);
                y_tip = yt + L*sin(phi_c);
                move_patch_center(target_cube.patch, [x_tip, y_tip, h]);
            end

            % 放置判据到位 → 与底盘绑定（保持“放下位置”的相对偏移）
            if ~retract_phase && abs(atan2(sin(theta_goal2-theta), cos(theta_goal2-theta))) < deg2rad(3) ...
                    && (abs(L-L_cmd)+abs(h-h_cmd) < 0.02)
               grabbed = false;
               placed  = true;                     % 开始与底盘绑定
               % 记录放置瞬间的相对偏移（保持落点不变，而不是贴底盘中心）
               phi_c = yaw + theta;
               x_tip = xt + L*cos(phi_c);
               y_tip = yt + L*sin(phi_c);
               drop_offset_xy = [x_tip - x_b, y_tip - y_b];
               place_h = deck_h;
               % 把方块放到当前末端落点的甲板高度
               move_patch_center(target_cube.patch, [x_b + drop_offset_xy(1), y_b + drop_offset_xy(2), place_h]);
               % 进入回位阶段
               state = STATE_RETURN;
            end

        case STATE_RETURN
            % 4) 回位：在放置高度基础上抬高5cm，并回初始姿态待机
            theta_cmd = theta_home;
            L_cmd     = L_min;
            h_cmd     = min(max(place_h + 0.05, h_min), h_max);
            if abs(atan2(sin(theta_cmd-theta), cos(theta_cmd-theta))) < deg2rad(2) ...
                    && abs(L - L_cmd) < 0.01 && abs(h - h_cmd) < 0.01
                state = STATE_DONE;
            end

        case STATE_DONE
            % 待机：保持回位后的姿态不变
            theta_cmd = theta; 
            L_cmd     = L; 
            h_cmd     = h;
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

    % 若已放在底盘上：每帧将方块中心跟随到底盘，但保留“放下位置”的相对偏移
    if placed
        place_h = base_thick + target_cube.h/2;
        move_patch_center(target_cube.patch, [x_b + drop_offset_xy(1), y_b + drop_offset_xy(2), place_h]);
    end

    % ====== 接触即绑定（移到位姿更新之后，更稳）======
    % 使用“两种方式择一命中”：1) 末端到“当前侧面接触点”的距离；2) 末端运动线段与方块AABB相交
    if ~grabbed && ~placed && (state==STATE_ALIGN || state==STATE_AIM_EXT)
        % 当前帧臂端
        phi_now = yaw + theta;
        x_tip_now = xt + L*cos(phi_now);
        y_tip_now = yt + L*sin(phi_now);

        % 1) 侧面接触点距离
        dir_now = [xt - cx, yt - cy]; unow = dir_now / (hypot(dir_now(1),dir_now(2)) + 1e-9);
        pick_xy_now = [cx, cy] - (cube_size/2 + suction_offset) * unow;
        near_hit = hypot(x_tip_now - pick_xy_now(1), y_tip_now - pick_xy_now(2)) < bind_xy_tol;

        % 2) 线段与方块AABB（XY平面）相交（带吸盘厚度）
        rect_center = [cx, cy];
        rect_size   = [cube_size + 2*suction_offset, cube_size + 2*suction_offset];
        seg_hit = seg_rect_intersect([x_tip_prev, y_tip_prev], [x_tip_now, y_tip_now], rect_center, rect_size);

        if (near_hit || seg_hit) && abs(h - cz) < bind_h_tol
            grabbed = true;
            state = STATE_CARRY;
            retract_phase = true;  % 触碰即进入“立即回收”阶段
            % 立即把方块吸附到臂端
            move_patch_center(target_cube.patch, [x_tip_now, y_tip_now, h]);
        end
        % 更新“上一帧臂端”
        x_tip_prev = x_tip_now;
        y_tip_prev = y_tip_now;
    end

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