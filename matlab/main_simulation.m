clc; clear; close all;

% 路径
thisdir = fileparts(mfilename('fullpath'));
addpath(thisdir);
addpath(fullfile(thisdir,'function'));

% ====== 参数 ======
map_size = 12.0;
cell_size = 1.2; nx = 3; ny = 4;
allowed_cube_ids = [1,2,3,4,6,7,9,10,11,12];
cube_ids = [3,7,11];
cube_size = 0.35;

% 机器人参数
base_size = 1.0; base_thick = 0.08;
turret_radius = 0.06;
h_min = 0.375; h_max = 0.775;
L_min = 0.67;  L_max = 0.80;     % 伸长量=0.13m
t_lift = 0.40;
t_extend = 0.15;                 % 0.13m/0.15s
t_spin   = 0.50;                 % 0.5s/圈
v_base = 0.90;                   % 0.5m/s
suction_offset = 0.02;           % 吸盘侧面间隙
safety_margin = 0.01;            % 与柱安全裕度
bind_xy_tol   = 0.050;           % 绑定水平容差(米) ← 放宽
bind_h_tol    = 0.030;           % 绑定高度容差(米)
rotate_start_dist = 1.00;
cube_rad      = cube_size/2 + suction_offset;
arm_radius = 0.025;
cup_radius = 0.050;
margin_arm = safety_margin + arm_radius;
margin_cup = safety_margin + cup_radius;
extend_full_130mm = true;       % 预测可命中后，是否一口气伸满 130mm 到 L_max
cup_len         = 0.05;        % 吸盘长度（米）
bind_depth_tol  = 0.005;       % 与侧面“法向距离”容差，用于触发吸附/穿模校正（米）
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
% 以“底盘前沿”作为触发判据（沿 +X 行驶，前沿 = x_b + base_size/2）
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
% 平行林带行进：“与林带前缘的间隙”
lane_clearance = 0.35;      % 与林带前缘的间隙（米）
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

% ====== 目标与“前一柱区域”触发线 ======
target_idx = 1;
target_cube = cubes(target_idx);
cz = target_cube.zbase + target_cube.h/2;

% 一开始就将高度抬到目标的中心高度
h_target = min(max(cz, h_min), h_max);
h = h_target;
update_robot_pose(patchBase, patchTurret, lineArmFix, lineArmExt, ...
    x_b, y_b, yaw, theta, L, h, base_size, base_thick, turret_radius, L_min);

% 计算目标方块所在“柱”的索引，以及“同一排(prev)柱”的区域起始线（沿 +X 行进）
% 规则：进入“前一柱的前方区域”（prev.x - prev.w/2）后，才开始旋转预判；此前保持 theta 不动
eps_row = 1e-6;
% 找到与 target_cube 最近的柱（目标柱）
[~, tgt_p] = min(hypot([pillars.x] - target_cube.x, [pillars.y] - target_cube.y));
tgt_y = pillars(tgt_p).y;
% 找到该排所有柱（y 相同）
row_idx = find(abs([pillars.y] - tgt_y) < eps_row);
% 按 x 从小到大排序，得到该排的列序
[~, sx] = sort([pillars(row_idx).x], 'ascend');
row_order = row_idx(sx);
% 在该排中找出目标列索引
col_pos = find(row_order == tgt_p, 1, 'first');
% 前一柱索引（若目标是第1列，则使用目标柱本列作为触发区域）
if col_pos > 1
    prev_p = row_order(col_pos - 1);
else
    prev_p = row_order(col_pos);
end
% 触发线 = 前一柱“前方区域”的起始 x（进入该区域即触发）
pre_rot_margin = 0.00;                         % 可调：>0 更靠后触发，<0 更靠前触发
prev_region_x_start = pillars(prev_p).x - pillars(prev_p).w/2 + pre_rot_margin;

% 新增：该排“前边界线”的法向方向（俯视图）
% 若机器人在林带前方（y 更小），法向为 +Y；否则为 -Y
face_sign = 1; % 你当前路径在林带前方，固定 +Y
phi_perp_world = face_sign * pi/2;  % 世界系法向方向（+Y 或 -Y）

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
allow_extend  = false;     % 仅当预测伸展安全时才允许从 L_min 伸出
place_center_margin = 0.01; % 放置中心到边界的最小边距，防止数值越界

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

    % 云台目标“垂直于边界线”的朝向（相对云台角）
    theta_perp = wrapTo2Pi(phi_perp_world - yaw);

    % 云台目标朝向（指向方块中心）仅用于计算接触点，不用于旋转目标
    cx = target_cube.x; cy = target_cube.y;
    theta_goal = wrapTo2Pi(atan2(cy - yt, cx - xt) - yaw);

    % 底盘前沿（沿 +X 行驶的前沿）
    x_front = x_b + base_size/2;

    switch state
        case STATE_ALIGN
            % 保持到目标高度 & 收臂（旋转时保持最小半径）
            h_cmd = h_target;
            L_cmd = L_min;

            % 仅当越过“前一柱的前方区域起始线”后，才开始做旋转预判；此前绝不旋转
            if x_front >= prev_region_x_start
                % 预判整段旋转（到“垂直于边界线”的角度”）全程不碰柱 → 放行全速旋转
                dth_des = sign(atan2(sin(theta_perp-theta), cos(theta_perp-theta))) ...
                          * min(omega_max*dt, abs(atan2(sin(theta_perp-theta), cos(theta_perp-theta))));
                if can_rotate_arm_clear(xt, yt, yaw, theta, theta_perp, L_min, h_target, v_base, pillars, margin_arm, omega_max, probe_dt_rot)
                    theta_cmd = theta + dth_des;   % 放行（≈14.4°/帧）
                else
                    theta_cmd = theta;             % 不安全 → 严格不转
                end
            else
                theta_cmd = theta; % 未到触发线，保持初始角度不动
            end

            % 对齐到“垂直边界线”后，进入“预测伸展”阶段
            if abs(atan2(sin(theta_perp-theta), cos(theta_perp-theta))) < deg2rad(2)
                state = STATE_AIM_EXT;
            end

        case STATE_AIM_EXT
            % 已对齐到“垂直边界线”，此后不再旋转，等待合适时机伸展
            theta_cmd = theta;      % 冻结角度在垂直方向
            h_cmd = h_target;       % 高度保持在目标高度

            % 法向/切向与侧面中心
            phi_perp = yaw + theta_perp;
            u_face   = [cos(phi_perp), sin(phi_perp)];      % 面法向（指向林内）
            t_face   = [-u_face(2), u_face(1)];             % 面切向（沿边界线）
            face_center = [cx, cy] - u_face * (cube_size/2);% 方块侧面中心
            % 吸盘前端面需与侧面中心齐平 ⇒ 吸盘中心目标
            target_cup_center = face_center - u_face*(cup_len/2);

            % 1) 切向对齐时间：tip 的切向投影仅由底盘运动决定（与 L 无关）
            % t_tan = (dot(face_center, t_face) - dot([xt,yt], t_face)) / (v_base * dot([1,0], t_face))
            denom_tan = v_base * t_face(1);         % = v_base * cos(t_face角)
            if abs(denom_tan) < 1e-6
                t_tan = inf;                        % 极端退化：几乎不沿切向相对运动
            else
                t_tan = (dot(face_center, t_face) - dot([xt,yt], t_face)) / denom_tan;
                t_tan = max(0, t_tan);              % 不允许负时间
            end

            % 2) 在 t_tan 时刻的法向所需长度（吸盘中心）
            base_future = [xt + v_base*t_tan, yt];
            L_needed = dot(target_cup_center - base_future, u_face);
            L_needed = min(max(L_needed, L_min), L_max);

            % 3) 现在起以最大伸速伸到 L_needed 需要的时长
            dL_req = max(0, L_needed - L);
            t_need = dL_req / max(vL_max,1e-6);

            % 4) 启动判据：只有“现在开始”能在 t_tan 时刻或更晚恰好到达（不提前）
            % 若现在启动会提前到（t_need < t_tan），则继续保持 L_min 等待
            start_now = isfinite(t_tan) && (t_need >= t_tan - 1e-6);

            % 5) 整段安全检查（考虑5cm臂径/10cm吸盘）
            margin_len = max(margin_arm, margin_cup);
            ext_ok = start_now && can_extend_now(xt, yt, yaw, theta_perp, L, L_needed, h, vh_max, h_target, v_base, pillars, margin_len, vL_max, probe_dt_ext);

            if ext_ok
                allow_extend = true;         % 放行伸展
                L_cmd = L_needed;            % 目标固定为“在 t_tan 时刻的所需长度”
            else
                allow_extend = false;        % 未到时机/不安全 → 严格保持 L_min
                L_cmd = L_min;
            end

            % 调试（需要时打开观察时序是否正确）
            % fprintf('AIM_EXT: t_tan=%.3f t_need=%.3f start=%d L=%.3f -> L_needed=%.3f\n', ...
            %     t_tan, t_need, start_now, L, L_needed);
        case STATE_CARRY
            % 3) 搬运回底盘并放置
            goal_xy = [x_b, y_b];
            deck_h  = base_thick + target_cube.h/2;

            % 云台朝后（与行进方向相反）
            theta_goal2 = wrapTo2Pi(pi);

            % 带方块整段预测旋转：臂段用臂半径，方块用 cube_rad
            dth = sign(atan2(sin(theta_goal2-theta), cos(theta_goal2-theta))) ...
                  * min(omega_max*dt, abs(atan2(sin(theta_goal2-theta), cos(theta_goal2-theta))));
            if can_rotate_with_payload_clear(xt, yt, yaw, theta, theta_goal2, L, h, v_base, pillars, margin_arm, omega_max, probe_dt_rot, cube_rad + safety_margin)
                theta_cmd = theta + dth;
            else
                theta_cmd = theta; % 不安全 → 等下一帧（通常等L回收到L_min或底盘再前进一点）
            end

            % 触碰后优先“立即回收”：先把臂收至 L_min，再进行放置流程（此时朝向已朝后）
            if retract_phase
                L_cmd = L_min;
                if abs(L - L_min) < 1e-3
                    retract_phase = false; % 回收到位，开始靠近并放置
                end
            else
                % 回收到位后，沿“朝后射线”把吸盘前端物理地推到甲板内某点，再下放
                v_back  = [cos(yaw+theta_goal2), sin(yaw+theta_goal2)];  % 朝后单位向量
                o_ray   = [xt, yt];                                      % 射线起点：云台支点
                c_rect  = [x_b, y_b];                                    % 甲板中心
                half    = base_size/2 - place_center_margin;             % 留少许内缩边距
                [s0, s1, hit] = ray_rect_intersection_param(o_ray, v_back, c_rect, [half, half]);
                if hit
                    % 选甲板内的中点作为“方块中心”目标（物理放置，不瞬移）
                    s_center = 0.5*(s0 + s1);                     % 该点在射线参数上的位置（方块中心）
                    % 吸盘中心需要落在“方块中心”外退 (cup_len/2 + cube_size/2) 处
                    s_cup_center = s_center - (cup_len/2 + cube_size/2);
                    % 需要的臂长（沿 v_back 的投影）：L_place
                    L_place = min(max(s_cup_center, L_min), L_max);
                    % 放置阶段命令长度：以最大伸缩速率接近 L_place
                    L_cmd = L_place;
                else
                    % 射线与甲板不相交（极少见），保持 L_min 等待
                    L_cmd = L_min;
                end
            end
            % 下放高度到甲板高度（方块中心与甲板齐平）
            h_cmd = min(max(deck_h, h_min), h_max);

            % 放置判据到位 → 与底盘绑定（保持“放下位置”的相对偏移）
            if ~retract_phase && abs(atan2(sin(theta_goal2-theta), cos(theta_goal2-theta))) < deg2rad(3) ...
                    && (abs(L-L_cmd)+abs(h-h_cmd) < 0.02)
                grabbed = false;
                placed  = true;                     % 开始与底盘绑定
                % 记录放置瞬间的相对偏移（使用机械臂当前放置到的真实位置，不做钳制）
                phi_c = yaw + theta;
                tip_xy = [xt + L*cos(phi_c), yt + L*sin(phi_c)];   % 吸盘中心XY
                v_back = [cos(phi_c), sin(phi_c)];
                cup_front = tip_xy + v_back*(cup_len/2);           % 吸盘前端面中心
                cube_center_xy = cup_front + v_back*(cube_size/2); % 方块中心XY（真实落点）
                drop_offset_xy = cube_center_xy - [x_b, y_b];
                place_h = deck_h;
                % 将方块保持在当前由机械臂放置到的真实位置（物理意义），随后绑定随底盘移动
                move_patch_center(target_cube.patch, [cube_center_xy(1), cube_center_xy(2), place_h]);
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
    theta_next = theta_cmd;

    % 高度
    h_next = h + sign(h_cmd - h) * min(vh_max*dt, abs(h_cmd - h));

    % 伸长
    if state==STATE_ALIGN || (state==STATE_AIM_EXT && ~allow_extend)
        % 旋转/瞄准但未放行伸展：严格保持 L_min，不做裁剪以免被“缩短”
        L_next = L + sign(L_min - L) * min(vL_max*dt, abs(L_min - L));
    else
        % 已放行伸展 或 其他阶段：按命令并做安全裁剪（考虑臂/吸盘厚度）
        dL_max = min(vL_max*dt, abs(L_cmd - L));
        L_try  = L + sign(L_cmd - L) * dL_max;
        phi_for_len = yaw + theta_next;
        margin_len = max(margin_arm, margin_cup);          % 考虑臂/吸盘直径
        L_maxSafe = max_safe_length_along_ray(xt, yt, phi_for_len, h_next, pillars, margin_len, L_try);
        L_next = min(L_try, L_maxSafe);
    end

    % 应用
    theta = theta_next; L = L_next; h = h_next;

    % 若已放在底盘上：每帧将方块中心跟随到底盘，但保留“放下位置”的相对偏移
    if placed
        place_h = base_thick + target_cube.h/2;
        % 保证跟随过程中中心仍在底盘范围内（底盘移动时也生效）
        half = base_size/2 - place_center_margin;
        x_center = x_b + drop_offset_xy(1);
        y_center = y_b + drop_offset_xy(2);
        x_center = min(max(x_center, x_b - half), x_b + half);
        y_center = min(max(y_center, y_b - half), y_b + half);
        move_patch_center(target_cube.patch, [x_center, y_center, place_h]);
    end

    % ====== 接触即绑定（位姿更新之后）======
    % 几何绑定：以“面法向+切向”度量触碰，并处理穿模回弹
    if ~grabbed && ~placed && (state==STATE_ALIGN || state==STATE_AIM_EXT)
        % 当前帧臂端（吸盘中心）
        phi_now = yaw + theta;
        tip_now = [xt + L*cos(phi_now), yt + L*sin(phi_now)];
        % 若未对齐也可使用当前姿态的法向（此时理论上处于垂直对齐）
        u_now = [cos(phi_now), sin(phi_now)];
        t_now = [-u_now(2), u_now(1)];
        % 方块侧面中心
        cx = target_cube.x; cy = target_cube.y;
        face_center = [cx, cy] - u_now * (cube_size/2);
        % 法向距离（吸盘中心到面中心沿法向，减去 cup_len/2）
        dist_to_plane = dot(face_center - tip_now, u_now) - cup_len/2;
        % 切向偏差（沿边界线方向）
        tan_off = dot(tip_now - face_center, t_now);

        touch_ok = (abs(dist_to_plane) <= bind_depth_tol) && (abs(tan_off) <= bind_xy_tol);
        penetrated = (dist_to_plane < -bind_depth_tol) && (abs(tan_off) <= bind_xy_tol);

        if (touch_ok || penetrated) && abs(h - cz) < bind_h_tol
            grabbed = true;
            state = STATE_CARRY;
            retract_phase = true;  % 触碰即进入“立即回收”
            % 模拟“推一下再吸住”：把方块放到“吸盘前端面齐平”位置后绑定
            % 吸盘前端面中心 = tip_now + u_now*(cup_len/2)
            cup_front = tip_now + u_now*(cup_len/2);
            cube_center_new = cup_front + u_now*(cube_size/2);
            move_patch_center(target_cube.patch, [cube_center_new(1), cube_center_new(2), h]);
        end

        % 更新“上一帧臂端”（保留调试/备用）
        x_tip_prev = tip_now(1);
        y_tip_prev = tip_now(2);
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

% ====== 安全缩步函数 ======
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

% ====== 整段预测“是否可从现在开始全速旋转” ======
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

% ====== 整段预测“是否可从现在开始全速伸展到 L_goal” ======
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

% 旋转整段预测：不带方块，仅检查“臂段”不碰柱（用于接近/对齐阶段尽早旋转）
function ok = can_rotate_arm_clear(xt, yt, yaw, theta, theta_goal, L_check, h_check, v_base, pillars, margin, omega_max, probe_dt)
    ok = true;
    d = atan2(sin(theta_goal-theta), cos(theta_goal-theta));
    s = sign(d); ang = abs(d);
    T = ang / max(omega_max,1e-6);
    t = 0.0;
    % 初始点
    phi_prev = yaw + theta;
    x_prev = xt + L_check*cos(phi_prev);
    y_prev = yt + L_check*sin(phi_prev);
    while t <= T
        th = theta + s*omega_max*min(t, T);
        phi = yaw + th;
        xti = xt + v_base*t; yti = yt;
        x_tip = xti + L_check*cos(phi);
        y_tip = yti + L_check*sin(phi);
        % 臂段与柱相交检查
        if arm_hits_pillars(xti, yti, x_tip, y_tip, h_check, pillars, margin)
            ok = false; return;
        end
        % 步进
        x_prev = x_tip; y_prev = y_tip;
        t = t + probe_dt;
    end
end

% 旋转整段预测：带方块。检查“臂段”与“方块中心轨迹对柱扩张AABB的相交”
function ok = can_rotate_with_payload_clear(xt, yt, yaw, theta, theta_goal, L_now, h_now, v_base, pillars, margin, omega_max, probe_dt, cube_rad)
    ok = true;
    d = atan2(sin(theta_goal-theta), cos(theta_goal-theta));
    s = sign(d); ang = abs(d);
    T = ang / max(omega_max,1e-6);
    t = 0.0;
    % 初始
    phi0 = yaw + theta;
    xt0 = xt; yt0 = yt;
    cx_prev = xt0 + L_now*cos(phi0);
    cy_prev = yt0 + L_now*sin(phi0);
    while t <= T
        th = theta + s*omega_max*min(t, T);
        phi = yaw + th;
        xti = xt + v_base*t; yti = yt;
        % 臂段
        x_tip = xti + L_now*cos(phi);
        y_tip = yti + L_now*sin(phi);
        if arm_hits_pillars(xti, yti, x_tip, y_tip, h_now, pillars, margin)
            ok = false; return;
        end
        % 方块中心轨迹相交：与“柱的AABB扩张 cube_rad+margin”相交则算干涉
        cx_now = x_tip; cy_now = y_tip;
        for k = 1:numel(pillars)
            if h_now <= pillars(k).h + 1e-6
                rect_center = [pillars(k).x, pillars(k).y];
                rect_size   = [pillars(k).w + 2*(cube_rad+margin), pillars(k).d + 2*(cube_rad+margin)];
                if seg_rect_intersect([cx_prev, cy_prev], [cx_now, cy_now], rect_center, rect_size)
                    ok = false; return;
                end
            end
        end
        cx_prev = cx_now; cy_prev = cy_now;
        t = t + probe_dt;
    end
end

% 射线与轴对齐矩形（XY）求交，返回射线参数区间 [s0,s1] 及是否命中
function [s0, s1, hit] = ray_rect_intersection_param(o, d, c, half)
    % o: 1x2 射线起点；d: 1x2 单位方向（不必严格单位）；
    % c: 1x2 矩形中心；half: 1x2 半边长
    xmin = c(1) - half(1); xmax = c(1) + half(1);
    ymin = c(2) - half(2); ymax = c(2) + half(2);

    smin = -inf; smax = inf;

    % X slabs
    if abs(d(1)) < 1e-9
        if o(1) < xmin || o(1) > xmax, hit=false; s0=0; s1=-1; return; end
    else
        tx1 = (xmin - o(1)) / d(1);
        tx2 = (xmax - o(1)) / d(1);
        tminx = min(tx1, tx2); tmaxx = max(tx1, tx2);
        smin = max(smin, tminx);
        smax = min(smax, tmaxx);
    end

    % Y slabs
    if abs(d(2)) < 1e-9
        if o(2) < ymin || o(2) > ymax, hit=false; s0=0; s1=-1; return; end
    else
        ty1 = (ymin - o(2)) / d(2);
        ty2 = (ymax - o(2)) / d(2);
        tminy = min(ty1, ty2); tmaxy = max(ty1, ty2);
        smin = max(smin, tminy);
        smax = min(smax, tmaxy);
    end

    % 限制为射线（s >= 0）
    s0 = max(smin, 0);
    s1 = smax;
    hit = (s1 >= s0);
end