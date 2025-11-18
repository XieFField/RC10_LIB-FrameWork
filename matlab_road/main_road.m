clear; clc; close all;

% ================== 参数 ==================
dt       = 0.01;
tau      = 0.35;        % 控制点距离比例(0~0.6)
tol_done = 0.01;        % 终点容差

% 机器人初始（全向底盘，yaw 不变）
x=0; y=0; yaw=0; theta=0; L_min=0.67; L=L_min; h=0.40;
base_size=0.60; base_thick=0.05; turret_r=0.08;

% 采集路径点
figure('Color','w'); hold on; axis equal; grid on;
xlabel('X'); ylabel('Y'); title('点击路径点, 回车结束');
xlim([-10 10]); ylim([-10 10]);
[patchBase, patchTurret, lineFix, lineExt] = draw_robot(x,y,yaw,theta,L,h,base_size,base_thick,turret_r);
try
    [wx,wy]=getpts(gca);
catch
    [wx,wy]=ginput;
end
wps=[wx,wy];
wps = dedup_close_points(wps,1e-4);
if isempty(wps)
    disp('无路径点'); return;
end
% 把初始位置作为首点插入（保证“起点→第1路径点”也贝塞尔化）
if norm([x,y]-wps(1,:))>1e-9
    wps=[ [x,y]; wps ];
end

plot(wps(:,1),wps(:,2),'ko','MarkerFaceColor',[1 1 0],'MarkerSize',6);
plot(wps(:,1),wps(:,2),'k--');

% ================== 构建三次贝塞尔段（C1 连续） ==================
n = size(wps,1);
T = zeros(n,2);
for i=1:n
    if i==1
        v = wps(2,:)-wps(1,:);
    elseif i==n
        v = wps(n,:)-wps(n-1,:);
    else
        v = wps(i+1,:)-wps(i-1,:);
    end
    nv = norm(v);
    if nv<1e-9, T(i,:)=[1 0]; else T(i,:)=v/nv; end
end

seg = struct('P0',{},'P1',{},'P2',{},'P3',{});
for i=1:n-1
    P0=wps(i,:); P3=wps(i+1,:);
    d = norm(P3-P0);
    k0 = tau*d; k1 = tau*d;
    P1 = P0 + k0*T(i,:);
    P2 = P3 - k1*T(i+1,:);
    seg(i).P0=P0; seg(i).P1=P1; seg(i).P2=P2; seg(i).P3=P3;

    % 预览
    tt=linspace(0,1,60); B = cubic_eval(P0,P1,P2,P3,tt);
    plot(B(:,1),B(:,2),'b-','LineWidth',1.5);
end
legend({'Waypoints','Polyline','Bezier path'},'Location','best');

% 为路径点建立段索引与曲线参数映射（第 j 路径点对应 j-1 段的终点，起点除外）
wp_map = struct('seg_idx',[],'t',[],'tan',[]);
for j = 1:n
    if j == 1
        wp_map(j).seg_idx = 1;
        wp_map(j).t = 0.0;
        d1 = cubic_derivative(seg(1).P0,seg(1).P1,seg(1).P2,seg(1).P3,0.0);
    else
        sidx = j-1;
        wp_map(j).seg_idx = sidx;
        wp_map(j).t = 1.0;
        d1 = cubic_derivative(seg(sidx).P0,seg(sidx).P1,seg(sidx).P2,seg(sidx).P3,1.0);
    end
    nd = norm(d1); if nd<1e-9, d1=[1,0]; nd=1; end
    wp_map(j).tan = d1/nd;   % 归一化切向
end

% ================== 跟踪控制（应用 FollowBezierController） ==================
% 控制器路径对象
path.segs = seg;

% ========== 逐路径点穿越配置 ==========
n_wp = size(wps,1);
wp_eps = 0.03*ones(n_wp,1);     % 每个路径点的死区(米)，确保“必经”
wp_eps(1) = 0.00;               % 起点无需判定
wp_eps(end) = 0.02;             % 终点更紧一点（可调）

wp_end_speed = 0.5*ones(n_wp,1); % 穿越该路径点时的目标速度(米/秒)
wp_end_speed(1) = 0.0;           % 起点无效
wp_end_speed(end) = 0;        % 终点速度可自定义（若需完全不停可设为>0）

gate = struct('idx', 2, ...      % 当前要“必经”的目标路径点，从第2个开始
              'eps', wp_eps, ...
              'v_end', wp_end_speed, ...
              'n', n_wp);

% 控制器参数（按“方案书”）
params = struct( ...
    'v_cruise',    1.5,   ... % 固定巡航速度（与曲线贴合更稳定）
    'v_max',       3.0,   ...
    'a_max',       1.0,   ...
    'k_e',         2.2,   ... % 法向误差增益（↑贴合，过大抖）
    'k_d',         0.4,   ... % 法向误差微分
    'k_snap',      0.05,  ... % 当横向误差大时提高法向拉回 (动态增益权重)
    'look_L_min',  0.10,  ...
    'look_L_max',  0.30,  ...
    'look_err_gain',0.6,  ... % 横向误差放大道路时减小 look
    'a_lat_max',   1.2,   ...
    'end_brake_a', 0.6,   ...
    'drag_b',      0.05,  ...
    'speed_floor', 0.05,  ...
    'e_int_clip',  0.15,  ...
    'offtrack_e',  0.05,  ...
    'recover_e',   0.015, ...
    'recover_time', 0.6,  ...
    'recover_boost',0.25, ...
    'drag_recover_scale',0.5,...
    'max_n_ratio', 0.8,   ...
    'final_stop',  true,  ... % 终点是否强制停车
    'final_stop_eps', 0.05, ... % 进入终点刹停半径
    'floor_final', 0.0,   ... % 终点速度地板（允许为0）
    'k_brake',     2.5    ... % 终点刹车系数（越大停得越快）
);

state = [];                   % 控制器内部状态
vx = 0.0; vy = 0.0;           % 当前速度（全向底盘）
end_pt = wps(end,:);

max_steps = 20000;            % 保险上限（避免意外死循环）
for k = 1:max_steps
    % 控制器输出期望速度（已做限速/限加速度/阻尼）
    [v_cmd, state, dbg] = FollowBezierController([x,y], [vx,vy], path, state, params, gate, dt);

    % 将指令速度作为实际速度
    vx = v_cmd(1);
    vy = v_cmd(2);

    % 位姿积分（全向底盘，yaw 不变）
    x = x + vx * dt;
    y = y + vy * dt;

    update_robot_pose(patchBase,patchTurret,lineFix,lineExt, ...
        x,y,yaw,theta,L,h,base_size,base_thick,turret_r,L_min);
    drawnow limitrate; pause(dt);
    if ~ishandle(patchBase), break; end

    % 命中当前必须穿越的路径点（进入死区）才切换到下一个
    if gate.idx <= gate.n
        if norm([x,y] - wps(gate.idx,:)) <= gate.eps(gate.idx)
            gate.idx = gate.idx + 1;
        end
    end
    % 到达终点判定（也可仅依赖上面的 gate 逻辑）
    if norm([x,y] - end_pt) <= tol_done
        break;
    end
end

if norm([x,y]-end_pt) < tol_done
    title('FollowBezierController：按曲线最大限度跟随（含惯性、前视与误差修正）');
else
    title('执行结束');
end

% ================== 函数区 ==================
function [v_cmd, state, dbg] = FollowBezierController(robot_pos, robot_vel, path, state, p, gate, dt)
    if isempty(state), state = struct(); end
    if ~isfield(state,'cache'), state.cache = build_cache(path); end
    if ~isfield(state,'t_seg'), state.t_seg = [1;0]; end
    if ~isfield(state,'e_prev_n'), state.e_prev_n = 0; end

    % 新增：恢复/脱轨状态默认与参数默认
    if ~isfield(state,'offtrack'),     state.offtrack = false; end
    if ~isfield(state,'recover_t'),    state.recover_t = 0.0;  end
    if ~isfield(state,'ontrack_time'), state.ontrack_time = 0; end
    if ~isfield(p,'offtrack_e'),       p.offtrack_e = 0.05; end
    if ~isfield(p,'recover_e'),        p.recover_e  = 0.015; end
    if ~isfield(p,'recover_time'),     p.recover_time = 0.6; end
    if ~isfield(p,'recover_boost'),    p.recover_boost = 0.25; end
    if ~isfield(p,'drag_recover_scale'), p.drag_recover_scale = 0.5; end
    if ~isfield(p,'max_n_ratio'),      p.max_n_ratio = 0.8; end
    if ~isfield(p,'v_cruise'),         p.v_cruise = min(1.0, p.v_max); end
    if ~isfield(p,'v_min_recover'),    p.v_min_recover = 0.5; end  % 新增：恢复期最小切向速度

    robot_pos = row2(robot_pos); robot_vel = row2(robot_vel);

    % 最近点 + 一次牛顿细化
    [seg_idx, t_near, p_near, d1_near] = nearest_point(robot_pos, state.cache, state.t_seg);
    t_near = refine_newton(state.cache.segs(seg_idx), t_near, robot_pos);
    seg = state.cache.segs(seg_idx);
    p_near = row2(cubic_eval(seg.P0,seg.P1,seg.P2,seg.P3,t_near));
    d1_near= row2(cubic_derivative(seg.P0,seg.P1,seg.P2,seg.P3,t_near));
    state.t_seg = [seg_idx; t_near];

    % 单位切/法向
    t_hat = d1_near; nt = norm(t_hat); if nt<1e-9, t_hat=[1,0]; nt=1; end; t_hat = t_hat/nt;
    n_hat = [-t_hat(2), t_hat(1)];

    % 误差与脱轨/回轨判定
    e_vec = robot_pos - p_near;
    e_n   = dot(e_vec, n_hat);
    e_norm= norm(e_vec);
    if abs(e_n) > p.offtrack_e || e_norm > 1.5*p.offtrack_e
        state.offtrack = true;
        state.ontrack_time = 0;
        state.recover_t = 0.0;
    else
        state.ontrack_time = state.ontrack_time + dt;
        if state.offtrack && abs(e_n) < p.recover_e
            state.recover_t = state.recover_t + dt;
            if state.recover_t >= p.recover_time
                state.offtrack = false; % 恢复期结束
            end
        else
            % 仍在恢复累计窗口之外，清零计时
            if ~state.offtrack
                state.recover_t = 0.0;
            end
        end
    end

    % 动态前视（误差大 -> look短）
    de_n = (e_n - state.e_prev_n)/dt; state.e_prev_n = e_n;
    look_L = p.look_L_max - (p.look_L_max - p.look_L_min) * (1 - exp(-p.look_err_gain * abs(e_n)));
    [~, t_look, ~, d1_look] = lookahead_point(seg_idx, t_near, look_L, 60, state.cache);
    v_dir = d1_look; nv = norm(v_dir); if nv<1e-9, v_dir = t_hat; nv=1; end; v_dir = v_dir/nv; %#ok<NASGU>

    % v_base：巡航 + 曲率限速
    v_base = p.v_cruise;
    d2_near = row2(cubic_second_derivative(seg.P0,seg.P1,seg.P2,seg.P3,t_near));
    denom = (norm(d1_near)^3 + 1e-9);
    kappa = abs(d1_near(1)*d2_near(2) - d1_near(2)*d2_near(1)) / denom;
    if kappa > 1e-6
        v_base = min(v_base, sqrt(p.a_lat_max / kappa));
    end

    % 仅在“非恢复期”应用路径点末端限速
    if gate.idx <= gate.n && ~(state.offtrack) && ~(state.recover_t > 0 && state.recover_t < p.recover_time)
        s_rem_wp = remaining_length_to_wp(state.cache, seg_idx, t_near, gate.idx);
        v_cap_wp = sqrt(max(0, 2*p.end_brake_a*max(0,s_rem_wp))) + p.speed_floor;
        v_base = min(v_base, v_cap_wp);
    end

    % 终点强制停车判定
    is_final_zone = false;
    if p.final_stop
        end_pt = path.segs(end).P3;            % 终点坐标
        if norm(robot_pos - end_pt) <= p.final_stop_eps
            is_final_zone = true;
        end
    end

    % 回轨恢复期与终点互斥：终点进入后不再 boost
    if is_final_zone
        v_ref = 0.0;           % 切向参考速度直接归零
        drag_scale = 1.0;      % 恢复阻尼正常
        state.recover_t = 0.0; % 停止恢复计时
    else
        if state.recover_t > 0 && state.recover_t < p.recover_time
            v_ref = min(p.v_max, max(p.v_min_recover, v_base + p.recover_boost));
            drag_scale = p.drag_recover_scale;
        else
            v_ref = v_base;
            drag_scale = 1.0;
        end
    end

    % 非终点保持常规速度底线；终点用 floor_final（可为0）
    floor_use = p.speed_floor;
    if is_final_zone
        floor_use = p.floor_final;
    end
    v_ref = max(floor_use, min(p.v_max, v_ref));

    % 切向/法向分离
    k_e_dyn = p.k_e + p.k_snap * min(1.0, abs(e_n)/max(1e-6,p.offtrack_e));
    v_t = v_ref;
    v_n = -(k_e_dyn * e_n + p.k_d * de_n);
    v_n = max(-p.max_n_ratio*v_t, min(p.max_n_ratio*v_t, v_n));
    v_cmd_pre = v_t * t_hat + v_n * n_hat;

    % 终点刹车：当进入终点区且残余速度仍>阈值，直接加阻尼
    if is_final_zone
        v_cmd_pre = v_cmd_pre - p.k_brake * robot_vel;  % 指令附加刹车
    elseif p.drag_b > 0
        v_cmd_pre = v_cmd_pre - (p.drag_b * drag_scale) * robot_vel;
    end

    % 限速/限加速度
    spd = norm(v_cmd_pre);
    if spd > p.v_max, v_cmd_pre = v_cmd_pre * (p.v_max/(spd+1e-9)); end
    dv_cmd = v_cmd_pre - robot_vel;
    dvn = norm(dv_cmd); max_dv = p.a_max * dt;
    if dvn > max_dv, dv_cmd = dv_cmd * (max_dv/(dvn+1e-9)); end
    v_cmd = robot_vel + dv_cmd;

    % 调试输出
    if nargout >= 3
        dbg.v_ref = v_ref; dbg.kappa = kappa;
        dbg.e_n = e_n; dbg.de_n = de_n;
        dbg.offtrack = state.offtrack; dbg.recover_t = state.recover_t;
    end
end

function t_ref = refine_newton(seg, t0, p)
    % 对 dist^2(t) 做一次 Newton（限制范围）
    t = t0;
    for i=1:1
        B  = cubic_eval(seg.P0,seg.P1,seg.P2,seg.P3,t);
        D1 = cubic_derivative(seg.P0,seg.P1,seg.P2,seg.P3,t);
        % 二阶导近似
        D2 = cubic_second_derivative(seg.P0,seg.P1,seg.P2,seg.P3,t);
        r  = B - p;
        f1 = 2 * dot(D1, r);
        f2 = 2 * (dot(D2, r) + dot(D1, D1));
        if abs(f2) < 1e-9, break; end
        t_new = t - f1 / f2;
        t = min(1.0, max(0.0, t_new));
    end
    t_ref = t;
end

% 二阶导（新增）
function d2 = cubic_second_derivative(P0,P1,P2,P3,t)
    d2 = 6*(1-t).*(P2 - 2*P1 + P0) + 6*t.*(P3 - 2*P2 + P1);
    d2 = row2(d2);
end

% 剩余弧长（保留）
function s_rem = remaining_length(cache, seg_idx, t)
    [Lseg, s_local] = t_to_s(cache, seg_idx, t);
    s_rem = Lseg - s_local;
    for s = (seg_idx+1):numel(cache.segs)
        s_rem = s_rem + cache.len(s);
    end
end

% 计算从当前(seg_idx,t)到“目标路径点 gate_wp_idx”的剩余弧长
% 约定：第 j 个路径点对应段索引 s_target = j-1（j>=2），即 seg(s_target) 的终点
function s_rem = remaining_length_to_wp(cache, seg_idx, t, gate_wp_idx)
    % 目标是第 gate_wp_idx 个路径点
    s_target = gate_wp_idx - 1;
    if s_target < 0, s_rem = 0; return; end
    % 当前段的剩余
    [Lseg, s_local] = t_to_s(cache, seg_idx, t);
    if s_target < seg_idx
        % 目标在身后，视作已到
        s_rem = 0.0; return;
    elseif s_target == seg_idx
        s_rem = max(0.0, Lseg - s_local); return;
    else
        s_rem = max(0.0, Lseg - s_local);
        for s = (seg_idx+1):s_target
            s_rem = s_rem + cache.len(s);
        end
    end
end

% ===================== 子函数区 =====================
function cache = build_cache(path)
    Nsample = 200;
    segs = path.segs;
    ns = numel(segs);
    cache = struct();
    cache.segs = segs;
    cache.Nsample = Nsample;
    cache.t_grid = linspace(0,1,Nsample+1).';
    cache.pos = cell(ns,1);
    cache.d1  = cell(ns,1);
    cache.len = zeros(ns,1);
    cache.cumlen = cell(ns,1);

    for i = 1:ns
        P0 = segs(i).P0; P1 = segs(i).P1; P2 = segs(i).P2; P3 = segs(i).P3;
        tt = cache.t_grid;
        B  = cubic_eval(P0,P1,P2,P3,tt);          % (N+1)x2
        D1 = cubic_derivative(P0,P1,P2,P3,tt);    % (N+1)x2
        d  = diff(B,1,1);
        seg_len = sum(sqrt(sum(d.^2,2)));
        cum = [0; cumsum(sqrt(sum(d.^2,2)))];

        cache.pos{i}    = B;
        cache.d1{i}     = D1;
        cache.len(i)    = seg_len;
        cache.cumlen{i} = cum;
    end
end

function [seg_idx, t_near, p_near, d1_near] = nearest_point(p, cache, t_warm)
    ns = numel(cache.segs);
    best_dist2 = inf; seg_idx = 1; idx = 1;

    % 粗搜
    for s = 1:ns
        B = cache.pos{s};                % (N+1)x2
        diffv = B - p;                   % 广播得到 (N+1)x2
        dist2 = sum(diffv.^2,2);
        [dmin, imin] = min(dist2);
        if dmin < best_dist2
            best_dist2 = dmin; seg_idx = s; idx = imin;
        end
    end

    % 细化（黄金分割仅返回 t）
    t0 = cache.t_grid(max(1,idx-1));
    t1 = cache.t_grid(min(length(cache.t_grid),idx+1));
    t_near = golden_find_min(@(t) dist2_curve(t, cache.segs(seg_idx), p), t0, t1, 10);

    seg = cache.segs(seg_idx);
    p_near = row2(cubic_eval(seg.P0, seg.P1, seg.P2, seg.P3, t_near));
    d1_near= row2(cubic_derivative(seg.P0, seg.P1, seg.P2, seg.P3, t_near));
end

function [seg_idx, t_look, p_look, d1_look] = lookahead_point(seg_idx, t_cur, L, steps, cache)
    ns = numel(cache.segs);
    sidx = seg_idx; t = t_cur;
    remain = L;
    for iter = 1:steps
        [seg_len, seg_s_now] = t_to_s(cache, sidx, t);
        seg_remain = seg_len - seg_s_now;
        if remain <= seg_remain + 1e-9
            t_look = s_to_t(cache, sidx, seg_s_now + remain);
            seg = cache.segs(sidx);
            p_look  = row2(cubic_eval(seg.P0, seg.P1, seg.P2, seg.P3, t_look));
            d1_look = row2(cubic_derivative(seg.P0, seg.P1, seg.P2, seg.P3, t_look));
            seg_idx = sidx; return;
        else
            remain = remain - seg_remain;
            if sidx < ns
                sidx = sidx + 1; t = 0.0;
            else
                % 已到最后段末，直接返回末端
                t_look = 1.0;
                seg = cache.segs(sidx);
                p_look  = row2(cubic_eval(seg.P0, seg.P1, seg.P2, seg.P3, t_look));
                d1_look = row2(cubic_derivative(seg.P0, seg.P1, seg.P2, seg.P3, t_look));
                seg_idx = sidx; return;
            end
        end
    end
    % 保护返回
    t_look = t;
    seg = cache.segs(sidx);
    p_look  = row2(cubic_eval(seg.P0, seg.P1, seg.P2, seg.P3, t_look));
    d1_look = row2(cubic_derivative(seg.P0, seg.P1, seg.P2, seg.P3, t_look));
    seg_idx = sidx;
end

function [seg_len, seg_s] = t_to_s(cache, seg_idx, t)
    tt = cache.t_grid;
    cum = cache.cumlen{seg_idx};
    Lseg = cache.len(seg_idx);
    if t <= 0, seg_len=Lseg; seg_s=0; return; end
    if t >= 1, seg_len=Lseg; seg_s=Lseg; return; end
    k = find(tt <= t, 1, 'last'); if k >= numel(tt), k = numel(tt)-1; end
    t0 = tt(k); t1 = tt(k+1);
    p = (t - t0) / (t1 - t0);
    s0 = cum(k); s1 = cum(k+1);
    seg_s = s0*(1-p) + s1*p;
    seg_len = Lseg;
end

function t = s_to_t(cache, seg_idx, s)
    cum = cache.cumlen{seg_idx};
    Lseg = cache.len(seg_idx);
    if s <= 0, t = 0; return; end
    if s >= Lseg, t = 1; return; end
    k = find(cum <= s, 1, 'last');
    if k >= numel(cum)
        t = 1; return;
    end
    s0 = cum(k); s1 = cum(k+1);
    if s1 - s0 < 1e-9
        % 退化保护：直接按索引比例给 t
        t = (k-1) / (numel(cum)-1);
        return;
    end
    p = (s - s0) / (s1 - s0);     % 段内比例
    tt = cache.t_grid;
    t0 = tt(k); t1 = tt(k+1);
    t = t0*(1-p) + t1*p;          % 线性插值得到 t
end

function val = dist2_curve(t, seg, p)
    q = row2(cubic_eval(seg.P0, seg.P1, seg.P2, seg.P3, t));
    d = q - row2(p);
    val = d(1)^2 + d(2)^2;
end

function t_best = golden_find_min(f, a, b, iters)
    gr = (sqrt(5)-1)/2;
    c = b - gr*(b-a); d = a + gr*(b-a);
    fc = f(c); fd = f(d);
    for i=1:iters
        if fc < fd
            b = d; d = c; fd = fc; c = b - gr*(b-a); fc = f(c);
        else
            a = c; c = d; fc = fd; d = a + gr*(b-a); fd = f(d);
        end
    end
    t_best = c; if fd < fc, t_best = d; end
end

function B = cubic_eval(P0,P1,P2,P3,t)
    % 标量t -> 1x2 行向量；向量t -> Nx2
    if isscalar(t)
        B = (1-t)^3.*P0 + 3*(1-t)^2.*t.*P1 + 3*(1-t).*t^2.*P2 + t^3.*P3;
        B = row2(B);
    else
        tt = t(:);
        B = (1-tt).^3.*P0 + 3*(1-tt).^2.*tt.*P1 + 3*(1-tt).*tt.^2.*P2 + tt.^3.*P3;
    end
end

function d1 = cubic_derivative(P0,P1,P2,P3,t)
    if isscalar(t)
        d1 = 3*(1-t)^2.*(P1-P0) + 6*(1-t).*t.*(P2-P1) + 3*t^2.*(P3-P2);
        d1 = row2(d1);
    else
        tt = t(:);
        d1 = 3*(1-tt).^2.*(P1-P0) + 6*(1-tt).*tt.*(P2-P1) + 3*tt.^2.*(P3-P2);
    end
end

function v = row2(v)
    v = reshape(v,1,[]);
    if numel(v) >= 2, v = v(1,1:2); end
end

function Q = dedup_close_points(P, epsd)
    % 去除相邻过近点，避免退化贝塞尔段
    if isempty(P), Q = P; return; end
    Q = P(1,:);
    for i = 2:size(P,1)
        if norm(P(i,:) - Q(end,:)) > epsd
            Q = [Q; P(i,:)];
        end
    end
end