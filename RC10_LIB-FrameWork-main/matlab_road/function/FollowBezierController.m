% filepath: f:\MyProjectFlies\STM32H7\Frame_T\matlab_road\function\FollowBezierController.m
function [v_cmd, state, dbg] = FollowBezierController(robot_pos, robot_vel, path, state, params, dt)
% FollowBezierController
% 输入:
%   robot_pos [1x2]  机器人当前位置 (x,y)
%   robot_vel [1x2]  机器人当前速度 (vx,vy)
%   path.segs(1..N): 每段为三次贝塞尔 {P0,P1,P2,P3} (各为1x2向量)
%   state: 控制器内部状态(首次可传 [])
%   params: 结构体
%       .k_speed        标量, 参考速度系数, v_ref = k_speed * |P'(t')|       (默认 1.0)
%       .v_max          速度模长上限 (默认 2.0 m/s)
%       .a_max          加速度上限 (默认 1.0 m/s^2)
%       .kp, .kd, .ki   2D PID 增益 (默认 kp=1.2, kd=0.2, ki=0)
%       .look_L         前视阈值距离 L (默认 0.20 m)
%       .look_steps     计算 t_look 的步数 step (默认 30)
%       .speed_floor    参考速度下限, 避免过小 (默认 0.05 m/s)
%       .drag_b         粘性阻尼系数(等效速度衰减, 可选, 默认 0)
%   dt: 控制周期(s)
%
% 输出:
%   v_cmd [1x2]  限加速度后的速度指令
%   state        更新后的内部状态 (需在下次调用时传回)
%   dbg          调试信息结构体

    % 参数默认
    if nargin < 6, dt = 0.01; end
    if nargin < 5, params = struct(); end
    p = set_default(params, ...
        'k_speed',     1.0, ...
        'v_max',       2.0, ...
        'a_max',       1.0, ...
        'kp',          1.2, ...
        'kd',          0.2, ...
        'ki',          0.0, ...
        'look_L',      0.20, ...
        'look_steps',  30, ...
        'speed_floor', 0.05, ...
        'drag_b',      0.0 ...
    );

    % 状态默认
    if isempty(state)
        state = struct();
    end
    if ~isfield(state,'cache')
        state.cache = build_cache(path);
    end
    if ~isfield(state,'t_seg')
        state.t_seg = [1; 0.0];  % [seg_idx; t_in_seg]
    end
    if ~isfield(state,'e_int')
        state.e_int = [0.0 0.0];
    end
    if ~isfield(state,'e_prev')
        state.e_prev = [0.0 0.0];
    end
    if ~isfield(state,'recover_t'), state.recover_t = 0; end

    % ---------- 脱轨/回轨检测状态 ----------
    if ~isfield(state,'offtrack'),     state.offtrack = false; end
    if ~isfield(state,'recover_t'),    state.recover_t = 0.0;  end
    if ~isfield(state,'ontrack_time'), state.ontrack_time = 0; end

    % 阈值（若未在 params 中提供，则设默认）
    if ~isfield(p,'offtrack_e'),      p.offtrack_e = 0.05; end   % 判定脱轨的法向/几何误差阈值
    if ~isfield(p,'recover_e'),       p.recover_e  = 0.015; end  % 认为已回轨的误差阈值
    if ~isfield(p,'recover_time'),    p.recover_time = 0.6; end  % 回轨后临时“加速恢复”时间(s)
    if ~isfield(p,'recover_boost'),   p.recover_boost = 0.25; end% 回轨后切向速度临时加成(m/s)
    if ~isfield(p,'drag_recover_scale'), p.drag_recover_scale = 0.5; end % 回轨阶段阻尼折减倍率
    if ~isfield(p,'max_n_ratio'),     p.max_n_ratio = 0.8; end   % 法向速度占切向的最大比例
    if ~isfield(p,'v_cruise'),        p.v_cruise = min(1.0, p.v_max); end

    % 计算几何误差（法向和欧氏）
    e_vec = robot_pos - p_near;
    e_n   = dot(e_vec, n_hat);
    e_norm = norm(e_vec);

    % 脱轨判定
    if abs(e_n) > p.offtrack_e || e_norm > 1.5*p.offtrack_e
        state.offtrack = true;
        state.ontrack_time = 0;
        state.recover_t = 0.0;
    else
        % 小误差视为在轨，累计在轨时间
        state.ontrack_time = state.ontrack_time + dt;
        if state.offtrack && abs(e_n) < p.recover_e
            % 开始“回轨恢复期”
            state.recover_t = state.recover_t + dt;
            if state.recover_t >= p.recover_time
                state.offtrack = false; % 恢复期结束
            end
        end
    end

    % ---------- 速度参考（切向基准） ----------
    % 基于巡航/曲率限速/路径点末端限速的 v_base
    v_base = p.v_cruise;
    % 曲率限速
    d2_near = row2(cubic_second_derivative(seg.P0, seg.P1, seg.P2, seg.P3, t_near));
    denom = (norm(d1_near)^3 + 1e-9);
    kappa = abs(d1_near(1)*d2_near(2) - d1_near(2)*d2_near(1)) / denom;
    if kappa > 1e-6 && isfield(p,'a_lat_max')
        v_base = min(v_base, sqrt(max(0.0, p.a_lat_max)/kappa));
    end
    % 路径点末端限速：仅在“非恢复期”才生效
    if isfield(p,'end_brake_a') && gate.idx <= gate.n && ~(state.offtrack) && ~(state.recover_t>0 && state.recover_t < p.recover_time)
        s_rem_wp = remaining_length_to_wp(state.cache, seg_idx, t_near, gate.idx);
        v_cap_wp = sqrt(max(0.0, 2 * p.end_brake_a * max(0.0, s_rem_wp))) + p.speed_floor;
        v_base = min(v_base, v_cap_wp);
    end

    % 回轨恢复：临时提高切向速度，帮助加速回到巡航
    if state.recover_t > 0 && state.recover_t < p.recover_time
        v_ref = min(p.v_max, v_base + p.recover_boost);
        drag_scale = p.drag_recover_scale;
    else
        v_ref = v_base;
        drag_scale = 1.0;
    end
    v_ref = max(p.speed_floor, min(p.v_max, v_ref));

    % ---------- 向量场合成：切向与法向分离 ----------
    % 法向误差微分
    if ~isfield(state,'e_prev_n'), state.e_prev_n = 0.0; end
    de_n = (e_n - state.e_prev_n)/dt; state.e_prev_n = e_n;

    % 动态法向增益（大偏差更强吸附）
    if ~isfield(p,'k_e'), p.k_e = 2.0; end
    if ~isfield(p,'k_d'), p.k_d = 0.4; end
    if ~isfield(p,'k_snap'), p.k_snap = 0.10; end
    k_e_dyn = p.k_e + p.k_snap * min(1.0, abs(e_n)/max(1e-6,p.offtrack_e));

    % 切向固定为 v_ref，法向叠加并限比
    v_t = v_ref;
    v_n = -(k_e_dyn * e_n + p.k_d * de_n);
    v_n = max(-p.max_n_ratio*v_t, min(p.max_n_ratio*v_t, v_n));

    v_cmd_pre = v_t * t_hat + v_n * n_hat;

    % 粘性阻尼（恢复期降低阻尼，加快恢复）
    if p.drag_b > 0
        v_cmd_pre = v_cmd_pre - (p.drag_b * drag_scale) * robot_vel;
    end

    % 限速与限加速度
    spd = norm(v_cmd_pre);
    if spd > p.v_max
        v_cmd_pre = v_cmd_pre * (p.v_max/(spd+1e-9));
    end
    dv_cmd = v_cmd_pre - robot_vel;
    dvn = norm(dv_cmd); max_dv = p.a_max * dt;
    if dvn > max_dv
        dv_cmd = dv_cmd * (max_dv/(dvn+1e-9));
    end
    v_cmd = robot_vel + dv_cmd;

    % 调试
    if nargout >= 3
        dbg.v_ref = v_ref;
        dbg.kappa = kappa;
        dbg.e_n = e_n;
        dbg.de_n = de_n;
        dbg.offtrack = state.offtrack;
        dbg.recover_t = state.recover_t;
        dbg.v_cmd_pre = v_cmd_pre;
    end

% ===================== 子函数区 =====================
function cache = build_cache(path)
% 预采样每段Bezier，用于快速最近点与弧长计算
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
        B  = cubic_eval(P0,P1,P2,P3,tt);
        D1 = cubic_derivative(P0,P1,P2,P3,tt);
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
% 在所有段上搜索最近点（粗采样+局部细化）
    ns = numel(cache.segs);
    best_dist2 = inf;
    seg_idx = 1; idx = 1;

    % 1) 粗搜
    for s = 1:ns
        B = cache.pos{s};     % (N+1)x2
        diffv = B - p;
        dist2 = sum(diffv.^2,2);
        [dmin, imin] = min(dist2);
        if dmin < best_dist2
            best_dist2 = dmin;
            seg_idx = s;
            idx = imin;
        end
    end

    % 2) 细化（黄金分割在相邻采样区间内）
    t0 = cache.t_grid(max(1,idx-1));
    t1 = cache.t_grid(min(length(cache.t_grid),idx+1));
    [t_near, p_near] = golden_find_min(@(t) dist2_curve(t, cache.segs(seg_idx), p), t0, t1, 10);
    d1_near = cubic_derivative(cache.segs(seg_idx).P0, cache.segs(seg_idx).P1, cache.segs(seg_idx).P2, cache.segs(seg_idx).P3, t_near);
end

function [seg_idx, t_look, p_look, d1_look] = lookahead_point(seg_idx, t_cur, L, steps, cache)
% 从(seg_idx, t_cur)开始，沿曲线推进，累计弧长>=L时得到前视点
    ns = numel(cache.segs);
    sidx = seg_idx; t = t_cur;
    remain = L;

    for iter = 1:steps
        % 当前段剩余弧长、单步推进
        [seg_len, seg_s_now] = t_to_s(cache, sidx, t);
        seg_remain = seg_len - seg_s_now;
        if remain <= seg_remain + 1e-9
            % 在本段内可满足remain -> 反解到新的 t
            t_look = s_to_t(cache, sidx, seg_s_now + remain);
            p_look = cubic_eval(cache.segs(sidx).P0, cache.segs(sidx).P1, cache.segs(sidx).P2, cache.segs(sidx).P3, t_look);
            d1_look= cubic_derivative(cache.segs(sidx).P0, cache.segs(sidx).P1, cache.segs(sidx).P2, cache.segs(sidx).P3, t_look);
            seg_idx = sidx;
            return;
        else
            % 用掉本段剩余，跳到下一段开头
            remain = remain - seg_remain;
            if sidx < ns
                sidx = sidx + 1;
                t = 0.0;
            else
                % 已到最后段末，直接返回末端
                t_look = 1.0;
                p_look = cubic_eval(cache.segs(sidx).P0, cache.segs(sidx).P1, cache.segs(sidx).P2, cache.segs(sidx).P3, t_look);
                d1_look= cubic_derivative(cache.segs(sidx).P0, cache.segs(sidx).P1, cache.segs(sidx).P2, cache.segs(sidx).P3, t_look);
                seg_idx = sidx;
                return;
            end
        end
    end
    % 保护：步数不够时，返回当前点
    t_look = t;
    p_look = cubic_eval(cache.segs(sidx).P0, cache.segs(sidx).P1, cache.segs(sidx).P2, cache.segs(sidx).P3, t_look);
    d1_look= cubic_derivative(cache.segs(sidx).P0, cache.segs(sidx).P1, cache.segs(sidx).P2, cache.segs(sidx).P3, t_look);
    seg_idx = sidx;
end

function [seg_len, seg_s] = t_to_s(cache, seg_idx, t)
% 将 t 映射为该段的弧长 s (0..L)
    tt = cache.t_grid;
    cum = cache.cumlen{seg_idx};
    Lseg = cache.len(seg_idx);
    if t <= 0
        seg_len = Lseg; seg_s = 0; return;
    elseif t >= 1
        seg_len = Lseg; seg_s = Lseg; return;
    end
    % 插值
    k = find(tt <= t, 1, 'last');
    if k >= numel(tt), k = numel(tt)-1; end
    t0 = tt(k); t1 = tt(k+1);
    p = (t - t0) / (t1 - t0);
    s0 = cum(k); s1 = cum(k+1);
    seg_s = s0*(1-p) + s1*p;
    seg_len = Lseg;
end

function t = s_to_t(cache, seg_idx, s)
% 将段内弧长 s(0..Lseg)反解为 t(0..1) (用查表线性插值)
    cum = cache.cumlen{seg_idx};
    Lseg = cache.len(seg_idx);
    if s <= 0, t=0; return; end
    if s >= Lseg, t=1; return; end
    % 在 cum 中找区间
    k = find(cum <= s, 1, 'last');
    if k >= numel(cum), t=1; return; end
    s0 = cum(k); s1 = cum(k+1);
    if s1 - s0 < 1e-9
        t = (k-1)/ (numel(cum)-1);
        return;
    end
    p = (s - s0) / (s1 - s0);
    tt = cache.t_grid;
    t0 = tt(k); t1 = tt(k+1);
    t = t0*(1-p) + t1*p;
end

function val = dist2_curve(t, seg, p)
% 点到三次贝塞尔在 t 处的距离平方
    q = cubic_eval(seg.P0, seg.P1, seg.P2, seg.P3, t);
    d = q - p;
    val = d(1)^2 + d(2)^2;
end

function [t_best, q_best] = golden_find_min(f, a, b, iters)
% 黄金分割搜索单峰最小值 (在 [a,b] 上)
    gr = (sqrt(5)-1)/2; % ~0.618
    c = b - gr*(b-a);
    d = a + gr*(b-a);
    fc = f(c);
    fd = f(d);
    for i=1:iters
        if fc < fd
            b = d; d = c; fd = fc; c = b - gr*(b-a); fc = f(c);
        else
            a = c; c = d; fc = fd; d = a + gr*(b-a); fd = f(d);
        end
    end
    if fc < fd
        t_best = c;
    else
        t_best = d;
    end
    q_best = []; % 调用方若需要可再求一次
end

function B = cubic_eval(P0,P1,P2,P3,t)
    % 支持标量 t 或列向量 t
    if numel(t) > 1
        tt = t(:);
        B = (1-tt).^3.*P0 + 3*(1-tt).^2.*tt.*P1 + 3*(1-tt).*tt.^2.*P2 + tt.^3.*P3;
    else
        tt = t;
        B = (1-tt)^3.*P0 + 3*(1-tt)^2.*tt.*P1 + 3*(1-tt).*tt^2.*P2 + tt^3.*P3;
    end
end

function d1 = cubic_derivative(P0,P1,P2,P3,t)
    d1 = 3*(1-t).^2.*(P1-P0) + 6*(1-t).*t.*(P2-P1) + 3*t.^2.*(P3-P2);
    if size(d1,1)==1, d1 = d1(:)'; end
end

function p = set_default(p, varargin)
    for i=1:2:numel(varargin)
        k = varargin{i}; v = varargin{i+1};
        if ~isfield(p,k) || isempty(p.(k)), p.(k) = v; end
    end
end
