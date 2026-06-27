classdef Path_Bezier < handle
% Path_Bezier 基于贝塞尔曲线的局部路径规划（MATLAB封装）
% - 一阶/二阶贝塞尔路径
% - 最近点 t 搜索（直线解析/二阶黄金分割）
% - 简化S型速度规划（对称加减速，位置型）
%
% 构造:
%   pb = Path_Bezier(start, end, params)                   % 一阶
%   pb = Path_Bezier(start, ctrl, end, params)             % 二阶
% params 字段(可选): initialSpeed, vmax, a, endSpeed
%  默认: initialSpeed=1e-3, vmax=0.6, a=1.0, endSpeed=0
%
% 主要方法:
%   v = pb.plan(point)         % 根据当前位置返回速度向量 [vx, vy]
%   pb.reset()
%   pb.update(start,end) 或 pb.update(start,ctrl,end)
%   finished = pb.isFinished()
%
% 可访问成员:
%   pb.t, pb.v_tangent, pb.v_resultant, pb.distance, pb.len

properties (Constant, Access=private)
    FIRST_ORDER_BEZIER = 1;
    SECOND_ORDER_BEZIER = 2;
    GOLDEN_RATIO = (sqrt(5)-1)/2;     % ~0.618
    BEZIER_SAMPLE_NUM = 100;
    FIND_NEAREST_STEPS = 10;
end

properties (Access=private)
    % 曲线数据
    order = 1;
    start_point (1,2) double = [0 0];
    end_point   (1,2) double = [0 0];
    control_point (1,2) double = [0 0];
    len double = 0;
    distance_list double = zeros(1,100);
    bezier_sample_step double = 1/100;
    tangent_vector (1,2) double = [1 0];

    % 速度约束（基于曲率的最大速度）
    end_vel double = 0;
    max_vel_list double = zeros(1,101);
    max_curvature_len double = 0;
    max_curvature_max_vel double = 1e6;
    current_max_vel double = 0;

    % 速度规划参数（简化S曲线-位置型）
    initialSpeed double = 1e-3;
    vmax double = 0.6;
    a double = 1.0;        % 对称加减速
    endSpeed double = 0.0;
end

properties (Access=public)
    % 运行态
    t double = 0;                 % 最近点参数 t
    v_resultant double = 0;       % 标量速度
    distance double = 0;          % 已行进距离（由plan根据点差累加）
    v_tangent (1,2) double = [0 0];
    point_last (1,2) double = [0 0];
    m_phase char {mustBeMember(m_phase,{'ACCEL','CRUISE','DECEL','FINISHED'})} = 'ACCEL';
end

methods
    function obj = Path_Bezier(p1, p2, p3, params)
        % 构造函数重载：一阶/二阶
        if nargin==0, return; end
        if nargin<=3
            % 一阶: (start,end[,params])
            start = p1;  finish = p2;
            obj.update1(start, finish);
            obj.initSpeedParams(params);
            obj.point_last = start;
        else
            % 二阶: (start,ctrl,end[,params])
            start = p1; ctrl = p2; finish = p3;
            obj.update2(start, ctrl, finish);
            obj.initSpeedParams(params);
            obj.point_last = start;
        end
    end

    function v = plan(obj, point)
        % 速度标量（位置型S曲线 + 曲率限速）
        obj.v_resultant = obj.planSpeed(obj.distance);
        % 最近点与切向
        obj.t = obj.getNearestT(point);
        obj.v_tangent = obj.getTangent(obj.t);
        % 距离累加
        obj.distance = obj.distance + norm(point - obj.point_last);
        obj.point_last = point;
        % 生成速度向量
        v = obj.v_tangent .* obj.v_resultant;
        % 完成判定
        if obj.distance >= obj.len - 1e-6
            obj.m_phase = 'FINISHED';
        end
    end

    function reset(obj)
        obj.m_phase = 'ACCEL';
        obj.point_last = obj.start_point;
        obj.distance = 0;
        obj.t = 0;
        obj.v_resultant = 0;
    end

    function update(obj, varargin)
        if numel(varargin)==2
            obj.update1(varargin{1}, varargin{2});
        elseif numel(varargin)==3
            obj.update2(varargin{1}, varargin{2}, varargin{3});
        else
            error('update 需要 (start,end) 或 (start,ctrl,end)');
        end
        % 更新速度终点距离
        obj.reset();
    end

    function tf = isFinished(obj)
        tf = strcmp(obj.m_phase,'FINISHED');
    end

    % 只读访问
    function L = Get_len(obj), L = obj.len; end
    function p = Get_Point(obj, t), p = obj.getPoint(t); end
end

methods (Access=private)
    function initSpeedParams(obj, params)
        if nargin<2 || isempty(params), params = struct(); end
        if ~isfield(params,'initialSpeed'), params.initialSpeed = 1e-3; end
        if ~isfield(params,'vmax'),         params.vmax = 0.6; end
        if ~isfield(params,'a'),            params.a = 1.0; end
        if ~isfield(params,'endSpeed'),     params.endSpeed = 0.0; end
        obj.initialSpeed = max(1e-5, params.initialSpeed);
        obj.vmax = params.vmax;
        obj.a = max(1e-6, params.a);
        obj.endSpeed = max(0, params.endSpeed);
    end

    function update1(obj, start_point_, end_point_)
        obj.order = obj.FIRST_ORDER_BEZIER;
        obj.start_point = start_point_;
        obj.end_point   = end_point_;
        d = end_point_ - start_point_;
        L = norm(d);
        obj.len = L;
        if L < 1e-9
            obj.tangent_vector = [1 0];
        else
            obj.tangent_vector = d ./ L;
        end
        % 直线无曲率限制
        obj.max_vel_list = inf(1, obj.BEZIER_SAMPLE_NUM+1);
        obj.max_curvature_len = 0;
        obj.max_curvature_max_vel = obj.max_vel_list(1);
    end

    function update2(obj, start_point_, control_point_, end_point_)
        obj.order = obj.SECOND_ORDER_BEZIER;
        obj.start_point   = start_point_;
        obj.control_point = control_point_;
        obj.end_point     = end_point_;
        obj.len = 0;
        obj.end_vel = 0;

        % 初始曲率与最大速度
        curv0 = obj.getCurvature(0.0);
        if curv0 < 1e-6, obj.max_vel_list(1) = 1e6;
        elseif curv0 > 1e6, obj.max_vel_list(1) = 0.1;
        else, obj.max_vel_list(1) = 1/sqrt(curv0);
        end
        obj.max_curvature_len = 0.0;
        obj.max_curvature_max_vel = obj.max_vel_list(1);

        % 采样长度与限速表
        t = 0;
        for i = 1:obj.BEZIER_SAMPLE_NUM
            if i < obj.BEZIER_SAMPLE_NUM
                seg = norm(obj.getPoint(t+obj.bezier_sample_step) - obj.getPoint(t));
                t = t + obj.bezier_sample_step;
                curv = obj.getCurvature(t);
            else
                seg = norm(obj.end_point - obj.getPoint(t));
                curv = obj.getCurvature(1.0);
            end
            obj.len = obj.len + seg;
            obj.distance_list(i) = obj.len;

            if curv < 1e-6, obj.max_vel_list(i+1) = 1e6;
            elseif curv > 1e6, obj.max_vel_list(i+1) = 0.1;
            else, obj.max_vel_list(i+1) = 1/sqrt(curv);
            end

            if obj.max_vel_list(i+1) < obj.max_curvature_max_vel
                obj.max_curvature_len = obj.len;
                obj.max_curvature_max_vel = obj.max_vel_list(i+1);
            end
        end
    end

    function v = planSpeed(obj, s)
        % 对称加减速的“位置型”速度上界 + 曲率限速
        L = max(obj.len, 1e-9);
        s = min(max(s,0), L);
        s_remain = L - s;

        v_acc = sqrt(max(0, obj.initialSpeed^2 + 2*obj.a*s));
        v_dec = sqrt(max(0, obj.endSpeed^2    + 2*obj.a*s_remain));
        v_profile = min([obj.vmax, v_acc, v_dec]);

        % 曲率限速（按最近 t）
        t_near = obj.t;
        vmax_curv = obj.getMaxVel(t_near);

        v = max(0, min(v_profile, vmax_curv));

        if s >= L - 1e-6
            obj.m_phase = 'FINISHED';
        elseif v_profile == v_acc && v < obj.vmax
            obj.m_phase = 'ACCEL';
        elseif v_profile == obj.vmax
            obj.m_phase = 'CRUISE';
        else
            obj.m_phase = 'DECEL';
        end
    end

    function p = getPoint(obj, t)
        if t <= 0, p = obj.start_point; return; end
        if t >= 1, p = obj.end_point;   return; end
        if obj.order == obj.FIRST_ORDER_BEZIER
            p = (1-t)*obj.start_point + t*obj.end_point;
        else
            p01 = (1-t)*obj.start_point + t*obj.control_point;
            p12 = (1-t)*obj.control_point + t*obj.end_point;
            p   = (1-t)*p01 + t*p12;
        end
    end

    function t = getNearestT(obj, point)
        if obj.order == obj.FIRST_ORDER_BEZIER
            d = obj.end_point - obj.start_point;
            v = point - obj.start_point;
            dd = dot(d,d);
            if dd < 1e-9, t = 0; return; end
            tval = dot(v,d)/dd;
            t = min(max(tval,0),1);
        else
            left = 0; right = 1;
            t1 = 1 - obj.GOLDEN_RATIO;
            t2 = obj.GOLDEN_RATIO;
            d1 = sum((obj.getPoint(t1)-point).^2);
            d2 = sum((obj.getPoint(t2)-point).^2);
            for i=1:obj.FIND_NEAREST_STEPS
                if d1 < d2
                    right = t2; t2 = t1; d2 = d1;
                    t1 = right - obj.GOLDEN_RATIO*(right-left);
                    d1 = sum((obj.getPoint(t1)-point).^2);
                else
                    left = t1; t1 = t2; d1 = d2;
                    t2 = left + obj.GOLDEN_RATIO*(right-left);
                    d2 = sum((obj.getPoint(t2)-point).^2);
                end
            end
            % 边界比较
            if d1 > d2, d1=d2; t1=t2; end
            dL = sum((obj.getPoint(left)-point).^2);
            if dL < d1, d1=dL; t1=left; end
            dR = sum((obj.getPoint(right)-point).^2);
            if dR < d1, t1=right; end
            t = t1;
        end
    end

    function v = getTangent(obj, t)
        if obj.order == obj.FIRST_ORDER_BEZIER
            v = obj.tangent_vector;
        else
            % 中心差分近似
            dt = 0.01;
            p1 = obj.getPoint(max(0,t-dt));
            p2 = obj.getPoint(min(1,t+dt));
            d  = p2 - p1;
            n  = norm(d);
            if n < 1e-9, v = obj.tangent_vector; else, v = d./n; end
            obj.tangent_vector = v;
        end
    end

    function c = getCurvature(obj, t)
        t = min(max(t,0),1);
        if obj.order == obj.FIRST_ORDER_BEZIER
            c = 0; return;
        end
        A = 2*(obj.control_point - obj.start_point);
        B = 2*(obj.end_point - 2*obj.control_point + obj.start_point);
        d1 = A + B.*t; dx = d1(1); dy = d1(2);
        if abs(dx)<1e-6 && abs(dy)<1e-6, c = 0; return; end
        d2 = 2*(obj.end_point - 2*obj.control_point + obj.start_point);
        ddx = d2(1); ddy = d2(2);
        num = abs(dx*ddy - dy*ddx);
        den2 = dx*dx + dy*dy;
        if den2 < 1e-6, c = 1e6; return; end
        c = num/(den2^(3/2));
        c = min(max(c,0),1e6);
    end

    function L = getCurrentLen(obj, t)
        t = min(max(t,0),1);
        if obj.order == obj.FIRST_ORDER_BEZIER
            L = norm(obj.getPoint(t)-obj.start_point);
            return;
        end
        idx = floor(t/obj.bezier_sample_step);
        p = (t - idx*obj.bezier_sample_step)/obj.bezier_sample_step;
        if idx==0
            L = obj.distance_list(1) * p;
        elseif idx < obj.BEZIER_SAMPLE_NUM
            L = obj.distance_list(idx) * (1-p) + obj.distance_list(idx+1)*p;
        else
            L = obj.len;
        end
    end

    function vmax = getMaxVel(obj, t)
        t = min(max(t,0),1);
        s = obj.getCurrentLen(t);
        % 末端剩余距离速度上界
        vmax_rem = sqrt(max(0, (obj.len - s)*2 + obj.end_vel^2));
        vmax = vmax_rem;
        if obj.order == obj.SECOND_ORDER_BEZIER
            idx = floor(t/obj.bezier_sample_step)+1;
            p = (t - (idx-1)*obj.bezier_sample_step)/obj.bezier_sample_step;
            if idx <= obj.BEZIER_SAMPLE_NUM
                vmax_curv = obj.max_vel_list(idx)*(1-p) + obj.max_vel_list(idx+1)*p;
            else
                vmax_curv = obj.max_vel_list(end);
            end
            vmax = min(vmax, vmax_curv);
            if s < obj.max_curvature_len
                vmax2 = sqrt(max(0, (obj.max_curvature_len - s)*2 + obj.max_curvature_max_vel^2));
                vmax = min(vmax, vmax2);
            end
        end
        obj.current_max_vel = vmax;
    end
end
end