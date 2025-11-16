clear; clc; close all;

% ================== 参数 ==================
dt       = 0.01;
v_start  = 0.05;
v_target = 1.5;
a_acc    = 1.5;
v_end    = v_target;    % 终点速度(若需减速设为其他值)
tau      = 0.35;        % 控制点距离比例(0~0.6)
tol_done = 0.01;

% 机器人初始
x=0; y=0; yaw=0; theta=0; L_min=0.67; L=L_min; h=0.40;
base_size=0.60; base_thick=0.05; turret_r=0.08;

% 采集路径点
figure('Color','w'); hold on; axis equal; grid on;
xlabel('X'); ylabel('Y'); title('点击路径点, 回车结束');
xlim([-2 2]); ylim([-2 2]);
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

% 把初始位置作为首点插入
if norm([x,y]-wps(1,:))>1e-9
    wps=[ [x,y]; wps ];
end

plot(wps(:,1),wps(:,2),'ko','MarkerFaceColor',[1 1 0],'MarkerSize',6);
plot(wps(:,1),wps(:,2),'k--');

% ================== 构建三次贝塞尔段 ==================
% 三次段：Pi -> Pi+1
% 控制点：B1 = Pi + k_i * T_i, B2 = Pi+1 - k_{i+1} * T_{i+1}
% T_i = normalize(P_{i+1}-P_{i-1}), 端点用单侧差分
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

seg = struct('P0',{},'P1',{},'P2',{},'P3',{},'L',{});

for i=1:n-1
    P0=wps(i,:); P3=wps(i+1,:);
    d = norm(P3-P0);
    k0 = tau*d; k1 = tau*d;
    P1 = P0 + k0*T(i,:);
    P2 = P3 - k1*T(i+1,:);
    % 弧长估算
    L = bezier_cubic_len(P0,P1,P2,P3);
    seg(i).P0=P0; seg(i).P1=P1; seg(i).P2=P2; seg(i).P3=P3; seg(i).L=L;
    % 预览
    tt=linspace(0,1,50); B = cubic_eval(P0,P1,P2,P3,tt);
    plot(B(:,1),B(:,2),'b-','LineWidth',1.5);
end
legend({'Waypoints','Polyline','Bezier path'},'Location','best');

% 累积长度
cumL = [0 cumsum([seg.L])];
total_len = cumL(end);

% ================== 速度规划：加速到 v_target 后保持匀速 ==================
s=0; v=v_start;

% 若需要终点减速，可添加减速段，这里保持匀速到终点（v_end=v_target）
while s < total_len - 1e-9
    % 加速
    if v < v_target
        v = v + a_acc*dt;
        if v > v_target, v = v_target; end
    end

    % 前进弧长
    s = s + v*dt;
    if s > total_len, s = total_len; end

    % 定位当前段
    idx = find(cumL <= s,1,'last');
    if idx >= length(cumL), idx = length(cumL)-1; end
    seg_s = s - cumL(idx);
    Lseg  = seg(idx).L;
    t = seg_s / (Lseg + 1e-9);    % 近似 t
    if t>1, t=1; end

    % 位置与切向
    P0=seg(idx).P0; P1=seg(idx).P1; P2=seg(idx).P2; P3=seg(idx).P3;
    pos = cubic_eval(P0,P1,P2,P3,t);
    d1  = cubic_derivative(P0,P1,P2,P3,t);
    dir = d1/(norm(d1)+1e-9);

    % 更新底盘
    x=pos(1); y=pos(2);
    update_robot_pose(patchBase,patchTurret,lineFix,lineExt,...
        x,y,yaw,theta,L,h,base_size,base_thick,turret_r,L_min);
    drawnow limitrate;
    pause(dt);
    if ~ishandle(patchBase), return; end
end

% 终点判定
if norm([x,y]-wps(end,:))<tol_done
    title('加速后匀速完成整条曲线');
else
    title('路径结束');
end
disp('完成');

% ================== 函数区 ==================
function B = cubic_eval(P0,P1,P2,P3,t)
    t = t(:);
    B = (1-t).^3.*P0 + 3*(1-t).^2.*t.*P1 + 3*(1-t).*t.^2.*P2 + t.^3.*P3;
end

function d1 = cubic_derivative(P0,P1,P2,P3,t)
    % 一阶导
    d1 = 3*(1-t).^2.*(P1-P0) + 6*(1-t).*t.*(P2-P1) + 3*t.^2.*(P3-P2);
    if size(d1,1)==1, d1=d1(:)'; end
end

function L = bezier_cubic_len(P0,P1,P2,P3)
    N=200; tt=linspace(0,1,N+1);
    B = cubic_eval(P0,P1,P2,P3,tt);
    d = diff(B,1,1);
    L = sum(sqrt(sum(d.^2,2)));
end

function Q = dedup_close_points(P,epsd)
    if isempty(P), Q=P; return; end
    Q=P(1,:);
    for i=2:size(P,1)
        if norm(P(i,:)-Q(end,:))>epsd
            Q=[Q;P(i,:)];
        end
    end
end