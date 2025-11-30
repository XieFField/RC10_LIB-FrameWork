clear; clc; close all;

% 添加函数路径
addpath('function');

% 初始化场景
figure('Color', 'w', 'Name', 'Robocon 2026 Simulation: Precise Approach & Grab');
axis equal; grid on; hold on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(3); 
% light('Position', [1 -2 1], 'Style', 'infinite');
% light('Position', [-1 2 0.5], 'Style', 'infinite');
% lighting gouraud;
xlim([-1.5 1.0]); ylim([-2.0 1.0]); zlim([0 1.5]);

% ==========================================
% 1. 放置武器架 & 矛杆
% ==========================================
build_stent('Position', [0, 0, 0]);
% 矛杆参数
weapon_len = 1027; 
weapon_radius = 20.3 / 2; 
hole_radius = 25 / 2; 
z_contact_lower = 80; 
y_hole_center = -110; % 修正后的孔位置
y_slot_back_wall = -45; 
z_contact_upper = 480; 

% 计算矛杆初始姿态向量
y_hole_front_edge = y_hole_center - hole_radius; 
y_axis_lower = y_hole_front_edge + weapon_radius; 
y_axis_upper = y_slot_back_wall - weapon_radius;
vec_y = y_axis_upper - y_axis_lower; 
vec_z = z_contact_upper - z_contact_lower;
dir_vec = [0, vec_y, vec_z]; dir_vec = dir_vec / norm(dir_vec);
init_vec = [1, 0, 0]; 
rot_axis = cross(init_vec, dir_vec); rot_axis = rot_axis / norm(rot_axis);
rot_angle = acos(dot(init_vec, dir_vec));
t_ground = -z_contact_lower / dir_vec(3);
offset_to_ground = t_ground * dir_vec;
center_offset = (weapon_len / 2) * dir_vec;

% 放置4根矛杆并保存句柄
hole_x_positions = [-300, -100, 100, 300];
h_weps = gobjects(1, length(hole_x_positions)); % 存储句柄
wep_init_transforms = cell(1, length(hole_x_positions)); % 存储初始变换矩阵

for i = 1:length(hole_x_positions)
    hx = hole_x_positions(i);
    ref_pos = [hx, y_axis_lower, z_contact_lower];
    ground_pos = ref_pos + offset_to_ground;
    final_pos_mm = ground_pos + center_offset;
    
    h_wep = build_weapon('Position', final_pos_mm/1000, 'Color', [0.85, 0.85, 0.9]);
    M_scale = makehgtform('scale', 0.001); 
    M_rot = makehgtform('axisrotate', rot_axis, rot_angle);
    M_trans = makehgtform('translate', final_pos_mm/1000);
    
    final_M = M_trans * M_rot * M_scale;
    set(h_wep, 'Matrix', final_M);
    
    h_weps(i) = h_wep;
    wep_init_transforms{i} = final_M;
end

% ==========================================
% 2. 运动轨迹规划 (Waypoints)
% ==========================================
robot_rot = 0; 

% --- 关键坐标计算 ---
x_contact = -0.11; 
x_start_gap = x_contact - 0.05; 

% 修正: Y轴接近位置 (y_approach)
% 逻辑: 凸起(130mm长)越过武器架前沿60mm. 
% 此时缺口台阶面距离武器架前沿 130 - 60 = 70mm.
% 武器架前沿 Y = -0.13m.
% 机器人凸起前端局部 Y = +0.49m.
% y_approach + 0.49 = -0.13 + 0.06 => y_approach = -0.56m
y_approach = -0.56;

y_final = -0.49; % 抵住下横梁的位置 (前进70mm后到达)
y_start_far = -1.5;

z_high = 980;
z_low = 515; % 夹爪下沿500mm -> 中心515mm

% --- 速度设置 ---
% 新增: 底盘最大速度限制 (m/s)
max_chassis_speed = 0.3; 

% 应用速度限制
speed_x = min(0.2, max_chassis_speed); % 横移速度通常较慢，保持原值或受限
speed_y = max_chassis_speed;           % 前进/后退使用最大速度
speed_z = 0.8;                         % 升降速度独立控制

% --- 阶段时间计算 ---
% Phase 1: Y轴接近 (到达 -0.56)
t1 = abs(y_approach - y_start_far) / speed_y;

% Phase 2: X轴横移 (贴靠侧边)
t2 = abs(x_contact - x_start_gap) / speed_x;

% Phase 3: Y轴继续前进 (消除70mm间隙, 抵住下横梁)
t3_move = abs(y_final - y_approach) / speed_y;
t3_z = abs(z_high - z_low) / 1000 / speed_z;
t3 = max(t3_move, t3_z);

% 新增: 夹取动作耗时 (停顿夹紧)
t_grab_duration = 0.5; 

% Phase 4: 后退至垂直 (Retreat to Vertical)
% 夹爪中心局部Y = 440mm (0.44m). 孔中心全局Y = -0.11m.
% 目标Robot Y = -0.11 - 0.44 = -0.55m.
y_vertical = -0.11 - 0.44; 
t4 = abs(y_vertical - y_final) / speed_y; 

% Phase 5: 垂直提升 (Lift)
% 优化: 分为两段
% 5a. 抬升至脱离孔 (Lift to Clear)
z_clear = z_low + 150; % 抬升150mm即可脱离
t5a = abs(z_clear - z_low) / 1000 / speed_z;

% 5b. 继续抬升至最高点 (Lift to High)
t5b = abs(z_high - z_clear) / 1000 / speed_z;

% Phase 6: 撤离 (Leave)
y_leave = -1.5;
t6 = abs(y_leave - y_vertical) / speed_y;

% 优化策略: 
% 动作1: 抬升到 z_clear (耗时 t5a)
% 动作2: 抬升剩余部分 (t5b) 和 后退 (t6) 同时进行
% 总耗时 = t5a + max(t5b, t6)

% --- 时间节点计算 ---
t_arrival = t1 + t2 + t3;              % 到达最低点时刻
t_grab_done = t_arrival + t_grab_duration; % 夹紧完成时刻
t_retreat_done = t_grab_done + t4;     % 后退完成时刻
t_clear_done = t_retreat_done + t5a;   % 脱离孔时刻
t_final_done = t_clear_done + max(t5b, t6); % 最终完成时刻

% 轨迹点定义 [Time, X, Y, Z]
waypoints = [
    0,              x_start_gap, y_start_far, z_high;  
    t1,             x_start_gap, y_approach,  z_high;  
    t1+t2,          x_contact,   y_approach,  z_high;  
    t_arrival,      x_contact,   y_final,     z_low;     % 到达最低点 (夹爪仍张开)
    t_grab_done,    x_contact,   y_final,     z_low;     % 原地停顿 (执行夹取)
    t_retreat_done, x_contact,   y_vertical,  z_low;     % 后退至垂直
    t_clear_done,   x_contact,   y_vertical,  z_clear;   % 抬升至脱离高度
    t_final_done,   x_contact,   y_leave,     z_high     % 同时后退和继续抬升
];

total_time = waypoints(end, 1);
fps = 30;
num_steps = ceil(total_time * fps);

h_robot = [];
title('Robocon 2026: 自动取矛流程仿真');

% 目标矛杆索引
target_wep_idx = 1; 
target_hole_pos = [hole_x_positions(target_wep_idx)/1000, y_hole_center/1000, z_contact_lower/1000];

% 等待用户点击“开始”后再运行仿真
fig = uifigure('Name','仿真控制','Position',[300 300 320 120],'Resize','off');
uilabel(fig,'Text','点击“开始”以运行仿真','HorizontalAlignment','center','Position',[10 70 300 30],'FontSize',12);
uibutton(fig,'Text','开始','Position',[110 20 100 36],'ButtonPushedFcn',@(src,event) uiresume(fig));
uiwait(fig);    % 等待按钮触发 uiresume
close(fig);

% ==========================================
% 3. 动画循环
% ==========================================
for step = 1:num_steps
    t_now = (step - 1) / fps;
    
    % --- 1. 机器人轨迹插值 ---
    idx = find(waypoints(:,1) >= t_now, 1);
    if isempty(idx), idx = size(waypoints, 1); end
    if idx == 1
        curr_x = waypoints(1,2); curr_y = waypoints(1,3); curr_z = waypoints(1,4);
    else
        t_prev = waypoints(idx-1, 1);
        t_next = waypoints(idx, 1);
        ratio = (t_now - t_prev) / (t_next - t_prev);
        p_prev = waypoints(idx-1, 2:4);
        p_next = waypoints(idx, 2:4);
        p_curr = p_prev + ratio * (p_next - p_prev);
        curr_x = p_curr(1); curr_y = p_curr(2); curr_z = p_curr(3);
    end
    
    % --- 2. 绘制机器人 ---
    if ~isempty(h_robot), delete(h_robot); end
    
    gripper_display_x = -190;
    
    % 夹爪状态控制: 
    % t < t_arrival: 张开 (1)
    % t_arrival <= t < t_grab_done: 线性闭合动画 (1 -> 0)
    % t >= t_grab_done: 闭合 (0)
    if t_now < t_arrival
        g_state = 1; 
    elseif t_now < t_grab_done
        % 线性插值实现平滑闭合效果
        g_state = 1 - (t_now - t_arrival) / t_grab_duration;
    else
        g_state = 0; 
    end

    h_robot = build_robot(...
        'Position', [curr_x, curr_y, 0], ...
        'Rotation', [0, 0, robot_rot], ...
        'GripperHeight', curr_z, ...
        'GripperX', gripper_display_x, ... 
        'GripperState', g_state, ... 
        'Color', [0.3, 0.3, 0.35]);
    
    % --- 3. 更新被抓矛杆姿态 ---
    % 只有当夹爪完全闭合后 (t >= t_grab_done)，矛杆才跟随运动
    if t_now >= t_grab_done
        % 计算夹爪中心全局坐标
        g_global_pos = [curr_x + gripper_display_x/1000, curr_y + 0.44, curr_z/1000];
        
        if t_now < t_retreat_done
            % Phase 4: 后退变垂直
            vec_spear = g_global_pos - target_hole_pos;
            len_current = norm(vec_spear);
            dir_spear = vec_spear / len_current;
            
            new_center = target_hole_pos + dir_spear * (weapon_len/2000);
            
            init_vec = [1, 0, 0];
            rot_axis_w = cross(init_vec, dir_spear); 
            if norm(rot_axis_w) < 1e-6, rot_axis_w = [0 1 0]; end
            rot_axis_w = rot_axis_w / norm(rot_axis_w);
            rot_angle_w = acos(dot(init_vec, dir_spear));
            
            M_scale = makehgtform('scale', 0.001);
            M_rot = makehgtform('axisrotate', rot_axis_w, rot_angle_w);
            M_trans = makehgtform('translate', new_center);
            set(h_weps(target_wep_idx), 'Matrix', M_trans * M_rot * M_scale);
            
        else
            % Phase 5 & 6: 垂直提升与撤离
            offset_z = (weapon_len/2000) - (z_low/1000 - z_contact_lower/1000);
            spear_center = g_global_pos + [0, 0, offset_z];
            
            dir_spear = [0, 0, 1];
            rot_axis_w = [0, -1, 0]; 
            rot_angle_w = pi/2;
            
            M_scale = makehgtform('scale', 0.001);
            M_rot = makehgtform('axisrotate', rot_axis_w, rot_angle_w);
            M_trans = makehgtform('translate', spear_center);
            set(h_weps(target_wep_idx), 'Matrix', M_trans * M_rot * M_scale);
        end
    end
    
    drawnow;
    
    % 强制等待，使动画播放速度与设定速度匹配
    pause(1/fps); 
end

text(0, 0, 1.2, '抓取完成', 'HorizontalAlignment', 'center', 'FontSize', 12, 'Color', 'r');