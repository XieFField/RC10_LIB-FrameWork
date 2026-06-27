function robot_h = build_robot(varargin)
% BUILD_ROBOT 创建机器人底盘及龙门架机构的3D模型 (支持动态夹爪开合)
%
% 输入参数 (Name-Value):
%   'Parent'        : 父对象句柄
%   'Position'      : [x, y, z] 机器人中心位置
%   'Rotation'      : [rx, ry, rz] 旋转角度
%   'Color'         : [r, g, b] 主体颜色
%   'GripperHeight' : 夹爪中心离地高度 (mm), 默认 900
%   'GripperY'      : (已废弃，夹爪Y位置固定为伸出状态)
%   'GripperX'      : 夹爪相对于机器人中心的X轴偏移 (mm)
%   'GripperState'  : 夹爪开合状态, 0(闭合) ~ 1(张开), 默认 0

    % 解析输入参数
    p = inputParser;
    addParameter(p, 'Parent', gca);
    addParameter(p, 'Position', [0, 0, 0]);
    addParameter(p, 'Rotation', [0, 0, 0]);
    addParameter(p, 'Color', [0.2, 0.2, 0.2]); 
    addParameter(p, 'GripperHeight', 900); 
    addParameter(p, 'GripperY', 490);      
    addParameter(p, 'GripperX', 0);
    addParameter(p, 'GripperState', 0);    
    parse(p, varargin{:});
    
    parent_h = p.Results.Parent;
    init_pos = p.Results.Position;
    init_rot = p.Results.Rotation;
    robot_color = p.Results.Color;
    gripper_z = p.Results.GripperHeight;
    gripper_x_offset = p.Results.GripperX;
    gripper_state = p.Results.GripperState; 

    robot_h = hgtransform('Parent', parent_h);
    
    % ==========================================
    % 1. 底盘建模 (Chassis)
    % ==========================================
    full_size = 980; 
    cut_width = 130; 
    cut_length = 780; 
    chassis_height = 100; 
    
    % 主体
    main_body_w = full_size - cut_width; 
    main_body_l = full_size; 
    draw_box(robot_h, [main_body_l, main_body_w, chassis_height], ...
        [0, -cut_width/2, chassis_height/2], robot_color);
        
    % 左后方凸起 (机械限位)
    % 外沿 Y = 490 (980/2)
    left_rem_l = full_size - cut_length; 
    left_rem_w = cut_width; 
    rem_center_x = -(full_size/2 - left_rem_l/2); 
    rem_center_y = (full_size/2 - left_rem_w/2);  
    draw_box(robot_h, [left_rem_l, left_rem_w, chassis_height], ...
        [rem_center_x, rem_center_y, chassis_height/2], robot_color);

    % ==========================================
    % 2. 龙门架机构 (Gantry)
    % ==========================================
    gantry_color = [0.8, 0.8, 0.8]; 
    total_gantry_height = 980;
    post_h = total_gantry_height - chassis_height; 
    post_sec = 40; 
    
    % 前立柱 (位于 Y=360 附近)
    post_x_front = 490 - 20; 
    post_y = 360 - 20; 
    draw_box(robot_h, [post_sec, post_sec, post_h], ...
        [post_x_front, post_y, chassis_height + post_h/2], gantry_color);
        
    % 后立柱
    post_x_back = -290; 
    draw_box(robot_h, [post_sec, post_sec, post_h], ...
        [post_x_back, post_y, chassis_height + post_h/2], gantry_color);
        
    % 顶部横梁
    beam_len = post_x_front - post_x_back;
    draw_box(robot_h, [beam_len, post_sec, post_sec], ...
        [(post_x_front+post_x_back)/2, post_y, chassis_height + post_h], gantry_color);
        
    % ==========================================
    % 3. 夹爪机构 (修正: 伸出130mm, 与机械限位平齐)
    % ==========================================
    % 几何定义:
    % 起点: Y = 360 (缺口台阶面/龙门架位置)
    % 终点: Y = 490 (机械限位外沿)
    % 长度: 130mm
    % 方向: Y+ (向前方伸出)
    
    g_start_y = 360;
    g_end_y = 490;
    g_len = 130;
    
    gx = gripper_x_offset;
    gz = gripper_z;
    
    % --- 垂直滑台 (Z轴) ---
    slide_z_h = 400;
    slide_z_pos = max(chassis_height + slide_z_h/2, min(total_gantry_height - slide_z_h/2, gripper_z));
    draw_box(robot_h, [100, 20, slide_z_h], ...
        [gx, post_y - 30, slide_z_pos], [0.3, 0.3, 0.3]);
    
    % --- 夹爪本体 ---
    
    % 1. 计算开口 (X轴方向开合)
    spear_diam = 20.3;
    gap_closed = spear_diam;       % 闭合: 20.3mm
    gap_open = spear_diam * 1.8;   % 张开: ~36.5mm
    current_gap = gap_closed + (gap_open - gap_closed) * gripper_state;
    
    % 2. 结构分段
    % 手指长度 100mm (前端), 基座长度 30mm (后端)
    finger_len = 100;
    base_len = g_len - finger_len; 
    
    % 3. 绘制基座 (Base)
    % Y范围: [360, 390]
    base_center_y = g_start_y + base_len/2;
    draw_box(robot_h, [60, base_len, 50], ...
        [gx, base_center_y, gz], [0.6, 0.1, 0.1]); % 红色基座
        
    % 4. 绘制手指 (Fingers)
    % Y范围: [390, 490] -> 末端刚好平齐 490
    finger_center_y = g_start_y + base_len + finger_len/2;
    finger_thick = 5;
    finger_height = 30;
    
    % 左指 (X-)
    draw_box(robot_h, [finger_thick, finger_len, finger_height], ...
        [gx - current_gap/2 - finger_thick/2, finger_center_y, gz], [0.2, 0.2, 0.2]); % 黑色碳纤维
    % 右指 (X+)
    draw_box(robot_h, [finger_thick, finger_len, finger_height], ...
        [gx + current_gap/2 + finger_thick/2, finger_center_y, gz], [0.2, 0.2, 0.2]);
        
    % 5. 电机 (Motor)
    % 模拟图中圆柱体电机, 位于基座上方
    draw_box(robot_h, [40, 50, 40], ...
        [gx, base_center_y, gz + 45], [0.8, 0.8, 0.8]); % 银色电机

    % ==========================================
    % 后处理
    % ==========================================
    scale_factor = 0.001;
    M_scale = makehgtform('scale', scale_factor);
    
    rx = init_rot(1); ry = init_rot(2); rz = init_rot(3);
    M_rot = makehgtform('xrotate', rx, 'yrotate', ry, 'zrotate', rz);
    M_trans = makehgtform('translate', init_pos);
    
    set(robot_h, 'Matrix', M_trans * M_rot * M_scale);
end

function draw_box(parent, size_vec, pos_vec, color)
    L = size_vec(1); W = size_vec(2); H = size_vec(3);
    v = [ -0.5 -0.5 -0.5; 0.5 -0.5 -0.5; 0.5 0.5 -0.5; -0.5 0.5 -0.5;
          -0.5 -0.5  0.5; 0.5 -0.5  0.5; 0.5 0.5  0.5; -0.5 0.5  0.5 ];
    v(:,1) = v(:,1)*L; v(:,2) = v(:,2)*W; v(:,3) = v(:,3)*H;
    v = v + pos_vec;
    f = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    patch('Vertices', v, 'Faces', f, 'FaceColor', color, 'EdgeColor', 'none', 'Parent', parent, 'FaceAlpha', 1.0);
end