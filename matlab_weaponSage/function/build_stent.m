function stent_h = build_stent(varargin)
% BUILD_STENT 创建2026 Robocon矛杆架的3D模型 (修正: 下横梁与底座前沿齐平)
%
%   stent_h = BUILD_STENT('Parent', h_axes, 'Position', [x,y,z])
%
%   输入参数 (可选):
%       'Parent'   - 父级 axes 或 hgtransform 句柄
%       'Position' - [x, y, z] 初始位置 (单位 m)
%       'Color'    - 模型颜色 [r, g, b]
%
%   返回:  
%       stent_h    - 矛杆架整体的 hgtransform 句柄
%
%   注意: 内部建模单位为 mm，但在输出时会缩放为 m (默认Robocon仿真使用米制单位)
%         如果您的仿真环境是mm，请注释掉最后的缩放代码。

    % 解析输入参数
    p = inputParser;
    addParameter(p, 'Parent', gca);
    addParameter(p, 'Position', [0, 0, 0]);
    addParameter(p, 'Color', [0.55, 0.27, 0.07]); 
    parse(p, varargin{:});
    
    parent_h = p.Results.Parent;
    init_pos = p.Results.Position;
    wood_color = p.Results.Color;

    % 创建整体变换对象
    stent_h = hgtransform('Parent', parent_h);
    
    % ==========================================
    % 尺寸定义 (单位 mm)
    % ==========================================
    % 底座总深 260mm (前段100 + 后段160)
    base_depth = 260; 
    base_width = 40;
    base_height = 40;
    
    total_width = 800;
    total_height = 500;
    
    post_section = 40; 
    upper_beam_section = 40; 
    
    % 下横梁参数
    lower_beam_height = 40;
    lower_beam_width = 40; 
    
    % --- Y轴定位 (以底座中心为0) ---
    % Y范围: -130 ~ 130
    % 修正: 下横梁是"最外侧"，即位于最前端 -130
    base_front_y = -base_depth / 2; % -130
    base_back_y = base_depth / 2;   % 130
    
    % 下横梁: 前沿与底座前沿齐平
    lower_beam_front_y = base_front_y; % -130
    lower_beam_back_y = lower_beam_front_y + lower_beam_width; % -90
    
    % 立柱定位
    % 间隙 20mm (下横梁后沿 到 立柱前表面)
    post_front_y = lower_beam_back_y + 20; % -90 + 20 = -70
    post_center_y = post_front_y + post_section/2; % -50
    

    % 上横梁前表面: 比立柱退后 5mm
    upper_beam_front_y = post_front_y + 5; % -65
    upper_beam_center_y = upper_beam_front_y + upper_beam_section/2; % -45
    
    % 孔中心: 距离下横梁前表面 20mm
    hole_center_y = lower_beam_front_y + 20; % -110
    
    % ==========================================
    % 1. 底部滑轨 (底座)
    % ==========================================
    % 底座 Z 中心 = 20
    draw_box(stent_h, [base_width, base_depth, base_height], ...
        [-total_width/2 + base_width/2, 0, base_height/2], wood_color);
    draw_box(stent_h, [base_width, base_depth, base_height], ...
        [total_width/2 - base_width/2, 0, base_height/2], wood_color);

    % ==========================================
    % 2. 立柱
    % ==========================================
    post_h = total_height - base_height - upper_beam_section; 
    post_z = base_height + post_h/2;
    
    draw_box(stent_h, [post_section, post_section, post_h], ...
        [-total_width/2 + base_width/2, post_center_y, post_z], wood_color);
    draw_box(stent_h, [post_section, post_section, post_h], ...
        [total_width/2 - base_width/2, post_center_y, post_z], wood_color);

    % ==========================================
    % 3. 下横梁
    % ==========================================
    lower_beam_z_center = lower_beam_height/2; 
    
    hole_centers_x = [-300, -100, 100, 300];
    hole_diam = 25;
    
    beam_start_x = -total_width/2 + base_width;
    current_x = beam_start_x;
    
    draw_beam_center_y = lower_beam_front_y + lower_beam_width/2;
    
    for i = 1:length(hole_centers_x)
        hx = hole_centers_x(i);
        hole_left_edge = hx - hole_diam/2;
        solid_len = hole_left_edge - current_x;
        if solid_len > 0.01
            solid_center_x = current_x + solid_len/2;
            draw_box(stent_h, [solid_len, lower_beam_width, lower_beam_height], ...
                [solid_center_x, draw_beam_center_y, lower_beam_z_center], wood_color);
        end
        
        block_width = lower_beam_width; 
        draw_holed_block(stent_h, [hole_diam, block_width, lower_beam_height], ...
            [hx, hole_center_y, lower_beam_z_center], hole_diam/2, wood_color);
            
        current_x = hx + hole_diam/2;
    end
    
    beam_end_x = total_width/2 - base_width;
    solid_len = beam_end_x - current_x;
    if solid_len > 0.01
        solid_center_x = current_x + solid_len/2;
        draw_box(stent_h, [solid_len, lower_beam_width, lower_beam_height], ...
            [solid_center_x, draw_beam_center_y, lower_beam_z_center], wood_color);
    end

    % ==========================================
    % 4. 上横梁
    % ==========================================
    top_beam_z = total_height - upper_beam_section/2;
    slot_depth = 20;
    back_thickness = upper_beam_section - slot_depth; 
    
    back_y_center = upper_beam_center_y + slot_depth/2;
    draw_box(stent_h, [total_width, back_thickness, upper_beam_section], ...
        [0, back_y_center, top_beam_z], wood_color);
    
    slot_width = 25;
    slots_x = [-300, -100, 100, 300];
    front_thickness = slot_depth;
    front_y_center = upper_beam_center_y - back_thickness/2;
    
    current_x = -total_width/2;
    for i = 1:length(slots_x)
        slot_left = slots_x(i) - slot_width/2;
        slot_right = slots_x(i) + slot_width/2;
        b_len = slot_left - current_x;
        b_center_x = current_x + b_len/2;
        draw_box(stent_h, [b_len, front_thickness, upper_beam_section], ...
            [b_center_x, front_y_center, top_beam_z], wood_color);
        current_x = slot_right;
    end
    b_len = total_width/2 - current_x;
    b_center_x = current_x + b_len/2;
    draw_box(stent_h, [b_len, front_thickness, upper_beam_section], ...
        [b_center_x, front_y_center, top_beam_z], wood_color);

    % 后处理
    set(stent_h, 'Matrix', makehgtform('scale', 0.001));
    M = get(stent_h, 'Matrix');
    M(1:3, 4) = init_pos(:);
    set(stent_h, 'Matrix', M);
end

function draw_box(parent, size_vec, pos_vec, color)
    % 简易画立方体
    L = size_vec(1); W = size_vec(2); H = size_vec(3);
    % 标准立方体顶点
    v = [ -0.5 -0.5 -0.5; 0.5 -0.5 -0.5; 0.5 0.5 -0.5; -0.5 0.5 -0.5;
          -0.5 -0.5  0.5; 0.5 -0.5  0.5; 0.5 0.5  0.5; -0.5 0.5  0.5 ];
    % 缩放
    v(:,1) = v(:,1)*L; v(:,2) = v(:,2)*W; v(:,3) = v(:,3)*H;
    % 平移
    v = v + pos_vec;
    % 面
    f = [1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 1 2 3 4; 5 6 7 8];
    patch('Vertices', v, 'Faces', f, 'FaceColor', color, 'EdgeColor', 'none', ...
          'Parent', parent, 'FaceAlpha', 1.0);
end

function draw_holed_block(parent, size_vec, pos_vec, radius, color)
    % 绘制一个中间有垂直圆孔的方块
    L = size_vec(1); W = size_vec(2); H = size_vec(3);
    cx = pos_vec(1); cy = pos_vec(2); cz = pos_vec(3);
    
    % 离散化圆
    n_seg = 20;
    theta = linspace(0, 2*pi, n_seg+1)';
    theta(end) = []; % 去掉重复点
    
    % 圆孔边缘坐标 (局部坐标)
    circ_x = radius * cos(theta);
    circ_y = radius * sin(theta);
    
    % 方块四个角 (局部坐标)
    box_x = [-L/2; L/2; L/2; -L/2];
    box_y = [-W/2; -W/2; W/2; W/2];
    
    % 构建顶面 (Z = H/2)
    % 这是一个带孔的多边形，为了简单，我们用三角剖分
    % 顶点列表：4个角 + 圆上的点
    z_top = H/2;
    z_bot = -H/2;
    
    % 顶点集合
    % 1-4: 矩形角点
    % 5-(4+n_seg): 圆周点
    verts_2d = [box_x, box_y; circ_x, circ_y];
    
    % 约束边 (圆周)
    constraints = [(5:4+n_seg-1)', (6:4+n_seg)'; 4+n_seg, 5];
    
    % Delaunay 三角剖分 (带孔)
    dt = delaunayTriangulation(verts_2d, constraints);
    is_inside = isInterior(dt);
    faces_top = dt.ConnectivityList(is_inside, :);
    
    % 过滤掉圆内部的三角形 (圆心是 0,0)
    % 计算每个三角形的重心，如果重心距离 < radius，则剔除
    centers = (verts_2d(faces_top(:,1),:) + verts_2d(faces_top(:,2),:) + verts_2d(faces_top(:,3),:))/3;
    dists = sqrt(centers(:,1).^2 + centers(:,2).^2);
    faces_top = faces_top(dists > radius - 0.01, :);
    
    % 生成3D顶点
    n_verts = size(verts_2d, 1);
    V_top = [verts_2d, repmat(z_top, n_verts, 1)];
    V_bot = [verts_2d, repmat(z_bot, n_verts, 1)];
    
    % 绘制顶面和底面
    % 平移到世界坐标
    V_top_w = V_top + [cx, cy, cz];
    V_bot_w = V_bot + [cx, cy, cz];
    
    patch('Vertices', V_top_w, 'Faces', faces_top, 'FaceColor', color, 'EdgeColor', 'none', 'Parent', parent);
    patch('Vertices', V_bot_w, 'Faces', faces_top, 'FaceColor', color, 'EdgeColor', 'none', 'Parent', parent);
    
    % 绘制外侧面 (4个矩形)
    % 1-2, 2-3, 3-4, 4-1
    side_faces = [1 2; 2 3; 3 4; 4 1];
    for i = 1:4
        idx1 = side_faces(i,1); idx2 = side_faces(i,2);
        v_side = [V_bot_w(idx1,:); V_bot_w(idx2,:); V_top_w(idx2,:); V_top_w(idx1,:)];
        patch('Vertices', v_side, 'Faces', [1 2 3 4], 'FaceColor', color, 'EdgeColor', 'none', 'Parent', parent);
    end
    
    % 绘制内孔面 (圆柱面)
    % 连接 V_top 的圆周点和 V_bot 的圆周点
    % 圆周点索引是 5 到 4+n_seg
    start_idx = 5;
    end_idx = 4 + n_seg;
    
    for i = start_idx:end_idx
        next_i = i + 1;
        if next_i > end_idx, next_i = start_idx; end
        
        % 构成一个四边形面
        v_cyl = [V_bot_w(i,:); V_bot_w(next_i,:); V_top_w(next_i,:); V_top_w(i,:)];
        % 颜色稍微深一点以体现孔的深度感
        patch('Vertices', v_cyl, 'Faces', [1 2 3 4], 'FaceColor', color*0.8, 'EdgeColor', 'none', 'Parent', parent);
    end
end