function weapon_h = build_weapon(varargin)
%
%   weapon_h = BUILD_WEAPON('Parent', h_axes, 'Position', [x,y,z], 'Rotation', [rx,ry,rz])
%
%   输入参数 (可选):
%       'Parent'   - 父级 axes 或 hgtransform 句柄
%       'Position' - [x, y, z] 初始位置 (单位 m)
%       'Rotation' - [rx, ry, rz] 初始旋转 (单位 rad)
%       'Color'    - 模型颜色 [r, g, b]
%
%   返回:
%       weapon_h   - 矛杆整体的 hgtransform 句柄

    % 解析输入参数
    p = inputParser;
    addParameter(p, 'Parent', gca);
    addParameter(p, 'Position', [0, 0, 0]);
    addParameter(p, 'Rotation', [0, 0, 0]); % 欧拉角
    addParameter(p, 'Color', [0.8, 0.8, 0.8]); % 默认银灰色/白色
    parse(p, varargin{:});
                       
    parent_h = p.Results.Parent;
    init_pos = p.Results.Position;
    init_rot = p.Results.Rotation;
    weapon_color = p.Results.Color;

    % 创建整体变换对象
    weapon_h = hgtransform('Parent', parent_h);
    
    % ==========================================
    % 尺寸定义 (单位 mm)
    % ==========================================
    total_length = 1027;
    diameter = 20.3;
    radius = diameter / 2;
    
    % ==========================================
    % 创建圆柱体
    % ==========================================
    % cylinder 默认生成半径为1，高度为1的圆柱，轴向沿 Z 轴
    % 范围 Z: [0, 1]
    [X, Y, Z] = cylinder(radius, 20); % 20边形近似圆形
    
    % 拉伸长度
    Z = Z * total_length;
    
    % 中心化 (可选，这里将几何中心设为原点，方便旋转)
    % 如果希望原点在矛杆一端，可以注释掉下面这行
    Z = Z - total_length / 2;
    
    % 默认 cylinder 是竖着的 (Z轴)，矛杆通常是横着的或者需要特定朝向
    % 这里我们保持默认 Z 轴朝向，通过外部 hgtransform 控制姿态
    % 或者在这里旋转几何体使其默认沿 X 轴 (Robocon仿真中常以X为前方)
    
    % 旋转几何体数据使其沿 X 轴 (将 Z 轴数据换到 X 轴)
    X_new = Z;
    Y_new = Y;
    Z_new = X;
    
    % 绘制
    surface('XData', X_new, 'YData', Y_new, 'ZData', Z_new, ...
            'FaceColor', weapon_color, 'EdgeColor', 'none', ...
            'Parent', weapon_h, 'FaceAlpha', 1.0);
        
    % 添加端盖 (封闭圆柱两端)
    patch(Y(1,:), X(1,:), repmat(-total_length/2, size(X(1,:))), weapon_color, ...
          'Parent', weapon_h, 'EdgeColor', 'none'); % 左端盖 (X负)
    patch(Y(2,:), X(2,:), repmat(total_length/2, size(X(2,:))), weapon_color, ...
          'Parent', weapon_h, 'EdgeColor', 'none'); % 右端盖 (X正)

    % ==========================================
    % 后处理：单位转换与定位
    % ==========================================
    
    % 缩放：毫米 -> 米
    scale_matrix = makehgtform('scale', 0.001);
    
    % 初始姿态
    rx = init_rot(1); ry = init_rot(2); rz = init_rot(3);
    rot_matrix = makehgtform('xrotate', rx, 'yrotate', ry, 'zrotate', rz);
    
    % 初始位置
    trans_matrix = makehgtform('translate', init_pos);
    
    % 组合变换矩阵 (注意顺序：缩放 -> 旋转 -> 平移)
    set(weapon_h, 'Matrix', trans_matrix * rot_matrix * scale_matrix);


    
end