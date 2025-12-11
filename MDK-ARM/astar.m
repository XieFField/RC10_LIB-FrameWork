function path = astar(map, start, goal)
% A* 路径规划算法
% map: 二维矩阵, 0 表示自由空间, 1 表示障碍物
% start: [x, y] 起点坐标
% goal: [x, y] 终点坐标
% path: 返回的路径点 [x1, y1; x2, y2; ...]

    % 定义8个移动方向
    motions = [-1, -1, 1.4; -1, 0, 1; -1, 1, 1.4; 0, -1, 1; 0, 1, 1; 1, -1, 1.4; 1, 0, 1; 1, 1, 1.4];
    
    [map_h, map_w] = size(map);
    
    % 开放列表和关闭列表
    % 节点格式: [g, h, f, x, y, parent_x, parent_y]
    g = 0;
    h = heuristic(start, goal);
    f = g + h;
    open_list = [g, h, f, start(1), start(2), start(1), start(2)];
    closed_list = [];
    
    path_found = false;
    
    while ~isempty(open_list)
        % 在开放列表中找到 f 值最小的节点
        [~, current_idx] = min(open_list(:, 3));
        current_node = open_list(current_idx, :);
        
        % 将当前节点从开放列表移到关闭列表
        closed_list = [closed_list; current_node];
        open_list(current_idx, :) = [];
        
        % 如果到达目标点
        if isequal(current_node(4:5), goal)
            path_found = true;
            break;
        end
        
        % 遍历所有可能的移动方向
        for i = 1:size(motions, 1)
            motion = motions(i, :);
            neighbor_pos = current_node(4:5) + motion(1:2);
            
            % 检查邻居节点是否有效
            if neighbor_pos(1) < 1 || neighbor_pos(1) > map_w || ...
               neighbor_pos(2) < 1 || neighbor_pos(2) > map_h || ...
               map(neighbor_pos(2), neighbor_pos(1)) == 1
                continue;
            end
            
            % 检查邻居节点是否已在关闭列表中
            if is_in_list(closed_list, neighbor_pos)
                continue;
            end
            
            % 计算邻居节点的 g, h, f 值
            g = current_node(1) + motion(3);
            h = heuristic(neighbor_pos, goal);
            f = g + h;
            
            % 如果邻居节点已在开放列表中，并且新的路径更优
            [in_open, idx] = is_in_list(open_list, neighbor_pos);
            if in_open && open_list(idx, 3) < f
                continue;
            elseif in_open
                open_list(idx, :) = []; % 从开放列表中移除旧节点
            end
            
            % 添加新节点到开放列表
            new_node = [g, h, f, neighbor_pos(1), neighbor_pos(2), current_node(4), current_node(5)];
            open_list = [open_list; new_node];
        end
    end
    
    % 回溯路径
    path = [];
    if path_found
        curr = closed_list(end, 4:5);
        while ~isequal(curr, start)
            path = [curr; path];
            parent_pos = closed_list(find(all(closed_list(:, 4:5) == curr, 2)), 6:7);
            curr = parent_pos;
        end
        path = [start; path];
    end
end

function h = heuristic(pos1, pos2)
    % 启发函数 (欧几里得距离)
    h = sqrt((pos1(1) - pos2(1))^2 + (pos1(2) - pos2(2))^2);
end

function [found, idx] = is_in_list(list, pos)
    % 检查节点是否在列表中
    found = false;
    idx = -1;
    if isempty(list)
        return;
    end
    
    for i = 1:size(list, 1)
        if isequal(list(i, 4:5), pos)
            found = true;
            idx = i;
            return;
        end
    end
end
