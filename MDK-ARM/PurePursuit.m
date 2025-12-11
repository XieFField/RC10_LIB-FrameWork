classdef PurePursuit < handle
    % Pure Pursuit 路径跟踪控制器
    
    properties
        waypoints       % 路径点 [x1, y1; x2, y2; ...]
        lookahead_dist  % 前瞻距离
        target_speed    % 目标速度
        last_target_idx % 上一个目标点的索引
    end
    
    methods
        function obj = PurePursuit(waypoints, lookahead_dist, target_speed)
            obj.waypoints = waypoints;
            obj.lookahead_dist = lookahead_dist;
            obj.target_speed = target_speed;
            obj.last_target_idx = 1;
        end
        
        function [linear_vel, angular_vel, target_point] = calculate_commands(obj, robot_state)
            % robot_state: [x, y, yaw]
            
            % 寻找路径上最近的点
            [closest_idx, ~] = obj.find_closest_point(robot_state);
            
            % 在最近点之后寻找前瞻目标点
            target_idx = obj.find_lookahead_point(robot_state, closest_idx);
            
            if target_idx > size(obj.waypoints, 1)
                target_idx = size(obj.waypoints, 1);
            end
            
            target_point = obj.waypoints(target_idx, :);
            
            % 计算曲率和角速度
            alpha = atan2(target_point(2) - robot_state(2), target_point(1) - robot_state(1)) - robot_state(3);
            dist_to_target = sqrt((target_point(1) - robot_state(1))^2 + (target_point(2) - robot_state(2))^2);
            
            % Pure Pursuit 公式
            curvature = 2 * sin(alpha) / dist_to_target;
            angular_vel = obj.target_speed * curvature;
            
            linear_vel = obj.target_speed;
            
            % 如果接近最后一个点，则减速
            if target_idx == size(obj.waypoints, 1) && dist_to_target < obj.lookahead_dist * 1.5
                linear_vel = obj.target_speed * (dist_to_target / (obj.lookahead_dist * 1.5));
                if dist_to_target < 0.1 % 到达终点
                    linear_vel = 0;
                    angular_vel = 0;
                end
            end
        end
        
        function [closest_idx, min_dist] = find_closest_point(obj, robot_state)
            dists = sqrt(sum((obj.waypoints - [robot_state(1), robot_state(2)]).^2, 2));
            [min_dist, closest_idx] = min(dists);
        end
        
        function target_idx = find_lookahead_point(obj, robot_state, start_idx)
            target_idx = obj.last_target_idx;
            for i = start_idx:size(obj.waypoints, 1)
                dist = sqrt((obj.waypoints(i, 1) - robot_state(1))^2 + (obj.waypoints(i, 2) - robot_state(2))^2);
                if dist > obj.lookahead_dist
                    target_idx = i;
                    break;
                end
                target_idx = i; % 如果循环结束都没找到，就用最后一个点
            end
            obj.last_target_idx = target_idx;
        end
    end
end
