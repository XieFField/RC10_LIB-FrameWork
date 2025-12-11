%% 主仿真脚本: 模拟 chassis_auto_control
clear; clc; close all;

%% 1. 环境和机器人参数设置
% 地图尺寸和分辨率 (对应 C++ 中的 CELL_SIZE_M)
CELL_SIZE = 1.2; % meters
MAP_GRID_WIDTH = 5;
MAP_GRID_HEIGHT = 6;

% 创建地图 (0: free, 1: obstacle)
map_grid = zeros(MAP_GRID_HEIGHT, MAP_GRID_WIDTH);
% C++: for (int y = 1; y <= 4; ++y) { for (int x = 1; x <= 3; ++x) }
% MATLAB 索引从 1 开始, C++ 从 0. C++ 的 x=1..3 -> MATLAB 的 x=2..4
% C++ 的 y=1..4 -> MATLAB 的 y=2..5
map_grid(2:5, 2:4) = 1;

% 起点和终点 (栅格坐标)
start_grid = [1, 1]; % C++ (0,0)
goal_grid = [5, 6];  % C++ (4,5)

% 机器人初始状态 [x, y, yaw] in meters and radians
% 强制从格号1的中心开始
start_world = (start_grid - 0.5) * CELL_SIZE;
robot_state = [start_world(1), start_world(2), 0]; % 初始朝向

% 仿真参数
dt = 0.1; % 时间步长 (s)
sim_time = 50; % 总仿真时间 (s)

% 控制器参数
TARGET_SPEED = 0.5; % m/s
LOOKAHEAD_DIST = 0.8; % m

%% 2. A* 路径规划
fprintf('Running A* planner...\n');
path_grid = astar(map_grid, start_grid, goal_grid);

if isempty(path_grid)
    error('A* did not find a path!');
end

% 将栅格路径转换为世界坐标路径 (取格子中心)
waypoints_world = (path_grid - 0.5) * CELL_SIZE;

%% 3. 初始化 Pure Pursuit 控制器
controller = PurePursuit(waypoints_world, LOOKAHEAD_DIST, TARGET_SPEED);

%% 4. 仿真主循环
fprintf('Starting simulation...\n');
robot_trajectory = []; % 存储机器人轨迹
h_fig = figure;
hold on; grid on; axis equal;
xlim([0, MAP_GRID_WIDTH * CELL_SIZE]);
ylim([0, MAP_GRID_HEIGHT * CELL_SIZE]);
title('Chassis Auto Control Simulation');
xlabel('X (m)');
ylabel('Y (m)');

% 绘制地图
for y = 1:MAP_GRID_HEIGHT
    for x = 1:MAP_GRID_WIDTH
        if map_grid(y, x) == 1
            rectangle('Position', [(x-1)*CELL_SIZE, (y-1)*CELL_SIZE, CELL_SIZE, CELL_SIZE], 'FaceColor', [0.5 0.5 0.5]);
        end
    end
end

% 绘制路径
plot(waypoints_world(:,1), waypoints_world(:,2), 'g-o', 'LineWidth', 2, 'DisplayName', 'A* Path');

% 仿真循环
for t = 0:dt:sim_time
    % 获取控制指令
    [linear_vel, angular_vel, target_point] = controller.calculate_commands(robot_state);
    
    % C++ 代码中的速度转换 (从全局速度到车体速度)
    % Pure pursuit 直接输出的是车体坐标系的角速度和期望的线速度,
    % 但 C++ 代码中有一个从全局速度到车体速度的转换。
    % 这里的 Pure Pursuit 实现更直接，已经输出了车体角速度。
    % C++ 代码中的速度分解部分在我们的仿真中可以简化。
    vx_body = linear_vel; % 纯追踪的速度就是车体前进速度
    vy_body = 0; % 纯追踪模型中没有侧向速度
    
    % 更新机器人状态 (简单的运动学模型)
    robot_state(1) = robot_state(1) + (vx_body * cos(robot_state(3)) - vy_body * sin(robot_state(3))) * dt;
    robot_state(2) = robot_state(2) + (vx_body * sin(robot_state(3)) + vy_body * cos(robot_state(3))) * dt;
    robot_state(3) = robot_state(3) + angular_vel * dt;
    robot_state(3) = atan2(sin(robot_state(3)), cos(robot_state(3))); % 角度归一化到 [-pi, pi]
    
    % 记录轨迹
    robot_trajectory = [robot_trajectory; robot_state];
    
    % 动态绘图
    if mod(t, 0.5) == 0 % 每0.5秒绘制一次
        if exist('h_robot', 'var') && isvalid(h_robot)
            delete(h_robot);
        end
        if exist('h_lookahead', 'var') && isvalid(h_lookahead)
            delete(h_lookahead);
        end
        h_robot = plot(robot_state(1), robot_state(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        h_lookahead = plot(target_point(1), target_point(2), 'm*', 'MarkerSize', 10);
        plot(robot_trajectory(:,1), robot_trajectory(:,2), 'b-', 'DisplayName', 'Robot Trajectory');
        drawnow;
    end
    
    % 检查是否到达终点
    dist_to_goal = sqrt((robot_state(1) - waypoints_world(end,1))^2 + (robot_state(2) - waypoints_world(end,2))^2);
    if dist_to_goal < 0.15
        fprintf('Goal reached!\n');
        break;
    end
end

%% 5. 最终结果显示
plot(robot_trajectory(:,1), robot_trajectory(:,2), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Robot Trajectory');
plot(robot_state(1), robot_state(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'Final Position');
legend('show');
hold off;

fprintf('Simulation finished.\n');
