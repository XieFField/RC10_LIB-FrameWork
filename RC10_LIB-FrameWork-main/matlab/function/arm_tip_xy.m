function [x_tip, y_tip] = arm_tip_xy(xt, yt, yaw, theta, L)
% 计算臂端俯视坐标：底盘朝向 yaw，云台相对角 theta，臂长 L
phi = yaw + theta;
x_tip = xt + L*cos(phi);
y_tip = yt + L*sin(phi);
end