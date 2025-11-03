function [xt, yt] = turret_mount_xy(xb, yb, yaw, base_size)
% 云台位于底盘“上沿”中点（底盘坐标 y=+base_size/2 方向）
xt = xb + (base_size/2)*cos(yaw+pi/2);
yt = yb + (base_size/2)*sin(yaw+pi/2);
end

