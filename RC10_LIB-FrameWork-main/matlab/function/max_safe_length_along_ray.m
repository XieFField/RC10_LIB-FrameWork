
function Lsafe = max_safe_length_along_ray(x0, y0, phi, h_arm, pillars, margin, L_target)
% 在 [0, L_target] 内二分搜索最大不与梅花桩相交的长度
lo = 0.0; hi = max(L_target, 0.0);
for it = 1:18
    mid = 0.5*(lo+hi);
    x1 = x0; y1 = y0;
    x2 = x0 + mid*cos(phi);
    y2 = y0 + mid*sin(phi);
    if arm_hits_pillars(x1,y1,x2,y2,h_arm,pillars,margin)
        hi = mid;  % 有碰撞 → 收短
    else
        lo = mid;  % 无碰撞 → 放长
    end
end
Lsafe = lo;
end