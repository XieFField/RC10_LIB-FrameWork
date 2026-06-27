function hit = base_hits_any(xb, yb, yaw, base_size, pillars)
% 本例底盘 yaw=0，若需通用 OBB 碰撞可扩展
hit = false;
hb = base_size/2;
for k = 1:numel(pillars)
    px = pillars(k).x; py = pillars(k).y;
    hw = pillars(k).w/2; hd = pillars(k).d/2;
    if abs(xb - px) <= (hb + hw) && abs(yb - py) <= (hb + hd)
        hit = true; return;
    end
end
end





