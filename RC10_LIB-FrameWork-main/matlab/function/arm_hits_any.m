% filepath: f:\MyProjectFlies\STM32H7\Frame_T\matlab\function\arm_hits_any.m
function hit = arm_hits_any(x1, y1, x2, y2, h_arm, pillars, margin)
% 机械臂线段在高度 h_arm 下，是否与任一梅花桩相交（含安全裕度）
hit = false;
for k = 1:numel(pillars)
    ph = pillars(k).h;
    if h_arm >= 0 && h_arm <= ph + 1e-6
        if seg_rect_intersect([x1, y1], [x2, y2], ...
                [pillars(k).x, pillars(k).y], ...
                [pillars(k).w + 2*margin, pillars(k).d + 2*margin])
            hit = true;
            return;
        end
    end
end
end