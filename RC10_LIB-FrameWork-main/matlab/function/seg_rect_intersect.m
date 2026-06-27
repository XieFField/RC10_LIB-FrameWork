function tf = seg_rect_intersect(p1, p2, rect_center, rect_size)
% 轴对齐矩形与线段是否相交（含端点在内）
% p1,p2: [x,y]; rect_center: [cx,cy]; rect_size: [w,d]
cx = rect_center(1); cy = rect_center(2);
w = rect_size(1); d = rect_size(2);
hx = w/2; hy = d/2;

% 平移到矩形坐标
p1 = [p1(1)-cx, p1(2)-cy]; 
p2 = [p2(1)-cx, p2(2)-cy];

% 端点在内
if point_in_aabb(p1, hx, hy) || point_in_aabb(p2, hx, hy)
    tf = true; return;
end

% 与四条边相交
edges = [ -hx, -hy,  hx, -hy;   % 下边
          -hx,  hy,  hx,  hy;   % 上边
          -hx, -hy, -hx,  hy;   % 左边
           hx, -hy,  hx,  hy ]; % 右边
for i=1:4
    a = edges(i,1:2); b = edges(i,3:4);
    if seg_seg_intersect(p1, p2, a, b)
        tf = true; return;
    end
end
tf = false;
end

% ===== 本文件子函数 =====
function tf = point_in_aabb(p, hx, hy)
tf = (abs(p(1))<=hx) && (abs(p(2))<=hy);
end

function tf = seg_seg_intersect(p1,p2,q1,q2)
o1 = orient(p1,p2,q1); o2 = orient(p1,p2,q2);
o3 = orient(q1,q2,p1); o4 = orient(q1,q2,p2);
tf = false;
if o1*o2<0 && o3*o4<0, tf = true; return; end
if o1==0 && on_seg(p1,p2,q1), tf=true; return; end
if o2==0 && on_seg(p1,p2,q2), tf=true; return; end
if o3==0 && on_seg(q1,q2,p1), tf=true; return; end
if o4==0 && on_seg(q1,q2,p2), tf=true; return; end
end

function v = orient(a,b,c)
v = (b(1)-a(1))*(c(2)-a(2)) - (b(2)-a(2))*(c(1)-a(1));
v = sign(v);
end

function tf = on_seg(a,b,c)
eps = 1e-9;
tf = (min(a(1),b(1))-eps<=c(1) && c(1)<=max(a(1),b(1))+eps && ...
      min(a(2),b(2))-eps<=c(2) && c(2)<=max(a(2),b(2))+eps);
end