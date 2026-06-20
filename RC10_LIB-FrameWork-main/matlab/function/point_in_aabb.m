function tf = point_in_aabb(p, hx, hy)
tf = (abs(p(1))<=hx) && (abs(p(2))<=hy);
end

function tf = seg_seg_intersect(p1,p2,q1,q2)
% 线段相交（含端点共线重叠）
o1 = orient(p1,p2,q1); o2 = orient(p1,p2,q2);
o3 = orient(q1,q2,p1); o4 = orient(q1,q2,p2);
tf = false;
if o1*o2<0 && o3*o4<0
    tf = true; return;
end
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
tf = (min(a(1),b(1))-1e-9<=c(1) && c(1)<=max(a(1),b(1))+1e-9 && ...
      min(a(2),b(2))-1e-9<=c(2) && c(2)<=max(a(2),b(2))+1e-9);
end