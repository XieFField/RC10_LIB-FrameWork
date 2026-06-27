function move_patch_center(p, new_center3)
% 将任意 patch 的几何中心平移到 new_center3（不改变朝向/尺寸）
V = get(p,'Vertices');
c = mean(V,1);
d = new_center3(:)' - c;
V = V + d;
set(p,'Vertices',V);
end