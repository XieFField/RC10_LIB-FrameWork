function h = draw_box(center, sizeXYZ, colorRGB, alpha)
% center=[x,y,z] 为几何中心；sizeXYZ=[w,d,h]
x = center(1); y = center(2); z = center(3);
w = sizeXYZ(1); d = sizeXYZ(2); hgt = sizeXYZ(3);
hx = w/2; hy = d/2; hz = hgt/2;

% 8 顶点
V = [x+[-hx -hx  hx  hx  -hx -hx  hx  hx]', ...
     y+[-hy  hy  hy -hy  -hy  hy  hy -hy]', ...
     z+[-hz -hz -hz -hz   hz  hz  hz  hz]'];

F = [1 2 3 4;  % 底
     5 6 7 8;  % 顶
     1 2 6 5;  % 侧
     2 3 7 6;
     3 4 8 7;
     4 1 5 8];

h = patch('Faces',F,'Vertices',V,'FaceColor',colorRGB,'FaceAlpha',alpha,'EdgeColor',[0.2 0.2 0.2]);
end