function cubes = place_cubes(pillars, allowed_ids, chosen_ids, cube_size)
arguments
    pillars
    allowed_ids (1,:) double
    chosen_ids (1,3) double
    cube_size (1,1) double {mustBePositive}
end

% 校验选择
if numel(chosen_ids)~=3 || any(~ismember(chosen_ids, allowed_ids))
    error('立方体编号必须从 {%s} 中选择 3 个', num2str(allowed_ids));
end

cubes = struct('id',{},'x',{},'y',{},'w',{},'d',{},'h',{},'zbase',{},'patch',{});
for t = 1:3
    id = chosen_ids(t);
    P = pillars(id);
    % 立方体放在桩顶中心
    zbase = P.h; 
    c = draw_box([P.x, P.y, zbase + cube_size/2], [cube_size, cube_size, cube_size], [1.0 0.9 0.2], 1.0);
    cubes(t).id = id;
    cubes(t).x = P.x; cubes(t).y = P.y;
    cubes(t).w = cube_size; cubes(t).d = cube_size; cubes(t).h = cube_size;
    cubes(t).zbase = zbase;
    cubes(t).patch = c;
end
end