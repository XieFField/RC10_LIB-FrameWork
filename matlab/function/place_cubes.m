function cubes = place_cubes(pillars, allowed_ids, chosen_ids, cube_size, nums)
% place_cubes: 在指定桩顶中心放置立方体
% - nums 可选；若未提供，则使用 numel(chosen_ids)
% - 若 chosen_ids 多于 nums，则只取前 nums 个；若少于 nums，报错

% 输入数量与默认处理
if nargin < 5 || isempty(nums)
    nums = numel(chosen_ids);
end

% 基本校验
validateattributes(allowed_ids, {'double'}, {'row'});
validateattributes(chosen_ids, {'double'}, {'row'});
validateattributes(cube_size, {'double'}, {'scalar','positive'});
validateattributes(nums, {'double'}, {'scalar','positive','integer'});

% 过滤出将要放置的 id 列表
if numel(chosen_ids) < nums
    error('chosen_ids 个数(%d)少于 nums(%d)。', numel(chosen_ids), nums);
end
ids = chosen_ids(1:nums);

% 校验是否在允许集合中
if any(~ismember(ids, allowed_ids))
    error('立方体编号必须从 {%s} 中选择。', num2str(allowed_ids));
end

% 生成 cubes 结构
cubes = struct('id',{},'x',{},'y',{},'w',{},'d',{},'h',{},'zbase',{},'patch',{});
for k = 1:nums
    id = ids(k);
    P = pillars(id);
    zbase = P.h; % 桩顶
    c = draw_box([P.x, P.y, zbase + cube_size/2], ...
                 [cube_size, cube_size, cube_size], ...
                 [1.0 0.9 0.2], 1.0);
    cubes(k).id = id;
    cubes(k).x = P.x;   cubes(k).y = P.y;
    cubes(k).w = cube_size; cubes(k).d = cube_size; cubes(k).h = cube_size;
    cubes(k).zbase = zbase;
    cubes(k).patch = c;
end
end