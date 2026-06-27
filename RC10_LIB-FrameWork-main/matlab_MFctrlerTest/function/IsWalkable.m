function tf = IsWalkable(map)
if map < 1 || map > 30
    tf = false; return;
end
[c, r] = Map_ToCR(map);
% 与 C++ 一样：中心区域 c=2..4 且 r=2..5 为障碍
tf = ~(c >= 2 && c <= 4 && r >= 2 && r <= 5);
end