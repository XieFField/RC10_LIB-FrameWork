function DrawGridPath(pathCells)
if isempty(pathCells), return; end
C = mf_consts();
XY = zeros(numel(pathCells),2);
for i = 1:numel(pathCells)
    p = C.MapNum_RealPos(pathCells(i),:);
    XY(i,:) = p(1:2);
end
plot(XY(:,1), XY(:,2), 'r-', 'LineWidth', 2);
end