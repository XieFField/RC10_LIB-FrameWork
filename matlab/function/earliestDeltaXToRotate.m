function dx = earliestDeltaXToRotate(D_side, L, margin)
% 并排行进越过前角后的最小前进量
R = L + margin;
t = R*R - D_side*D_side;
dx = (t<=0) * 0.0 + (t>0) * sqrt(t);
end