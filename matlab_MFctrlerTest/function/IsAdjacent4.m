function tf = IsAdjacent4(a, b)
if a < 1 || a > 30 || b < 1 || b > 30
    tf = false; return;
end
[c1, r1] = Map_ToCR(a);
[c2, r2] = Map_ToCR(b);
dc = abs(c1 - c2);
dr = abs(r1 - r2);
tf = (dc + dr) == 1;

end