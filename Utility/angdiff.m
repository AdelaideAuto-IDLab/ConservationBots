function delta = angdiff(x, y)
if nargin == 1
    d = diff(x);
else
    d = y - x;
end

delta = wrapToPi(d);

end
