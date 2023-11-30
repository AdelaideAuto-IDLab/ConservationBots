
function clipped_r= limit_range(r)
    r(r>0.999)=0.999;
    r(r<0.001)=0.001;
    clipped_r= r;
end