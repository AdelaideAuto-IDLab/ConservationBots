function d = angdiffd(u1, u2)
    u1_r = deg2rad(u1);
    u2_r = deg2rad(u2);
    
    d_r = angdiff(u1_r, u2_r);
    
    d = rad2deg(d_r);
end