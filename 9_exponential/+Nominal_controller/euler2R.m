function R = euler2R(euler)

    r = euler(1);
    p = euler(2);
    y = euler(3);
    
    RotX = [1      0       0;
            0 cos(r) -sin(r);
            0 sin(r)  cos(r)];
    RotY = [ cos(p) 0 sin(p);
                  0 1      0;
            -sin(p) 0 cos(p)];
    RotZ = [cos(y) -sin(y) 0;
            sin(y)  cos(y) 0;
                 0       0 1];
             
    R = RotZ*RotY*RotX;
    
end