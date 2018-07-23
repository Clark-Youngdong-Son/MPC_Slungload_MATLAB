function rpyDot = pqr2RpyDot(pqr, rpy)

    phi = rpy(1);
    theta = rpy(2);
    
    conversion = [1 sin(phi)*tan(theta)  cos(phi)*tan(theta);
                  0 cos(phi)            -sin(phi);
                  0 sin(phi)*sec(theta)  cos(phi)*sec(theta)];
    rpyDot = conversion*pqr;

end