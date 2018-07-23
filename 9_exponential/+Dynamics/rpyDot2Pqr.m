function pqr = rpyDot2Pqr(rpyDot, rpy)

    phi = rpy(1);
    theta = rpy(2);
    
    conversion = [1         0 -sin(theta);
                  0  cos(phi)  sin(phi)*cos(theta);
                  0 -sin(phi)  cos(phi)*cos(theta)];
    pqr = conversion*rpyDot;

end