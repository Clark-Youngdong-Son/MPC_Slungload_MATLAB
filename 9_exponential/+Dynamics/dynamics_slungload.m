function x_new = dynamics_slungload(x_now, u_now, param)

    %State parsing
    position_now = x_now(1:3);
    rpy_now = x_now(4:6);
    p_now = x_now(7:9);
    velocity_now = x_now(10:12);
    pqr_now = x_now(13:15);
    omega_now = x_now(16:18);

    %Input parsing
    thrust = u_now(1);
    moments = u_now(2:4);
    
    %Compute constants
    R = Dynamics.euler2R(rpy_now);
    rpy_dot_now = Dynamics.pqr2RpyDot(pqr_now, rpy_now);
    p_dot_now = cross(omega_now, p_now);
    e3 = [0 0 1].';

    %Compute accelerations & Frame conversion
    acceleration_now = (thrust*dot(p_now,R*e3)-param.MASS_Q*param.LENGTH*dot(p_dot_now, p_dot_now))*p_now/(param.MASS_Q + param.MASS_L) - param.G*e3;
    
    pqr_dot_now = param.I\(moments-cross(pqr_now,param.I*pqr_now));
    rpy_ddot_now = Dynamics.pqr2RpyDot(pqr_dot_now, rpy_now);
    
    omega_dot_now = -cross(p_now, thrust*R*e3)/(param.MASS_Q*param.LENGTH);
    p_ddot_now = cross(omega_dot_now, p_now) + cross(omega_now, cross(omega_now, p_now));
    
    %Integrate (2nd order)
%     position_new = position_now + velocity_now*param.dt + 0.5*acceleration_now*param.dt*param.dt;
%     velocity_new = velocity_now + acceleration_now*param.dt;
% 
%     attitude_new = rpy_now + rpy_dot_now*param.dt + 0.5*rpy_ddot_now*param.dt*param.dt;
%     pqr_new = pqr_now + pqr_dot_now*param.dt;
% 
%     p_new = p_now + p_dot_now*param.dt + 0.5*p_ddot_now*param.dt*param.dt;
%     p_new = p_new/norm(p_new);
%     omega_new = omega_now + omega_dot_now*param.dt;
    position_new = position_now + velocity_now*param.dt;
    velocity_new = velocity_now + acceleration_now*param.dt;

    attitude_new = rpy_now + rpy_dot_now*param.dt;
    pqr_new = pqr_now + pqr_dot_now*param.dt;

    p_new = p_now + p_dot_now*param.dt;
    p_new = p_new/norm(p_new);
    omega_new = omega_now + omega_dot_now*param.dt;
%     omega_new(3) = 0;
    %Output state generation
    x_new = [position_new; attitude_new;     p_new;
             velocity_new;     pqr_new; omega_new];
     
end