function input = controller_geometric(x_now, x_ref, param)

    position_now = x_now(1:3);
    velocity_now = x_now(10:12);
    attitude_now = x_now(4:6);
    R_now = Nominal_controller.euler2R(attitude_now);
    pqr_now = x_now(13:15);
    p_now = x_now(7:9);
    omega_now = x_now(16:18);
    p_dot_now = cross(omega_now, p_now);
    
    position_des = x_ref(1:3);
    velocity_des = x_ref(4:6);
    acceleration_des = x_ref(7:9);
   
    %Load position controller
    e_x = position_now - position_des;
    e_v = velocity_now - velocity_des;
    
    A = -param.k_x*e_x - param.k_v*e_v + (param.MASS_Q + param.MASS_L)*(acceleration_des + param.G*[0 0 1].') + ...
         param.MASS_Q*param.LENGTH*dot(p_dot_now, p_dot_now)*p_now;
	p_des = - A/norm(A);
    F_n = dot(A,p_des)*p_des;
    
    %Load attitude controller
    e_p = Nominal_controller.skew(p_now)^2*p_des;
    e_p_dot = p_dot_now;
%     F_n = -dot(p_des,p_now)*p_now;
    F_pd = -param.k_p*e_p - param.k_w*e_p_dot;
    F = F_n - F_pd;
    
    b3_c = F/norm(F);
    b1_d = [1 0 0].';
    b1_c = -(cross(b3_c,cross(b3_c,b1_d)))/norm(cross(b3_c,b1_d));
    
    R_des = [b1_c cross(b3_c, b1_c) b3_c];
    pqr_des = zeros(3,1);
    thrust = dot(F,R_now*[0 0 1].');
    
    %Attitude controller
    e_R = 0.5*Nominal_controller.skew_inverse(R_des.'*R_now - R_now.'*R_des);
    e_pqr = pqr_now - R_now.'*R_des*pqr_des;
    moments = -param.k_R/(param.epsilon^2)*e_R - param.k_pqr/(param.epsilon)*e_pqr + cross(pqr_now, param.I*pqr_now);  
    
    input = [thrust; moments];
end