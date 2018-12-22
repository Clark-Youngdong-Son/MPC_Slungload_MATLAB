function omega_estimation = omegaEstimation(x_trajectory, dynamics_param)

    length = size(x_trajectory,2);
    omega_estimation = zeros(3,length);
    for i=1:length
        x_L = x_trajectory(1:3,i);
        p_now = x_trajectory(7:9,i);
        x_L_dot = x_trajectory(10:12,i);
        omega_now = x_trajectory(16:18,i);
        x_Q = x_L - dynamics_param.LENGTH*p_now;
        x_Q_dot = x_L_dot - dynamics_param.LENGTH*cross(omega_now, p_now);
        
        p_predict = (x_L + x_L_dot*dynamics_param.dt) - (x_Q + x_Q_dot*dynamics_param.dt);
        p_predict = p_predict/norm(p_predict);
        
        p_dot = (p_predict - p_now)/dynamics_param.dt;
        
        A = [p_now(1) p_now(2) p_now(3); 0 p_now(3) -p_now(2); -p_now(3) 0 p_now(1)];
        b = [0 p_dot(1) p_dot(2)].';
        omega_estimation(:,i) = A\b;
    end

end
