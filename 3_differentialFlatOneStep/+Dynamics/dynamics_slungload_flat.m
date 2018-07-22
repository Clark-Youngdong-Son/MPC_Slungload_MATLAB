function x_new = dynamics_slungload_flat(x_now, u_now, param)

    x_new = x_now + [x_now(4:end); zeros(3,1)]*param.dt + [zeros(15,1); u_now];
%     x_new = x_now + [x_now(4:end); zeros(3,1)]*param.dt + 0.5*[x_now(7:end); zeros(6,1)]*param.dt^2 + [zeros(15,1); u_now];
     
end