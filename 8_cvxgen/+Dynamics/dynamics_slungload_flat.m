function x_new = dynamics_slungload_flat(x_now, u_now, param)

    x_new = x_now + [x_now(4:end); zeros(3,1)]*param.dt + [zeros(15,1); u_now*param.dt];
     
end