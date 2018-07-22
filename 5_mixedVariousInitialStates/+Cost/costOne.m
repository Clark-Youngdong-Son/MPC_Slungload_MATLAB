function cost = costOne(x_n, u_n, param, i)

cost_Q = 0.5*(x_n-param.x_ref(:,i)).'*param.Q*(x_n-param.x_ref(:,i));
cost_R = 0.5*(u_n-param.u_ref).'*param.R*(u_n-param.u_ref);
if(param.obstacle_enable)
    for j=1:size(param.obstacle,2)
        if(strcmp(param.obstacle_consider,'xy'))
            cost_O = param.alpha*exp(1/(1+sqrt(sum((x_n(1:2)-param.obstacle(1:2,j)).^2)))); %consider XY
        elseif(strcmp(param.obstacle_consider,'xyz'))
            cost_O = param.alpha*exp(1/(1+sqrt(sum((x_n(1:3)-param.obstacle(1:3,j)).^2)))); %consider XYZ
        else
            disp('COST: wrong parameter (obstacle_consider)');
        end  
    end
else
    cost_O = 0;
end

cost = param.dt*(cost_Q + cost_R + cost_O);