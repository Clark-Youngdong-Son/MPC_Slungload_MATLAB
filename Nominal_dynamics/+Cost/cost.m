function cost = cost(x_n, u_n, param)

cost_L = 0.5*(x_n(:,end)-param.x_f).'*param.L*(x_n(:,end)-param.x_f);

cost_Q = 0;
cost_R = 0;
cost_O = 0;
for i= 1:param.N-1
    cost_Q = cost_Q + 0.5*(x_n(:,i)-param.x_f).'*param.Q*(x_n(:,i)-param.x_f);
    cost_R = cost_R + 0.5*(u_n(:,i)-param.u_ref).'*param.R*(u_n(:,i)-param.u_ref);
    if(param.obstacle_enable)
        for j=1:param.obstacle_number
            if(strcmp(param.obstacle_consider,'xy'))
                cost_O = cost_O + param.alpha*exp(1/(1+sqrt(sum((x_n(1:2,i)-param.obstacle(1:2,j)).^2)))); %consider XY
            elseif(strcmp(param.obstacle_consider,'xyz'))
                cost_O = cost_O + param.alpha*exp(1/(1+sqrt(sum((x_n(1:3,i)-param.obstacle(1:3,j)).^2)))); %consider XYZ
            else
                disp('COST: wrong parameter (obstacle_consider)');
            end  
        end
    end
end

cost = cost_L + cost_Q + cost_R + cost_O;