function [x_nominal, u_nominal, time, cost_new, iteration] = SLQ(x_now, x_trajec, param)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Allocate static arrays
x_nominal = zeros(param.n, param.N+1);
x_line = zeros(param.n, param.N+1, param.maxLineSearch);

u_nominal = zeros(param.m, param.N);
u_line = zeros(param.m, param.N, param.maxLineSearch);

persistent K l;
if isempty(K)
	K = zeros(param.m,param.n,param.N);
end
if isempty(l)
    l = zeros(param.m,param.N);
end

persistent decay alpha;
if isempty(decay)
    decay = -(0:param.maxLineSearch-1)*(1/(param.maxLineSearch+1));
end
if isempty(alpha)
    alpha = 10.^decay;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic;
costDecreased = false;
%Main function
x_nominal(:,1) = x_now;
%This nominal controller must be changed for each dynamic system
% if((param.warmstart_original || param.warmstart_modified) && ~isempty(x_trajec)) %With warmstart
%     if(param.warmstart_original) %Use previous input
%         u_last = u_nominal(:,end);
%         u_nominal(:,1:end-1) = u_nominal(:,2:end);
%         u_nominal(:,end) = u_last;
%     elseif(param.warmstart_modified) %Follow previous trajectory
%         %Forward simulation with the previous input trajectory
%         for i=1:param.N
%             x_nominal(:,i+1) = param.dynamics(x_nominal(:,i), x_trajec(:,i), param.dynamics_param);
%         end
%         for i=1:param.N
%             u_nominal(:,i) = param.nominal_controller(x_nominal(:,i), [x_trajec(1:3,i+1); x_trajec(10:12,i+1); zeros(3,1)], param.nominal_controller_param);
%             x_nominal(:,i+1) = param.dynamics(x_nominal(:,i), u_nominal(:,i), param.dynamics_param);
%         end
%     end
% else %Without warmstart
%     for i=1:param.N
%         u_nominal(:,i) = param.nominal_controller(x_nominal(:,i), [param.x_f(1:3); zeros(6,1)], param.nominal_controller_param);
%         x_nominal(:,i+1) = param.dynamics(x_nominal(:,i), u_nominal(:,i), param.dynamics_param);
%     end
% end

% u_nominal = randn(size(u_nominal));
for i=1:param.maxIter %SLQ iteration
    cost_old = param.cost(x_nominal, u_nominal, param.cost_param);
    s1 = param.cost_param.L*(x_nominal(:,end)-param.x_f);
    S2 = param.cost_param.L;
    %Backward solve
    for j=param.N:-1:1
        %Linearize dynamics and cost function
        [A, B] = param.dynamicsGradient(x_nominal(:,j), u_nominal(:,j), param.dynamics, param.dynamics_param);
        [q1, r1, Q2, R2] = param.costGradient(x_nominal(:,j), u_nominal(:,j), param.costOne, param.cost_param);
        
        H = R2 + B.'*S2*B;
        H = (H+H.')/2;
        H_inv = inv(H);
        H_inv = (H_inv+H_inv.')/2;
        G = B.'*S2*A;
        g = r1 + B.'*s1;
        
        s1 = q1 + A.'*s1 - G.'*H_inv*g;
        S2 = Q2 + A.'*S2*A - G.'*H_inv*G;
        
        K(:,:,j) = -H_inv*G;
        l(:,j) = -H_inv*g;
    end
    %Line search
    costs = zeros(param.maxLineSearch, 1);
    for j=1:param.maxLineSearch
        x_line(:,1,j) = x_nominal(:,1);
        for k=1:param.N
            u_line(:,k,j) = u_nominal(:,k) + alpha(j)*l(:,k) + K(:,:,k)*(x_line(:,k,j)-x_nominal(:,k));
            x_line(:,k+1,j) = param.dynamics(x_line(:,k,j), u_line(:,k,j), param.dynamics_param);
        end
        costs(j) = param.cost(x_line(:,:,j), u_line(:,:,j), param.cost_param);
    end
    [cost_temp, minIndex] = min(costs);
    
    %Convergence check
    if(cost_temp<cost_old)
        if(~costDecreased)
            costDecreased = true;
        end
        cost_new = cost_temp;
        fprintf('Cost old : %5.2f    Cost_new : %5.2f\n', cost_old, cost_new);
        x_nominal = x_line(:,:,minIndex);
        u_nominal = u_line(:,:,minIndex);
        delta = cost_old - cost_temp;
        if(delta<param.convergenceTol)
            break;
        end
    else
        if(costDecreased)
            disp('Cost does not decrease');
            break;
        else
            fprintf('Cost does not decrease at all. cost_old:%6.2f cost_new:%6.2f\n', cost_old, cost_temp);
            if(i==param.maxIter)
                cost_new = NaN;
            end
        end
    end
end
iteration = i;
time = toc;
end