function [A, B] = dynamicsGradient(x_now, u_now, dynamics, dynamics_param)

n = length(x_now);
m = length(u_now);

A = zeros(n,n);
B = zeros(n,m);

for i=1:n
    dX = zeros(n,1);
    dX(i) = dynamics_param.dx;
    A(:,i) = (dynamics(x_now+dX, u_now, dynamics_param)-dynamics(x_now, u_now, dynamics_param))/dynamics_param.dx;
end

for i=1:m
    dU = zeros(m,1);
    dU(i) = dynamics_param.du;
    B(:,i) = (dynamics(x_now, u_now+dU, dynamics_param)-dynamics(x_now, u_now, dynamics_param))/dynamics_param.dx;
end
end