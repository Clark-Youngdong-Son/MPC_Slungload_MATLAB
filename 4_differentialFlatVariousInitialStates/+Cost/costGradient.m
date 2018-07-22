function [q1, r1, Q2, R2] = costGradient(x_now, u_now, costOne, cost_param)

n = length(x_now);
m = length(u_now);

Q2 = zeros(n,n);
R2 = zeros(m,m);

q1 = xGradient(n, x_now, u_now, costOne, cost_param);
r1 = uGradient(m, x_now, u_now, costOne, cost_param);
for i=1:n
    dX = zeros(n,1);
    dX(i) = cost_param.dx;
    Q2(i,:) = ((xGradient(n, x_now+dX, u_now, costOne, cost_param)-q1)/cost_param.dx).';
end
for i=1:m
    dU = zeros(m,1);
    dU(i) = cost_param.du;
    R2(i,:) = ((uGradient(m, x_now, u_now+dU, costOne, cost_param)-r1)/cost_param.dx).';
end

end

function q1 = xGradient(n, x_now, u_now, costOne, cost_param)

q1 = zeros(n,1);
for i=1:n
    dX = zeros(n,1);
    dX(i) = cost_param.dx;
    q1(i) = (costOne(x_now+dX, u_now, cost_param)-costOne(x_now, u_now, cost_param))/cost_param.dx;
end

end

function r1 = uGradient(m, x_now, u_now, costOne, cost_param)

r1 = zeros(m,1);
for i=1:m
    dU = zeros(m,1);
    dU(i) = cost_param.du;
    r1(i) = (costOne(x_now, u_now+dU, cost_param)-costOne(x_now, u_now, cost_param))/cost_param.du;
end

end