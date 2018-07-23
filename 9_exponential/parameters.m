%Simulation
dt = 0.01;
N = 500;
t = 0:dt:dt*N;

n = 3*6;
m = 4;
x_final = zeros(n,1);
x_waypoint = zeros(n,1);
k_waypoint = N/2;

%System
initialNumber = 1;
x_flat_trajec_optimal = zeros(n,N+1,initialNumber);
x_trajec_optimal = zeros(n,N+1,initialNumber);
u_flat_trajec_optimal = zeros(3, N, initialNumber);
u_trajec_optimal = zeros(4, N, initialNumber);
cost_new = zeros(initialNumber,1);
iteration = zeros(initialNumber,1);
convergenceTime = zeros(initialNumber,1);
obstacleDistance = zeros(N+1, initialNumber);
x_initial = zeros(n,initialNumber);
for i=1:initialNumber
%     x_initial(4:6,i) = randn(3,1)*1;
%     x_initial(7:9,i) = randn(3,1)*1; %attitude & cable together
%     x_initial(10:12,i) = randn(3,1)*1; %attitude rate & cable rate
%     x_initial(13:15,i) = randn(3,1)*5; %quadrotor attitude
%     x_initial(16:18,i) = randn(3,1)*10; %quadrotor attitude rate
%     x_initial(10:18,i) = rand(9,1)*2;
end
% x_initial(1,1) = 0;
% x_initial(1,1) = 0;
% x_initial(4,1) = 0;
% x_initial(5,1) = 0;
% x_initial(6,1) = 0;
% x_initial(7,1) = 0;%cable direction
% x_initial(8,1) = 0;
% x_initial(9,1) = 0;
% x_initial(10,1) = 0;%cable omega
% x_initial(11,1) = 0;
% x_initial(12,1) = 0;
% x_initial(13,1) = 0;%multirotor attitude
% x_initial(14,1) = 0;
% x_initial(15,1) = 0;
% x_initial(16,1) = 0;%multirotor attitude
% x_initial(17,1) = 0;
% x_initial(18,1) = 0;
x_final(1,1) = 3;
x_final(2,1) = 0;
x_final(3,1) = 0;
x_waypoint(1,1) = 2;
x_waypoint(2,1) = 3;
x_waypoint(3,1) = 0;
d_1 = 2;

G = 9.81;
MASS_Q = 2.45;
MASS_L = 0.05;
LENGTH = 1;
I = diag([0.0447563 0.0458929 0.1277]);

%Dynamics
dynamics = @Dynamics.dynamics_slungload_flat;
dynamics_param.dt = dt;
dynamics_param.dx = 0.000001;
dynamics_param.du = 0.000001;
dynamicsGradient = @Dynamics.dynamicsGradient;

%Converter
converter_param.MASS_Q = MASS_Q;
converter_param.MASS_L = MASS_L;
converter_param.LENGTH = LENGTH;
converter_param.G = G;
converter_param.I = I;

%Cost
cost = @Cost.cost;
cost_param.dt = dt;
cost_param.x_f = x_final;
cost_param.L = diag([1000 1000 1000 100 100 100 10 10 10 10 10 10 10 10 10 10 10 10 ]);
% cost_param.L = diag([300 300 300 100 100 100 100 100 100 1*ones(1,9)]);
cost_param.Q = cost_param.L;
% cost_param.Q = zeros(n,n);
cost_param.R = diag([1 1 1]);
cost_param.alpha = 300;
cost_param.u_ref = 0;
cost_param.N = N;
cost_param.dx = 0.000001;
cost_param.du = 0.000001;
costOne = @Cost.costOne;
costGradient = @Cost.costGradient;
cost_param.obstacle_enable = false;
cost_param.obstacle = zeros(3,1);
cost_param.obstacle(:,1) = [1.5 0.05 0].';
if cost_param.obstacle_enable
    cost_param.obstacle_consider = 'xy';
%     cost_param.obstacle_consider = 'xyz';
end

%QP
A = diag(ones(1,18));
for i=1:15
    A(i,i+3) = dt;
end
% A = zeros(18,18);
% for i=1:15
%     A(i,i+3) = 1;
% end
% A = expm(A*dt);
B = zeros(18,3);
B(16,1) = dt;
B(17,2) = dt;
B(18,3) = dt;

A_eq1 = zeros(18, 3*N);
for i=1:N
    A_eq1(:,((i-1)*3+1):i*3) = A^(N-i)*B;
end
b_eq1 = x_final - A^N*x_initial;

A_eq2 = zeros(18, 3*(k_waypoint-1));
for i=1:k_waypoint-1
    A_eq2(:,((i-1)*3+1):i*3) = A^(k_waypoint-1-i)*B;
end
A_eq2 = [A_eq2 zeros(18,3*(N-(k_waypoint-1)))];
b_eq2 = x_waypoint - A^(k_waypoint-1)*x_initial;

x_obstacle = [1.5 0.0 0.0].';
A_o = zeros(N*18,3*N);
A_o(1:18,1:3) = B;
B_o = zeros(18*N,18);
B_o(1:18,1:18) = A;
for i=2:N
    A_o((18*(i-1))+1:18*i,1:3) = A*A_o((18*(i-2))+1:18*(i-1),1:3);
    A_o((18*(i-1))+1:18*i,4:end) = A_o((18*(i-2))+1:18*(i-1),1:(end-3));
    B_o(18*(i-1)+1:18*i,:) = A*B_o(18*(i-2)+1:18*(i-1),:);
end

%Final only
A_eq = [A_eq1];
b_eq = [b_eq1];

%Final & Waypoint
% A_eq = [A_eq1; A_eq2(1:3,:)];
% b_eq = [b_eq1; b_eq2(1:3,:)];

QP_option = optimoptions('quadprog', 'Display', 'iter');

QP_param.H = eye(3*N);
QP_param.A = A;
QP_param.B = B;
QP_param.A_eq = A_eq;
QP_param.b_eq = b_eq;
QP_param.option = QP_option;