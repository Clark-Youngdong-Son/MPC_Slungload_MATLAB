%Simulation
dt = 0.04;
N = 100;
t = 0:dt:dt*N;

n = 3*6;
m = 3;
x_final = zeros(n,1);

%System
initialNumber = 2;
x_trajec_optimal = zeros(n,N+1,initialNumber);
u_trajec_optimal = zeros(m,N, initialNumber);
cost_new = zeros(initialNumber,1);
iteration = zeros(initialNumber,1);
convergenceTime = zeros(initialNumber,1);
obstacleDistance = zeros(N+1, initialNumber);
x_initial = zeros(n,initialNumber);
for i=1:initialNumber
%     x_initial(1:6,i) = randn(6,1)*1;
%     x_initial(7:9,i) = randn(3,1)*1; %attitude & cable together
%     x_initial(10:12,i) = randn(3,1)*1; %attitude rate & cable rate
%     x_initial(13:15,i) = randn(3,1)*5; %quadrotor attitude
%     x_initial(16:18,i) = randn(3,1)*10; %quadrotor attitude rate
%     x_initial(10:18,i) = rand(9,1)*2;
end
    x_initial(7:9,1) = randn(3,1)*1; %attitude & cable together
    x_initial(7:9,2) = x_initial(7:9,2)*0.5; %attitude & cable together
% x_initial(1,1) = 0;
% x_initial(1,1) = 0;
% x_initial(4,1) = 0;
% x_initial(5,1) = 0;
% x_initial(6,1) = 0;
% x_initial(16,1) = 3;
% x_initial(17,1) = 0;
% x_initial(18,1) = 0;
x_final(1,1) = 3;
x_final(2,1) = 0;
x_final(3,1) = 0;
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

%Cost
cost = @Cost.cost;
cost_param.dt = dt;
cost_param.x_f = x_final;
% cost_param.L = diag([100 100 100 10 10 10 1 1 1 zeros(1,9)]);
cost_param.L = diag([10 10 1 1 1 100 10 10 10 10 10 10 0*ones(1,6)]);
% cost_param.Q = diag([10 10 1 1 1 100 10 10 10 10 10 10 10*ones(1,6)])*0.1;
cost_param.Q = zeros(18,18);
cost_param.R = diag([1 1 1]);
cost_param.alpha = 100;
cost_param.u_ref = 0;
cost_param.N = N;
cost_param.dx = 0.000001;
cost_param.du = 0.000001;
costOne = @Cost.costOne;
costGradient = @Cost.costGradient;
cost_param.obstacle_enable = false;
cost_param.obstacle = zeros(3,1);
cost_param.obstacle(:,1) = [1.5 0 0].';
if cost_param.obstacle_enable
    cost_param.obstacle_consider = 'xy';
%     cost_param.obstacle_consider = 'xyz';
end

%SLQ
%Tunable
SLQ_param.warmstart_original = false; %use one-step-shifted input trajectory
SLQ_param.warmstart_modified = false; %use the input which follows the previous state trajectory
SLQ_param.maxIter = 10;
SLQ_param.maxLineSearch = 10;
SLQ_param.convergenceTol = 1;
%Static
SLQ_param.n = n;
SLQ_param.m = m;
SLQ_param.N = N;
SLQ_param.x_f = x_final;
SLQ_param.dynamics = dynamics;
SLQ_param.dynamicsGradient = dynamicsGradient;
SLQ_param.dynamics_param = dynamics_param;
SLQ_param.cost = cost;
SLQ_param.costOne = costOne;
SLQ_param.costGradient = costGradient;
SLQ_param.cost_param = cost_param;