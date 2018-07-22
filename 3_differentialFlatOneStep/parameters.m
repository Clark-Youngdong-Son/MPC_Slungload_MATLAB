%Simulation
dt = 0.01;
N = 200;
t = 0:dt:dt*N;

n = 3*6;
m = 3;
x_final = zeros(n,1);

%System
initialNumber = 1;
x_trajec_optimal = zeros(n,N+1,initialNumber);
u_trajec_optimal = zeros(m,N, initialNumber);
cost_new = zeros(initialNumber,1);
iteration = zeros(initialNumber,1);
convergenceTime = zeros(initialNumber,1);
obstacleDistance = zeros(N+1, initialNumber);
x_initial = zeros(n,initialNumber);
% for i=1:initialNumber
%    x_initial(:,i) = randn(n,1)*0.1;
%    x_initial(7:9,i) = x_initial(7:9,i)/norm(x_initial(7:9,i));
%    if(x_initial(9,i)>0)
%        x_initial(7:9,i) = -x_initial(7:9,i);
%    end
% end
% x_initial(1,1) = 0;
% x_initial(1,1) = 0;
% x_initial(4,1) = 0;
% x_initial(5,1) = 0;
% x_initial(6,1) = 0;
x_initial(7,1) = 0;%cable direction
x_initial(8,1) = 0;
x_initial(9,1) = 0;
x_initial(10,1) = 0;%cable omega
x_initial(11,1) = 0;
x_initial(12,1) = 0;
x_initial(13,1) = 0;%multirotor attitude
x_initial(14,1) = 0;
x_initial(15,1) = 0;
x_initial(16,1) = 100;%multirotor attitude
x_initial(17,1) = 0;
x_initial(18,1) = 0;
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
cost_param.L = diag([1000 1000 1000 100 100 100 10 10 10 10 10 10 10 10 10 zeros(1,3)]);
% cost_param.L = diag([300 300 300 100 100 100 100 100 100 1*ones(1,9)]);
% cost_param.Q = cost_param.L;
cost_param.Q = zeros(n,n);
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

%SLQ
%Tunable
SLQ_param.warmstart_original = false; %use one-step-shifted input trajectory
SLQ_param.warmstart_modified = false; %use the input which follows the previous state trajectory
SLQ_param.maxIter = 10;
SLQ_param.maxLineSearch = 10;
SLQ_param.convergenceTol = 0.01;
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