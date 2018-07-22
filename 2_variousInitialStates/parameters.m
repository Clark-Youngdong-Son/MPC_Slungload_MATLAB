%Simulation
dt = 0.04;
N = 60;
t = 0:dt:dt*N;

n = 18;
m = 4;
x_final = zeros(n,1);

%System
initialNumber = 2500;
x_trajec_optimal = zeros(n,N+1,initialNumber);
u_trajec_optimal = zeros(m,N, initialNumber);
cost_new = zeros(initialNumber,1);
iteration = zeros(initialNumber,1);
convergenceTime = zeros(initialNumber,1);
obstacleDistance = zeros(N+1, initialNumber);
x_initial = zeros(n,initialNumber);
for i=1:initialNumber
   x_initial(:,i) = randn(n,1)*0.1;
   x_initial(7:9,i) = x_initial(7:9,i)/norm(x_initial(7:9,i));
   if(x_initial(9,i)>0)
       x_initial(7:9,i) = -x_initial(7:9,i);
   end
end
x_final(1,1) = 3;
x_final(9,1) = -1;
G = 9.81;
MASS_Q = 2.45;
MASS_L = 0.05;
LENGTH = 1;
I = diag([0.0447563 0.0458929 0.1277]);

%Nominal controller
nominal_controller = @Nominal_controller.controller_geometric;
nominal_controller_param.G = G;
nominal_controller_param.MASS_Q = MASS_Q;
nominal_controller_param.MASS_L = MASS_L;
nominal_controller_param.LENGTH = LENGTH;
nominal_controller_param.I = I;
nominal_controller_param.k_x = diag([3 3 10]);
nominal_controller_param.k_v = diag([3 3 10]);
nominal_controller_param.k_p = diag([1 1 5]);
nominal_controller_param.k_w = diag([6 6 10]);
nominal_controller_param.k_R = diag([2 2 2]);
nominal_controller_param.k_pqr = diag([0.6 0.6 0.6]);
nominal_controller_param.epsilon = 0.5;
nominal_controller_param.x_f = x_final;

%Dynamics
dynamics = @Dynamics.dynamics_slungload;
dynamics_param.G = G;
dynamics_param.MASS_Q = MASS_Q;
dynamics_param.MASS_L = MASS_L;
dynamics_param.LENGTH = LENGTH;
dynamics_param.I = I;
dynamics_param.dt = dt;
dynamics_param.dx = 0.000001;
dynamics_param.du = 0.000001;
dynamicsGradient = @Dynamics.dynamicsGradient;

%Cost
cost = @Cost.cost;
cost_param.dt = dt;
cost_param.x_f = x_final;
cost_param.L = diag([100 100 300 1000 1000 1000 100 100 100 60 60 180 100 100 100 1 1 0]);
cost_param.Q = diag([100 100 300 1000 1000 1000 100 100 100 60 60 180 100 100 100 1 1 0]);
cost_param.R = diag([1 60 60 60]);
cost_param.alpha = 100;
cost_param.u_ref = [(2.45 + 0.05)*9.81 0 0 0].';
cost_param.N = N;
cost_param.dx = 0.000001;
cost_param.du = 0.000001;
costOne = @Cost.costOne;
costGradient = @Cost.costGradient;
cost_param.obstacle_enable = true;
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
SLQ_param.nominal_controller = nominal_controller;
SLQ_param.nominal_controller_param = nominal_controller_param;
SLQ_param.cost = cost;
SLQ_param.costOne = costOne;
SLQ_param.costGradient = costGradient;
SLQ_param.cost_param = cost_param;