clear all; close all;
%% Simulation parameters
n = 6;
m = 3;
N = 25;
V_max = 2;
g = [0 0 -9.81].';
p0 = [-8 -1 0].';
v0 = [0 0 0].';
pc_1 = [-1 0].';
r1 = 3;
lambda = 0;
tf = 15;
epsilon = 1e-6;
u_max = 13.33;
theta_cone = deg2rad(30);
pf = [8 1 0.5].';
vf = [0 0 0].';
pc_2 = [4 01].';
r2 = 1.5;
n_hat = [0 0 1].';

dt = tf/(N-1);
H = zeros(2,3);
H(1,1) = 1;
H(2,2) = 1;

x = zeros(n,N);
u = zeros(m,N-1);
y = [reshape(x,n*N,1); reshape(u,m*(N-1),1)];

%% Dynamics (double integrator)
A = zeros(n,n);
A(1,1) = 1;
A(2,2) = 1;
A(3,3) = 1;
A(1,4) = dt;
A(2,5) = dt;
A(3,6) = dt;
A(4,4) = 1;
A(5,5) = 1;
A(6,6) = 1;

B = zeros(n,m);
B(4,1) = 1;
B(5,2) = 1;
B(6,3) = 1;

%% Simulation
%Initial guess with waypoint-QP


while(1)    
    %Convexification
    
    
    %Optimization
    
end