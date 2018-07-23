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

%% Initial guess with waypoint-QP
%cost
H = zeros(n*N + m*(N-1), n*N + m*(N-1));
for i=(n*N+1):(n*N + m*(N-1))
    H(i,i) = 1;
end

%dynamics
A_dynamics = zeros(n*(N-1), n*N + m*(N-1));
for i=1:N-1
    A_dynamics(((i-1)*n+1):(i*n),((i-1)*n+1):((i+1)*n)) = [-A eye(n)];
    A_dynamics(((i-1)*n+1):(i*n),(n*N+1+(i-1)*m):(n*N+i*m)) = -B;
end
b_dynamics = zeros(n*(N-1),1);
for i=1:N-1
    b_dynamics(((i-1)*n+1):(i*n),:) = B*g;
end

%initial condition
A_initial = zeros(n,n*N + m*(N-1));
A_initial(:,1:n) = eye(n);
b_initial = zeros(n,1);
b_initial(1:3) = p0;
b_initial(4:6) = vf;

%final condition
A_final = zeros(n,n*N + m*(N-1));
A_final(:,(n*(N-1)+1):(n*N)) = eye(n);
b_final = zeros(n,1);
b_final(1:3) = pf;
b_final(4:6) = vf;

%waypoint condition
x_waypoint = [0 -5 0].';
k_waypoint = round(N/2);
A_waypoint = zeros(3,n*N + m*(N-1));
A_waypoint(:,((n*(k_waypoint-1)+1):(n*k_waypoint))) = [eye(3) zeros(3,3)];
b_waypoint = zeros(3,1);
b_waypoint(1:3) = x_waypoint;

A_eq = [A_dynamics; A_initial; A_final; A_waypoint];
b_eq = [b_dynamics; b_initial; b_final; b_waypoint];
QP_option = optimoptions('quadprog', 'Display', 'iter');
y_initial = quadprog(H,[],[],[],A_eq,b_eq,[],[],[],QP_option);
x_initial = y_initial(1:n*N);
x_initial = reshape(x_initial,n,N);
u_initial = y_initial(n*N+1:n*N + m*(N-1));
u_initial = reshape(u_initial,m,N-1);

figure(1);
t = 0:dt:tf;
subplot(3,2,1); plot(t, x_initial(1,:)); hold on; plot(t(k_waypoint),x_waypoint(1),'bo','MarkerFaceColor','b');
subplot(3,2,2); plot(t, x_initial(4,:)); 
subplot(3,2,3); plot(t, x_initial(2,:)); hold on; plot(t(k_waypoint),x_waypoint(2),'bo','MarkerFaceColor','b');
subplot(3,2,4); plot(t, x_initial(5,:));
subplot(3,2,5); plot(t, x_initial(3,:)); hold on; plot(t(k_waypoint),x_waypoint(3),'bo','MarkerFaceColor','b');
subplot(3,2,6); plot(t, x_initial(6,:));

figure(2);
plot3(x_initial(1,:),x_initial(2,:),x_initial(3,:),'-k'); hold on;
plot3(p0(1),p0(2),p0(3),'ok','MarkerFaceColor','k');
plot3(pf(1),pf(2),pf(3),'or','MarkerFaceColor','r');
plot3(x_waypoint(1),x_waypoint(2),x_waypoint(3),'ob','MarkerFaceColor','b');
axis equal;
axis([-8.5 8.5 -5 5 -1 8]);
%% Sequential Convexification
% while(1)    
    %Convexification
    
    
    %Optimization
    
% end