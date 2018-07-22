clc; clear all; close all;
%Estimation of up to fifth derivative of load position

%Load simulated data
load('flatData.mat');

%System definition
n = 18;
m = 9;
dt = 0.01;

A = zeros(n,n);
% for i=1:15
%     A(i,i) = 1;
%     A(i,i+3) = dt;
% end
% A(16,16) = 1;
% A(17,17) = 1;
% A(18,18) = 1;
for i=1:15
   A(i,i+3) = 1;
end
A = expm(A*dt);
H = zeros(6,n);
H(1,1) = 1;
H(2,2) = 1;
H(3,3) = 1;
H(4,4) = 1;
H(5,5) = 1;
H(6,6) = 1;
H(7,7) = 1;
H(8,8) = 1;
H(9,9) = 1;

x_trajec_estimate = zeros(size(x_trajec_optimal));
x_trajec_estimate(:,1) = x_trajec_optimal(:,1);
P_estimate = zeros(n,n,size(x_trajec_optimal,2));
P(:,:,1) = 1*eye(18);

sigma_w = zeros(18,1);
sigma_w(16) = 1;
sigma_w(17) = 1;
sigma_w(18) = 1;
Q = diag(sigma_w);

sigma_v = 0.01*ones(9,1);
sigma_v(4) = 0.05;
sigma_v(5) = 0.05;
sigma_v(6) = 0.05;
sigma_v(7) = 0.1;
sigma_v(8) = 0.1;
sigma_v(9) = 0.1;
R = diag(sigma_v);
for i=2:size(x_trajec_optimal,2)
    %prediction
    w = randn(18,1).*sigma_w;
    x_priori = A*x_trajec_estimate(:,i-1) + w;
    P_priori = A*P(:,:,i-1)*A.' + Q;
    
    %estimation
    v = randn(9,1).*sigma_v;
    S = R + H*P_priori*H.';
    K = P_priori*H.'*inv(S);
    y = x_trajec_optimal(1:9,i) + v;
    x_trajec_estimate(:,i) = x_priori + K*(y-H*x_priori);
    P(:,:,i) = (eye(n) - K*H)*P_priori;
end

figure(1);
subplot(611);
plot(1:size(x_trajec_optimal,2), x_trajec_optimal(1,:),'-k'); hold on;
plot(1:size(x_trajec_optimal,2), x_trajec_estimate(1,:),'-b');
subplot(612);
plot(1:size(x_trajec_optimal,2), x_trajec_optimal(4,:),'-k'); hold on;
plot(1:size(x_trajec_optimal,2), x_trajec_estimate(4,:),'-b');
subplot(613);
plot(1:size(x_trajec_optimal,2), x_trajec_optimal(7,:),'-k'); hold on;
plot(1:size(x_trajec_optimal,2), x_trajec_estimate(7,:),'-b');
subplot(614);
plot(1:size(x_trajec_optimal,2), x_trajec_optimal(10,:),'-k'); hold on;
plot(1:size(x_trajec_optimal,2), x_trajec_estimate(10,:),'-b');
subplot(615);
plot(1:size(x_trajec_optimal,2), x_trajec_optimal(13,:),'-k'); hold on;
plot(1:size(x_trajec_optimal,2), x_trajec_estimate(13,:),'-b');
subplot(616);
plot(1:size(x_trajec_optimal,2), x_trajec_optimal(16,:),'-k'); hold on;
plot(1:size(x_trajec_optimal,2), x_trajec_estimate(16,:),'-b');