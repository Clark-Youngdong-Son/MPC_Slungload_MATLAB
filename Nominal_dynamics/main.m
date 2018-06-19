%% Load parameters
clc; clear all; close all;
parameters;

%% Simulation
% for i=1:maxIter
for i=1:1
    %Divide for warmstart
    if(i==1)
        [x_trajec_optimal(:,:,i), u_trajec_optimal(:,:,i), time, cost_new] = SLQ(x(:,i), [], SLQ_param);
    else
        [x_trajec_optimal(:,:,i), u_trajec_optimal(:,:,i), time, cost_new] = SLQ(x(:,i), x_trajec_optimal(:,:,i-1), SLQ_param);
    end
    
    
    figure(1);
    subplot(311); plot(t, x_trajec_optimal(1,:,i)); hold on; plot(t(end), x_final(1), 'ro', 'MarkerFaceColor', 'r');
    subplot(312); plot(t, x_trajec_optimal(2,:,i)); hold on; plot(t(end), x_final(2), 'ro', 'MarkerFaceColor', 'r');
    subplot(313); plot(t, x_trajec_optimal(3,:,i)); hold on; plot(t(end), x_final(3), 'ro', 'MarkerFaceColor', 'r');
    figure(2);
    subplot(411); plot(t(1:end-1), u_trajec_optimal(1,:,i)); hold on;
    subplot(412); plot(t(1:end-1), u_trajec_optimal(2,:,i)); hold on;
    subplot(413); plot(t(1:end-1), u_trajec_optimal(3,:,i)); hold on;
    subplot(414); plot(t(1:end-1), u_trajec_optimal(4,:,i)); hold on;
end