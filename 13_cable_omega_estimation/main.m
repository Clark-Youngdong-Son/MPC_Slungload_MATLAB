%% Load parameters
clc; clear all; close all;
parameters;

%% Simulation
[x_trajec_optimal, u_trajec_optimal, time, cost_new] = SLQ(x(:,1), [], SLQ_param);
omega_estimation = omegaEstimation(x_trajec_optimal, dynamics_param);
%% Plot
ylabels = {'$$x_L(m)$$', '$$y_L(m)$$', '$$z_L(m)$$', '$$\phi_Q(deg)$$', '$$\theta_Q(deg)$$', '$$\psi_Q(deg)$$', '$$p_x$$', '$$p_y$$', '$$p_z$$', ...
    '$$v_{x.L}(m/s)$$', '$$v_{y.L}(m/s)$$', '$$v_{z.L}(m/s)$$', '$$p_Q(deg/s)$$', '$$q_Q(deg/s)$$', '$$r_Q(deg/s)$$', '$$w_x(/s)$$', '$$w_y(/s)$$', '$$w_z(/s)$$'};
subplot(611); plot(t, x_trajec_optimal(1,:),'LineWidth',1.5); hold on; plot(t(end), x_final(1), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(1),'Interpreter','latex');
title('Position & Velocity of the load');
subplot(612); plot(t, x_trajec_optimal(2,:),'LineWidth',1.5); hold on; plot(t(end), x_final(2), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(2),'Interpreter','latex');
subplot(613); plot(t, x_trajec_optimal(3,:),'LineWidth',1.5); hold on; plot(t(end), x_final(3), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(3),'Interpreter','latex');
subplot(614); plot(t, x_trajec_optimal(10,:),'LineWidth',1.5); hold on; plot(t(end), x_final(10), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(10),'Interpreter','latex');
subplot(615); plot(t, x_trajec_optimal(11,:),'LineWidth',1.5); hold on; plot(t(end), x_final(11), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(11),'Interpreter','latex');
subplot(616); plot(t, x_trajec_optimal(12,:),'LineWidth',1.5); hold on; plot(t(end), x_final(12), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(12),'Interpreter','latex');

figure(2);
subplot(611); plot(t, x_trajec_optimal(4,:),'LineWidth',1.5); hold on; plot(t(end), x_final(4), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(4),'Interpreter','latex');
title('Attitude & attitude rate of the drone');
subplot(612); plot(t, x_trajec_optimal(5,:),'LineWidth',1.5); hold on; plot(t(end), x_final(5), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(5),'Interpreter','latex');
subplot(613); plot(t, x_trajec_optimal(6,:),'LineWidth',1.5); hold on; plot(t(end), x_final(6), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(6),'Interpreter','latex');
subplot(614); plot(t, x_trajec_optimal(13,:),'LineWidth',1.5); hold on; plot(t(end), x_final(13), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(13),'Interpreter','latex');
subplot(615); plot(t, x_trajec_optimal(14,:),'LineWidth',1.5); hold on; plot(t(end), x_final(14), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(14),'Interpreter','latex');
subplot(616); plot(t, x_trajec_optimal(15,:),'LineWidth',1.5); hold on; plot(t(end), x_final(15), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(15),'Interpreter','latex');

figure(3);
subplot(611); plot(t, x_trajec_optimal(7,:),'LineWidth',1.5); hold on; plot(t(end), x_final(7), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(7),'Interpreter','latex');
title('Direction & rate of the cable');
subplot(612); plot(t, x_trajec_optimal(8,:),'LineWidth',1.5); hold on; plot(t(end), x_final(8), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(8),'Interpreter','latex');
subplot(613); plot(t, x_trajec_optimal(9,:),'LineWidth',1.5); hold on; plot(t(end), x_final(9), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(9),'Interpreter','latex');
subplot(614); plot(t, x_trajec_optimal(16,:),'LineWidth',1.5); hold on; plot(t(end), x_final(16), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(16),'Interpreter','latex');
subplot(615); plot(t, x_trajec_optimal(17,:),'LineWidth',1.5); hold on; plot(t(end), x_final(17), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(17),'Interpreter','latex');
subplot(616); plot(t, x_trajec_optimal(18,:),'LineWidth',1.5); hold on; plot(t(end), x_final(18), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(18),'Interpreter','latex');
subplot(614); plot(t, omega_estimation(1,:),'b','LineWidth',1.5); hold on; plot(t(end), x_final(16), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(16),'Interpreter','latex');
subplot(615); plot(t, omega_estimation(2,:),'b','LineWidth',1.5); hold on; plot(t(end), x_final(17), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(17),'Interpreter','latex');
subplot(616); plot(t, omega_estimation(3,:),'b','LineWidth',1.5); hold on; plot(t(end), x_final(18), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(18),'Interpreter','latex');

ylabels = {'$$f(N)$$', '$$M_x(N \cdot m)$$', '$$M_y(N \cdot m)$$', '$$M_z(N \cdot m)$$'};
figure(4);
subplot(411); plot(t(1:end-1), u_trajec_optimal(1,:),'LineWidth',1.5); hold on; ylabel(ylabels(1),'Interpreter','latex');
title('Control input');
subplot(412); plot(t(1:end-1), u_trajec_optimal(2,:),'LineWidth',1.5); hold on; ylabel(ylabels(2),'Interpreter','latex');
subplot(413); plot(t(1:end-1), u_trajec_optimal(3,:),'LineWidth',1.5); hold on; ylabel(ylabels(3),'Interpreter','latex');
subplot(414); plot(t(1:end-1), u_trajec_optimal(4,:),'LineWidth',1.5); hold on; ylabel(ylabels(4),'Interpreter','latex');

figure(5)
plot3(x_trajec_optimal(1,:), x_trajec_optimal(2,:), x_trajec_optimal(3,:)); axis equal; hold on;
plot3(x_trajec_optimal(1,1), x_trajec_optimal(1,1), x_trajec_optimal(1,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot3(x_final(1), x_final(2), x_final(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
if(cost_param.obstacle_enable)
    for i=1:size(cost_param.obstacle,2)
        plot3(cost_param.obstacle(1,i), cost_param.obstacle(2,i), cost_param.obstacle(3,i), 'o', 'MarkerSize',15, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
    end
end
xlim([-1 4]); ylim([-2.5 2.5]); zlim([-1 4]);
% conditionTest1(x_trajec_optimal);
conditionTest1([x_trajec_optimal(1:15,:); omega_estimation]);
autoArrangeFigures();