%% Load parameters
parameters;
load('flatData.mat');
%% Simulation
for i=1:initialNumber
    fprintf('==================[%d/%d]==================\n',i,initialNumber);
    [x_trajec_optimal2(:,:,i), u_trajec_optimal(:,:,i), convergenceTime(i), cost_new(i)] = SLQ(x_initial(:,i), x_trajec_optimal, SLQ_param);
end
%% Plot
for j=1:initialNumber
    figure(1);
    ylabels = {'$$x_L(m)$$', '$$y_L(m)$$', '$$z_L(m)$$', '$$\phi_Q(deg)$$', '$$\theta_Q(deg)$$', '$$\psi_Q(deg)$$', '$$p_x$$', '$$p_y$$', '$$p_z$$', ...
        '$$v_{x.L}(m/s)$$', '$$v_{y.L}(m/s)$$', '$$v_{z.L}(m/s)$$', '$$p_Q(deg/s)$$', '$$q_Q(deg/s)$$', '$$r_Q(deg/s)$$', '$$w_x(/s)$$', '$$w_y(/s)$$', '$$w_z(/s)$$'};
    subplot(611); plot(t, x_trajec_optimal2(1,:,j),'LineWidth',1); hold on; plot(t(end), x_trajec_optimal(1,end), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(1),'Interpreter','latex');
    plot(t, x_trajec_optimal(1,:),'--k','LineWidth',1.5);
    title('Position & Velocity of the load');
    subplot(612); plot(t, x_trajec_optimal2(2,:,j),'LineWidth',1); hold on; plot(t(end), x_trajec_optimal(2,end), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(2),'Interpreter','latex');
    plot(t, x_trajec_optimal(2,:),'--k','LineWidth',1.5);
    subplot(613); plot(t, x_trajec_optimal2(3,:,j),'LineWidth',1); hold on; plot(t(end), x_trajec_optimal(3,end), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(3),'Interpreter','latex');
    plot(t, x_trajec_optimal(3,:),'--k','LineWidth',1.5);
    subplot(614); plot(t, x_trajec_optimal2(10,:,j),'LineWidth',1); hold on; plot(t(end), x_trajec_optimal(10,end), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(10),'Interpreter','latex');
    plot(t, x_trajec_optimal(10,:),'--k','LineWidth',1.5);
    subplot(615); plot(t, x_trajec_optimal2(11,:,j),'LineWidth',1); hold on; plot(t(end), x_trajec_optimal(11,end), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(11),'Interpreter','latex');
    plot(t, x_trajec_optimal(11,:),'--k','LineWidth',1.5);
    subplot(616); plot(t, x_trajec_optimal2(12,:,j),'LineWidth',1); hold on; plot(t(end), x_trajec_optimal(12,end), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(12),'Interpreter','latex');
    plot(t, x_trajec_optimal(12,:),'--k','LineWidth',1.5);

    figure(2);
    subplot(611); plot(t, rad2deg(x_trajec_optimal2(4,:,j)),'LineWidth',1); hold on; plot(t(end), rad2deg(x_trajec_optimal(4,end)), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(4),'Interpreter','latex');
    plot(t, rad2deg(x_trajec_optimal(4,:)),'--k','LineWidth',1.5);
    title('Attitude & attitude rate of the drone');
    subplot(612); plot(t, rad2deg(x_trajec_optimal2(5,:,j)),'LineWidth',1); hold on; plot(t(end), rad2deg(x_trajec_optimal(5,end)), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(5),'Interpreter','latex');
    plot(t, rad2deg(x_trajec_optimal(5,:)),'--k','LineWidth',1.5);
    subplot(613); plot(t, rad2deg(x_trajec_optimal2(6,:,j)),'LineWidth',1); hold on; plot(t(end), rad2deg(x_trajec_optimal(6,end)), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(6),'Interpreter','latex');
    plot(t, rad2deg(x_trajec_optimal(6,:)),'--k','LineWidth',1.5);
    subplot(614); plot(t, rad2deg(x_trajec_optimal2(13,:,j)),'LineWidth',1); hold on; plot(t(end), rad2deg(x_trajec_optimal(13,end)), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(13),'Interpreter','latex');
    plot(t, rad2deg(x_trajec_optimal(13,:)),'--k','LineWidth',1.5);
    subplot(615); plot(t, rad2deg(x_trajec_optimal2(14,:,j)),'LineWidth',1); hold on; plot(t(end), rad2deg(x_trajec_optimal(14,end)), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(14),'Interpreter','latex');
    plot(t, rad2deg(x_trajec_optimal(14,:)),'--k','LineWidth',1.5);
    subplot(616); plot(t, rad2deg(x_trajec_optimal2(15,:,j)),'LineWidth',1); hold on; plot(t(end), rad2deg(x_trajec_optimal(15,end)), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(15),'Interpreter','latex');
    plot(t, rad2deg(x_trajec_optimal(15,:)),'--k','LineWidth',1.5);

    figure(3);
    subplot(611); plot(t, x_trajec_optimal2(7,:,j),'LineWidth',1); hold on; plot(t(end), x_trajec_optimal(7,end), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(7),'Interpreter','latex');
    title('Direction & rate of the cable');
    plot(t, x_trajec_optimal(7,:),'--k','LineWidth',1.5);
    subplot(612); plot(t, x_trajec_optimal2(8,:,j),'LineWidth',1); hold on; plot(t(end), x_trajec_optimal(8,end), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(8),'Interpreter','latex');
    plot(t, x_trajec_optimal(8,:),'--k','LineWidth',1.5);
    subplot(613); plot(t, x_trajec_optimal2(9,:,j),'LineWidth',1); hold on; plot(t(end), x_trajec_optimal(9,end), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(9),'Interpreter','latex');
    plot(t, x_trajec_optimal(9,:),'--k','LineWidth',1.5);
    subplot(614); plot(t, x_trajec_optimal2(16,:,j),'LineWidth',1); hold on; plot(t(end), x_trajec_optimal(16,end), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(16),'Interpreter','latex');
    plot(t, x_trajec_optimal(16,:),'--k','LineWidth',1.5);
    subplot(615); plot(t, x_trajec_optimal2(17,:,j),'LineWidth',1); hold on; plot(t(end), x_trajec_optimal(17,end), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(17),'Interpreter','latex');
    plot(t, x_trajec_optimal(17,:),'--k','LineWidth',1.5);
    subplot(616); plot(t, x_trajec_optimal2(18,:,j),'LineWidth',1); hold on; plot(t(end), x_trajec_optimal(18,end), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(18),'Interpreter','latex');
    plot(t, x_trajec_optimal(18,:),'--k','LineWidth',1.5);

    ylabels = {'$$f(N)$$', '$$M_x(N \cdot m)$$', '$$M_y(N \cdot m)$$', '$$M_z(N \cdot m)$$'};
    figure(4);
    subplot(411); plot(t(1:end-1), u_trajec_optimal(1,:,j),'LineWidth',1); hold on; ylabel(ylabels(1),'Interpreter','latex');
    subplot(412); plot(t(1:end-1), u_trajec_optimal(2,:,j),'LineWidth',1); hold on; ylabel(ylabels(1),'Interpreter','latex');
    subplot(413); plot(t(1:end-1), u_trajec_optimal(3,:,j),'LineWidth',1); hold on; ylabel(ylabels(1),'Interpreter','latex');
    subplot(414); plot(t(1:end-1), u_trajec_optimal(4,:,j),'LineWidth',1); hold on; ylabel(ylabels(1),'Interpreter','latex');
    title('Control input');
end
autoArrangeFigures();
%% video
v = VideoWriter('result.avi');
v.FrameRate = 1/dt;
open(v);
for j=1:initialNumber
    figure(5);
    plot3(x_trajec_optimal(1,:), x_trajec_optimal(2,:), x_trajec_optimal(3,:), '--'); axis equal; hold on;
    plot3(x_trajec_optimal2(1,:,j), x_trajec_optimal2(2,:,j), x_trajec_optimal2(3,:,j), '-');
    for i=1:size(x_trajec_optimal2,2)
        h1 = drawRotors([x_trajec_optimal2(1:3,i,j)-LENGTH*x_trajec_optimal2(7:9,i,j); x_trajec_optimal2(4:6,i,j)], 0.2);
        h2 = drawCable(x_trajec_optimal2(1:3,i,j), x_trajec_optimal2(7:9,i,j), LENGTH);
        h3 = drawLoad(x_trajec_optimal2(1:3,i,j));
        plot3(cost_param.obstacle(1), cost_param.obstacle(2), cost_param.obstacle(3), 'o', 'MarkerSize',15, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
        xlim([-1 4]); ylim([-2.5 2.5]); zlim([-1 4]);
        drawnow;
        frame = getframe(gcf);
        writeVideo(v,frame);
        if i~=size(x_trajec_optimal2,2)
            delete(h1{1});
            delete(h1{2});
            delete(h2{1});
            delete(h3);
        end
    end
end
close(v);