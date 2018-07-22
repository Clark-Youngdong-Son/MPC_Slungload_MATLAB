%% Check flatness validity by applying computed input to the original system
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

x_real = zeros(n,N+1);
x_real(:,1) = x_trajec_optimal(:,1);
for i=1:N
%     x_real(4:6,i) = x_trajec_optimal(4:6,i);
%     x_real(13:15,i) = x_trajec_optimal(13:15,i);
    x_real(:,i+1) = Dynamics.dynamics_slungload(x_real(:,i), u_trajec_optimal(:,i), dynamics_param);
%     x_real(4:6,i+1) = x_trajec_optimal(4:6,i+1);
%     x_real(13:15,i+1) = x_trajec_optimal(13:15,i+1);
end

%Plot
figure(1);
subplot(611); plot(t, x_real(1,:), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(1), 'ro', 'MarkerFaceColor', 'r');
plot(t(k_waypoint), x_waypoint(1), 'o', 'MarkerSize', 5,'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
subplot(612); plot(t, x_real(2,:), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(2), 'ro', 'MarkerFaceColor', 'r');
plot(t(k_waypoint), x_waypoint(2), 'o', 'MarkerSize', 5,'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
subplot(613); plot(t, x_real(3,:), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(3), 'ro', 'MarkerFaceColor', 'r');
plot(t(k_waypoint), x_waypoint(3), 'o', 'MarkerSize', 5,'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
subplot(614); plot(t, x_real(10,:), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(10), 'ro', 'MarkerFaceColor', 'r');
subplot(615); plot(t, x_real(11,:), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(11), 'ro', 'MarkerFaceColor', 'r');
subplot(616); plot(t, x_real(12,:), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(12), 'ro', 'MarkerFaceColor', 'r');

figure(2);
subplot(611); plot(t, rad2deg(x_real(4,:)), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(4), 'ro', 'MarkerFaceColor', 'r');
subplot(612); plot(t, rad2deg(x_real(5,:)), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(5), 'ro', 'MarkerFaceColor', 'r');
subplot(613); plot(t, rad2deg(x_real(6,:)), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(6), 'ro', 'MarkerFaceColor', 'r');
subplot(614); plot(t, rad2deg(x_real(13,:)), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(13), 'ro', 'MarkerFaceColor', 'r');
subplot(615); plot(t, rad2deg(x_real(14,:)), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(14), 'ro', 'MarkerFaceColor', 'r');
subplot(616); plot(t, rad2deg(x_real(15,:)), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(15), 'ro', 'MarkerFaceColor', 'r');

figure(3);
subplot(611); plot(t, x_real(7,:), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(7), 'ro', 'MarkerFaceColor', 'r');
title('Direction & rate of the cable');
subplot(612); plot(t, x_real(8,:), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(8), 'ro', 'MarkerFaceColor', 'r');
subplot(613); plot(t, x_real(9,:), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(9), 'ro', 'MarkerFaceColor', 'r');
subplot(614); plot(t, x_real(16,:), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(16), 'ro', 'MarkerFaceColor', 'r');
subplot(615); plot(t, x_real(17,:), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(17), 'ro', 'MarkerFaceColor', 'r');
subplot(616); plot(t, x_real(18,:), 'r', 'LineWidth',1.5); hold on; plot(t(end), x_final(18), 'ro', 'MarkerFaceColor', 'r');