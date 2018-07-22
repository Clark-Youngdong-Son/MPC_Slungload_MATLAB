%% Load parameters
clc; clear all; close all;
parameters;
time = clock;

%% Simulation
for i=1:initialNumber
    fprintf('=========[MPC] iteration: %3d/%3d\n',i,initialNumber);
    [x_trajec_optimal(:,:,i), u_trajec_optimal(:,:,i), convergenceTime(i), cost_new(i), iteration(i)] = SLQ(x_initial(:,i), [], SLQ_param);
    for j=1:N
        obstacleDistance(j,i) = sqrt(sum(((x_trajec_optimal(1:3,j))-(cost_param.obstacle(1:3))).^2));
    end
end
save(['monteCarlo_' num2str(time(1)) '_' num2str(time(2)) '_' num2str(time(3)) '_' num2str(time(4)) '_' num2str(time(5)) '.mat'], 'x_trajec_optimal', 'cost_new', 'iteration');

%% MakeVideo
v = VideoWriter(['monteCarlo_' num2str(time(1)) '_' num2str(time(2)) '_' num2str(time(3)) '_' num2str(time(4)) '_' num2str(time(5)) '.avi']);
open(v);
for i=1:initialNumber
    figure(1);
    fprintf('Recording [%4d/%4d]\n',i,initialNumber);
    subplot(121);
    drawRotors([x_trajec_optimal(1:3,1,i)-LENGTH*x_trajec_optimal(7:9,1,i); x_trajec_optimal(4:6,1,i)], 0.2);
    drawCable(x_trajec_optimal(1:3,1,i), x_trajec_optimal(7:9,1,i), LENGTH);
    drawLoad(x_trajec_optimal(1:3,1,i));
    
    plot3(x_trajec_optimal(1,:,i), x_trajec_optimal(2,:,i), x_trajec_optimal(3,:,i)); axis equal; hold on;
    plot3(x_trajec_optimal(1,1,i), x_trajec_optimal(2,1,i), x_trajec_optimal(3,1,i), 'go', 'MarkerSize', 3, 'MarkerFaceColor', 'g');
    plot3(x_final(1), x_final(2), x_final(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    if(cost_param.obstacle_enable)
        for j=1:size(cost_param.obstacle,2)
            plot3(cost_param.obstacle(1,j), cost_param.obstacle(2,j), cost_param.obstacle(3,j), 'o', 'MarkerSize',15, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
        end
    end
    xlim([-1 4]); ylim([-2.5 2.5]); zlim([-1 4]);
    grid on;
    xlabel('x(m)');
    ylabel('y(m)');
    zlabel('z(m)');
    
    subplot(122);
    hold off;
    drawRotors([x_trajec_optimal(1:3,1,i)-LENGTH*x_trajec_optimal(7:9,1,i); x_trajec_optimal(4:6,1,i)], 0.2);
    drawCable(x_trajec_optimal(1:3,1,i), x_trajec_optimal(7:9,1,i), LENGTH);
    drawLoad(x_trajec_optimal(1:3,1,i));
    
    plot3(x_trajec_optimal(1,:,i), x_trajec_optimal(2,:,i), x_trajec_optimal(3,:,i)); axis equal; hold on;
    plot3(x_trajec_optimal(1,1,i), x_trajec_optimal(2,1,i), x_trajec_optimal(3,1,i), 'go', 'MarkerSize', 3, 'MarkerFaceColor', 'g');
    plot3(x_final(1), x_final(2), x_final(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    if(cost_param.obstacle_enable)
        for j=1:size(cost_param.obstacle,2)
            plot3(cost_param.obstacle(1,j), cost_param.obstacle(2,j), cost_param.obstacle(3,j), 'o', 'MarkerSize',15, 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');
        end
    end
    xlim([-1 4]); ylim([-2.5 2.5]); zlim([-1 4]);
    grid on;
    xlabel('x(m)');
    ylabel('y(m)');
    zlabel('z(m)');
    view([-90 90]);
    frame = getframe(gcf);
    writeVideo(v,frame);
end

for i=1:initialNumber
    figure(2);
    if(~isnan(cost_new(i)))
        subplot(211); plot(i, cost_new(i), '.'); hold on;
        subplot(212); plot(i, iteration(i), '.'); hold on;
    else
        subplot(211); plot(i, 0, 'o', 'MarkerFaceColor', 'r'); hold on;
        subplot(212); plot(i, 0, 'o', 'MarkerFaceColor', 'r'); ylabel('convergence iteration'); hold on;
    end
end
subplot(211); ylabel('cost');
subplot(212); xlabel('sequence'); ylabel('convergence iteration');
autoArrangeFigures();

fprintf('Optimization failure %d/%d\n', sum(isnan(cost_new)), initialNumber);