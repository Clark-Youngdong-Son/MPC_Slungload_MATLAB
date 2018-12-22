clear all; close all;

n = 18;
m = 3;
N = 121;
simulations = 200;
x0_flat = zeros(n,simulations);
x_flat_optimal = zeros(n,N,simulations);
u_flat_optimal = zeros(m,N-1,simulations);
x_original_optimal = zeros(n,N,simulations);
u_original_optimal = zeros(4,N-1,simulations);
for ii=1:simulations
    %% Simulation parameters
    p0 = [-3 0 0.5].';
    x0(1:3) = p0;
    x0(4:6) = randn(3,1)*0.2;
    x0(7:9) = randn(3,1)*0.3; %attitude & cable together
    x0(10:12) = randn(3,1)*0.2; %attitude rate & cable rate
    x0(13:15) = randn(3,1)*0.5; %quadrotor attitude
    x0(16:18) = randn(3,1)*1; %quadrotor attitude rate
    x0(10:18) = rand(9,1)*2;
    x0_flat(:,ii) = x0;
    tf = 10;
    pf = [3 0 0.5].';
    vf = [0 0 0].';

    pc_1 = [-1.3 -0.25].';
    pc_2 = [1.3 0.25].';
    pc_3 = [3.5 0].';
    pc_4 = [-2.0 -0.4].';
    pc_5 = [-1.7 0.5].';
    pc_6 = [-0.8 0.1].';
    pc_7 = [-1.5 -0.4].';
    pc_8 = [0.3 0.4].';
    pc_9 = [0.5 -0.3].';
    pc_10 = [1 0.3].';
    r1 = 0.4;
    r2 = 0.4;
    r3 = 0.2;
    r4 = 0.2;
    r5 = 0.2;
    r6 = 0.2;
    r7 = 0.2;
    r8 = 0.2;
    r9 = 0.2;
    r10 = 0.2;

    dt = tf/(N-1);
    t = 0:dt:tf;

    x = zeros(n,N);
    u = zeros(m,N-1);
    y = [reshape(x,n*N,1); reshape(u,m*(N-1),1)];
    n_obstacle = 2;
    r = [r1 r2];
    pc = [pc_1 pc_2];
%     r = [r1 r2 r3 r4 r5 r6 r7 r8 r9 r10];
%     pc = [pc_1 pc_2 pc_3 pc_4 pc_5 pc_6 pc_7 pc_8 pc_9 pc_10];

    %% Dynamics (flat system)
    A = diag(ones(1,n));
    for i=1:15
        A(i,i+3) = dt;
    end

    B = zeros(n,m);
    B(16,1) = dt;
    B(17,2) = dt;
    B(18,3) = dt;

    %% Initial guess with A*
    gridSize = 0.1;
    xMin = -4.5;
    xMax = 4.5;
    yMin = -3;
    yMax = 3;

    % Generating a MAP
    mapRows = (xMax-xMin)/gridSize;
    mapCols = (yMax-yMin)/gridSize;
    MAP=int8(zeros(mapRows, mapCols));
    GoalRegister=int8(zeros(mapRows, mapCols));

    % Fill obstacles
    for i=1:n_obstacle
        oxMin = pc(1,i) - r(i);
        oxMax = pc(1,i) + r(i);
        oyMin = pc(2,i) - r(i);
        oyMax = pc(2,i) + r(i);

        colMin = round(((oyMax - yMax)/(yMin - yMax)*mapCols));
        colMax = round(((oyMin - yMax)/(yMin - yMax)*mapCols));
        rowMin = round(((oxMax - xMax)/(xMin - xMax)*mapRows));
        rowMax = round(((oxMin - xMax)/(xMin - xMax)*mapRows));
        MAP(rowMin:rowMax, colMin:colMax) = 1;
    end

    % Set start & Goal Position
    startCol = round(((p0(2) - yMax)/(yMin - yMax)*mapCols));
    startRow = round(((p0(1) - xMax)/(xMin - xMax)*mapRows));
    goalCol = round(((pf(2) - yMax)/(yMin - yMax)*mapCols));
    goalRow = round(((pf(1) - xMax)/(xMin - xMax)*mapRows));
    GoalRegister(goalRow, goalCol)=1;

    Connecting_Distance=8; %Avoid to high values Connecting_Distances for reasonable runtimes. 

    % Running PathFinder
    OptimalPath=ASTARPATH(startCol,startRow,MAP,GoalRegister,Connecting_Distance);
%     if size(OptimalPath,2)>1
%         figure(10)
%         imagesc((MAP))
%             colormap(flipud(gray));
% 
%         hold on;
%         axis equal
%         plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
%         plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
%         plot(OptimalPath(:,2),OptimalPath(:,1),'r')
%         legend('Goal','Start','Path')
%     end
%     set(gcf,'Color',[1 1 1]);

    % Conver to physical coordinates
    pathLength = size(OptimalPath,1);
    initialPath = zeros(2,pathLength);
    for i=1:pathLength
        initialPath(1,i) = xMax - OptimalPath(pathLength-i+1,1)*gridSize;
        initialPath(2,i) = yMax - OptimalPath(pathLength-i+1,2)*gridSize;
    end

    timeBasis = tf*0.5*(sin((0:(length(t)-1))/(length(t)-1)*pi + 3/2*pi)+1);
    initialPath_intp = zeros(2,length(t));
    initialPath_intp(1,:) = interp1(linspace(0,t(end),pathLength),initialPath(1,:), timeBasis, 'linear');
    initialPath_intp(2,:) = interp1(linspace(0,t(end),pathLength),initialPath(2,:), timeBasis, 'linear');
    % initialPath_intp(1,:) = interp1(linspace(0,t(end),pathLength),initialPath(1,:), t, 'linear');
    % initialPath_intp(2,:) = interp1(linspace(0,t(end),pathLength),initialPath(2,:), t, 'linear');

    y_initial = zeros(n*N + m*(N-1),1);
    for i=1:N
        y_initial((i-1)*n+1) = initialPath_intp(1,i);
        y_initial((i-1)*n+2) = initialPath_intp(2,i);
    end
    % 
    % y_ref = zeros(n*N + m*(N-1),1);
    % H = zeros(n*N + m*(N-1), n*N + m*(N-1));
    % % Position weight
    % for i=1:N
    %     H((i-1)*n+1,(i-1)*n+1) = 20;
    %     H((i-1)*n+2,(i-1)*n+2) = 20;
    %     y_ref((i-1)*n+1) = initialPath_intp(1,i);
    %     y_ref((i-1)*n+2) = initialPath_intp(2,i);
    % end
    % % Input weight
    % for i=(n*N+1):(n*N + m*(N-1))
    %     H(i,i) = 1;
    % end
    %dynamics
    A_dynamics = zeros(n*(N-1), n*N + m*(N-1));
    for i=1:N-1
        A_dynamics(((i-1)*n+1):(i*n),((i-1)*n+1):((i+1)*n)) = [-A eye(n)];
        A_dynamics(((i-1)*n+1):(i*n),(n*N+1+(i-1)*m):(n*N+i*m)) = -B;
    end
    b_dynamics = zeros(n*(N-1),1);

    %initial condition
    A_initial = zeros(n,n*N + m*(N-1));
    A_initial(:,1:n) = eye(n);
    b_initial = zeros(n,1);
    b_initial(1:3) = p0;
    b_initial(4:18) = x0(4:18);

    %final condition
    A_final = zeros(n,n*N + m*(N-1));
    A_final(:,(n*(N-1)+1):(n*N)) = eye(n);
    b_final = zeros(n,1);
    b_final(1:3) = pf;
    b_final(4:6) = vf;
    % 
    % %waypoint condition
    % % x_waypoint = [0 -5 0].';
    % % k_waypoint = round(N/2);
    % % A_waypoint = zeros(3,n*N + m*(N-1));
    % % A_waypoint(:,((n*(k_waypoint-1)+1):(n*k_waypoint))) = [eye(3) zeros(3,n-3)];
    % % b_waypoint = zeros(3,1);
    % % b_waypoint(1:3) = x_waypoint;
    % 
    % % A_eq = [A_dynamics; A_initial; A_final; A_waypoint];
    % % b_eq = [b_dynamics; b_initial; b_final; b_waypoint];
    % A_eq = [A_dynamics; A_initial;];
    % b_eq = [b_dynamics; b_initial;];
    % 
    % cvx_begin
    %         variable y_initial(n*N + m*(N-1))
    %         
    %         minimize( (y_initial-y_ref).'*H*(y_initial-y_ref) )
    %         
    %         subject to
    %             A_eq*y_initial==b_eq
    % cvx_end
    % 
    % cost = y_initial.'*H*y_initial;
    x_initial = y_initial(1:n*N);
    x_initial = reshape(x_initial,n,N);
    u_initial = y_initial(n*N+1:n*N + m*(N-1));
    u_initial = reshape(u_initial,m,N-1);
    % 
%     figure(1);
    % % subplot(3,2,1); plot(t, x_initial(1,:),'-k'); hold on; plot(t(k_waypoint),x_waypoint(1),'bo','MarkerFaceColor','b'); ylabel('x_L(m)');
    % % subplot(3,2,2); plot(t, x_initial(4,:),'-k'); hold on; ylabel('v_L.x(m)');
    % % subplot(3,2,3); plot(t, x_initial(2,:),'-k'); hold on; plot(t(k_waypoint),x_waypoint(2),'bo','MarkerFaceColor','b'); ylabel('y_L(m)');
    % % subplot(3,2,4); plot(t, x_initial(5,:),'-k'); hold on; ylabel('v_L.y(m)');
    % % subplot(3,2,5); plot(t, x_initial(3,:),'-k'); hold on; plot(t(k_waypoint),x_waypoint(3),'bo','MarkerFaceColor','b'); ylabel('z_L(m)');
    % % subplot(3,2,6); plot(t, x_initial(6,:),'-k'); hold on; ylabel('v_L.z(m)');
%     subplot(3,2,1); plot(t, initialPath_intp(1,:), '-k'); hold on; ylabel('x_L(m)');
%     subplot(3,2,3); plot(t, initialPath_intp(2,:), '-k'); hold on; ylabel('y_L(m)');
%     set(gcf,'Color',[1 1 1]);

    figure(2);
%     plot3(x_initial(1,:),x_initial(2,:),x_initial(3,:),'-k'); hold on; axis equal;
    plot3(p0(1),p0(2),p0(3),'ok','MarkerFaceColor','k', 'MarkerSize',8); hold on;
    plot3(pf(1),pf(2),pf(3),'or','MarkerFaceColor','r', 'MarkerSize',8);
    set(gcf,'Color',[1 1 1]);
    % plot3(x_waypoint(1),x_waypoint(2),x_waypoint(3),'ob','MarkerFaceColor','b');
    for i=1:n_obstacle
        R = [r(i) r(i)];
        n_ = 200;
        [X, Y, Z] = cylinder(R,n_);
        X = X + pc(1,i);
        Y = Y + pc(2,i);
        Z(2,:) = 1.0;
        obstacle1 = surf(X,Y,Z);
        set(obstacle1,'FaceAlpha',0.5,'FaceColor','red','EdgeColor','none','EdgeAlpha',1.0)
        pos = [2 4 2 2]; 
        rectangle('Position',[pc(1,i)-0.4 pc(2,i)-0.4 0.8 0.8],'Curvature',[1 1],'FaceColor', [221 255 151]/255);
    end
    axis equal;
    axis([-4.0 4.0 -3 3 -1 2]);
    view([0 90]);
    xlabel('x(m)');
    ylabel('y(m)');
    zlabel('z(m)');
    set(gca,'Color',[1 1 1]);
    autoArrangeFigures(3,3,1);
    
    figure(2);
    plot3(x_initial(1,:),x_initial(2,:),x_initial(3,:),'-k'); hold on; axis equal;
    plot3(p0(1),p0(2),p0(3),'og','MarkerFaceColor','k');
    plot3(pf(1),pf(2),pf(3),'or','MarkerFaceColor','r');
    set(gcf,'Color',[1 1 1]);

    %% Sequential Convexification
    maxIter = 1;
    costs = zeros(1,1+maxIter);
    costs(1) = 0;
    z_temp = zeros(n*N + m*(N-1),1);
    H = zeros(n*N + m*(N-1), n*N + m*(N-1));
    % Input weight
    for i=(n*N+1):(n*N + m*(N-1))
        H(i,i) = 1;
    end
    alpha = 50;
    for i=2:(N)
        H((i-1)*n+1,(i-1)*n+1) = alpha;
        H((i-1)*n+2,(i-1)*n+2) = 4*alpha;
%         H((i-1)*n+3,(i-1)*n+3) = alpha;
    end
    for iter=1:maxIter
        %Convexification
        z = zeros(n*N + m*(N-1),n_obstacle);
        l = zeros(n*N + m*(N-1),n_obstacle);
        for i=1:n_obstacle
    %         cvx_begin quiet
    %         variable z_temp(n*N + m*(N-1))
    %         
    %         minimize( norm(y_initial - z_temp) )
    %         subject to
    %             for j=1:N
    %                 norm(z_temp(((j-1)*n+1):((j-1)*n+2)) - pc(:,i)) <= r(i)
    %             end
    %         cvx_end
            for j=1:N
                distance = sqrt(sum((x_initial(1:2,j) - pc(:,i)).^2));
                offsetVector = (r(i)-distance)*(x_initial(1:2,j) - pc(:,i))/norm(x_initial(1:2,j) - pc(:,i));
                z_temp(((j-1)*n+1):((j-1)*n+2)) = x_initial(1:2,j) + offsetVector;
            end
            z(:,i) = z_temp;
            %Linearization
            for j=2:N-1
                delta = z(((j-1)*n+1):((j-1)*n+2),i)-pc(:,i);
                l(((j-1)*n+1):((j-1)*n+2),i) = delta/norm(delta);
            end
        end    

        %Optimization
        A_eq = [A_dynamics; A_initial; A_final];
        b_eq = [b_dynamics; b_initial; b_final];
        cvx_begin
        variable y(n*N + m*(N-1))

        obj = y.'*H*y;
        for k=2:N
            obj = obj -2*alpha*y((k-1)*n+1)*pf(1);
            obj = obj -2*4*alpha*y((k-1)*n+2)*pf(2);
%             obj = obj -2*alpha*y((k-1)*n+3)*pf(3);
        end
        minimize( obj )
        subject to
                A_eq*y==b_eq
                %Linearized constraint
                for i=1:n_obstacle
                    for j=2:N-1
                       l(((j-1)*n+1):((j-1)*n+2),i).'*(y(((j-1)*n+1):((j-1)*n+2))-z(((j-1)*n+1):((j-1)*n+2),i))>=0 
                    end
                end
                for i=2:N
                    %x
                    y((i-1)*n+1)>=-4
                    y((i-1)*n+1)<=4
                    %y
                    y((i-1)*n+2)>=-2
                    y((i-1)*n+2)<=2
                    %z
                    y((i-1)*n+3)>=0
                    y((i-1)*n+3)<=1.5
                end

    %             y(n*(round(N/2))+1:n*(round(N/2))+3) == x_waypoint.';
        cvx_end

        costs(iter+1) = y.'*H*y;
        y_initial = y;
        x_initial = y_initial(1:n*N);
        x_initial = reshape(x_initial,n,N);
        u_initial = y(n*N+1:n*N + m*(N-1));
        u_initial = reshape(u_initial,m,N-1);

%         figure(1);
    %     subplot(3,2,1); plot(t, x_initial(1,:)); hold on; plot(t(k_waypoint),x_waypoint(1),'bo','MarkerFaceColor','b'); ylabel('x_L(m)');
    %     subplot(3,2,2); plot(t, x_initial(4,:)); hold on; ylabel('v_L.x(m)');
    %     subplot(3,2,3); plot(t, x_initial(2,:)); hold on; plot(t(k_waypoint),x_waypoint(2),'bo','MarkerFaceColor','b'); ylabel('y_L(m)');
    %     subplot(3,2,4); plot(t, x_initial(5,:)); hold on; ylabel('v_L.y(m)');
    %     subplot(3,2,5); plot(t, x_initial(3,:)); hold on; plot(t(k_waypoint),x_waypoint(3),'bo','MarkerFaceColor','b'); ylabel('z_L(m)');
    %     subplot(3,2,6); plot(t, x_initial(6,:)); hold on; ylabel('v_L.z(m)');
%         subplot(3,2,1); plot(t, x_initial(1,:)); hold on; ylabel('x_L(m)');
%         subplot(3,2,2); plot(t, x_initial(4,:)); hold on; ylabel('v_L.x(m)');
%         subplot(3,2,3); plot(t, x_initial(2,:)); hold on; ylabel('y_L(m)');
%         subplot(3,2,4); plot(t, x_initial(5,:)); hold on; ylabel('v_L.y(m)');
%         subplot(3,2,5); plot(t, x_initial(3,:)); hold on; ylabel('z_L(m)');
%         subplot(3,2,6); plot(t, x_initial(6,:)); hold on; ylabel('v_L.z(m)');

        figure(2);
        plot3(x_initial(1,:),x_initial(2,:),x_initial(3,:)); hold on;

        distance = zeros(n_obstacle,length(t)-2);
        for i=2:length(t)-1
            for j=1:n_obstacle
                distance(j,i-1) = sqrt(sum((x_initial(1:2,i)-pc(:,j)).^2));
            end
        end
        delta_cost = costs(iter+1) - costs(iter);
        constraints = zeros(n_obstacle,1);
        for i=1:n_obstacle
            constraints(i) = -min(distance(i,:)-r(i));
        end

        fprintf('iter: %2d cost_new: %4.2f cost_old: %4.2f',...
            iter, costs(iter+1), costs(iter));
        for i=1:n_obstacle
            fprintf(' c%1d: %4.2f',i, constraints(i));
        end
        fprintf('\n');
        if(sum(constraints<=0.01)==n_obstacle && (delta_cost>=-1))
            disp('Converged');
            break;
        end
        drawnow;
    end

    x_flat_optimal(:,:,ii) = x_initial;
    u_flat_optimal(:,:,ii) = u_initial;
    converter_param.MASS_Q = 1.5;
    converter_param.MASS_L = 0.5;
    converter_param.LENGTH = 1.0;
    converter_param.G = 9.81;
    converter_param.I = diag([0.0447563 0.0458929 0.1277]);
    [x_trajec_optimal, u_trajec_optimal] = convert2physical(real(x_initial), real(u_initial), converter_param);
    x_original_optimal(:,:,ii) = x_trajec_optimal;
    u_original_optimal(:,:,ii) = u_trajec_optimal;

%     %% SLQ
%     parameters;
%     [x_trajec_optimal_SLQ, u_trajec_optimal_SLQ, time, cost_new] = SLQ(x(:,1), [], SLQ_param);

%     %% Plot
%     figure(3);
%     ylabels = {'$$x_L(m)$$', '$$y_L(m)$$', '$$z_L(m)$$', '$$\phi_Q(deg)$$', '$$\theta_Q(deg)$$', '$$\psi_Q(deg)$$', '$$p_x$$', '$$p_y$$', '$$p_z$$', ...
%         '$$v_{x.L}(m/s)$$', '$$v_{y.L}(m/s)$$', '$$v_{z.L}(m/s)$$', '$$p_Q(deg/s)$$', '$$q_Q(deg/s)$$', '$$r_Q(deg/s)$$', '$$w_x(/s)$$', '$$w_y(/s)$$', '$$w_z(/s)$$'};
%     subplot(611); plot(t, x_trajec_optimal(1,:),'LineWidth',1.5); hold on; plot(t(end), pf(1), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(1),'Interpreter','latex');
%     % plot(t(k_waypoint), x_waypoint(1), 'o', 'MarkerSize', 5,'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
%     title('Position & Velocity of the load');
%     subplot(612); plot(t, x_trajec_optimal(2,:),'LineWidth',1.5); hold on; plot(t(end), pf(2), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(2),'Interpreter','latex');
%     % plot(t(k_waypoint), x_waypoint(2), 'o', 'MarkerSize', 5,'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
%     subplot(613); plot(t, x_trajec_optimal(3,:),'LineWidth',1.5); hold on; plot(t(end), pf(3), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(3),'Interpreter','latex');
%     % plot(t(k_waypoint), x_waypoint(3), 'o', 'MarkerSize', 5,'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
%     subplot(614); plot(t, x_trajec_optimal(10,:),'LineWidth',1.5); hold on; ylabel(ylabels(10),'Interpreter','latex');
%     subplot(615); plot(t, x_trajec_optimal(11,:),'LineWidth',1.5); hold on; ylabel(ylabels(11),'Interpreter','latex');
%     subplot(616); plot(t, x_trajec_optimal(12,:),'LineWidth',1.5); hold on; ylabel(ylabels(12),'Interpreter','latex');
%     set(gcf,'Color',[1 1 1]);
% 
%     figure(4);
%     subplot(611); plot(t, rad2deg(x_trajec_optimal(4,:)),'LineWidth',1.5); hold on; plot(t(end), vf(1), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(4),'Interpreter','latex');
%     title('Attitude & attitude rate of the drone');
%     subplot(612); plot(t, rad2deg(x_trajec_optimal(5,:)),'LineWidth',1.5); hold on; plot(t(end), vf(2), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(5),'Interpreter','latex');
%     subplot(613); plot(t, rad2deg(x_trajec_optimal(6,:)),'LineWidth',1.5); hold on; plot(t(end), vf(3), 'ro', 'MarkerFaceColor', 'r'); ylabel(ylabels(6),'Interpreter','latex');
%     subplot(614); plot(t, rad2deg(x_trajec_optimal(13,:)),'LineWidth',1.5); hold on; ylabel(ylabels(13),'Interpreter','latex');
%     subplot(615); plot(t, rad2deg(x_trajec_optimal(14,:)),'LineWidth',1.5); hold on; ylabel(ylabels(14),'Interpreter','latex');
%     subplot(616); plot(t, rad2deg(x_trajec_optimal(15,:)),'LineWidth',1.5); hold on; ylabel(ylabels(15),'Interpreter','latex');
%     set(gcf,'Color',[1 1 1]);
% 
%     figure(5);
%     subplot(611); plot(t, x_trajec_optimal(7,:),'LineWidth',1.5); hold on; ylabel(ylabels(7),'Interpreter','latex');
%     title('Direction & rate of the cable');
%     subplot(612); plot(t, x_trajec_optimal(8,:),'LineWidth',1.5); hold on; ylabel(ylabels(8),'Interpreter','latex');
%     subplot(613); plot(t, x_trajec_optimal(9,:),'LineWidth',1.5); hold on; ylabel(ylabels(9),'Interpreter','latex');
%     subplot(614); plot(t, x_trajec_optimal(16,:),'LineWidth',1.5); hold on; ylabel(ylabels(16),'Interpreter','latex');
%     subplot(615); plot(t, x_trajec_optimal(17,:),'LineWidth',1.5); hold on; ylabel(ylabels(17),'Interpreter','latex');
%     subplot(616); plot(t, x_trajec_optimal(18,:),'LineWidth',1.5); hold on; ylabel(ylabels(18),'Interpreter','latex');
%     set(gcf,'Color',[1 1 1]);
% 
%     ylabels = {'$$f(N)$$', '$$M_x(N \cdot m)$$', '$$M_y(N \cdot m)$$', '$$M_z(N \cdot m)$$'};
%     figure(6);
%     subplot(411); plot(t(1:end-1), u_trajec_optimal(1,:),'LineWidth',1.5); hold on; ylabel(ylabels(1),'Interpreter','latex');
%     subplot(412); plot(t(1:end-1), u_trajec_optimal(2,:),'LineWidth',1.5); hold on; ylabel(ylabels(2),'Interpreter','latex');
%     subplot(413); plot(t(1:end-1), u_trajec_optimal(3,:),'LineWidth',1.5); hold on; ylabel(ylabels(3),'Interpreter','latex');
%     subplot(414); plot(t(1:end-1), u_trajec_optimal(4,:),'LineWidth',1.5); hold on; ylabel(ylabels(4),'Interpreter','latex');
%     title('Control input');
%     set(gcf,'Color',[1 1 1]);

    % flatnessValidation;

%     figure(7);
%     autoArrangeFigures(3,3,1);
%     plot3(x_trajec_optimal(1,:), x_trajec_optimal(2,:), x_trajec_optimal(3,:)); axis equal; hold on;
%     % plot3(x_waypoint(1), x_waypoint(2), x_waypoint(3), 'o', 'MarkerSize', 6,'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
%     plot3(x_trajec_optimal(1,1), x_trajec_optimal(1,1), x_trajec_optimal(1,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
%     set(gcf,'Color',[1 1 1]);
%     plot3(pf(1), pf(2), pf(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
%     for i=1:n_obstacle
%         R = [r(i) r(i)];
%         n_ = 100;
%         [X, Y, Z] = cylinder(R,n_);
%         X = X + pc(1,i);
%         Y = Y + pc(2,i);
%         Z(2,:) = 0.5;
%         obstacle1 = surf(X,Y,Z);
%         set(obstacle1,'FaceAlpha',0.5,'FaceColor','red','EdgeColor','red','EdgeAlpha',0.3)
%     end
%     axis equal;
%     axis([-4.5 4.5 -3 3 -1 2]);
%     view([-30 50]);
%     xlabel('x(m)');
%     ylabel('y(m)');
%     zlabel('z(m)');
%     % v = VideoWriter('result.avi');
%     % v.FrameRate = 1/dt;
%     % open(v);
%     for i=1:1:size(x_trajec_optimal,2)
%         if i~=1
%             delete(h1{1});
%             delete(h1{2});
%             delete(h2{1});
%             delete(h3);
%     %         delete(h4{1});
%     %         delete(h4{2});
%     %         delete(h5{1});
%     %         delete(h6);
%         end
%         h1 = drawRotors([x_trajec_optimal(1:3,i)-converter_param.LENGTH*x_trajec_optimal(7:9,i); x_trajec_optimal(4:6,i)], 0.2);
%         h2 = drawCable(x_trajec_optimal(1:3,i), x_trajec_optimal(7:9,i), converter_param.LENGTH);
%         h3 = drawLoad(x_trajec_optimal(1:3,i));
% 
%     %     h4 = drawRotors([x_real(1:3,i)-LENGTH*x_real(7:9,i); x_real(4:6,i)], 0.2);
%     %     h5 = drawCable(x_real(1:3,i), x_real(7:9,i), LENGTH);
%     %     h6 = drawLoad(x_real(1:3,i));
%         drawnow;
%     %     frame = getframe(gcf);
%     %     writeVideo(v,frame);
%     end
%     % close(v);
end

%% SLQ
for ii=1:simulations
    parameters;
    [x_trajec_optimal_SLQ, u_trajec_optimal_SLQ, time, cost_new] = SLQ(x(:,1), [], SLQ_param);
    figure(3);
    hold on;
    for i=1:n_obstacle
        R = [r(i) r(i)];
        n_ = 200;
        [X, Y, Z] = cylinder(R,n_);
        X = X + pc(1,i);
        Y = Y + pc(2,i);
        Z(2,:) = 1.0;
        pos = [2 4 2 2]; 
        rectangle('Position',[pc(1,i)-0.4 pc(2,i)-0.4 0.8 0.8],'Curvature',[1 1],'FaceColor', [221 255 151]/255);
    end
    plot3(x_trajec_optimal_SLQ(1,:),x_trajec_optimal_SLQ(2,:),x_trajec_optimal_SLQ(3,:)); hold on;
    axis equal;
    axis([-4.0 4.0 -3 3 -1 2]);
    view([0 90]);
    xlabel('x(m)');
    ylabel('y(m)');
    zlabel('z(m)');
    set(gca,'Color',[1 1 1]);
    drawnow;
end