function conditionTest1(x_trajectory)

    length = size(x_trajectory,2);
    result = zeros(1,length);
    for i=1:length
        p_now = x_trajectory(7:9,i);
        omega_now = x_trajectory(16:18,i);
        result(i) = dot(p_now, omega_now);
    end
    figure;
    plot(1:length, result, 1:length, zeros(1,length),'-r');
end
