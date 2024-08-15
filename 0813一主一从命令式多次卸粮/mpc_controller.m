function mpc_controller
    % 初始参数设置
    dt = 0.01; % 离散时间步长
    N = 20; % 预测时域长度
    model = @(x, u) vehicle_model(x, u, dt); % 车辆动力学模型
    weights.tracking_error = 1e2; % 路径跟踪误差权重
    weights.input_rate = 1; % 输入变化率权重
    constraints = []; % 假设没有控制输入和状态的约束

    % 初始状态 [x, y, theta, v]，这里需要根据实际情况设置
    x0 = [0, 0, 0, 10]; % 车辆初始位置、朝向和速度

    % 路径点，这里使用之前生成的路径点 r
    global r;
    path_x = r(:, 2);
    path_y = r(:, 3);
    path_points = [path_x'; path_y'];

    % 循环调用MPC控制器
    for i = 1:N
        % 预测当前时刻的控制输入
        [u, fval, exitflag, output] = fmincon(@(u) mpc_objective(x0, u, model, path_points, weights, N), ...
                                               zeros(1, N), [], [], [], [], [], [], constraints);

        % 应用控制输入
        x0 = model(x0, u);

        % 绘制跟踪效果
        plot(x0(1), x0(2));
        hold on;
        plot(path_x, path_y, 'r');
        drawnow;

        % 更新状态
        x0 = [x0(1:2) + x0(4) * cos(x0(3)) * dt, x0(3:4)];
    end
end

function dx = vehicle_model(x, u, dt)
    % 简单的自行车模型
    v = x(4);
    dx = [x(4) * cos(x(3)); x(4) * sin(x(3)); u; 0];
end

function cost = mpc_objective(x0, u, model, path_points, weights, N)
    % 初始化预测轨迹和成本
    X = zeros(4, N+1);
    U = zeros(1, N);
    X(:, 1) = x0;
    cost = 0;

    % 预测轨迹
    for i = 1:N
        U(i) = u(i);
        X(:, i+1) = model(X(:, i), U(i), dt);
    end

    % 计算成本函数
    for i = 1:N
        % 计算路径跟踪误差
        error = path_points(1:2:end, mod(i, size(path_points, 2))+1) - X(1:2, i+1);
        cost = cost + norm(error) * weights.tracking_error;

        % 计算控制输入变化率
        if i > 1
            cost = cost + (U(i) - U(i-1))^2 * weights.input_rate;
        end
    end
end