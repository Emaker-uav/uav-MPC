function Fu = uavNMPC(X_states,X_des,Q,R,N,dT_MPC)
tic
global u0;
X_ref = generateReference(X_states,X_des,N);
poslim = [-10;10;
          -10;10;
          -10;10];
nonlcon = @(u) stateConstraints(u, X_states, N, dT_MPC, 0.5,poslim);
fun = @(u)costFunction(u,X_states,X_ref, Q, R, N,dT_MPC);
options = optimoptions('fmincon','Display','iter','Algorithm','sqp');
lb = [-10;
      -10;
      -180;
      0;
      -10;
      -10;
      -180;
      0;
      -10;
      -10;
      -180;
      0;
      -10;
      -10;
      -180;
      0;
      -10;
      -10;
      -180;
      0;];
ub = [10;
      10;
      180;
      25;
      10;
      10;
      180;
      25;
      10;
      10;
      180;
      25;
      10;
      10;
      180;
      25;
      10;
      10;
      180;
      25];
Fu = fmincon(fun,u0,[],[],[],[],lb,ub,nonlcon,options);
u0 = Fu;
toc
end
%% 
function x_ref = generateReference(X_state, X_des, h)
    % 初始化参考路径矩阵
    x_ref = zeros(size(X_state, 1), h);
    
    % 为每个维度（x, y, z）生成贝塞尔曲线
    for i = 1:size(X_state, 1)
        x_ref(i, :) = bezierPath(X_state(i), X_des(i), h);
    end
end

% 三次贝塞尔曲线生成
function path = bezierPath(start_value, end_value, num_points)
    % 设定四个控制点
    P0 = start_value;  % 起始点
    P1 = start_value + 0.5 * (end_value - start_value); % 第一个控制点
    P2 = start_value + 0.5 * (end_value - start_value); % 第二个控制点
    P3 = end_value;  % 终点

    % 生成t参数
    t = linspace(0, 1, num_points);

    % 计算贝塞尔曲线路径
    path = (1 - t).^3 * P0 + 3 * (1 - t).^2 .* t * P1 + 3 * (1 - t) .* t.^2 * P2 + t.^3 * P3;
end

function [c, ceq] = stateConstraints(u, X_states, N, dT_MPC, v_max,pos_limits)
    % 初始化约束
    c = [];
    ceq = [];
    
    % 初始状态
    X_current = X_states;
    % 解包位置约束 [xmin, xmax, ymin, ymax, zmin, zmax]
    xmin = pos_limits(1); xmax = pos_limits(2);
    ymin = pos_limits(3); ymax = pos_limits(4);
    zmin = pos_limits(5); zmax = pos_limits(6);
    % 循环预测时域
    for k = 1:N
        % 当前输入
        u_k = u(4*(k-1)+1 : 4*k);
        
        % 根据系统动力学更新状态
        derx = uavDynamics(X_current, u_k);
        X_current = X_current + dT_MPC * derx;
        % 提取速度 (vx, vy, vz)
        x = X_current(1);
        y = X_current(2);
        z = X_current(3);
        vx = X_current(4);
        vy = X_current(5);
        vz = X_current(6);
        
        % 添加速度约束：-v_max <= v <= v_max
        c = [c; vx - v_max; -vx - v_max; vy - v_max; -vy - v_max; vz - v_max; -vz - v_max];
        % 添加位置约束：xmin <= x <= xmax, ymin <= y <= ymax, zmin <= z <= zmax
        c = [c; x - xmax; xmin - x; y - ymax; ymin - y; z - zmax; zmin - z];
    end
end
