clc;
clear;

%% 串口配置
% 初始化串口通信
% 设置串口端口和波特率
portName = "COM64"; % 替换为实际串口号
baudRate = 500000;   % 设置波特率
u = usart(portName, baudRate);

%% 无人机接收信息帧
global datastruct;
datastruct.pit = single(0);
datastruct.rol = single(0);
datastruct.yaw = single(0);
datastruct.x = single(0);
datastruct.y = single(0);
datastruct.z = single(0);
datastruct.vx = single(0);
datastruct.vy = single(0);
datastruct.vz = single(0);

%% 无人机参数
global m  g  u0 rst;
m = 1.7;
g = 9.81;   
u0 = zeros(20,1);
rst = 0;
count = uint32(0);

%% 无人机初状态以及权重设置
X_state = [0; 0; 0; 0; 0; 0]; 
X_des = [1; 1; 6; 0; 0; 0]; 
Q = diag([400, 400, 800, 10, 10, 10]);
R = diag([0.001, 0.001, 0.001, 0.001]);
N = 5; 
dT_MPC = 0.01;

%% 图像显示设置
maxPoints = 100; % 最大显示的点数
t = 0;

numStates = 6; % 状态数量
rows = 2; % 子图行数
cols = 3; % 子图列数
timeData = cell(numStates, 1); % 每个状态的时间存储
plotData = cell(numStates, 1); % 每个状态的值存储
refData = cell(numStates, 1);
hAxes = gobjects(numStates, 1); % Axes 对象数组
hPlots = gobjects(numStates, 1); % 每个状态对应的绘图对象
refPlots = gobjects(numStates, 1);

for i = 1:numStates
    hAxes(i) = subplot(rows, cols, i); % 创建子图
    hPlots(i) = plot(nan, nan, 'b-'); % 创建每个子图的绘图对象
    hold on;
    refPlots(i) = plot(nan, nan, 'r-');
    hold on;
    xlabel('Time (s)');
    ylabel(['State ', num2str(i)]);
    title(['X\_state(', num2str(i), ')']);
    grid on;
end

figure;
numFu = 4;
rows2 = 4; % 子图行数
cols2 = 1; % 子图列数
time2Data = cell(numFu,1);
plot2Data = cell(numFu,1);
fPlots = gobjects(numFu,1);
for i = 1:numFu
    subplot(rows2, cols2, i); % 创建子图
    fPlots(i) = plot(nan, nan, 'b-'); % 创建每个子图的绘图对象
    grid on;
end

% 创建 3D 图形窗口
trackPoints = []; % 实际轨迹点
figure;
hold on;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Trajectory Tracking');
view(3); % 设置3D视角
axis equal;

% 绘制静态期望点
plot3(X_state(1), X_state(2), X_state(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'g'); % 期望位置点
plot3(X_des(1), X_des(2), X_des(3), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r'); % 期望位置点

% 创建动态轨迹曲线
hTrack = plot3(nan, nan, nan, 'b-', 'LineWidth', 1.5); % 实际轨迹曲线

% 定义无人机机臂长度
arm_length = 0.5; % 每个机臂的长度

% 四个机臂在局部坐标系的端点
arms = arm_length * [
    1,  1, 0;  % 第一根机臂 (X形右上)
   -1, -1, 0;  % 第二根机臂 (X形左下)
    1, -1, 0;  % 第三根机臂 (X形右下)
   -1,  1, 0   % 第四根机臂 (X形左上)
]';

% 定义螺旋桨的半径
propeller_radius = 0.3;
propeller_angle = 0;

% 创建四个螺旋桨的绘制对象
hPropellers = gobjects(4, 1);
for i = 1:4
    hPropellers(i) = plot3(nan, nan, nan, 'k-', 'LineWidth', 2);
end

while true  

    rst = 1;
    
    Fu  = uavNMPC(X_state, X_des, Q, R, N, dT_MPC);
    derx = uavDynamics(X_state, Fu);
    X_state = X_state + 0.01 * derx;
    t = t + 0.01;

    for i = 1:numStates
        % 更新数据
        if isempty(timeData{i})
            timeData{i} = t; % 初始化时间数据
            plotData{i} = X_state(i); % 初始化状态数据
            refData{i} = X_des(i);
        else
            timeData{i} = [timeData{i}, t]; % 添加时间点
            plotData{i} = [plotData{i}, X_state(i)]; % 添加对应状态的值
            refData{i} = [refData{i}, X_des(i)];
        end

        % 保留最近 maxPoints 个数据
        if length(timeData{i}) > maxPoints
            timeData{i} = timeData{i}(end-maxPoints+1:end);
            plotData{i} = plotData{i}(end-maxPoints+1:end);
            refData{i} = refData{i}(end-maxPoints+1:end);
        end

        % 更新子图
        set(hPlots(i), 'XData', timeData{i}, 'YData', plotData{i});
        set(refPlots(i), 'XData', timeData{i}, 'YData', refData{i});
        set(hAxes(i), 'YLim', [-0.5, 1]); % Y 轴范围（根据需要调整）
    end

    for i = 1:numFu
        % 更新数据
        if isempty(time2Data{i})
            time2Data{i} = t; % 初始化时间数据
            plot2Data{i} = Fu(i); % 初始化状态数据
        else
            time2Data{i} = [time2Data{i}, t]; % 添加时间点
            plot2Data{i} = [plot2Data{i}, Fu(i)]; % 添加对应状态的值
        end

        % 保留最近 maxPoints 个数据
        if length(time2Data{i}) > maxPoints
            time2Data{i} = time2Data{i}(end-maxPoints+1:end);
            plot2Data{i} = plot2Data{i}(end-maxPoints+1:end);
        end
        % 更新子图
        set(fPlots(i), 'XData', time2Data{i}, 'YData', plot2Data{i});
    end

    % 提取当前位置
    currentPoint = X_state(1:3)'; % 当前 [x, y, z] 位置
    trackPoints = [trackPoints; currentPoint]; % 添加到轨迹点集合

    % 更新轨迹曲线
    set(hTrack, 'XData', trackPoints(:, 1), 'YData', trackPoints(:, 2), 'ZData', trackPoints(:, 3));

    % 动态调整坐标轴范围（根据需要可扩展为动态调整）
    xlim([-1, 1]); ylim([-1, 1]); zlim([0, 1]);

    % 提取欧拉角（弧度制）
    alpha = datastruct.rol * pi/180; % 滚转角 (roll)
    beta = -datastruct.pit*pi/180;  % 俯仰角 (pitch)
    gamma = -datastruct.yaw*pi/180; % 偏航角 (yaw)
%     alpha = Fu(1) * pi/180; % 滚转角 (roll)
%     beta = Fu(2) * pi/180;  % 俯仰角 (pitch)
%     gamma = Fu(3) * pi/180; % 偏航角 (yaw)

    % 计算无人机的旋转矩阵 (XYZ 顺序)
    R_body = eul2rotm([alpha, beta, gamma], 'XYZ');

    % 转换机臂端点到世界坐标系
    arms_world = R_body * arms + X_state(1:3);

    % 绘制无人机框架
    if exist('hArms', 'var')
        delete(hArms);
    end
    hArms = plot3(... 
        [arms_world(1, [1, 2]), nan, arms_world(1, [3, 4])], ...
        [arms_world(2, [1, 2]), nan, arms_world(2, [3, 4])], ...
        [arms_world(3, [1, 2]), nan, arms_world(3, [3, 4])], ...
        'k-', 'LineWidth', 2); % 无人机 X 型

    % 更新螺旋桨的旋转位置
    for i = 1:4
        % 根据 Fu(4) 控制螺旋桨转速，设定旋转角度
        if i == 3 || i == 4  % 1号和3号螺旋桨为逆时针旋转
            propeller_angle = Fu(4) * t; % 逆时针旋转（正方向）
        else  % 2号和4号螺旋桨为顺时针旋转
            propeller_angle = -Fu(4) * t; % 顺时针旋转（反方向）
        end
        
        propeller_center = arms_world(:, i); % 螺旋桨中心位置
        propeller_offset = propeller_radius * [cos(propeller_angle); sin(propeller_angle); 0]; % 旋转位置
    
        % 更新螺旋桨的位置
        set(hPropellers(i), 'XData', propeller_center(1) + [-propeller_offset(1), propeller_offset(1)], ...
                             'YData', propeller_center(2) + [-propeller_offset(2), propeller_offset(2)], ...
                             'ZData', propeller_center(3) + [-propeller_offset(3), propeller_offset(3)]);
    end

    % 更新轨迹曲线
    currentPoint = X_state(1:3)'; 
    trackPoints = [trackPoints; currentPoint];
    set(hTrack, 'XData', trackPoints(:, 1), 'YData', trackPoints(:, 2), 'ZData', trackPoints(:, 3));

    % 动态调整坐标轴范围（根据需要调整）
    xlim([-3, 3] + X_state(1));
    ylim([-3, 3] + X_state(2));
    zlim([0, 10]);

    % 刷新图像
    drawnow;
end
