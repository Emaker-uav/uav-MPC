clc;
clear;

%% cart-pole properties
global m l g Ixx Iyy Izz cT cM gJ u0
m = 1.4;
l = 0.2;
Ixx = 0.0211;
Iyy = 0.0219;
Izz = 0.0366;
cT = 1.201e-5;
cM = 1.574e-7;
g = 9.81;   
u0 = zeros(20,1);
%% cart-pole initial condition
X_state = [0;0;]; 
X_des = [ 0.5; 0.1;]; 
Q = diag([1000,10]);
R = diag(0.001);
N = 5; % # of horizons
dT_MPC = 0.01; % sec

% X_state = [0;0;0;
%            0;0;0;
%            0;0;0;
%            0;0;0]; 
% X_des = [0.01;0;0.5;
%          0;0;0;
%          0;0;0;
%          0;0;0]; 
% Q = diag([1,1,2000,1,1,10,1,1,1,1,1,1]);
% R = diag([0.001,0.001,0.001,0.001]);
% N = 5; 
% dT_MPC = 0.01; 
%% Controllers
t = 0;




maxPoints = 200; % 最大显示的点数
numStates = 12; % 状态数量
numFu = 4;
rows = 4; % 子图行数
cols = 3; % 子图列数
timeData = cell(numStates, 1); % 每个状态的时间存储
FutimeData = cell(numFu, 1);
plotData = cell(numStates, 1); % 每个状态的值存储
hPlots = gobjects(numStates, 1); % 每个状态对应的绘图对象
desData = cell(numStates, 1);
desPlots = gobjects(numStates, 1);
figure;
for i = 1:numStates
    subplot(rows, cols, i); 
    hPlots(i) = plot(nan, nan, 'b-'); 
    hold on;
    desPlots(i) = plot(nan , nan, 'r-');
    grid on;
end
fuData = cell(numFu,1);
fuPlots = gobjects(numFu,1);
figure
for i = 1:numFu
    subplot(4, 1, i); 
    fuPlots(i) = plot(nan, nan, 'b-'); 
    hold on;
    grid on;
end
while true  
    Fu  = uavNMPC(X_state,X_des,Q,R,N,dT_MPC);
    derx = uavDynamics(X_state, Fu);
    X_state = X_state + 0.01*derx;
    pause(dT_MPC);

    for i = 1:numStates
        % 更新数据
        if isempty(timeData{i})
            timeData{i} = t; % 初始化时间数据
            plotData{i} = X_state(i); % 初始化状态数据
            desData{i} = X_des(i);
        else
            timeData{i} = [timeData{i}, t]; % 添加时间点
            plotData{i} = [plotData{i}, X_state(i)]; % 添加对应状态的值
            desData{i} = [desData{i},X_des(i)];
        end

        % 保留最近 maxPoints 个数据
        if length(timeData{i}) > maxPoints
            timeData{i} = timeData{i}(end-maxPoints+1:end);
            plotData{i} = plotData{i}(end-maxPoints+1:end);
            desData{i} = desData{i}(end-maxPoints+1:end);
        end

        % 更新子图
        set(hPlots(i), 'XData', timeData{i}, 'YData', plotData{i});
        set(desPlots(i), 'XData', timeData{i}, 'YData', desData{i});
    end

    for i = 1:numFu
        % 更新数据
        if isempty(FutimeData{i})
            FutimeData{i} = t; % 初始化时间数据
            fuData{i} = Fu(i); % 初始化状态数据
        else
            FutimeData{i} = [FutimeData{i}, t]; % 添加时间点
            fuData{i} = [fuData{i}, Fu(i)]; % 添加对应状态的值
        end
    
        if length(FutimeData{i}) > maxPoints
            FutimeData{i} = FutimeData{i}(end-maxPoints+1:end);
            fuData{i} = fuData{i}(end-maxPoints+1:end);
        end
        set(fuPlots(i), 'XData', FutimeData{i}, 'YData', fuData{i}); 
    end
    % 刷新图像
    drawnow;
    t = t + dT_MPC;



    %% xyz
%     figure(1);
%     subplot(3, 4, 1);
%     hold on;  % ����֮ǰ�Ļ�ͼ
%     plot(t, X_state(1), 'b*');  % ��̬���ݵ�
%     plot(t, X_des(1), 'r*'); % �ο���
% %     ylim([-5 5]);
%     title('X State(x)');
%     xlabel('Time');
%     ylabel('Value');
%     grid on;
%     hold off;
% 
%     subplot(3, 4, 5);
%     hold on;  % ����֮ǰ�Ļ�ͼ
%     plot(t, X_state(2), 'b*');  % ��̬���ݵ�
%     plot(t, X_des(2), 'r*'); % �ο���
% %     ylim([-5 5]);
%     title('X State(y)');
%     xlabel('Time');
%     ylabel('Value');
%     grid on;
%     hold off;
% 
%     subplot(3, 4, 9);
%     hold on;  % ����֮ǰ�Ļ�ͼ
%     plot(t, X_state(3), 'b*');  % ��̬���ݵ�
%     plot(t, X_des(3), 'r*'); % �ο���
% %     ylim([-5 5]);
%     title('X State(z)');
%     xlabel('Time');
%     ylabel('Value');
%     grid on;
%     hold off;
% 
%     %% vxvyvz
%     subplot(3, 4, 2);
%     hold on;  % ����֮ǰ�Ļ�ͼ
%     plot(t, X_state(4), 'b*');  % ��̬���ݵ�
%     plot(t, X_des(4), 'r*'); % �ο���
% %     ylim([-5 5]);
%     title('X State(vx)');
%     xlabel('Time');
%     ylabel('Value');
%     grid on;
%     hold off;
% 
%     subplot(3, 4, 6);
%     hold on;  % ����֮ǰ�Ļ�ͼ
%     plot(t, X_state(5), 'b*');  % ��̬���ݵ�
%     plot(t, X_des(5), 'r*'); % �ο���
% %     ylim([-5 5]);
%     title('X State(vy)');
%     xlabel('Time');
%     ylabel('Value');
%     grid on;
%     hold off;
% 
%     subplot(3, 4, 10);
%     hold on;  % ����֮ǰ�Ļ�ͼ
%     plot(t, X_state(6), 'b*');  % ��̬���ݵ�
%     plot(t, X_des(6), 'r*'); % �ο���
% %     ylim([-5 5]);
%     title('X State(vz)');
%     xlabel('Time');
%     ylabel('Value');
%     grid on;
%     hold off;
%     %% alpha beta gama
%     subplot(3, 4, 3);
%     hold on;  % ����֮ǰ�Ļ�ͼ
%     plot(t, X_state(7), 'b*');  % ��̬���ݵ�
%     plot(t, X_des(7), 'r*'); % �ο���
% %     ylim([-5 5]);
%     title('X State(alpha)');
%     xlabel('Time');
%     ylabel('Value');
%     grid on;
%     hold off;
% 
%     subplot(3, 4, 7);
%     hold on;  % ����֮ǰ�Ļ�ͼ
%     plot(t, X_state(8), 'b*');  % ��̬���ݵ�
%     plot(t, X_des(8), 'r*'); % �ο���
% %     ylim([-5 5]);
%     title('X State(beta)');
%     xlabel('Time');
%     ylabel('Value');
%     grid on;
%     hold off;
% 
%     subplot(3, 4, 11);
%     hold on;  % ����֮ǰ�Ļ�ͼ
%     plot(t, X_state(9), 'b*');  % ��̬���ݵ�
%     plot(t, X_des(9), 'r*'); % �ο���
% %     ylim([-5 5]);
%     title('X State(gama)');
%     xlabel('Time');
%     ylabel('Value');
%     grid on;
%     hold off;
%     %% pqr
%     subplot(3, 4, 4);
%     hold on;  % ����֮ǰ�Ļ�ͼ
%     plot(t, X_state(10), 'b*');  % ��̬���ݵ�
%     plot(t, X_des(10), 'r*'); % �ο���
% %     ylim([-5 5]);
%     title('X State(p)');
%     xlabel('Time');
%     ylabel('Value');
%     grid on;
%     hold off;
% 
%     subplot(3, 4, 8);
%     hold on;  % ����֮ǰ�Ļ�ͼ
%     plot(t, X_state(11), 'b*');  % ��̬���ݵ�
%     plot(t, X_des(11), 'r*'); % �ο���
% %     ylim([-5 5]);
%     title('X State(q)');
%     xlabel('Time');
%     ylabel('Value');
%     grid on;
%     hold off;
% 
%     subplot(3, 4, 12);
%     hold on;  % ����֮ǰ�Ļ�ͼ
%     plot(t, X_state(12), 'b*');  % ��̬���ݵ�
%     plot(t, X_des(12), 'r*'); % �ο���
% %     ylim([-5 5]);
%     title('X State(r)');
%     xlabel('Time');
%     ylabel('Value');
%     grid on;
%     hold off;
%     %% �������
%     figure(2);
%     for i = 1 : size(R,1)
%         subplot(size(R,1), 1, i);
%         hold on;  % ����֮ǰ�Ļ�ͼ
%         plot(t, Fu(i), 'b*');  % ��̬���ݵ�
%         xlabel('Time');
%         ylabel('Value');
%         grid on;
%         hold off;
%     end
%     drawnow;
end