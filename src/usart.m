classdef usart
    properties
        SerialObj   % 串口对象
    end

    methods
        % 构造函数，初始化串口
        function obj = usart(portName, baudRate)
            obj.SerialObj = serialport(portName, baudRate);
            configureCallback(obj.SerialObj, "byte", 60, @(src, ~) obj.U1_Data_Receive(src));
            fprintf("串口已打开: %s, 波特率: %d\n", portName, baudRate);
        end

        % 数据接收回调函数
        function U1_Data_Receive(obj, serialObj)
            persistent u1_rxstate u1_rxbuf u1_data_len u1_data_cnt;
            global rst;
            if isempty(u1_rxstate)
                u1_rxstate = 0;
                u1_rxbuf = uint8([]);
                u1_data_len = 0;
                u1_data_cnt = 0;
            end

            % 读取串口数据
            data = read(serialObj, serialObj.NumBytesAvailable, "uint8");
            if(rst == 1)
                rst = 0;
                % 状态机实现数据解析
                for i = 1:length(data)
                    byte = data(i);
                    switch u1_rxstate
                        case 0
                            if byte == 0xAA
                                u1_rxstate = 1;
                                u1_rxbuf = uint8(byte); % 初始化数据缓冲区
                            end
                        case 1
                            if byte == 0xFF
                                u1_rxstate = 2;
                                u1_rxbuf(end+1) = byte;
                            else
                                u1_rxstate = 0;
                            end
                        case 2
                            u1_rxstate = 3;
                            u1_rxbuf(end+1) = byte; % 保存CMD字节
                        case 3
                            if byte < 50
                                u1_rxstate = 4;
                                u1_rxbuf(end+1) = byte; % 保存数据长度
                                u1_data_len = byte;
                                u1_data_cnt = 0;
                            else
                                u1_rxstate = 0;
                            end
                        case 4
                            if u1_data_len > 0
                                u1_data_len = u1_data_len - 1;
                                u1_rxbuf(end+1) = byte; % 保存数据
                                u1_data_cnt = u1_data_cnt + 1;
                            end
                            if u1_data_len == 0
                                u1_rxstate = 5;
                            end
                        case 5
                            u1_rxstate = 6;
                            u1_rxbuf(end+1) = byte; % 保存校验字节sc
                        case 6
                            u1_rxstate = 0;
                            u1_rxbuf(end+1) = byte; % 保存校验字节ac
                            % 调用数据解析函数
                            obj.U1_Data_Anl(u1_rxbuf);
                            % 清空缓冲区
                            u1_rxbuf = uint8([]);
                        otherwise
                            u1_rxstate = 0;
                    end
                end
            end
        end

        % 数据解析函数
        function U1_Data_Anl(obj, data)
%             keyboard;
            global datastruct;
            len = length(data);
            if len < 6
                return; % 数据长度不足
            end
            % 校验逻辑
            sc = uint32(0);  % 使用 uint16 来存储中间结果
            ac = uint32(0);  % 使用 uint16 来存储中间结果
            for i = 1:len-2
                sc = sc + uint32(data(i));  % 将 data(i) 显式转换为 uint16 类型进行加法
                ac = ac + sc;
            end
            % 最终结果转换为 uint8 并只保留低八位
            sc = uint8(bitand(sc, 255));  % 保留低八位并转换回 uint8
            ac = uint8(bitand(ac, 255));  % 保留低八位并转换回 uint8
            if sc ~= data(len-1) || ac ~= data(len)
                fprintf("校验失败\n");
                return;
            end
            % 数据头校验
            if data(1) ~= 0xAA || data(2) ~= 0xFF
                fprintf("数据头错误\n");
                return;
            end

            % 数据解析
            cmd = data(3); % 获取CMD字节
            switch cmd
                case 0xF9
                    datastruct.pit = typecast(data(5:8), 'single');  % MATLAB 索引从 1 开始，所以 +4 对应的是 5 到 8
                    datastruct.rol = typecast(data(9:12), 'single');  % 对应 data+8
                    datastruct.yaw = typecast(data(13:16), 'single');  % 对应 data+12
                    datastruct.x = typecast(data(17:20), 'single');  % 对应 data+16
                    datastruct.y = typecast(data(21:24), 'single');  % 对应 data+20
                    datastruct.z = typecast(data(25:28), 'single');  % 对应 data+24
                    datastruct.vx = typecast(data(29:32), 'single');  % 对应 data+16
                    datastruct.vy = typecast(data(33:36), 'single');  % 对应 data+20
                    datastruct.vz = typecast(data(37:40), 'single');  % 对应 data+24
                    fprintf('Pitch: %.4f\n', datastruct.pit);
                    fprintf('Roll: %.4f\n', datastruct.rol);
                    fprintf('Yaw: %.4f\n', datastruct.yaw);
                    fprintf('X: %.4f\n', datastruct.x);
                    fprintf('Y: %.4f\n', datastruct.y);
                    fprintf('Z: %.4f\n', datastruct.z);
                    fprintf('VX: %.4f\n', datastruct.vx);
                    fprintf('VY: %.4f\n', datastruct.vy);
                    fprintf('VZ: %.4f\n', datastruct.vz);
%                     fprintf("收到复位命令\n");
                    obj.U1_Check_Reset();
                otherwise
                    fprintf("未知命令: 0x%X\n", cmd);
            end
        end

        % 模拟复位命令的处理函数
        function U1_Check_Reset(obj)
%             fprintf("复位指令已处理\n");
        end
    end
end
