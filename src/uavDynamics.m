function dX = cartPoleDynamics(X, f)
% global m l g Ixx Iyy Izz cT cM
% T = (f(1)+f(2)+f(3)+f(4));
% taox = sqrt(2) / 2 * l * (-f(1) + f(2) + f(3) - f(4));
% taoy = sqrt(2) / 2 * l * (-f(1) - f(2) + f(3) + f(4));
% taoz = 0.01145 * (-f(1) + f(2) - f(3) + f(4));
% x = X(1);
% y = X(2);
% z = X(3);
% vx = X(4);
% vy = X(5);
% vz = X(6);
% alpha = X(7);
% beta = X(8);
% gama = X(9);
% p = X(10);
% q = X(11);
% r = X(12);               
% % full dynamics:
% dx = vx;
% dy = vy;
% dz = vz;
% dvx =  (T / m) * (cos(alpha)*sin(beta)*cos(gama) + sin(alpha)*sin(gama));
% dvy =  (T / m) * (cos(alpha)*sin(beta)*sin(gama) - sin(alpha)*cos(gama));
% dvz = (T / m) * cos(alpha) * cos(beta) - g;
% dalpha = p + sin(alpha)*tan(beta)*q + cos(alpha)*tan(beta)*r;
% dbeta = cos(alpha)*q - sin(alpha)*r;
% dgama = (sin(alpha)/cos(beta))*q + (cos(alpha)/cos(beta))*r;
% dp = (1 / Ixx) * (taox + q * r *(Iyy - Izz));
% dq = (1 / Iyy) * (taoy + p * r * (Izz - Ixx));
% dr = (1 / Izz) * (taoz + p * q * (Ixx - Iyy));
% 
% dX = [dx;dy;dz; 
%       dvx; dvy; dvz;
%       dalpha; dbeta; dgama;
%       dp; dq; dr];
%% 原来状态
x = X(1);
vx = X(2);
%% 现有输入f 求增量
dx = vx;
dvx = f(1);
dX = [dx;
      dvx];
end