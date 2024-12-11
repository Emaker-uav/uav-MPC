function dX = uavDynamics(X, f)
global m g
alpha = f(1);
beta = f(2);
gama = f(3);
T = f(4);
x = X(1);
y = X(2);
z = X(3);
vx = X(4);
vy = X(5);
vz = X(6);              
dx = vx;
dy = vy;
dz = vz;
dvx = (T / m)*(cos(alpha*(pi/180))*sin(beta*(pi/180))*cos(gama*(pi/180))+sin(alpha*(pi/180))*sin(gama*(pi/180)));
dvy = (T / m)*(cos(alpha*(pi/180))*sin(beta*(pi/180))*sin(gama*(pi/180))-sin(alpha*(pi/180))*cos(gama*(pi/180)));
dvz = (T / m)*cos(alpha*(pi/180))*cos(beta*(pi/180)) - g;

dX = [dx;dy;dz; 
      dvx; dvy; dvz];
end