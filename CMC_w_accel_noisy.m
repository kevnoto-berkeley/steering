function [z_next] = CMC_w_accel_noisy(z,u,dt,delta_psi)
% Car Model, Curvilinear, with acceleration
% z = [s, x, psi, v]
% s = curvilinear distance
% x = lateral distance from centerline
% psi = heading, absolute reference

% v = car velocity, curvilinear

% u = [d, a]
% d = steering angle, radians
% a = acceleration, curvilinear

% dt = time step
% psi_err = current heading error
lr = 2.25;
psiDot = z(4)/lr * sin(u(1));

psi = z(3) + psiDot*dt - delta_psi;

s = z(1) + z(4)*dt*cos(psi) + normrnd(0,1);
x = z(2) + z(4)*dt*sin(psi) + normrnd(0,.05);
% v = z(4) + u(2)*dt + normrnd(0,.5);

z_next = [s, x, psi, z(4)]';