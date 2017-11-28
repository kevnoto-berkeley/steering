function [z_next] = car_model_curvilinear(z,u,v,dt,psi_err)
% z = [ s, x, psi]
% u = [d]
% d = steering angle relative to curvilinear tangent at s
% s = curvilinear distance
% x = perpendicular distance
% psi = relative heading to curvilinear tangent at s
% v = velocity
% dt = timestep
lr = 2.25;

psiDot = v/lr * sin(u(1));

s = z(1) + v*dt*cos(psi_err);
x = z(2) + v*dt*sin(psi_err);
psi = z(3) + psiDot*dt;

z_next = [s, x, psi]';