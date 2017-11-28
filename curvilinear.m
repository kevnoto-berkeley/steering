%%
clc; clear all; close all;
%% Track
%total is 12.42 at scale = 1
s = 1:.1:12.42;
s = s*100;
psi = arrayfun(@track,s);
N = numel(s);
x = zeros(N,1);
y = zeros(N,1);
for i=1:N-1
    x(i+1) = x(i) + cos(psi(i));
    y(i+1) = y(i) + sin(psi(i));
end
figure(1)
plot(x,y,'k--')
axis equal

%% MPC Parameters for short-sighted solution
% open-loop iterations
N = 25; 
% timestep
dt = .5; 
% velocity
v = 50; 
% initial conditions
z0 = [0,0,0]'; 
% short-sighted horizon
M = 5;
% Iteratively solve and propogate solution
xOpt = [];
uOpt = [];
xOpt = [xOpt, z0];
z_ol_ref = z0;
for i = 1:N
    z_next = [v*dt+z_ol_ref(1,i);0;track(v*dt+z_ol_ref(1,i))];
    z_ol_ref = [z_ol_ref z_next];
end
for j = 1:N-M
    fprintf('Solving t=%d\n', j)
    z_ref = z0;
    for i = 1:M
        z_next = [v*dt+z_ref(1,i); 0; track(v*dt+z_ref(1,i))];
        z_ref = [z_ref z_next];
    end
    [f,x,u,j] = solve_car(z0,v,dt,M);
    if f == false
        disp('infeasible!')
        break
    end
    z0 = x(:,2);
    xOpt = [xOpt, z0];
    uOpt = [uOpt, u(:,2)]; 
end
%% plot
sol = xOpt;
u = uOpt;
z_ref = z_ol_ref;
figure
subplot(3,1,1)
plot(z_ref(1,:))
hold on
plot(sol(1,:))
ylabel('s')
subplot(3,1,2)
plot(z_ref(2,:))
hold on
plot(sol(2,:))
ylabel('x')
subplot(3,1,3)
plot(z_ref(3,:))
hold on
plot(sol(3,:))
ylabel('psi')
figure
plot(arrayfun(@radtodeg,u),'bo-')
%%
s = 0:dt*v:12.4*100*2;
psi = arrayfun(@track,s);
n = size(xOpt,2);
x_opt = zeros(n,1);
y_opt = zeros(n,1);
x = zeros(n,1);
y = zeros(n,1);
for i = 1:n-1
    x_opt(i+1) = x_opt(i) + v*dt*cos(xOpt(3,i+1));
    y_opt(i+1) = y_opt(i) + v*dt*sin(xOpt(3,i+1));
    x(i+1) = x(i) + v*dt*cos(psi(i));
    y(i+1) = y(i) + v*dt*sin(psi(i));
end
figure
plot(x,y,'kx--','linewidth',2)
axis equal
hold on
plot(x_opt,y_opt,'bo-')
