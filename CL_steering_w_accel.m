%%
clc; clear all; close all;
%% Track
%total is 12.42 at scale = 1
s = 1:.5:12.42;
s = s*100;
psi = arrayfun(@track,s);
% plot(psi);

%% MPC Parameters for short-sighted solution
% open-loop iterations
N = 40; 
% timestep
dt = .5; 
% velocity
v = repmat(50,1,N+1); 
% initial conditions
z0 = [0,0,0]'; 
% short-sighted horizon
M = 8;
% Iteratively solve and propogate solution
xOpt = [];
uOpt = [];
xOpt = [xOpt, [z0;v(1)]];
z_ol_ref = [z0;v(1)];
for i = 1:N
    z_next = [v(i)*dt+z_ol_ref(1,i);0;0;v(i)];
    z_ol_ref = [z_ol_ref z_next];
end
for j = 1:N-M
    fprintf('Solving t=%d\n', j)
    [f,x,u,jo] = solve_car_w_accel(z0(1:3),v,dt,M);
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
subplot(4,1,1)
plot(z_ref(1,:))
hold on
plot(sol(1,:))
ylabel('s')
subplot(4,1,2)
plot(z_ref(2,:))
hold on
plot(sol(2,:))
ylabel('x')
subplot(4,1,3)
plot(z_ref(3,:))
hold on
plot(sol(3,:))
ylabel('psi')
subplot(4,1,4)
plot(z_ref(4,:))
hold on
plot(sol(4,:))
ylabel('v')
figure
subplot(1,1,1)
plot(arrayfun(@radtodeg,u(1,:)),'bo-')
ylabel('Steering input in degrees')
% subplot(2,1,2)
% plot(u(2,:),'bo-')
% ylabel('Acceleration')
%%
s = 0:dt*v(1):12.4*100*2;
psi = arrayfun(@track,s);
n = size(xOpt,2);
x_opt = zeros(n,1);
y_opt = zeros(n,1);
x = zeros(n,1);
y = zeros(n,1);
for i = 1:n-1
    x_opt(i+1) = x_opt(i) + xOpt(4,i+1)*dt*cos(psi(i)+xOpt(3,i+1));
    y_opt(i+1) = y_opt(i) + xOpt(4,i+1)*dt*sin(psi(i)+xOpt(3,i+1));
    x(i+1) = x(i) + v(1)*dt*cos(psi(i));
    y(i+1) = y(i) + v(1)*dt*sin(psi(i));
end
figure
plot(x,y,'kx--','linewidth',1)
axis equal
hold on
plot(x_opt,y_opt,'bo-')
