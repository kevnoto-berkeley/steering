%% The Course
clc;clear all; close all;
N = 10;
dt = .1;
v = 10;
z0 = [0,0,0]';
Y_ref = zeros(N,1);
psi_ref = zeros(N,1);
for i=1:N
    Y_ref(i+1) = Y_ref(i) + v*dt;
    psi_ref(i+1) = road(Y_ref(i+1));
end

X_ref = zeros(N,1);
Y_ref = zeros(N,1);
for i=1:N-1
    X_ref(i+1) = X_ref(i) + v*dt*cos(psi_ref(i));
    Y_ref(i+1) = Y_ref(i) + v*dt*sin(psi_ref(i));
end

%% MPC Solve 1 step

objectives = 0;

z = sdpvar(3,N);
u = sdpvar(1,N);
opts = sdpsettings('solver','ipopt');
constraints = [z(:,1) == z0];
disp('Constraints')
for i=1:N-1
    constraints = [constraints, z(:,i+1)==bikeFE(z(:,i),u(1,i),v,dt)];
    constraints = [constraints, abs(u(i+1)-u(i)) <= 0.05];
end
disp('Objectives')
for i =1 :N
    objectives = objectives + (z(1,i)-X_ref(i))^2 + (z(2,i)-Y_ref(i))^2 + (z(3,i)-psi_ref(i))^2;
end
optimize(constraints, objectives, opts)
%% plot
sol = value(z);

figure
subplot(3,1,1)
plot(X_ref)
hold on
plot(sol(1,:))
ylabel('x')
subplot(3,1,2)
plot(Y_ref)
hold on
plot(sol(2,:))
ylabel('y')
subplot(3,1,3)
plot(psi_ref)
hold on
plot(sol(3,:))
ylabel('psi')
figure
plot(arrayfun(@radtodeg,value(u)),'bo-')

figure
plot(X_ref,Y_ref,'b--')
hold on
plot(sol(1,:),sol(2,:),'r-')
