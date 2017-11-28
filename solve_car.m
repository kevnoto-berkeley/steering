function [feas, xOpt, uOpt, JOpt] = solve_car(z0, v, dt, N)
% Add angle and acceleration to inputs
% make it maximize distance traveled

% 

z_ref = z0;
for i = 1:N
    psi = track(z_ref(1,i));
    z_next = [v*dt + z_ref(1,i); 0; psi];
    z_ref = [z_ref z_next];
end

objectives = 0;

z = sdpvar(3,N);
u = sdpvar(1,N);
opts = sdpsettings('solver','ipopt','verbose',0);
constraints = [z(:,1) == z0];

for i=1:N-1
    constraints = [constraints, z(:,i+1)==car_model_curvilinear(z(:,i),u(1,i),v,dt,z(3,i)-z_ref(3,i))];
    constraints = [constraints, -3.7/2 <= z(2,i) <= 3.7/2];
    constraints = [constraints, -degtorad(10)*dt <= (u(i+1)-u(i)) <= degtorad(10)*dt];
end

for i = 1:N
    objectives = objectives + (z(1,i)-z_ref(1,i))^2 + (z(2,i))^2 + (z(3,i)-z_ref(3,i))^2;
end
status = optimize(constraints, objectives, opts);

feas = ~status.problem;
if feas == 1
    xOpt = value(z);
    uOpt = value(u);
    JOpt = value(objectives);
else
    xOpt = []; uOpt = []; JOpt = 0;
end
