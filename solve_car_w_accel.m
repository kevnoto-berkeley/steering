function [feas, xOpt, uOpt, JOpt] = solve_car_w_accel(z0, v, dt, N)
% Solves MPC that minimizes reference error and achieves target velocity
% v is a vector of given velocities for N steps
if numel(z0) > 3
    disp('z0 needs to be a 3 state vector, velocity is handled via v')
    return
end
z0 = [z0;v(1)];
z_ref = [z0(1);z0(2);track(z0(1));z0(4)];
for i = 1:N
    psi = track(z_ref(1,i));
    z_next = [v(i)*dt + z_ref(1,i); 0; psi; v(i)];
    z_ref = [z_ref z_next];
end

objectives = 0;

z = sdpvar(4,N);
u = sdpvar(1,N);
opts = sdpsettings('solver','ipopt','verbose',0);
constraints = [z(:,1) == z0];

for i=1:N-1
    % Model constraints
    z_next = CMC_w_accel_noisy(z(:,i),u(:,i),dt,z_ref(3,i+1)-z_ref(3,i));
    constraints = [constraints, z(1:3,i+1)==z_next(1:3)];
    % Velocity match constraint
    constraints = [constraints, z(4,i+1) == v(i+1)];
    % Road size constraints, 3.7m wide road
    constraints = [constraints, -3.7/2 <= z(2,i) <= 3.7/2];
    % Heading error constraints, +/- 5 deg
    constraints = [constraints degtorad(-5) <= z(3,i) <= degtorad(5)];
    % Steering input constraints, 90 degrees motion
    constraints = [constraints, -degtorad(60) <= (u(1,i)) <= degtorad(60)];
    % Steering rate constraints, 10 deg/s
    constraints = [constraints, -degtorad(10)*dt <= (u(1,i+1)-u(1,i)) <= degtorad(10)*dt];
%     % Acceleration constraint, .5 m/s
%     constraints = [constraints, -2*dt <= u(2,i) <= 2*dt];
%     % Acceleration rate constraint, .5 m/s^2
%     constraints = [constraints, -.5*dt <= u(2,i+1) - u(2,i) <= .5*dt];
end

for i = 1:N
    % Minimize curvilinear offset from v_tar constant motion
    objectives = objectives + (z(1,i)-z_ref(1,i))^2;
    % Minimize centerline offset
    objectives = objectives + (z(2,i))^2 ;
    % Minimize heading error
    objectives = objectives + (z(3,i))^2;
    % Minimize v_tar error
    objectives = objectives + (v(i) - z(4,i))^2;
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
