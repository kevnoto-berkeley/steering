function [z_next] = car_model(z,u,v,dt)
    % z = [X, Y, psi]
    % X = lateral location on road
    % Y = longitudinal location on road
    % psi = heading error?

    % v = velocity, constant/given

    % U = d
    % d = steering angle
    lr = 1;

    X = z(1);
    Y = z(2);
    psi = z(3);
    d = u(1);
    psiDot = (v/lr)*sin(d);


    X_next = X + v*dt*cos(psi);
    Y_next = Y + v*dt*sin(psi);
    psi_next = psi + dt*psiDot;
    z_next = [X_next,Y_next,psi_next]';