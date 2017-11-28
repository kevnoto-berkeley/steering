A = [1.2 1; 0 1];
B = [0;1];
Q = eye(2);
R = 1;
ulo = -1;
uhi = 1;
xlo = [-15;-15];
xhi = [15;15];
x0 = [2;1];

P = eye(2);
nx = size(A,2);
nu = size(B,2);

M = 25; %Simulation horizon
xOpt = zeros (nx,M_1);
uOpt = zeros(nu,M);
xOpt(:,1) = x0;

N = 3;
xPred = zeros(xn,N+1,M);
predErr = zeros(nx,M-n);

feas = false([1,M);
xN = [0;0];

figure
for t = 1:M
    fprintf('solving simstep: %i\n',t)
    pfeas(t), x, u] = solve_cftoc(A,B,P,Q,R,N,xopt(:,t),xlo,xhi,ulo,uhi,xN,[]);
    if ~feas(t)
        xOpt = []
        uOpt = []
        predErr = []
        return
    end
    % save open loop prediction
    xPred(:,:,t) = x;
    
    % save closed loop trajectory only first action
    xOpt (:,t+1) = x(:,2);
    uOpt(:,t) = u(1);
    
    % plot open loop
    plot(x(1,:),x(2,:),'r--')
    hold on
    pause(0.1)
end





function[feas,xop
    yalmip('clear')
    nx = size(A,2)
    nu = size(B,2)
    x = sdpvar(nx,N+1)
    u = sdpvar(nu,N);
    feas - FAlse
    constr = [x(:,1) == x0]
    if isemptu(Af)
    constr = [constr x(:,N+1)==bf];
    else
        constr = [constr Af*x(:,N+1)<=bf];
    end
    cost = x(:,N+1)'(p(x(:,N+1);
    
    for k = 1:N
        % Add box constraints to every time
        constr  = [constr,x(:,l+1) == A*x(:,k) + b*u(:,l), uL <= u(:,
        cost = cost _ x(:,k)'*q*x(:,k) + u(:,k)'*R*u(L,)
