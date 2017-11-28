%%
close all; clear all;

%%
% Grid the state space
% z = [s,x,psi,v]
% Assume s can be any number, no constraint
% Constrain road to 3.7m wide
% Assume heading is always 0, so we constrain on heading absolutely to +/-
% 10 deg
% Constrain v to constant (for now)

dt = .5;
zl = [-inf;-1;-degtorad(5);20];
zu = [inf;1;degtorad(5);50];

u1_grid = linspace(-degtorad(10),degtorad(10),50);
% Grid the states from 0 to positive inf, assuming symmetric performance
psi_grid = linspace(degtorad(0),degtorad(10),50);
v_grid = linspace(20,50,50);
x_grid = linspace(0,3.7,50);

cost = @(z) z(2)^2 + z(3)^2;
feas = {};
N = 1;
z2_points = linspace(zl(2),zu(2),50);
z3_points = linspace(zl(3),zu(3),20);
z4_points = linspace(zl(4),zu(4),50);
poly = {};
poly{1} = Polyhedron('lb',zl(2:4),'ub',zu(2:4));
%%
N = 10;
parfor delta_psi = [0,degtorad(5),degtorad(10)]
    for n = 1:N
        fprintf("Iteration: %d\n", n)
        feas{n} = [];
        A = poly{n}.H(:,1:3);
        b = poly{n}.H(:,4);
        for i = 1:numel(x_grid)
            for j = 1:numel(psi_grid)
                for k = 1:numel(v_grid)
                    for u = u1_grid
                        z = [0;x_grid(i);psi_grid(j);v_grid(k)];
                        z_next = CMC_w_accel(z,[u;0],dt,delta_psi);
                        if all(A * z_next(2:4) <= b)
                            feas{n} = [feas{n}, z];
                            break
                        end
                    end
                end
            end
        end
        z2_points = feas{n}(2,:)';
        z3_points = feas{n}(3,:)';
        z4_points = feas{n}(4,:)';
        points = feas{n}(2:4,:)';
        id = convhull(points);
        id = unique(id);
        points = points(id,:);
        poly{n+1}= Polyhedron(points);
        if n >=2
            if poly{n} == poly{n-1}
                disp('Set hasnt changed')
                fprintf("Iteration: %d\n", n)
                break
            end
        end
    end
end
%%
figure
X = Polyhedron('lb',[-3.7;-.2;20],'ub',[3.7;.2;50]);
plot(X,'color','b','alpha',.5)
xlabel('lane error (m)')
ylabel('heading error (rad)')
zlabel('velocity (m/s)')
hold on
plot(poly{10})
legend({'Feasible Range','Controllable Range'})
%%
pause
hold on
xlim([-5,5])
ylim([-.2,.2])
zlim([20,50])
for i = 1:numel(poly)
    plot(poly{i},'alpha',.2)
    pause
end
xlabel('psi')
ylabel('x')