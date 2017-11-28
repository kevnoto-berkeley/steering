function [z] = bikeFE(z,u,v,TS)
x = z(1);
y = z(2);
psi = z(3);
lr = 1.738;
lf = 1.738;
% B = atan2((lr*tan(deltaF)),(lf+lr));
B = u(1);
psiDot = (v/lr)*sin(B);
yDot = v*sin(psi+B);
xDot = v*cos(psi+B);

xp = x + TS*xDot;
yp = y + TS*yDot;
psip = psi + TS*psiDot;
z = [xp;yp;psip];