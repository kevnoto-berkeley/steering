function [psi] = road(y)
r = 20;
if (y < 5)
    psi = 0;
elseif (y >= 5) && (y < 10)
    psi = (y-5)/(2*pi*r/4);
elseif (y >= 10) && (y < 20)
    psi = (10-5)/(2*pi*r/4) - (y-10)/(2*pi*r/4);
else
    psi = 0;
end