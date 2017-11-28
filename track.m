function [psi] = track(y)
s = 100;

r = 1*s;
rs = 2*r/4;
total = 2*(pi*r + r + pi*rs);
if y < 2*r
    psi = 0;
elseif (y >= 2*r) && (y < (2*r + pi*r))
    psi = (y - 2*r) / r;
elseif (y >= (2*r + pi*r)) && (y < (2*r + pi*r + 2*pi*rs/4))
    psi = pi + (y - (2*r + pi*r)) / rs;
elseif (y >= (2*r + pi*r + 2*pi*rs/4)) && (y < (2*r + pi*r + 6*pi*rs/4))
    psi = 3*2*pi/4 - (y - (2*r + pi*r + 2*pi*rs/4)) / rs;
elseif (y >= (2*r + pi*r + 6*pi*rs/4)) && (y < (2*r + pi*r + 8*pi*rs/4))
    psi = pi/2 + (y - (2*r + pi*r + 6*pi*rs/4)) / rs;
elseif (y >= (2*r + pi*r + 8*pi*rs/4)) && (y < total)
    psi = pi + (y - (2*r + pi*r + 8*pi*rs/4)) / r;
else
    psi = track(y-total) + 2*pi;
end
    