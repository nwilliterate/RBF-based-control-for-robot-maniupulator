% Copyright (C) 2022 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Feb, 03, 2022
% Last Updated: Feb, 03, 2022
%
% -------------------------------------------------
% observer plant
% two-link manipulator
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
function sys=obs_plant(x, u)
global K bar D
q(1) = x(1);
q(2) = x(2);
dq(1) = x(3);
dq(2) = x(4);
y(1) = u(1);
y(2) = u(2);
delta(1) = u(3);
delta(2) = u(4);
ut(1) = u(5);
ut(2) = u(6);
q = q'; dq = dq'; y = y'; ut = ut'; delta = delta';

% bar = 0.4;
M = get_MassMatrix(q) * bar;
Cq = get_CoriolisVector(q, dq) * bar;
G = get_GravityVector(q) * bar;
F = get_FrictionVector(dq) * bar;

% D=0.8;
v=-D*sign(y-q);

% K = diag([75 75])*0.2;

dot = dq+K*(y-q);
ddot = inv(M)*(-Cq-G-F+ut)+K*(y-q)+delta-v;
sys = [dot(1);dot(2);ddot(1);ddot(2);];
end