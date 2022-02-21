% Copyright (C) 2022 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Feb, 03, 2022
% Last Updated: Feb, 03, 2022
%
% -------------------------------------------------
% 2 DoF Manipulator Plant Function
% -------------------------------------------------
% Equation)
%       ddq = M(q)ddq + C(q, dq)dq
%
% Input)
%       x   : Joint Postion and Velocity
%       u   : Joint Torque
% Output)
%       dxdt  : Joint Velocity and Acceleration
%
% the following code has been tested on Matlab 2021a
function dxdt = plant(x,u)
q(1) = x(1);
q(2) = x(2);
dq(1) = x(3);
dq(2) = x(4);

M = get_MassMatrix(q);
Cq = get_CoriolisVector(q, dq);
G = get_GravityVector(q);
F = get_FrictionVector(dq);
ddot = inv(M)*(-Cq-G-F+u);
dxdt = [x(3);x(4);ddot(1);ddot(2);];
end