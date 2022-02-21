% Copyright (C) 2022 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Feb, 21, 2022
% 
% -------------------------------------------------
% Runge-Kutta Method
% -------------------------------------------------
% Equation)
%       dx = Ax+Bu
% Input)
%       x   : state x
%       u   : Input u
%       T   : Simulation Time 
% Output)
%       dx  : Differentiation of State x
%
% the following code has been tested on Matlab 2021a
function dx = rk(x, u, T)
k1 = plant(x, u)*T;
k2 = plant(x+k1*0.5, u)*T;
k3 = plant(x+k2*0.5, u)*T;
k4 = plant(x+k3, u)*T;
dx = x+((k1+k4)/6+(k2+k3)/3);
end