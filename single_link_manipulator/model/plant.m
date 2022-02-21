% Copyright (C) 2022 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Feb, 21, 2022
% Last Updated: Feb, 21, 2022
% 
% -------------------------------------------------
% single-link manipulator plant
% 
% -------------------------------------------------
%
% the following code has been tested on matlab 2021a
function sys=plant(x, u)
m=1;l=1;M=0.5;g=9.8;
fx=-0.5*m*g*l*sin(x(1))/M;
gx=1/M;
sys = [x(2); fx+gx*u];
end