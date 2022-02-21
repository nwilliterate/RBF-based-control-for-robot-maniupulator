% Copyright (C) 2022 All rights reserved.
% Authors:     Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Feb, 21, 2022
% Last Updated: Feb, 21, 2022
% 
% -------------------------------------------------
% single-link manipulator
% observer plant
% -------------------------------------------------
%
% the following code has been tested on matlab 2021a
function sys=obs_plant(x, u)
y=u(1);
ut=u(2);
fxp=u(3);
gxp=u(4);

global D K1 K2
A=[0 1;0 0];
b=[0 1]';
C=[1 0];

K=[K1 K2]';

ye=y-x(1);

% D=.8;
v=-D*sign(ye);

dx=A*x+b*(fxp+gxp*ut-v)+K*(y-C*x);
sys = [dx(1); dx(2)];
end