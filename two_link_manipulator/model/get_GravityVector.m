% Copyright (C) 2022 All rights reserved.
% Authors:      Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Feb, 03, 2022
% Last Updated: Feb, 03, 2022
%
% -------------------------------------------------
% Get gravity vector 
% two-link manipulator
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
function G = get_GravityVector(q)
l1 = 0.2; l2 = 0.1;
m1 = 10; m2 = 5;
g = 9.81;
G = [m2*l2*g*cos(q(1)+q(2))+(m1+m2)*l1*g*cos(q(1)); 
    m2*l2*g*cos(q(1)+q(2))];
end