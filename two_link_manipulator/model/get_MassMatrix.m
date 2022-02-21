% Copyright (C) 2022 All rights reserved.
% Authors:      Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Feb, 03, 2022
% Last Updated: Feb, 03, 2022
%
% -------------------------------------------------
% Get mass matrix 
% two-link manipulator
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
function M = get_MassMatrix(q)
l1 = 0.2; l2 = 0.1;
m1 = 10; m2 = 5;
g = 9.81;
M = [(l2^2)*m2 + 2*l1*l2*m2*cos(q(2))+(l1^2)*(m1+m2) (l2^2)*m2+l1*l2*m2*cos(q(2));
    (l2^2)*m2+l1*l2*m2*cos(q(2)) (l2^2)*m2];
end