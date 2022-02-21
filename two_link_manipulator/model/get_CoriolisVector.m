% Copyright (C) 2022 All rights reserved.
% Authors:      Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Feb, 03, 2022
% Last Updated: Feb, 03, 2022
%
% -------------------------------------------------
% Get coriolis vector 
% two-link manipulator
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
function Cdq = get_CoriolisVector(q, dq)
x(1) = q(1);
x(2) = q(2);
x(3) = dq(1);
x(4) = dq(2);

l1 = 0.2; l2 = 0.1;
m1 = 10; m2 = 5;
g = 9.81;
Cdq = [-m2*l1*l2*sin(x(2))*(x(4)^2)-2*m2*l1*l2*sin(x(2))*x(3)*x(4);
    m2*l1*l2*sin(x(2))*(x(4)^2)];
end