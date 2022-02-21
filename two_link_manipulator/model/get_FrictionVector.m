% Copyright (C) 2022 All rights reserved.
% Authors:      Seonghyeon Jo <cpsc.seonghyeon@gmail.com>
%
% Date:         Feb, 03, 2022
% Last Updated: Feb, 03, 2022
%
% -------------------------------------------------
% Get friction vector
% two-link manipulator
% -------------------------------------------------
%
% the following code has been tested on Matlab 2021a
function F = get_FrictionVector(dq)
F = 5*[sign(dq(1)); sign(dq(2))];
end