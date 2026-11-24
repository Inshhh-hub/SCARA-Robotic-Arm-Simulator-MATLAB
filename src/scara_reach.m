function [rMin, rMax] = scara_reach(robot)
%SCARA_REACH Minimum and maximum horizontal reach of the SCARA arm.
%
%   [RMIN, RMAX] = SCARA_REACH(ROBOT) returns the smallest and largest
%   distance from the base axis to the quill axis that the arm can reach,
%   taking the limits of joint 2 into account. The radial distance is
%
%       r(theta2) = sqrt(a1^2 + a2^2 + 2*a1*a2*cos(theta2)),
%
%   which decreases monotonically with |theta2| on [0, pi]. Joint 1 only
%   rotates the arm, so it does not change the reach.
%
%   See also SCARA_WORKSPACE, SCARA_WORKSPACE_AREA.

if nargin < 1 || isempty(robot)
    robot = scara_params();
end

lo = robot.theta2Lim(1);
hi = robot.theta2Lim(2);

if lo <= 0 && hi >= 0
    tNear = 0;                          % fully stretched arm is allowed
else
    tNear = min(abs(lo), abs(hi));
end
tFar = max(abs(lo), abs(hi));           % most folded configuration

r = @(t) sqrt(max(0, robot.a1^2 + robot.a2^2 + 2*robot.a1*robot.a2*cos(t)));
rMax = r(tNear);
rMin = r(tFar);
end
