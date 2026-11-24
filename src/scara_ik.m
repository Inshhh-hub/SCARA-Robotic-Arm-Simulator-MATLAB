function [q, ok, info] = scara_ik(pose, robot, elbow)
%SCARA_IK Closed-form inverse kinematics of the SCARA arm.
%
%   Q = SCARA_IK(POSE, ROBOT) returns the joint vector [theta1 theta2 d3
%   theta4] that places the tool tip at POSE = [x y z] or [x y z phi]
%   (metres, radians). The tool yaw phi defaults to 0.
%
%   Q = SCARA_IK(POSE, ROBOT, ELBOW) selects the arm configuration:
%       +1     - theta2 >= 0  (default)
%       -1     - theta2 <= 0
%       'auto' - the first of (+1, -1) that respects all joint limits
%
%   [Q, OK, INFO] = SCARA_IK(...) also returns OK = true when the pose is
%   reachable AND every joint is inside its limits, plus a struct INFO with
%   fields reachable, withinLimits, elbow and message.
%
%   Derivation (see docs/kinematics.md):
%       cos(theta2) = (x^2 + y^2 - a1^2 - a2^2) / (2 a1 a2)
%       theta2      = atan2(+/- sqrt(1 - c2^2), c2)
%       theta1      = atan2(y, x) - atan2(a2 sin(theta2), a1 + a2 cos(theta2))
%       d3          = d1 - d4 - z
%       theta4      = theta1 + theta2 - phi
%
%   See also SCARA_FK.

if nargin < 2 || isempty(robot)
    robot = scara_params();
end
if nargin < 3 || isempty(elbow)
    elbow = 1;
end

pose = double(pose(:).');
if numel(pose) == 3
    pose(4) = 0;
end
if numel(pose) ~= 4
    error('scara_ik:InvalidInput', 'pose must be [x y z] or [x y z phi].');
end

% ---- automatic selection of the elbow configuration -----------------
if ischar(elbow) || isstring(elbow)
    if ~strcmpi(elbow, 'auto')
        error('scara_ik:InvalidInput', 'elbow must be +1, -1 or ''auto''.');
    end
    [q, ok, info] = scara_ik(pose, robot, 1);
    if ~ok
        [q2, ok2, info2] = scara_ik(pose, robot, -1);
        if ok2
            q = q2;  ok = ok2;  info = info2;
        end
    end
    return
end

if elbow >= 0
    e = 1;
else
    e = -1;
end

x = pose(1);  y = pose(2);  z = pose(3);  phi = pose(4);
a1 = robot.a1;  a2 = robot.a2;

% ---- planar 2R sub-problem -----------------------------------------
c2 = (x^2 + y^2 - a1^2 - a2^2) / (2*a1*a2);
reachable = abs(c2) <= 1 + 1e-9;
c2 = min(1, max(-1, c2));
s2 = e * sqrt(1 - c2^2);

theta2 = atan2(s2, c2);
theta1 = atan2(y, x) - atan2(a2*s2, a1 + a2*c2);

% ---- vertical axis and tool roll -----------------------------------
d3     = robot.d1 - robot.d4 - z;
theta4 = theta1 + theta2 - phi;

theta1 = local_fit_angle(theta1, robot.theta1Lim);
theta4 = local_fit_angle(theta4, robot.theta4Lim);
q = [theta1, theta2, d3, theta4];

% ---- limit check ----------------------------------------------------
lims  = [robot.theta1Lim; robot.theta2Lim; robot.d3Lim; robot.theta4Lim];
names = {'theta1', 'theta2', 'd3', 'theta4'};
tol   = 1e-9;
bad   = q(:) < lims(:, 1) - tol | q(:) > lims(:, 2) + tol;

info.reachable    = reachable;
info.withinLimits = ~any(bad);
info.elbow        = e;
if ~reachable
    info.message = 'Target lies outside the annular reach of the arm.';
elseif any(bad)
    info.message = ['Joint limit violated: ', strjoin(names(bad), ', ')];
else
    info.message = 'OK';
end
ok = info.reachable && info.withinLimits;
end

% =====================================================================
function a = local_fit_angle(a, lim)
%LOCAL_FIT_ANGLE Wrap to (-pi, pi], then shift by 2*pi if that fits the limits.
a = mod(a + pi, 2*pi) - pi;
if a < lim(1) || a > lim(2)
    for cand = [a + 2*pi, a - 2*pi]
        if cand >= lim(1) && cand <= lim(2)
            a = cand;
            return
        end
    end
end
end
