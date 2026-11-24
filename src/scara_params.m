function robot = scara_params(varargin)
%SCARA_PARAMS Geometry, joint limits and home pose of the SCARA manipulator.
%
%   ROBOT = SCARA_PARAMS() returns the default robot description used by
%   the simulator. All lengths are in metres and all angles in radians.
%
%   ROBOT = SCARA_PARAMS(Name, Value, ...) overrides individual fields:
%
%       robot = scara_params('a1', 0.40, 'a2', 0.30, 'd3Lim', [0 0.15]);
%
%   Fields
%     name       - descriptive name of the arm
%     d1         - height of the arm plane above the base frame {0}   [m]
%     a1         - length of link 1 (shoulder -> elbow)               [m]
%     a2         - length of link 2 (elbow -> quill axis)             [m]
%     d4         - fixed tool offset below the quill flange           [m]
%     theta1Lim  - [min max] limits of joint 1 (revolute)           [rad]
%     theta2Lim  - [min max] limits of joint 2 (revolute)           [rad]
%     d3Lim      - [min max] stroke of joint 3 (prismatic)            [m]
%     theta4Lim  - [min max] limits of joint 4 (revolute, tool roll) [rad]
%     qHome      - home configuration [theta1 theta2 d3 theta4]
%     jointType  - joint sequence, 'RRPR'
%
%   See also SCARA_DH_TABLE, SCARA_FK, SCARA_WORKSPACE.

deg = pi/180;

robot.name      = 'SCARA (RRPR)';
robot.d1        = 0.40;
robot.a1        = 0.35;
robot.a2        = 0.25;
robot.d4        = 0.05;
robot.theta1Lim = [-150 150] * deg;
robot.theta2Lim = [-145 145] * deg;
robot.d3Lim     = [0 0.20];
robot.theta4Lim = [-180 180] * deg;
robot.qHome     = [30*deg, 45*deg, 0.05, 0];
robot.jointType = 'RRPR';

% ---- Name/Value overrides -------------------------------------------
if mod(numel(varargin), 2) ~= 0
    error('scara_params:InvalidInput', ...
        'Optional arguments must be given as Name, Value pairs.');
end
for k = 1:2:numel(varargin)
    name = varargin{k};
    if isstring(name) && isscalar(name)
        name = char(name);
    end
    if ~ischar(name)
        error('scara_params:InvalidInput', 'Parameter names must be text.');
    end
    if ~isfield(robot, name)
        error('scara_params:UnknownField', ...
            'Unknown SCARA parameter "%s".', name);
    end
    robot.(name) = varargin{k + 1};
end

robot = local_validate(robot);
end

% =====================================================================
function robot = local_validate(robot)
%LOCAL_VALIDATE Check that the robot description is physically sensible.

positive = {'d1', 'a1', 'a2'};
for k = 1:numel(positive)
    v = robot.(positive{k});
    if ~(isnumeric(v) && isscalar(v) && isfinite(v) && v > 0)
        error('scara_params:InvalidValue', ...
            '%s must be a positive, finite scalar.', positive{k});
    end
end

if ~(isnumeric(robot.d4) && isscalar(robot.d4) && isfinite(robot.d4) && robot.d4 >= 0)
    error('scara_params:InvalidValue', 'd4 must be a non-negative scalar.');
end

limits = {'theta1Lim', 'theta2Lim', 'd3Lim', 'theta4Lim'};
for k = 1:numel(limits)
    v = robot.(limits{k});
    if ~(isnumeric(v) && numel(v) == 2 && all(isfinite(v)) && v(1) < v(2))
        error('scara_params:InvalidValue', ...
            '%s must be a vector [min max] with min < max.', limits{k});
    end
    robot.(limits{k}) = double(v(:).');
end

if robot.theta1Lim(2) - robot.theta1Lim(1) > 2*pi + 1e-12
    error('scara_params:InvalidValue', 'theta1Lim may span at most 2*pi.');
end
if any(abs(robot.theta2Lim) > pi + 1e-12)
    error('scara_params:InvalidValue', 'theta2Lim must lie within [-pi, pi].');
end
if ~(isnumeric(robot.qHome) && numel(robot.qHome) == 4)
    error('scara_params:InvalidValue', 'qHome must have four elements.');
end
robot.qHome = double(robot.qHome(:).');
end
