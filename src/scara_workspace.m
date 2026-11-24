function ws = scara_workspace(robot, n)
%SCARA_WORKSPACE Sample and analyse the reachable workspace of the SCARA arm.
%
%   WS = SCARA_WORKSPACE(ROBOT) samples joints 1 and 2 on a regular
%   120 x 120 grid inside their limits, maps every sample to the tool
%   position (translation column of 0T4 from SCARA_FK) and returns the
%   point cloud together with workspace metrics.
%
%   WS = SCARA_WORKSPACE(ROBOT, N) uses N samples per revolute joint.
%
%   Output struct fields
%     theta1, theta2 - N x N joint-space grids                        [rad]
%     X, Y           - N x N tool-tip coordinates                       [m]
%     manip          - N x N manipulability  a1*a2*|sin(theta2)|      [m^2]
%     rMin, rMax     - minimum / maximum horizontal reach               [m]
%     zRange         - [zMin zMax] height range of the tool tip          [m]
%     stroke         - vertical stroke of the prismatic joint            [m]
%     area           - planar workspace area                          [m^2]
%     volume         - reachable volume = area * stroke                [m^3]
%     boundary       - cell array of 2xM curves: images of the joint-limit
%                      edges and of the singular arc theta2 = 0. Together
%                      they form the envelope of the workspace.
%     boundaryName   - description of each envelope curve
%     n              - samples per joint
%
%   The tool position follows from the DH product 0T4 = A1*A2*A3*A4:
%       x = a1 cos(theta1) + a2 cos(theta1 + theta2)
%       y = a1 sin(theta1) + a2 sin(theta1 + theta2)
%       z = d1 - d3 - d4
%   It is evaluated here in vectorised form for speed; the unit tests check
%   that it matches SCARA_FK.
%
%   See also SCARA_PLOT_WORKSPACE, SCARA_WORKSPACE_AREA, SCARA_REACH.

if nargin < 1 || isempty(robot), robot = scara_params(); end
if nargin < 2 || isempty(n), n = 120; end
n = max(2, round(n));

a1  = robot.a1;
a2  = robot.a2;
th1 = robot.theta1Lim;
th2 = robot.theta2Lim;

[T1, T2] = meshgrid(linspace(th1(1), th1(2), n), linspace(th2(1), th2(2), n));

ws.theta1 = T1;
ws.theta2 = T2;
ws.X      = a1*cos(T1) + a2*cos(T1 + T2);
ws.Y      = a1*sin(T1) + a2*sin(T1 + T2);
ws.manip  = a1*a2*abs(sin(T2));

[ws.rMin, ws.rMax] = scara_reach(robot);
ws.zRange = robot.d1 - robot.d4 - fliplr(robot.d3Lim);   % [zMin zMax]
ws.stroke = robot.d3Lim(2) - robot.d3Lim(1);
ws.area   = scara_workspace_area(robot);
ws.volume = ws.area * ws.stroke;

[ws.boundary, ws.boundaryName] = local_envelope(robot, 240);
ws.n = n;
end

% =====================================================================
function [curves, names] = local_envelope(robot, m)
%LOCAL_ENVELOPE Images of the joint-space rectangle edges (and theta2 = 0).
th1 = robot.theta1Lim;
th2 = robot.theta2Lim;
s1 = linspace(th1(1), th1(2), m);
s2 = linspace(th2(1), th2(2), m);
one = ones(1, m);

xy = @(t1, t2) [robot.a1*cos(t1) + robot.a2*cos(t1 + t2); ...
                robot.a1*sin(t1) + robot.a2*sin(t1 + t2)];

curves = {xy(s1, th2(1)*one), xy(s1, th2(2)*one), ...
          xy(th1(1)*one, s2), xy(th1(2)*one, s2)};
names  = {'theta2 = min', 'theta2 = max', 'theta1 = min', 'theta1 = max'};

if th2(1) < 0 && th2(2) > 0
    curves{end + 1} = xy(s1, 0*one);
    names{end + 1}  = 'theta2 = 0 (outer singular arc)';
end
end
