function A = scara_workspace_area(robot, method, res)
%SCARA_WORKSPACE_AREA Area of the planar (XY) workspace of the SCARA arm.
%
%   A = SCARA_WORKSPACE_AREA(ROBOT) returns the area in m^2 swept by the
%   quill axis over the full range of joints 1 and 2. Joint 3 only moves
%   vertically, so the reachable volume is A * stroke (see SCARA_WORKSPACE).
%
%   A = SCARA_WORKSPACE_AREA(ROBOT, METHOD) selects the algorithm:
%
%   'integral' (default) - semi-analytical. In polar form the tool radius
%       r depends on theta2 only, and r dr = a1*a2*sin(t) dt with
%       t = |theta2|. At a given radius the arm covers an arc of length
%       L = span(theta1) for each elbow branch (+t and -t) that lies inside
%       the theta2 limits; the two arcs are rotated against each other by
%       2*psi(t), psi = atan2(a2 sin t, a1 + a2 cos t). Hence
%
%           A = integral_0^pi  U(t) * a1 * a2 * sin(t) dt,
%
%       where U(t) is the angular length of the union of the arcs. With
%       full rotation of both joints this gives the annulus area
%       pi*((a1+a2)^2 - (a1-a2)^2) = 4*pi*a1*a2.
%
%   'grid' - independent numerical check. Joint space is sampled densely
%       enough that neighbouring samples are less than half a cell apart,
%       mapped through the kinematics and accumulated in an occupancy grid
%       of RES x RES cells (default RES = 300). Boundary cells count as half.
%
%   See also SCARA_WORKSPACE, SCARA_REACH.

if nargin < 1 || isempty(robot), robot = scara_params(); end
if nargin < 2 || isempty(method), method = 'integral'; end
if nargin < 3 || isempty(res), res = 300; end

switch lower(char(method))
    case 'integral'
        A = local_integral_area(robot);
    case 'grid'
        A = local_grid_area(robot, round(res));
    otherwise
        error('scara_workspace_area:InvalidMethod', ...
            'method must be ''integral'' or ''grid''.');
end
end

% =====================================================================
function A = local_integral_area(robot)
a1 = robot.a1;
a2 = robot.a2;
lo = robot.theta2Lim(1);
hi = robot.theta2Lim(2);
L  = min(robot.theta1Lim(2) - robot.theta1Lim(1), 2*pi);  % arc per branch

% Split [0, pi] where branch membership changes, so each piece is smooth.
brk = unique([0, pi, min(pi, abs([lo, hi]))]);

A = 0;
for k = 1:numel(brk) - 1
    ta = brk(k);
    tb = brk(k + 1);
    tm = 0.5*(ta + tb);
    inPos = (tm >= lo) && (tm <= hi);      % branch theta2 = +t allowed
    inNeg = (-tm >= lo) && (-tm <= hi);    % branch theta2 = -t allowed
    if ~(inPos || inNeg)
        continue
    end
    if inPos && inNeg && L < 2*pi
        % Union of two arcs of length L whose starts differ by delta
        t = linspace(ta, tb, 4001);
        psi = atan2(a2*sin(t), a1 + a2*cos(t));
        delta = mod(2*psi, 2*pi);
        overlap = max(0, L - delta) + max(0, L - (2*pi - delta));
        U = min(2*pi, 2*L - overlap);
        A = A + a1*a2*trapz(t, U .* sin(t));
    else
        % One branch only (or full rotation): constant arc length L
        A = A + L*a1*a2*(cos(ta) - cos(tb));
    end
end
end

% =====================================================================
function A = local_grid_area(robot, res)
th1 = robot.theta1Lim;
th2 = robot.theta2Lim;
[~, rMax] = scara_reach(robot);

R = 1.02*rMax;                 % half-width of the square grid
h = 2*R/res;                   % cell size

% Neighbouring samples must be < h/2 apart so that no interior cell is missed
n1 = max(2, ceil((th1(2) - th1(1)) * rMax / (0.5*h)) + 1);
n2 = max(2, ceil((th2(2) - th2(1)) * robot.a2 / (0.5*h)) + 1);
t1 = linspace(th1(1), th1(2), n1);
t2 = linspace(th2(1), th2(2), n2);

occ = false(res, res);
chunk = max(1, floor(2e6 / n1));          % bound the memory footprint
for k = 1:chunk:n2
    idx = k:min(n2, k + chunk - 1);
    [T1, T2] = meshgrid(t1, t2(idx));
    X = robot.a1*cos(T1) + robot.a2*cos(T1 + T2);
    Y = robot.a1*sin(T1) + robot.a2*sin(T1 + T2);
    ix = min(res, max(1, floor((X(:) + R)/h) + 1));
    iy = min(res, max(1, floor((Y(:) + R)/h) + 1));
    occ(sub2ind([res, res], iy, ix)) = true;
end

% Occupied cells with an empty 4-neighbour lie on the boundary: count as half
pad = false(res + 2);
pad(2:end-1, 2:end-1) = occ;
interior = pad(1:end-2, 2:end-1) & pad(3:end, 2:end-1) & ...
           pad(2:end-1, 1:end-2) & pad(2:end-1, 3:end);
edgeCells = occ & ~interior;

A = (nnz(occ) - 0.5*nnz(edgeCells)) * h^2;
end
