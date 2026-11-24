% Unit tests for the SCARA kinematics library (script-based test suite).
%
% Run from the repository root:
%     addpath src
%     results = runtests('tests');
%     table(results)
%
% The code before the first %% section is shared setup for every test.

if exist('scara_fk', 'file') ~= 2
    here = fileparts(mfilename('fullpath'));
    candidates = {fullfile(here, '..', 'src'), fullfile(pwd, 'src'), fullfile(pwd, '..', 'src')};
    for c = 1:numel(candidates)
        if exist(fullfile(candidates{c}, 'scara_fk.m'), 'file')
            addpath(candidates{c});
            break
        end
    end
end

robot = scara_params();
tol   = 1e-10;
wrap  = @(a) mod(a + pi, 2*pi) - pi;
lims  = [robot.theta1Lim; robot.theta2Lim; robot.d3Lim; robot.theta4Lim];
randq = @() (lims(:, 1) + rand(4, 1) .* (lims(:, 2) - lims(:, 1))).';

% Closed-form 0T4 derived by hand from the DH table (docs/kinematics.md)
closedForm = @(q, r) [ ...
    cos(q(1)+q(2)-q(4)),  sin(q(1)+q(2)-q(4)),  0, r.a1*cos(q(1)) + r.a2*cos(q(1)+q(2)); ...
    sin(q(1)+q(2)-q(4)), -cos(q(1)+q(2)-q(4)),  0, r.a1*sin(q(1)) + r.a2*sin(q(1)+q(2)); ...
    0,                    0,                   -1, r.d1 - q(3) - r.d4; ...
    0,                    0,                    0, 1];

%% DH transform is a proper rigid-body transform
rng(1);
for k = 1:20
    A = dh_transform(2*pi*rand - pi, rand, rand, 2*pi*rand - pi);
    R = A(1:3, 1:3);
    assert(max(max(abs(R.'*R - eye(3)))) < tol, 'Rotation part is not orthonormal.');
    assert(abs(det(R) - 1) < tol, 'Rotation part must have det = +1.');
    assert(isequal(A(4, :), [0 0 0 1]), 'Last row must be [0 0 0 1].');
end

%% Home position matches hand calculation
T = scara_fk([0 0 0 0], robot);
expected = [robot.a1 + robot.a2; 0; robot.d1 - robot.d4];
assert(max(abs(T(1:3, 4) - expected)) < tol, 'Stretched-arm position is wrong.');
assert(max(max(abs(T(1:3, 1:3) - diag([1 -1 -1])))) < tol, 'Tool z-axis must point down.');

%% Quarter turns of the shoulder and elbow
T = scara_fk([pi/2 0 0 0], robot);
assert(max(abs(T(1:2, 4) - [0; robot.a1 + robot.a2])) < tol);
T = scara_fk([0 pi/2 0 0], robot);
assert(max(abs(T(1:2, 4) - [robot.a1; robot.a2])) < tol);

%% Prismatic joint lowers the tool
T0 = scara_fk([0.3 0.4 0.00 0.1], robot);
T1 = scara_fk([0.3 0.4 0.12 0.1], robot);
assert(abs((T0(3, 4) - T1(3, 4)) - 0.12) < tol, 'd3 must move the tool straight down.');
assert(max(abs(T0(1:2, 4) - T1(1:2, 4))) < tol, 'd3 must not move the tool sideways.');

%% DH chain equals the closed-form expression
rng(2);
for k = 1:200
    q = randq();
    T = scara_fk(q, robot);
    assert(max(max(abs(T - closedForm(q, robot)))) < 1e-12, 'DH product differs from closed form.');
end

%% Frame origins and pose output
rng(3);
q = randq();
[T, frames, P, pose] = scara_fk(q, robot);
assert(isequal(size(frames), [4 4 5]) && isequal(size(P), [3 5]));
assert(max(abs(P(:, 1))) < tol, 'Frame 0 must be at the origin.');
assert(max(abs(P(:, 2) - [robot.a1*cos(q(1)); robot.a1*sin(q(1)); robot.d1])) < tol, 'Elbow position.');
assert(max(abs(pose(1:3).' - T(1:3, 4))) < tol);
assert(abs(wrap(pose(4) - (q(1) + q(2) - q(4)))) < tol, 'Tool yaw must be theta1 + theta2 - theta4.');

%% Inverse kinematics round trip for both elbow configurations
rng(4);
for k = 1:200
    q = randq();
    [~, ~, ~, pose] = scara_fk(q, robot);
    elbow = 1;
    if q(2) < 0, elbow = -1; end
    [qIK, ok] = scara_ik(pose, robot, elbow);
    assert(ok, 'IK reported failure for a reachable pose.');
    assert(max(abs(wrap(qIK([1 2 4]) - q([1 2 4])))) < 1e-9, 'IK angles do not match.');
    assert(abs(qIK(3) - q(3)) < 1e-12, 'IK d3 does not match.');
end

%% Inverse kinematics flags unreachable targets
[~, ok, info] = scara_ik([2*(robot.a1 + robot.a2), 0, 0.2, 0], robot);
assert(~ok && ~info.reachable, 'Target beyond r_max must be rejected.');
[~, ok, info] = scara_ik([0.3, 0.1, robot.d1 + 1, 0], robot);
assert(~ok && info.reachable && ~info.withinLimits, 'd3 limit violation must be reported.');

%% Geometric Jacobian matches finite differences
rng(5);
h = 1e-6;
for k = 1:30
    q = randq();
    [J, w] = scara_jacobian(q, robot);
    Jnum = zeros(4, 4);
    for j = 1:4
        dq = zeros(1, 4);
        dq(j) = h;
        [~, ~, ~, pP] = scara_fk(q + dq, robot);
        [~, ~, ~, pM] = scara_fk(q - dq, robot);
        d = (pP - pM) / (2*h);
        d(4) = wrap(pP(4) - pM(4)) / (2*h);
        Jnum(:, j) = d.';
    end
    assert(max(max(abs(J([1 2 3 6], :) - Jnum))) < 1e-7, 'Jacobian differs from finite differences.');
    assert(max(max(abs(J([4 5], :)))) < tol, 'SCARA cannot rotate about x or y.');
    assert(abs(w - robot.a1*robot.a2*abs(sin(q(2)))) < 1e-12, 'Manipulability formula.');
end

%% Singular configurations have zero manipulability
[~, w0] = scara_jacobian([0.4 0 0.1 0], robot);
assert(w0 < 1e-12, 'Stretched arm must be singular.');

%% Reach limits
[rMin, rMax] = scara_reach(robot);
assert(abs(rMax - (robot.a1 + robot.a2)) < tol);
t2 = robot.theta2Lim(2);
assert(abs(rMin - sqrt(robot.a1^2 + robot.a2^2 + 2*robot.a1*robot.a2*cos(t2))) < tol);

%% Area with full joint rotation equals the annulus area
r = scara_params('theta1Lim', [-pi pi], 'theta2Lim', [-pi pi]);
A = scara_workspace_area(r);
Aexact = pi*((r.a1 + r.a2)^2 - (r.a1 - r.a2)^2);      % = 4*pi*a1*a2
assert(abs(A - Aexact)/Aexact < 1e-9, 'Full-rotation area must equal the annulus.');

%% Area of a single elbow branch matches the analytical integral
r = scara_params('theta1Lim', [0 1], 'theta2Lim', [0.3 2.0]);
A = scara_workspace_area(r);
Aexact = 1 * r.a1 * r.a2 * (cos(0.3) - cos(2.0));
assert(abs(A - Aexact)/Aexact < 1e-9, 'Single-branch area is wrong.');

%% Integral and occupancy-grid area estimates agree
cases = {robot, scara_params('theta1Lim', [-0.5 2.5], 'theta2Lim', [-2.8 1.0])};
for k = 1:numel(cases)
    Aint  = scara_workspace_area(cases{k}, 'integral');
    Agrid = scara_workspace_area(cases{k}, 'grid', 300);
    assert(abs(Agrid - Aint)/Aint < 0.02, 'Grid and integral areas disagree by more than 2 %.');
end

%% Workspace samples match forward kinematics and lie inside the reach annulus
ws = scara_workspace(robot, 40);
R = hypot(ws.X, ws.Y);
assert(all(R(:) >= ws.rMin - 1e-9 & R(:) <= ws.rMax + 1e-9), 'Sample outside the reach annulus.');
k = 777;
T = scara_fk([ws.theta1(k), ws.theta2(k), 0, 0], robot);
assert(abs(T(1, 4) - ws.X(k)) < tol && abs(T(2, 4) - ws.Y(k)) < tol, 'Workspace map differs from FK.');
assert(abs(ws.volume - ws.area*ws.stroke) < tol);
assert(abs(diff(ws.zRange) - ws.stroke) < tol);

%% Parameter validation rejects bad input
bad = {{'a1', -0.1}, {'theta2Lim', [-4 4]}, {'d3Lim', [0.2 0.1]}, {'nonexistent', 1}};
for k = 1:numel(bad)
    threw = false;
    try
        scara_params(bad{k}{:});
    catch
        threw = true;
    end
    assert(threw, 'scara_params accepted an invalid value.');
end
