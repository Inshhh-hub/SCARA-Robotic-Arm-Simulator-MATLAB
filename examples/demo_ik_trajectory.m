%DEMO_IK_TRAJECTORY Follow a Cartesian path with inverse kinematics.
%
%   The tool tip traces a horizontal circle with a "pick" dip. Joint values
%   come from SCARA_IK; SCARA_FK is used to verify that FK(IK(x)) = x.
%   The arm is animated and the joint trajectories are plotted.

clear; clc;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
addpath(fullfile(here, '..', 'src'));

robot = scara_params();

%% Cartesian path  [x y z phi]
N  = 240;
s  = linspace(0, 2*pi, N).';
c  = [0.38, 0.12];          % circle centre (m)
R  = 0.12;                  % circle radius (m)
z0 = 0.30;                  % travel height (m)
traj = [c(1) + R*cos(s), ...
        c(2) + R*sin(s), ...
        z0 - 0.10*(1 - cos(2*s))/2, ...   % two dips down to z0 - 0.10
        zeros(N, 1)];                    % constant tool yaw

%% Inverse kinematics along the path
Q  = zeros(N, 4);
ok = false(N, 1);
for k = 1:N
    [Q(k, :), ok(k)] = scara_ik(traj(k, :), robot, 'auto');
end
if ~all(ok)
    warning('demo_ik_trajectory:unreachable', ...
        '%d path points are outside the workspace or the joint limits.', nnz(~ok));
end

err = zeros(N, 1);
for k = 1:N
    [~, ~, ~, pose] = scara_fk(Q(k, :), robot);
    err(k) = norm(pose(1:3) - traj(k, 1:3));
end
fprintf('max |FK(IK(x)) - x| along the path = %.2e m\n', max(err));

%% Animate
fig = figure('Name', 'SCARA - IK trajectory', 'Color', 'w');
ax = axes('Parent', fig);
h = scara_draw(ax, Q(1, :), robot, [], struct('showFrames', false));
hold(ax, 'on');
plot3(ax, traj(:, 1), traj(:, 2), traj(:, 3), '--', 'Color', [0.6 0.6 0.6]);
hPath = plot3(ax, nan, nan, nan, '-', 'Color', [0.55 0.20 0.75], 'LineWidth', 2);
hold(ax, 'off');
title(ax, 'Tool tip following a circular pick path (inverse kinematics)');

for k = 1:N
    if ~isvalid(fig), break; end
    h = scara_draw(ax, Q(k, :), robot, h, struct('showFrames', false));
    set(hPath, 'XData', traj(1:k, 1), 'YData', traj(1:k, 2), 'ZData', traj(1:k, 3));
    drawnow limitrate
    pause(0.01);
end

%% Joint trajectories
figure('Name', 'SCARA - joint trajectories', 'Color', 'w');
labels = {'\theta_1 (deg)', '\theta_2 (deg)', 'd_3 (mm)', '\theta_4 (deg)'};
scale  = [180/pi, 180/pi, 1000, 180/pi];
for j = 1:4
    subplot(4, 1, j);
    v = Q(:, j);
    if j ~= 3
        v = unwrap(v);       % remove +/-180 deg wrap jumps for plotting
    end
    plot(s*180/pi, v*scale(j), 'LineWidth', 1.8);
    grid on
    ylabel(labels{j});
end
xlabel('Path parameter (deg)');
subplot(4, 1, 1);
title('Joint trajectories from inverse kinematics');
