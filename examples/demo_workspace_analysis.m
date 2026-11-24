%DEMO_WORKSPACE_ANALYSIS Reach, area and volume of the SCARA workspace.
%
%   1. Samples the workspace of the default arm and plots it.
%   2. Cross-checks the planar area with an independent grid estimate.
%   3. Design study: for a fixed total reach a1 + a2, how should the length
%      be split between the two links to maximise the workspace?

clear; clc;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
addpath(fullfile(here, '..', 'src'));

robot = scara_params();

%% 1. Workspace of the default arm
ws = scara_workspace(robot, 150);
fprintf('--- Workspace of the default SCARA -------------------------\n');
fprintf('Link lengths     a1 = %.0f mm, a2 = %.0f mm\n', 1000*robot.a1, 1000*robot.a2);
fprintf('Joint limits     theta1 = [%g %g] deg, theta2 = [%g %g] deg\n', ...
    round(robot.theta1Lim*180/pi), round(robot.theta2Lim*180/pi));
fprintf('Reach            r_min = %.1f mm, r_max = %.1f mm\n', 1000*ws.rMin, 1000*ws.rMax);
fprintf('Tool-tip height  z = %.0f .. %.0f mm (stroke %.0f mm)\n', ...
    1000*ws.zRange(1), 1000*ws.zRange(2), 1000*ws.stroke);
fprintf('Planar area      %.4f m^2\n', ws.area);
fprintf('Volume           %.4f m^3\n', ws.volume);

scara_plot_workspace(ws, robot);

%% 2. Independent check of the area (occupancy grid)
tic;
Agrid = scara_workspace_area(robot, 'grid', 400);
fprintf('\nArea, polar integral : %.5f m^2\n', ws.area);
fprintf('Area, occupancy grid : %.5f m^2  (%.2f %% difference, %.2f s)\n', ...
    Agrid, 100*(Agrid - ws.area)/ws.area, toc);

%% 3. Design study: split of a fixed total reach between a1 and a2
L = robot.a1 + robot.a2;
ratio = linspace(0.15, 0.85, 71);          % a1 / (a1 + a2)
A = zeros(size(ratio));
rMin = zeros(size(ratio));
for k = 1:numel(ratio)
    r = robot;
    r.a1 = ratio(k) * L;
    r.a2 = (1 - ratio(k)) * L;
    A(k) = scara_workspace_area(r);
    rMin(k) = scara_reach(r);
end
[Abest, kBest] = max(A);
fprintf('\nFor a1 + a2 = %.0f mm the largest area (%.4f m^2) is at a1/(a1+a2) = %.2f\n', ...
    1000*L, Abest, ratio(kBest));

figure('Name', 'SCARA - link length design study', 'Color', 'w');
yyaxis left
plot(ratio, A, 'LineWidth', 2);
ylabel('Planar workspace area (m^2)');
yyaxis right
plot(ratio, 1000*rMin, '--', 'LineWidth', 2);
ylabel('Minimum reach r_{min} (mm)');
yyaxis left
hold on
plot(ratio(kBest), Abest, 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
hold off
grid on
xlabel('a_1 / (a_1 + a_2)');
title(sprintf('Workspace vs. link-length split  (a_1 + a_2 = %.0f mm)', 1000*L));
