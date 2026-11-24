function [axTop, ax3] = scara_plot_workspace(ws, robot, axTop, ax3, colorBy)
%SCARA_PLOT_WORKSPACE Plot the workspace computed by SCARA_WORKSPACE.
%
%   SCARA_PLOT_WORKSPACE(WS, ROBOT) opens a new figure with a top view
%   (XY plane) of the reachable area and a 3-D view of the reachable volume.
%
%   SCARA_PLOT_WORKSPACE(WS, ROBOT, AXTOP, AX3) draws into existing axes
%   (regular axes or UIAxes); used by the simulator GUI.
%
%   SCARA_PLOT_WORKSPACE(..., COLORBY) colours the sample points by
%       'manipulability' - a1*a2*|sin(theta2)|  (default)
%       'elbow'          - sign of theta2 (the two arm configurations)
%
%   [AXTOP, AX3] = SCARA_PLOT_WORKSPACE(...) returns the axes used.
%
%   See also SCARA_WORKSPACE, SCARA_DRAW.

if nargin < 5 || isempty(colorBy), colorBy = 'manipulability'; end
if nargin < 4 || isempty(axTop) || isempty(ax3)
    fig = figure('Name', 'SCARA workspace', 'Color', 'w', ...
        'Position', [100 100 1200 520]);
    axTop = axes('Parent', fig, 'OuterPosition', [0.00 0 0.5 1]);
    ax3   = axes('Parent', fig, 'OuterPosition', [0.50 0 0.5 1]);
end

X = ws.X(:);
Y = ws.Y(:);

switch lower(colorBy)
    case 'elbow'
        C = sign(ws.theta2(:));
        C(C == 0) = 1;
        cmap = [0.16 0.44 0.78; 0.94 0.55 0.16];
        cl = [-1 1];
        cbLabel = 'Elbow configuration';
        cbTicks = [-0.5 0.5];
        cbTickLabels = {'\theta_2 < 0', '\theta_2 \geq 0'};
    otherwise
        C = ws.manip(:);
        cmap = parula(256);
        cl = [0, max(robot.a1*robot.a2, eps)];
        cbLabel = 'Manipulability  w = a_1a_2|sin\theta_2|  (m^2)';
        cbTicks = [];
        cbTickLabels = {};
end

ang = linspace(0, 2*pi, 361);
envColor = [0.12 0.12 0.15];
lim = 1.12*max(ws.rMax, eps);

% =================== top view (XY plane) =============================
cla(axTop);
hold(axTop, 'on');
scatter(axTop, X, Y, 6, C, 'filled');

hEnv = gobjects(0);
for k = 1:numel(ws.boundary)
    b = ws.boundary{k};
    hEnv(end + 1) = plot(axTop, b(1, :), b(2, :), '-', ...
        'Color', envColor, 'LineWidth', 1.3); %#ok<AGROW>
end
hMax = plot(axTop, ws.rMax*cos(ang), ws.rMax*sin(ang), '--', ...
    'Color', [0.50 0.50 0.56], 'LineWidth', 1);
hMin = plot(axTop, ws.rMin*cos(ang), ws.rMin*sin(ang), ':', ...
    'Color', [0.50 0.50 0.56], 'LineWidth', 1.3);
plot(axTop, 0, 0, 'k+', 'MarkerSize', 10, 'LineWidth', 1.5, ...
    'HandleVisibility', 'off');
hold(axTop, 'off');

colormap(axTop, cmap);
set(axTop, 'CLim', cl, 'XLim', [-lim lim], 'YLim', [-lim lim], ...
    'DataAspectRatio', [1 1 1]);
cb = colorbar(axTop);
cb.Label.String = cbLabel;
if ~isempty(cbTicks)
    cb.Ticks = cbTicks;
    cb.TickLabels = cbTickLabels;
end
if ~isempty(hEnv)
    legend(axTop, [hEnv(1), hMax, hMin], ...
        {'Envelope (joint limits, \theta_2 = 0)', 'r_{max}', 'r_{min}'}, ...
        'Location', 'southoutside', 'Orientation', 'horizontal', ...
        'FontSize', 8, 'AutoUpdate', 'off');
end
grid(axTop, 'on');
box(axTop, 'on');
xlabel(axTop, 'X (m)');
ylabel(axTop, 'Y (m)');
title(axTop, sprintf('Top view  -  planar area %.4f m^2', ws.area));

% =================== 3-D reachable volume ============================
zB = ws.zRange(1);
zT = ws.zRange(2);
step = max(1, floor(numel(X) / 6000));
idx = (1:step:numel(X)).';
nI = numel(idx);

cla(ax3);
hold(ax3, 'on');
scatter3(ax3, X(idx), Y(idx), zT*ones(nI, 1), 5, C(idx), 'filled');
scatter3(ax3, X(idx), Y(idx), zB*ones(nI, 1), 5, C(idx), 'filled');
for k = 1:numel(ws.boundary)
    b = ws.boundary{k};
    m = size(b, 2);
    surface('Parent', ax3, 'XData', [b(1, :); b(1, :)], ...
        'YData', [b(2, :); b(2, :)], 'ZData', [zB*ones(1, m); zT*ones(1, m)], ...
        'FaceColor', [0.25 0.45 0.75], 'FaceAlpha', 0.12, 'EdgeColor', 'none');
    plot3(ax3, b(1, :), b(2, :), zT*ones(1, m), '-', 'Color', envColor, 'LineWidth', 1);
    plot3(ax3, b(1, :), b(2, :), zB*ones(1, m), '-', 'Color', envColor, 'LineWidth', 1);
end
plot3(ax3, [0 0], [0 0], [0 robot.d1], '-', 'Color', [0.30 0.30 0.32], 'LineWidth', 5);
hold(ax3, 'off');

colormap(ax3, cmap);
zPad = 0.08*max(ws.rMax, eps);
set(ax3, 'CLim', cl, 'XLim', [-lim lim], 'YLim', [-lim lim], ...
    'ZLim', [min(0, zB) - zPad, max(robot.d1, zT) + zPad], ...
    'DataAspectRatio', [1 1 1]);
view(ax3, -35, 25);
grid(ax3, 'on');
xlabel(ax3, 'X (m)');
ylabel(ax3, 'Y (m)');
zlabel(ax3, 'Z (m)');
title(ax3, sprintf('Reachable volume %.4f m^3  (stroke %.0f mm)', ...
    ws.volume, 1000*ws.stroke));
end
