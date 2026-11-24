function varargout = scara_simulator()
%SCARA_SIMULATOR Interactive SCARA robotic arm simulator.
%
%   SCARA_SIMULATOR opens an interactive simulator of a 4-DOF SCARA (RRPR)
%   manipulator built on Denavit-Hartenberg kinematic modelling:
%
%     * 3D Simulation tab - drive the joints [theta1 theta2 d3 theta4] with
%       sliders or numeric fields, change the link lengths a1, a2, the
%       column height d1, the tool offset d4 and the quill stroke, and watch
%       the live DH table, the base-to-tool transform 0T4, the tool pose,
%       the manipulability and the singularity status.
%     * Workspace Analysis tab - samples the joint space, plots the
%       reachable area (top view) and volume (3-D) coloured by
%       manipulability or elbow configuration, and reports the reach,
%       planar area and workspace volume. The joint limits of theta1 and
%       theta2 can be changed here.
%
%   FIG = SCARA_SIMULATOR returns the figure handle.
%
%   Requirements: MATLAB R2018b or newer. No toolboxes are needed.
%
%   See also SCARA_FK, SCARA_WORKSPACE, SCARA_DRAW.

rootDir = fileparts(mfilename('fullpath'));
addpath(fullfile(rootDir, 'src'));

% ======================= simulator state ==============================
robot     = scara_params();
q         = robot.qHome;            % [theta1 theta2 d3 theta4] (rad, rad, m, rad)
trail     = zeros(0, 3);            % tool-tip history
MAX_TRAIL = 800;
ws        = [];                     % last workspace analysis
animating = false;

hRobot = [];   hTrail = [];   hFoot = [];
hWsArm = [];   hWsTip = [];

D2R = pi/180;
R2D = 180/pi;
TH  = char(952);                    % theta
AL  = char(945);                    % alpha
PHI = char(966);                    % phi
SQ  = char(178);                    % superscript 2
CU  = char(179);                    % superscript 3

jointNames = {[TH '1'], [TH '2'], 'd3', [TH '4']};
jointUnits = {'deg', 'deg', 'mm', 'deg'};
jointScale = [R2D, R2D, 1000, R2D];  % internal (rad / m) -> UI (deg / mm)
limField   = {'theta1Lim', 'theta2Lim', 'd3Lim', 'theta4Lim'};

bg    = [0.95 0.96 0.98];
white = [1 1 1];

% ======================= window and layout ============================
scr = get(groot, 'ScreenSize');
W = min(1360, scr(3) - 60);
H = min(830, scr(4) - 90);
fig = uifigure('Name', 'SCARA Robotic Arm Simulator', ...
    'Position', [max(20, (scr(3) - W)/2), max(40, (scr(4) - H)/2), W, H], ...
    'Color', bg);
fig.CloseRequestFcn = @(~, ~) onClose();

root = uigridlayout(fig, [1 2]);
root.ColumnWidth = {380, '1x'};
root.RowHeight = {'1x'};
root.Padding = [10 10 10 10];
root.ColumnSpacing = 12;
root.BackgroundColor = bg;

% ----------------------- left column: controls ------------------------
left = uigridlayout(root, [5 1]);
left.Layout.Row = 1;
left.Layout.Column = 1;
left.RowHeight = {56, 190, 140, 168, '1x'};
left.Padding = [0 0 0 0];
left.RowSpacing = 8;
left.BackgroundColor = bg;
if isprop(left, 'Scrollable')               % R2020b+: scroll on small screens
    left.Scrollable = 'on';
end

hdr = uigridlayout(left, [2 1]);
hdr.RowHeight = {30, 18};
hdr.Padding = [4 0 0 0];
hdr.RowSpacing = 0;
hdr.BackgroundColor = bg;
uilabel(hdr, 'Text', 'SCARA Robotic Arm Simulator', 'FontSize', 19, ...
    'FontWeight', 'bold', 'FontColor', [0.10 0.20 0.36]);
uilabel(hdr, 'Text', 'DH-based forward kinematics & workspace analysis', ...
    'FontSize', 11, 'FontColor', [0.36 0.40 0.48]);

% Joint variables ---------------------------------------------------------
pJ = uipanel(left, 'Title', 'Joint variables', 'FontWeight', 'bold', ...
    'BackgroundColor', white);
gJ = uigridlayout(pJ, [4 3]);
gJ.ColumnWidth = {72, '1x', 72};
gJ.RowHeight = {'1x', '1x', '1x', '1x'};
gJ.BackgroundColor = white;
sld = cell(1, 4);
edt = cell(1, 4);
for iJ = 1:4
    uilabel(gJ, 'Text', sprintf('%s  (%s)', jointNames{iJ}, jointUnits{iJ}), ...
        'FontWeight', 'bold');
    sld{iJ} = uislider(gJ, ...
        'Limits', robot.(limField{iJ}) * jointScale(iJ), ...
        'Value', q(iJ) * jointScale(iJ), ...
        'MajorTicks', [], 'MinorTicks', [], ...
        'ValueChangingFcn', @(src, evt) setJoint(iJ, evt.Value, true), ...
        'ValueChangedFcn',  @(src, evt) setJoint(iJ, src.Value, true));
    edt{iJ} = uieditfield(gJ, 'numeric', ...
        'Limits', robot.(limField{iJ}) * jointScale(iJ), ...
        'Value', q(iJ) * jointScale(iJ), ...
        'ValueDisplayFormat', '%.1f', 'HorizontalAlignment', 'center', ...
        'ValueChangedFcn', @(src, evt) setJoint(iJ, src.Value, false));
end

% Link lengths ------------------------------------------------------------
pL = uipanel(left, 'Title', 'Link lengths & stroke', 'FontWeight', 'bold', ...
    'BackgroundColor', white);
gL = uigridlayout(pL, [3 4]);
gL.ColumnWidth = {78, '1x', 78, '1x'};
gL.RowHeight = {22, 22, 22};
gL.BackgroundColor = white;
linkSpec = { 'a1',     'a1  (mm)',     [50 1000],  robot.a1;
             'a2',     'a2  (mm)',     [50 1000],  robot.a2;
             'd1',     'd1  (mm)',     [100 1500], robot.d1;
             'd4',     'd4  (mm)',     [0 300],    robot.d4;
             'stroke', 'Stroke (mm)',  [10 800],   robot.d3Lim(2) - robot.d3Lim(1) };
for iL = 1:size(linkSpec, 1)
    uilabel(gL, 'Text', linkSpec{iL, 2});
    uispinner(gL, 'Limits', linkSpec{iL, 3}, ...
        'Value', round(1000*linkSpec{iL, 4}), 'Step', 10, ...
        'ValueDisplayFormat', '%.0f', ...
        'ValueChangedFcn', @(src, evt) onGeometryChanged(linkSpec{iL, 1}, src.Value));
end

% Display & motion --------------------------------------------------------
pD = uipanel(left, 'Title', 'Display & motion', 'FontWeight', 'bold', ...
    'BackgroundColor', white);
gD = uigridlayout(pD, [4 2]);
gD.RowHeight = {22, 22, 28, 28};
gD.RowSpacing = 6;
gD.BackgroundColor = white;
cbFrames = uicheckbox(gD, 'Text', 'Show DH frames', 'Value', true, ...
    'ValueChangedFcn', @(~, ~) refresh(false));
cbTrail = uicheckbox(gD, 'Text', 'Show tool trail', 'Value', true, ...
    'ValueChangedFcn', @(~, ~) refresh(false));
cbFoot = uicheckbox(gD, 'Text', 'Workspace footprint', 'Value', true, ...
    'ValueChangedFcn', @(~, ~) updateFootprint());
cbXray = uicheckbox(gD, 'Text', 'Transparent links', 'Value', false, ...
    'ValueChangedFcn', @(~, ~) refresh(false));
uibutton(gD, 'Text', 'Home', 'ButtonPushedFcn', @(~, ~) onHome());
uibutton(gD, 'Text', 'Random pose', 'ButtonPushedFcn', @(~, ~) onRandom());
btnAnim = uibutton(gD, 'Text', 'Animate', 'ButtonPushedFcn', @(~, ~) onAnimate());
uibutton(gD, 'Text', 'Clear trail', 'ButtonPushedFcn', @(~, ~) onClearTrail());

% End-effector pose -------------------------------------------------------
pP = uipanel(left, 'Title', 'End-effector pose (tool tip)', 'FontWeight', 'bold', ...
    'BackgroundColor', white);
gP = uigridlayout(pP, [6 2]);
gP.ColumnWidth = {170, '1x'};
gP.RowHeight = {20, 20, 20, 20, 20, 26};
gP.RowSpacing = 6;
gP.Padding = [10 6 10 6];
gP.BackgroundColor = white;
poseNames = {'X  (mm)', 'Y  (mm)', 'Z  (mm)', [PHI '  tool yaw  (deg)'], ...
             ['Manipulability  w  (m' SQ ')']};
lblPose = cell(1, 5);
for iP = 1:5
    uilabel(gP, 'Text', poseNames{iP});
    lblPose{iP} = uilabel(gP, 'Text', '-', 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'right');
end
gS = uigridlayout(gP, [1 2]);
gS.Layout.Row = 6;
gS.Layout.Column = [1 2];
gS.ColumnWidth = {22, '1x'};
gS.Padding = [0 0 0 0];
gS.BackgroundColor = white;
lamp = uilamp(gS, 'Color', [0.20 0.75 0.30]);
lblStatus = uilabel(gS, 'Text', 'Regular configuration');

% ----------------------- right column: tabs ---------------------------
tabs = uitabgroup(root);
tabs.Layout.Row = 1;
tabs.Layout.Column = 2;
tabSim = uitab(tabs, 'Title', '3D Simulation', 'BackgroundColor', white);
tabWs  = uitab(tabs, 'Title', 'Workspace Analysis', 'BackgroundColor', white);

% 3D simulation tab --------------------------------------------------------
gSim = uigridlayout(tabSim, [2 1]);
gSim.RowHeight = {'1x', 196};
gSim.BackgroundColor = white;
ax3D = uiaxes(gSim);
title(ax3D, 'SCARA manipulator  -  3D view');

gTab = uigridlayout(gSim, [1 2]);
gTab.ColumnWidth = {'1.1x', '1x'};
gTab.Padding = [0 0 0 0];
gTab.BackgroundColor = white;
pDH = uipanel(gTab, 'Title', 'DH parameter table   (* = joint variable)', ...
    'FontWeight', 'bold', 'BackgroundColor', white);
gDH = uigridlayout(pDH, [1 1]);
gDH.Padding = [4 4 4 4];
tblDH = uitable(gDH, 'ColumnName', {'Link', [TH '_i (deg)'], 'd_i (mm)', ...
    'a_i (mm)', [AL '_i (deg)']}, 'RowName', {}, 'ColumnEditable', false);
pT = uipanel(gTab, 'Title', 'Base-to-tool transform   T04 = A1 A2 A3 A4   (m)', ...
    'FontWeight', 'bold', 'BackgroundColor', white);
gT = uigridlayout(pT, [1 1]);
gT.Padding = [4 4 4 4];
tblT = uitable(gT, 'ColumnName', {'n  (x4)', 's  (y4)', 'a  (z4)', 'p  (origin)'}, ...
    'RowName', {}, 'ColumnEditable', false);

% Workspace tab ------------------------------------------------------------
gWs = uigridlayout(tabWs, [2 1]);
gWs.RowHeight = {34, '1x'};
gWs.BackgroundColor = white;
gCtl = uigridlayout(gWs, [1 10]);
gCtl.ColumnWidth = {104, 62, 104, 62, 54, 66, 62, 150, '1x', 96};
gCtl.ColumnSpacing = 8;
gCtl.Padding = [6 4 6 4];
gCtl.BackgroundColor = white;
uilabel(gCtl, 'Text', [TH '1 limit  ' char(177) ' (deg)']);
spnT1 = uispinner(gCtl, 'Limits', [10 180], 'Step', 5, ...
    'Value', round(robot.theta1Lim(2)*R2D), ...
    'ValueChangedFcn', @(~, ~) onLimitsChanged());
uilabel(gCtl, 'Text', [TH '2 limit  ' char(177) ' (deg)']);
spnT2 = uispinner(gCtl, 'Limits', [10 180], 'Step', 5, ...
    'Value', round(robot.theta2Lim(2)*R2D), ...
    'ValueChangedFcn', @(~, ~) onLimitsChanged());
uilabel(gCtl, 'Text', 'Samples');
spnN = uispinner(gCtl, 'Limits', [40 400], 'Step', 20, 'Value', 120, ...
    'ValueChangedFcn', @(~, ~) recomputeWorkspace());
uilabel(gCtl, 'Text', 'Colour by');
ddColor = uidropdown(gCtl, 'Items', {'Manipulability', 'Elbow configuration'}, ...
    'Value', 'Manipulability', 'ValueChangedFcn', @(~, ~) recomputeWorkspace());
uilabel(gCtl, 'Text', '');
uibutton(gCtl, 'Text', 'Recompute', 'ButtonPushedFcn', @(~, ~) recomputeWorkspace());

gWsB = uigridlayout(gWs, [1 3]);
gWsB.ColumnWidth = {'1x', '1x', 270};
gWsB.Padding = [0 0 0 0];
gWsB.BackgroundColor = white;
axTop = uiaxes(gWsB);
axWs3 = uiaxes(gWsB);
pM = uipanel(gWsB, 'Title', 'Workspace metrics', 'FontWeight', 'bold', ...
    'BackgroundColor', white);
gM = uigridlayout(pM, [2 1]);
gM.RowHeight = {'1x', 96};
gM.BackgroundColor = white;
tblM = uitable(gM, 'ColumnName', {'Metric', 'Value'}, 'RowName', {}, ...
    'ColumnWidth', {130, 'auto'}, 'ColumnEditable', false);
uilabel(gM, 'FontSize', 10, 'FontColor', [0.38 0.40 0.46], ...
    'VerticalAlignment', 'top', 'Text', { ...
    ['Area: exact polar integral over ' TH '2;'], ...
    'volume = area x vertical stroke.', ...
    'Black curves: images of the joint-limit', ...
    ['edges and of the singular arc ' TH '2 = 0.'], ...
    'Star: current tool position.'});

% ======================= initial render ===============================
hRobot = scara_draw(ax3D, q, robot, [], struct('showFrames', true));
hTrail = line('Parent', ax3D, 'XData', nan, 'YData', nan, 'ZData', nan, ...
    'Color', [0.55 0.20 0.75], 'LineWidth', 1.5);
hFoot = line('Parent', ax3D, 'XData', nan, 'YData', nan, 'ZData', nan, ...
    'Color', [0.20 0.50 0.85], 'LineWidth', 1.2, 'LineStyle', '--');

recomputeWorkspace();
refresh(true);

if nargout > 0
    varargout{1} = fig;
end

% ======================================================================
%                              callbacks
% ======================================================================
    function setJoint(jIdx, uiValue, fromSlider)
        % Slider / numeric field moved: update q and the twin control.
        q(jIdx) = uiValue / jointScale(jIdx);
        edt{jIdx}.Value = uiValue;
        if ~fromSlider
            sld{jIdx}.Value = uiValue;
        end
        refresh(true);
    end

    function onGeometryChanged(fieldName, mmValue)
        % Link length / column height / tool offset / stroke changed.
        if strcmp(fieldName, 'stroke')
            robot.d3Lim = [0, mmValue/1000];
            applyJointLimits();
        else
            robot.(fieldName) = mmValue/1000;
        end
        trail = zeros(0, 3);
        recomputeWorkspace();
        refresh(false);
    end

    function onLimitsChanged()
        % Symmetric joint limits of theta1 / theta2 changed.
        robot.theta1Lim = [-1 1] * spnT1.Value * D2R;
        robot.theta2Lim = [-1 1] * spnT2.Value * D2R;
        applyJointLimits();
        trail = zeros(0, 3);
        recomputeWorkspace();
        refresh(false);
    end

    function onHome()
        moveTo(clampToLimits(robot.qHome), 40, false);
    end

    function onRandom()
        moveTo(randomPose(0.9), 50, false);
    end

    function onClearTrail()
        trail = zeros(0, 3);
        refresh(false);
    end

    function onAnimate()
        % Toggle a continuous demo motion through random poses.
        if animating
            animating = false;            % the running loop will stop
            return
        end
        animating = true;
        btnAnim.Text = 'Stop';
        while animating && isvalid(fig)
            moveTo(randomPose(0.85), 70, true);
        end
        if isvalid(fig)
            btnAnim.Text = 'Animate';
        end
    end

    function onClose()
        animating = false;
        delete(fig);
    end

% ======================================================================
%                               helpers
% ======================================================================
    function refresh(addTrail)
        % Redraw the arm and update every read-out.
        if ~isvalid(fig)
            return
        end
        drawOpts = struct('showFrames', cbFrames.Value, 'transparent', cbXray.Value);
        hRobot = scara_draw(ax3D, q, robot, hRobot, drawOpts);
        [T04, ~, Pts, pose] = scara_fk(q, robot);
        [~, wManip] = scara_jacobian(q, robot);

        tipPos = T04(1:3, 4).';
        if addTrail && cbTrail.Value
            if isempty(trail) || norm(trail(end, :) - tipPos) > 1e-6
                trail(end + 1, :) = tipPos;
                if size(trail, 1) > MAX_TRAIL
                    trail(1:end - MAX_TRAIL, :) = [];
                end
            end
        end
        if cbTrail.Value && ~isempty(trail)
            set(hTrail, 'XData', trail(:, 1), 'YData', trail(:, 2), ...
                'ZData', trail(:, 3), 'Visible', 'on');
        else
            set(hTrail, 'Visible', 'off');
        end

        updateTables(T04);
        updatePose(pose, wManip);
        updateWorkspaceMarker(Pts);
        drawnow limitrate
    end

    function updateTables(T04)
        DH = scara_dh_table(q, robot);
        isVar = logical([1 0 0 0; 1 0 0 0; 0 1 0 0; 1 0 0 0]);  % [theta d a alpha]
        dhScale = [R2D, 1000, 1000, R2D];
        data = cell(4, 5);
        for rr = 1:4
            data{rr, 1} = sprintf('%d', rr);
            for cc = 1:4
                str = sprintf('%.1f', DH(rr, cc) * dhScale(cc));
                if isVar(rr, cc)
                    str = [str ' *']; %#ok<AGROW>
                end
                data{rr, cc + 1} = str;
            end
        end
        tblDH.Data = data;

        T04(abs(T04) < 5e-5) = 0;               % avoid "-0.0000"
        tblT.Data = arrayfun(@(x) sprintf('%.4f', x), T04, 'UniformOutput', false);
    end

    function updatePose(pose, wManip)
        lblPose{1}.Text = sprintf('%.1f', 1000*pose(1));
        lblPose{2}.Text = sprintf('%.1f', 1000*pose(2));
        lblPose{3}.Text = sprintf('%.1f', 1000*pose(3));
        lblPose{4}.Text = sprintf('%.1f', R2D*pose(4));
        lblPose{5}.Text = sprintf('%.4f', wManip);

        if pose(3) < 0
            lamp.Color = [0.85 0.20 0.20];
            lblStatus.Text = 'Tool tip below the base plane (z < 0)';
        elseif abs(sin(q(2))) < sin(10*D2R)
            lamp.Color = [0.95 0.65 0.10];
            lblStatus.Text = ['Near singularity  (' TH '2 close to 0' char(176) ...
                ' or ' char(177) '180' char(176) ')'];
        else
            lamp.Color = [0.20 0.75 0.30];
            lblStatus.Text = 'Regular configuration';
        end
    end

    function recomputeWorkspace()
        % Sample the workspace, redraw both workspace plots and the metrics.
        ws = scara_workspace(robot, round(spnN.Value));
        colorMode = 'manipulability';
        if strcmp(ddColor.Value, 'Elbow configuration')
            colorMode = 'elbow';
        end
        scara_plot_workspace(ws, robot, axTop, axWs3, colorMode);

        hold(axTop, 'on');
        hWsArm = plot(axTop, nan, nan, '-o', 'Color', [0.10 0.10 0.12], ...
            'LineWidth', 2.5, 'MarkerSize', 5, 'MarkerFaceColor', [1 1 1], ...
            'HandleVisibility', 'off');
        hWsTip = plot(axTop, nan, nan, 'p', 'MarkerSize', 15, ...
            'MarkerFaceColor', [0.90 0.20 0.20], 'MarkerEdgeColor', [0 0 0], ...
            'HandleVisibility', 'off');
        hold(axTop, 'off');

        updateMetrics();
        updateFootprint();
        [~, ~, Pts] = scara_fk(q, robot);
        updateWorkspaceMarker(Pts);
    end

    function updateMetrics()
        tblM.Data = { ...
            'Max reach  r_max',      sprintf('%.1f mm', 1000*ws.rMax);
            'Min reach  r_min',      sprintf('%.1f mm', 1000*ws.rMin);
            'Planar area',           sprintf('%.4f m%s', ws.area, SQ);
            'Workspace volume',      sprintf('%.4f m%s', ws.volume, CU);
            'Vertical stroke',       sprintf('%.0f mm', 1000*ws.stroke);
            'Tool-tip Z range',      sprintf('%.0f to %.0f mm', 1000*ws.zRange(1), 1000*ws.zRange(2));
            [TH '1 range'],          sprintf('%.0f to %.0f deg', R2D*robot.theta1Lim(1), R2D*robot.theta1Lim(2));
            [TH '2 range'],          sprintf('%.0f to %.0f deg', R2D*robot.theta2Lim(1), R2D*robot.theta2Lim(2));
            'Max manipulability',    sprintf('%.4f m%s', robot.a1*robot.a2, SQ);
            'Mean manipulability',   sprintf('%.4f m%s', mean(ws.manip(:)), SQ);
            'Samples',               sprintf('%d x %d', ws.n, ws.n)};
    end

    function updateFootprint()
        % Dashed outline of the workspace envelope on the floor of the 3-D view.
        if isempty(ws) || ~isvalid(fig)
            return
        end
        pts = zeros(2, 0);
        for kb = 1:numel(ws.boundary)
            pts = [pts, ws.boundary{kb}, [nan; nan]]; %#ok<AGROW>
        end
        zFloor = 0.002*(robot.a1 + robot.a2);
        set(hFoot, 'XData', pts(1, :), 'YData', pts(2, :), ...
            'ZData', zFloor*ones(1, size(pts, 2)), ...
            'Visible', onoff(cbFoot.Value));
    end

    function updateWorkspaceMarker(Pts)
        % Arm projection and tool position on the workspace top view.
        if ~isempty(hWsArm) && isvalid(hWsArm)
            set(hWsArm, 'XData', Pts(1, 1:3), 'YData', Pts(2, 1:3));
            set(hWsTip, 'XData', Pts(1, 5), 'YData', Pts(2, 5));
        end
    end

    function applyJointLimits()
        % Push robot joint limits to the sliders / fields, clamping q.
        q = clampToLimits(q);
        for jIdx = 1:4
            jl = robot.(limField{jIdx});
            uiVal = q(jIdx) * jointScale(jIdx);
            sld{jIdx}.Value = uiVal;          % valid for old and new limits
            edt{jIdx}.Value = uiVal;
            sld{jIdx}.Limits = jl * jointScale(jIdx);
            edt{jIdx}.Limits = jl * jointScale(jIdx);
        end
    end

    function qc = clampToLimits(qIn)
        qc = qIn;
        for jIdx = 1:4
            jl = robot.(limField{jIdx});
            qc(jIdx) = min(max(qIn(jIdx), jl(1)), jl(2));
        end
    end

    function qr = randomPose(frac)
        % Random configuration in the central FRAC of each joint range.
        qr = zeros(1, 4);
        for jIdx = 1:4
            jl = robot.(limField{jIdx});
            mid = mean(jl);
            qr(jIdx) = mid + frac*(rand - 0.5)*(jl(2) - jl(1));
        end
    end

    function moveTo(qTarget, nSteps, stopWhenIdle)
        % Smooth (cosine-blended) joint-space motion to qTarget.
        qStart = q;
        for st = 1:nSteps
            if ~isvalid(fig) || (stopWhenIdle && ~animating)
                return
            end
            blend = 0.5 - 0.5*cos(pi*st/nSteps);
            q = qStart + blend*(qTarget - qStart);
            syncControls();
            refresh(true);
            pause(0.012);
        end
    end

    function syncControls()
        for jIdx = 1:4
            uiVal = q(jIdx) * jointScale(jIdx);
            sld{jIdx}.Value = uiVal;
            edt{jIdx}.Value = uiVal;
        end
    end
end

% ======================================================================
function s = onoff(tf)
if tf
    s = 'on';
else
    s = 'off';
end
end
