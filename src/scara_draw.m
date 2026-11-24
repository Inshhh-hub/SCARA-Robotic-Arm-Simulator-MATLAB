function h = scara_draw(ax, q, robot, h, opts)
%SCARA_DRAW Render (or update) a 3-D model of the SCARA arm.
%
%   H = SCARA_DRAW(AX, Q, ROBOT) draws the arm in configuration Q into the
%   axes AX (regular axes or UIAxes) and returns a struct of graphics
%   handles.
%
%   H = SCARA_DRAW(AX, Q, ROBOT, H) updates an existing drawing in place.
%   Only vertex data is changed, so this is fast enough for interactive
%   sliders and animation. Always keep the returned H.
%
%   H = SCARA_DRAW(..., OPTS) accepts a struct with optional fields
%       showFrames  - draw the DH coordinate frames {0}..{4}   (true)
%       transparent - semi-transparent links ("x-ray" view)    (false)
%
%   Example
%       robot = scara_params();
%       figure; ax = axes;
%       h = scara_draw(ax, robot.qHome, robot);
%       for t = linspace(0, pi/2, 60)
%           h = scara_draw(ax, [t, pi/4, 0.05, 0], robot, h);
%           drawnow
%       end
%
%   See also SCARA_FK, SCARA_PLOT_WORKSPACE.

if nargin < 4, h = []; end
if nargin < 5 || isempty(opts), opts = struct(); end
if ~isfield(opts, 'showFrames'),  opts.showFrames  = true;  end
if ~isfield(opts, 'transparent'), opts.transparent = false; end

[~, frames, P] = scara_fk(q, robot);
dims = local_dims(robot);

if isempty(h) || ~isstruct(h) || ~isfield(h, 'ground') || ~isvalid(h.ground)
    h = local_create(ax);
end

o1 = P(:, 2).';                 % elbow (origin of frame 1)
o2 = P(:, 3).';                 % quill axis at the arm plane (frame 2)
o3 = P(:, 4).';                 % bottom of the quill (frame 3)
o4 = P(:, 5).';                 % tool tip (frame 4)
d1 = robot.d1;
t  = dims.t;

% ---- static parts (scale with the link lengths) ---------------------
g = dims.ground;
set(h.ground, 'XData', [-g; g; g; -g], 'YData', [-g; -g; g; g], 'ZData', [0; 0; 0; 0]);
local_cyl_update(h.plate,  h, [0 0], dims.rp, 0, dims.hp);
local_cyl_update(h.column, h, [0 0], dims.rb, dims.hp, max(dims.hp, d1 - t));

% ---- moving links ----------------------------------------------------
local_link_update(h.link1, h, [0 0], o1, dims.w1, d1 - t, d1);
local_link_update(h.link2, h, o1,    o2, dims.w2, d1,     d1 + t);

% ---- prismatic quill and tool ----------------------------------------
zq = o3(3);
local_cyl_update(h.quill, h, o2, dims.rq, zq, zq + dims.Lq);
local_cyl_update(h.tool,  h, o2, dims.rt, o4(3), zq);

% ---- gripper (shows the tool roll theta4) ----------------------------
x4 = frames(1:3, 1, 5).';
gA = o4 + dims.grip*x4;
gB = o4 - dims.grip*x4;
dn = [0 0 -dims.finger];
G  = [gA; gB; nan(1, 3); gA; gA + dn; nan(1, 3); gB; gB + dn];
set(h.gripper, 'XData', G(:, 1), 'YData', G(:, 2), 'ZData', G(:, 3));
set(h.tip, 'XData', o4(1), 'YData', o4(2), 'ZData', o4(3));

% ---- DH coordinate frames ---------------------------------------------
Lf = dims.frameLen;
XX = nan(15, 3);  YY = XX;  ZZ = XX;
for k = 1:5
    o = frames(1:3, 4, k).';
    R = frames(1:3, 1:3, k);
    rows = 3*k - 2 : 3*k;
    XX(rows, :) = [o; o + Lf*R(:, 1).'; nan(1, 3)];
    YY(rows, :) = [o; o + Lf*R(:, 2).'; nan(1, 3)];
    ZZ(rows, :) = [o; o + Lf*R(:, 3).'; nan(1, 3)];
    set(h.frameText(k), 'Position', o + Lf*[0.18 0.18 0.10]);
end
set(h.axX, 'XData', XX(:, 1), 'YData', XX(:, 2), 'ZData', XX(:, 3));
set(h.axY, 'XData', YY(:, 1), 'YData', YY(:, 2), 'ZData', YY(:, 3));
set(h.axZ, 'XData', ZZ(:, 1), 'YData', ZZ(:, 2), 'ZData', ZZ(:, 3));

vis = local_onoff(opts.showFrames);
set([h.axX, h.axY, h.axZ], 'Visible', vis);
set(h.frameText, 'Visible', vis);

% ---- transparency ---------------------------------------------------
alpha = 1;
if opts.transparent, alpha = 0.45; end
if ~isfield(h, 'alpha') || h.alpha ~= alpha
    set(h.solids, 'FaceAlpha', alpha);
    h.alpha = alpha;
end

% ---- axis limits only when the geometry changes (keeps user zoom) ----
key = [robot.a1, robot.a2, robot.d1, robot.d4, robot.d3Lim];
if ~isfield(h, 'key') || ~isequal(h.key, key)
    zTop = d1 - robot.d3Lim(1) + dims.Lq + 0.05*dims.s;
    zLow = min(0, d1 - robot.d3Lim(2) - robot.d4 - dims.finger - 0.04*dims.s);
    set(ax, 'XLim', [-g g], 'YLim', [-g g], 'ZLim', [zLow zTop]);
    h.key = key;
end
end

% =====================================================================
function d = local_dims(robot)
%LOCAL_DIMS Visual dimensions, proportional to the total reach.
s = robot.a1 + robot.a2;
d.s        = s;
d.w1       = 0.120*s;     % link-1 width
d.w2       = 0.095*s;     % link-2 width
d.t        = 0.045*s;     % link thickness
d.rb       = 0.060*s;     % column radius
d.rp       = 0.120*s;     % base plate radius
d.hp       = 0.020*s;     % base plate height
d.rq       = 0.016*s;     % quill radius
d.rt       = 0.010*s;     % tool radius
d.grip     = 0.035*s;     % half-width of the gripper
d.finger   = 0.030*s;     % finger length
d.frameLen = 0.140*s;     % length of the frame axes
d.ground   = 1.250*s;     % half-size of the floor
d.Lq       = robot.d3Lim(2) + d.t + 0.08*s;   % quill length
end

% =====================================================================
function h = local_create(ax)
%LOCAL_CREATE Create every graphics object once (geometry is set later).
cBase  = [0.30 0.32 0.36];
cLink1 = [0.16 0.44 0.78];
cLink2 = [0.94 0.55 0.16];
cQuill = [0.78 0.79 0.82];
cTool  = [0.85 0.20 0.20];

[h.ucx, h.ucy, h.ucz] = cylinder(1, 40);     % unit cylinder template

h.ground = patch('Parent', ax, 'XData', nan(4, 1), 'YData', nan(4, 1), ...
    'ZData', zeros(4, 1), 'FaceColor', [0.93 0.94 0.96], ...
    'EdgeColor', [0.78 0.80 0.84], 'FaceLighting', 'none');

h.plate  = local_cyl_create(ax, 0.85*cBase);
h.column = local_cyl_create(ax, cBase);
h.link1  = local_link_create(ax, cLink1);
h.link2  = local_link_create(ax, cLink2);
h.quill  = local_cyl_create(ax, cQuill);
h.tool   = local_cyl_create(ax, cTool);

h.solids = [h.plate.side, h.plate.caps, h.column.side, h.column.caps, ...
    h.link1.box, h.link1.endA.side, h.link1.endA.caps, h.link1.endB.side, h.link1.endB.caps, ...
    h.link2.box, h.link2.endA.side, h.link2.endA.caps, h.link2.endB.side, h.link2.endB.caps, ...
    h.quill.side, h.quill.caps, h.tool.side, h.tool.caps];

h.gripper = line('Parent', ax, 'XData', nan, 'YData', nan, 'ZData', nan, ...
    'Color', 0.75*cTool, 'LineWidth', 3);
h.tip = line('Parent', ax, 'XData', nan, 'YData', nan, 'ZData', nan, ...
    'LineStyle', 'none', 'Marker', 'o', 'MarkerSize', 6, ...
    'MarkerFaceColor', [1 1 1], 'MarkerEdgeColor', [0 0 0]);

h.axX = line('Parent', ax, 'XData', nan, 'YData', nan, 'ZData', nan, ...
    'Color', [0.86 0.15 0.15], 'LineWidth', 2);
h.axY = line('Parent', ax, 'XData', nan, 'YData', nan, 'ZData', nan, ...
    'Color', [0.10 0.62 0.20], 'LineWidth', 2);
h.axZ = line('Parent', ax, 'XData', nan, 'YData', nan, 'ZData', nan, ...
    'Color', [0.12 0.32 0.90], 'LineWidth', 2);

h.frameText = gobjects(1, 5);
for k = 1:5
    h.frameText(k) = text(0, 0, 0, sprintf('{%d}', k - 1), 'Parent', ax, ...
        'Interpreter', 'none', 'FontSize', 9, 'FontWeight', 'bold', ...
        'Color', [0.20 0.20 0.25]);
end

try %#ok<TRYNC> lighting is cosmetic; skip it where unsupported
    light('Parent', ax, 'Style', 'infinite', 'Position', [1 -1.5 2]);
    light('Parent', ax, 'Style', 'infinite', 'Position', [-2 1 1], ...
        'Color', [0.45 0.45 0.45]);
end

set(ax, 'DataAspectRatio', [1 1 1]);
view(ax, -38, 24);
grid(ax, 'on');
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
zlabel(ax, 'Z (m)');
end

% =====================================================================
function c = local_cyl_create(ax, color)
c.side = surface('Parent', ax, 'XData', zeros(2), 'YData', zeros(2), ...
    'ZData', zeros(2), 'FaceColor', color, 'EdgeColor', 'none', ...
    'FaceLighting', 'gouraud', 'AmbientStrength', 0.45, ...
    'DiffuseStrength', 0.7, 'SpecularStrength', 0.3);
c.caps = patch('Parent', ax, 'Vertices', zeros(3), 'Faces', [1 2 3], ...
    'FaceColor', color, 'EdgeColor', 'none', 'FaceLighting', 'flat', ...
    'AmbientStrength', 0.45);
end

% =====================================================================
function local_cyl_update(c, h, center, r, z0, z1)
X = center(1) + r*h.ucx;
Y = center(2) + r*h.ucy;
Z = z0 + (z1 - z0)*h.ucz;
set(c.side, 'XData', X, 'YData', Y, 'ZData', Z);

n = size(h.ucx, 2) - 1;               % last point repeats the first
ring = [X(1, 1:n).', Y(1, 1:n).'];
V = [ring, z0*ones(n, 1); ring, z1*ones(n, 1)];
set(c.caps, 'Vertices', V, 'Faces', [1:n; n+1:2*n]);
end

% =====================================================================
function L = local_link_create(ax, color)
L.box = patch('Parent', ax, 'Vertices', zeros(8, 3), ...
    'Faces', [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8], ...
    'FaceColor', color, 'EdgeColor', 'none', 'FaceLighting', 'flat', ...
    'AmbientStrength', 0.45);
L.endA = local_cyl_create(ax, color);
L.endB = local_cyl_create(ax, color);
end

% =====================================================================
function local_link_update(L, h, pA, pB, width, z0, z1)
%LOCAL_LINK_UPDATE Rounded slab from pA to pB (XY) between heights z0, z1.
d = pB(1:2) - pA(1:2);
len = norm(d);
if len < eps
    u = [1 0];
else
    u = d / len;
end
nrm = [-u(2), u(1)] * (width/2);
c = [pA(1:2) - nrm; pB(1:2) - nrm; pB(1:2) + nrm; pA(1:2) + nrm];
set(L.box, 'Vertices', [c, z0*ones(4, 1); c, z1*ones(4, 1)]);
local_cyl_update(L.endA, h, pA, width/2, z0, z1);
local_cyl_update(L.endB, h, pB, width/2, z0, z1);
end

% =====================================================================
function s = local_onoff(tf)
if tf
    s = 'on';
else
    s = 'off';
end
end
