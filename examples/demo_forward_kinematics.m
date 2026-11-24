%DEMO_FORWARD_KINEMATICS Forward kinematics of the SCARA arm via DH matrices.
%
%   Builds the DH table for one configuration, prints every link transform
%   A1..A4, multiplies them into 0T4 and checks the result against the
%   closed-form expression. Finally draws the arm with its DH frames.

clear; clc;
here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end
addpath(fullfile(here, '..', 'src'));

robot = scara_params();
deg = pi/180;
q = [40*deg, -60*deg, 0.08, 25*deg];        % [theta1 theta2 d3 theta4]

%% DH table
DH = scara_dh_table(q, robot);
fprintf('DH table  [theta_i (deg)   d_i (m)   a_i (m)   alpha_i (deg)]\n');
fprintf('  Link %d: %8.2f  %8.3f  %8.3f  %8.2f\n', ...
    [1:4; DH(:, 1).'/deg; DH(:, 2).'; DH(:, 3).'; DH(:, 4).'/deg]);

%% Link transforms and their product
T = eye(4);
for i = 1:4
    Ai = dh_transform(DH(i, 1), DH(i, 2), DH(i, 3), DH(i, 4));
    fprintf('\nA%d =\n', i);
    disp(round(Ai, 4));
    T = T * Ai;
end
fprintf('0T4 = A1*A2*A3*A4 =\n');
disp(round(T, 4));

%% Compare with the library function and the closed form
[Tfk, ~, ~, pose] = scara_fk(q, robot);
phi = q(1) + q(2) - q(4);
Tcf = [cos(phi),  sin(phi),  0, robot.a1*cos(q(1)) + robot.a2*cos(q(1) + q(2));
       sin(phi), -cos(phi),  0, robot.a1*sin(q(1)) + robot.a2*sin(q(1) + q(2));
       0,         0,        -1, robot.d1 - q(3) - robot.d4;
       0,         0,         0, 1];

fprintf('Tool tip : x = %.4f m, y = %.4f m, z = %.4f m, phi = %.2f deg\n', ...
    pose(1), pose(2), pose(3), pose(4)/deg);
fprintf('max |scara_fk - manual product| = %.2e\n', max(abs(Tfk(:) - T(:))));
fprintf('max |scara_fk - closed form|    = %.2e\n', max(abs(Tfk(:) - Tcf(:))));

[~, w] = scara_jacobian(q, robot);
fprintf('Manipulability w = %.4f m^2 (a1*a2*|sin(theta2)| = %.4f m^2)\n', ...
    w, robot.a1*robot.a2*abs(sin(q(2))));

%% Draw the arm
figure('Name', 'SCARA - forward kinematics', 'Color', 'w');
ax = axes;
scara_draw(ax, q, robot);
title(ax, 'SCARA at q = [40^\circ, -60^\circ, 80 mm, 25^\circ] with DH frames');
