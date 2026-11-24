function [J, w] = scara_jacobian(q, robot)
%SCARA_JACOBIAN Geometric Jacobian and manipulability of the SCARA arm.
%
%   J = SCARA_JACOBIAN(Q, ROBOT) returns the 6x4 geometric Jacobian that
%   maps joint rates [dtheta1 dtheta2 dd3 dtheta4] to the tool twist
%   [v; omega] expressed in the base frame. Column i is built from the DH
%   frames returned by SCARA_FK:
%
%       revolute joint i :  [ z_{i-1} x (o_4 - o_{i-1}) ;  z_{i-1} ]
%       prismatic joint i:  [ z_{i-1}                   ;     0    ]
%
%   [J, W] = SCARA_JACOBIAN(Q, ROBOT) also returns Yoshikawa's
%   manipulability of the translational part, W = sqrt(det(Jv*Jv')),
%   which for the SCARA reduces to  W = a1 * a2 * |sin(theta2)|.
%   W = 0 at the singular configurations theta2 = 0 (arm fully stretched)
%   and theta2 = +/- pi (arm folded back).
%
%   See also SCARA_FK, SCARA_WORKSPACE.

if nargin < 2 || isempty(robot)
    robot = scara_params();
end

[~, frames] = scara_fk(q, robot);
oEnd = frames(1:3, 4, 5);

J = zeros(6, 4);
for i = 1:4
    z = frames(1:3, 3, i);     % z_{i-1}
    o = frames(1:3, 4, i);     % o_{i-1}
    if robot.jointType(i) == 'R'
        J(1:3, i) = cross(z, oEnd - o);
        J(4:6, i) = z;
    else
        J(1:3, i) = z;
    end
end

Jv = J(1:3, :);
w = sqrt(max(det(Jv * Jv.'), 0));
end
