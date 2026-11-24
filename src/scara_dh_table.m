function DH = scara_dh_table(q, robot)
%SCARA_DH_TABLE Standard DH parameter table of the SCARA (RRPR) arm.
%
%   DH = SCARA_DH_TABLE(Q, ROBOT) returns a 4x4 matrix whose row i is
%   [theta_i, d_i, a_i, alpha_i] for link i, evaluated at the joint vector
%   Q = [theta1 theta2 d3 theta4] (rad, rad, m, rad).
%
%       Link |  theta_i  |  d_i  |  a_i  | alpha_i
%       -----+-----------+-------+-------+--------
%         1  |  theta1*  |  d1   |  a1   |   0
%         2  |  theta2*  |  0    |  a2   |   pi
%         3  |  0        |  d3*  |  0    |   0
%         4  |  theta4*  |  d4   |  0    |   0
%
%   (* = joint variable). The twist alpha2 = pi turns z2 downwards, so a
%   positive d3 lowers the quill, as on a real SCARA.
%
%   See also DH_TRANSFORM, SCARA_FK, SCARA_PARAMS.

if nargin < 2 || isempty(robot)
    robot = scara_params();
end
if numel(q) ~= 4
    error('scara_dh_table:InvalidInput', ...
        'q must be [theta1 theta2 d3 theta4].');
end

DH = [ q(1),  robot.d1,  robot.a1,  0;
       q(2),  0,         robot.a2,  pi;
       0,     q(3),      0,         0;
       q(4),  robot.d4,  0,         0 ];
end
