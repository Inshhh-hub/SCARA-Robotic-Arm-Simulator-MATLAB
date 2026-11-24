function [T, frames, P, pose] = scara_fk(q, robot)
%SCARA_FK Forward kinematics of the SCARA arm from its DH parameters.
%
%   T = SCARA_FK(Q, ROBOT) returns the 4x4 homogeneous transform of the
%   tool frame {4} relative to the base frame {0}:
%
%       T = 0T4 = A1(theta1) * A2(theta2) * A3(d3) * A4(theta4)
%
%   where Ai = DH_TRANSFORM(theta_i, d_i, a_i, alpha_i) is built from row i
%   of SCARA_DH_TABLE.
%
%   [T, FRAMES, P, POSE] = SCARA_FK(Q, ROBOT) also returns
%     FRAMES - 4x4x5 array with FRAMES(:,:,k+1) = 0Tk for k = 0..4
%              (FRAMES(:,:,1) is the identity, i.e. the base frame)
%     P      - 3x5 matrix whose columns are the frame origins o0 .. o4
%     POSE   - [x y z phi]: tool-tip position (m) and tool yaw (rad).
%              For this DH assignment phi = theta1 + theta2 - theta4.
%
%   Q is [theta1 theta2 d3 theta4] in (rad, rad, m, rad). ROBOT defaults to
%   SCARA_PARAMS().
%
%   Example
%       robot = scara_params();
%       T = scara_fk([pi/6, pi/4, 0.05, 0], robot)
%
%   See also SCARA_IK, SCARA_JACOBIAN, SCARA_DH_TABLE.

if nargin < 2 || isempty(robot)
    robot = scara_params();
end

DH = scara_dh_table(q, robot);

frames = zeros(4, 4, 5);
frames(:, :, 1) = eye(4);
for i = 1:4
    Ai = dh_transform(DH(i, 1), DH(i, 2), DH(i, 3), DH(i, 4));
    frames(:, :, i + 1) = frames(:, :, i) * Ai;
end

T = frames(:, :, 5);
P = reshape(frames(1:3, 4, :), 3, 5);
pose = [T(1:3, 4).', atan2(T(2, 1), T(1, 1))];
end
