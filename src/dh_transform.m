function A = dh_transform(theta, d, a, alpha)
%DH_TRANSFORM Homogeneous link transform, standard Denavit-Hartenberg convention.
%
%   A = DH_TRANSFORM(THETA, D, A, ALPHA) returns the 4x4 matrix that maps
%   coordinates in frame {i} to frame {i-1}:
%
%       A = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
%
%         = [ cos(theta)  -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)  a*cos(theta)
%             sin(theta)   cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)  a*sin(theta)
%                 0             sin(alpha)              cos(alpha)             d
%                 0                 0                       0                  1      ]
%
%   Inputs (scalars)
%     THETA - joint angle about z_{i-1}               [rad]
%     D     - link offset along z_{i-1}                 [m]
%     A     - link length along x_i                     [m]
%     ALPHA - link twist about x_i                    [rad]
%
%   See also SCARA_DH_TABLE, SCARA_FK.

ct = cos(theta);  st = sin(theta);
ca = cos(alpha);  sa = sin(alpha);

A = [ ct, -st*ca,  st*sa, a*ct;
      st,  ct*ca, -ct*sa, a*st;
       0,     sa,     ca,    d;
       0,      0,      0,    1 ];
end
