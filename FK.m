% Delta Robot - Inverse Kinematics
function E_0 = FK(q1, q2, q3, alpha, f, e, rf, re)

sym pi;
syms x y z;

OFi = f/2 * tan(30*pi/180);
EEi = e/2 * tan(30*pi/180);
R = OFi - EEi;

% Rotation matrix about y-axis for virtual elbow joints J1' J2' J3'
Roty1 = [cos(alpha(1))  0 sin(alpha(1));
             0       1     0;
        -sin(alpha(1)) 0 cos(alpha(1))];

Roty2 = [cos(alpha(2))  0 sin(alpha(2));
             0       1     0;
        -sin(alpha(2)) 0 cos(alpha(2))];

Roty3 = [cos(alpha(3))  0 sin(alpha(3));
             0       1     0;
        -sin(alpha(3)) 0 cos(alpha(3))];

% Coordinates for virtual elbow joints J1' J2' J3'    
J1 = [0; -re*sin(q1); R+rf*cos(q1)];
J1p = Roty1 * J1;

J2 = [0; -re*sin(q2); R+rf*cos(q2)];
J2p = Roty2 * J2;

J3 = [0; -re*sin(q3); R+rf*cos(q3)];
J3p = Roty3 * J3;

% Standard equation of a circle whose center is at (h,k)
%   => (x-h)^2 + (y-k)^2 = r^2

% Sphere with center at J1'
sphere_J1p = x^2 + (y-J1p(2))^2 + (z-J1p(3))^2 - re^2;

% Sphere with center at J2'
sphere_J2p = (x-J2p(1))^2 + (y-J2p(2))^2 + (z-J2p(3))^2 - re^2;

% Sphere with center at J3'
sphere_J3p = (x-J3p(1))^2 + (y-J3p(2))^2 + (z-J3p(3))^2 - re^2;

% ------- testing findSphereIntersection function ----
c1 = [0 J1p(2) J1p(3)];
r1 = re;

c2 = [J2p(1) J2p(2) J2p(3)];
r2 = re;

c3 = [J3p(1) J3p(2) J3p(3)];
r3 = re;

% Find intersection of two circles
[E_0(1), E_0(2), E_0(3)] = findSphereIntersection(c1,r1,c2,r2,c3,r3);

% ---- end testing of func ----

% % Sphere J1' equation subtracted by sphere J2' equation
% J1J2 = sphere_J1p - sphere_J2p
% 
% % Sphere J1' equation subtracted by sphere J3' equation
% J1J3 = sphere_J1p - sphere_J3p
% 
% % Solve J1J2 for x
% x = solve(J1J2,x)
% 
% % Solve J1J3 for z
% z = solve(J1J2,z)
% 
% % Sub in x and z then solve equation for y. Two solutions appear. The
% % moving platform has negative y-coordinate due to placement of origin.
% % Therefore, the solution we want for y is the negative solution of
% % equation below.
% y = solve(subs(sphere_J1p,x,z),y) %doesn't seem to give me equation i want still has multiple terms in it
end








