% Delta Robot - Inverse Kinematics
clc; clear;

sym pi;
syms x y z;

% -----temp variable
f = 567; % equilateral triangle side of fixed platform
e = 76; % equilateral triangle side of moving platform
rf = 524; % Upper arm length in mm
re = 1244; % Forearm length in mm
q1 = 15;
q2 = 20;
q3 = 25;
% -----temp variable

OFi = f/2 * tan(30*pi/180);
EEi = e/2 * tan(30*pi/180);
R = OFi - EEi;

% Rotation matrix about y-axis for virtual elbow joints J1' J2' J3'
Roty1 = [1 0 0
         0 1 0
         0 0 1];

Roty2 = [cos(2*pi/3)  0 sin(2*pi/3);
             0       1     0;
        -sin(2*pi/3) 0 cos(2*pi/3)];

Roty3 = [cos(-2*pi/3)  0 sin(-2*pi/3);
             0       1     0;
        -sin(-2*pi/3) 0 cos(-2*pi/3)];

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
[z_Ji, y_Ji, x_Ji] = findSphereIntersection(c1,r1,c2,r2,c3,r3)

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









