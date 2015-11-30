% Delta Robot - Inverse Kinematics
function [qi, Fi, Ji, Ei] = IK(E, alpha, f, e, rf, re)

% Robot Parameters of the Delta Robot
sym pi;
syms x_Ji y_Ji z_Ji % Coordinates of the origin
syms x_Fi y_Fi z_Fi % Coordinates of joint Fi

% Rotation matrix about y-axis
Roty = [cos(alpha)  0 sin(alpha);
             0       1     0;
        -sin(alpha) 0 cos(alpha)];
    
E_0 = Roty * E; % Coordinate of TCP

% Coordinates of TCP, E(x0,y0,z0)
x0 = E_0(1);
y0 = E_0(2);
z0 = E_0(3);

EEi = e/2 * tan(30*pi/180); % from joint E to Ei
OFi = f/2 * tan(30*pi/180); % from joint O to Fi
EipJi = sqrt(re^2-x0^2); % Ei'Ji

% Standard equation of a circle whose center is at (h,k)
%   => (x-h)^2 + (y-k)^2 = r^2

% Equation of circle whose center is at joint Fi.
circ_Fi = (z_Ji-OFi)^2 + (y_Ji)^2 - (rf)^2;
c1 = [OFi 0];
r1 = rf;

% Equation of circle whose center is at joint Ei'. 
circ_Eip = (z_Ji-(z0+EEi))^2 + (y_Ji-y0)^2 - (EipJi)^2;
c2 = [z0+EEi y0];
r2 = EipJi;

% Find intersection of two circles
[z_Ji, y_Ji] = findCircIntersection(c1,r1,c2,r2);

% We can now solve for the joint angle for actuator Fi using atan2.
qi = atan2( -y_Ji, z_Ji-OFi ); 
Fi = Roty * [0, 0, OFi]';
Ji = Roty * [0, y_Ji, z_Ji]';
% Ei = Roty * [x0, y0, z0+EEi]';
Ei = Roty * [E_0(1), E_0(2), E_0(3)+EEi]';
end






