% Delta Robot - Simulation
clc;clear;close all;

%% Robot Parameters of the Delta Robot
f = 380; % equilateral triangle side of fixed platform
e = 116; % equilateral triangle side of moving platform
rf = 154; % Upper arm length in mm
re = 345; % Forearm length in mm
alpha = [0 -120*pi/180 120*pi/180]; % Angle exploiting symmetry when computing IK. -120deg/+120deg

%% Inverse Kinematics - returns joint angles given end-effector position
disp('Desired TCP position:')
E = [0; -300; 0] % User desired pose of TCP (Tool Center Point)

[q1, F1, J1, E1] = IK(E, alpha(1), f, e, rf, re);
[q2, F2, J2, E2] = IK(E, alpha(2), f, e, rf, re);
[q3, F3, J3, E3] = IK(E, alpha(3), f, e, rf, re);
disp('IK solution for angles (q1, q2, q3) in degrees:')
[q1*180/pi, q2*180/pi, q3*180/pi]'

%% Forward Kinematics - returns end-effector position given joint angles
disp('TCP solved using FK with joint angles solved from IK as input:')
% Using results from IK as input into FK as a answer check
E_0 = FK(q1, q3, q2, alpha, f, e, rf, re)'

%% Plot Workspace
plotWorkspace(alpha, f, e, rf, re)

%% Plot Delta robot
figure
hold on 
grid on
axis square
view(3)
xlabel('X')
ylabel('Y')
zlabel('Z')

% Top fixed platform
plot3(F1(1), F1(2), F1(3), '*r')
plot3(F2(1), F2(2), F2(3), '*r')
plot3(F3(1), F3(2), F3(3), '*r')
top = [F1'; F2'; F3'; F1'];
plot3(top(:,1), top(:,2), top(:,3), 'r')

% Three arms connection fixed and moving platforms
plot3(J1(1), J1(2), J1(3), 'ob')
plot3(J2(1), J2(2), J2(3), '*b')
plot3(J3(1), J3(2), J3(3), 'xb')
arm1 = [F1'; J1'; E1'];
plot3(arm1(:,1), arm1(:,2), arm1(:,3), 'b')
arm2 = [F2'; J2'; E2'];
plot3(arm2(:,1), arm2(:,2), arm2(:,3), 'b')
arm3 = [F3'; J3'; E3'];
plot3(arm3(:,1), arm3(:,2), arm3(:,3), 'b')

% Bottom moving platform
plot3(E1(1), E1(2), E1(3), '*g')
plot3(E2(1), E2(2), E2(3), '*g')
plot3(E3(1), E3(2), E3(3), '*g')
bot = [E1'; E2'; E3'; E1'];
plot3(bot(:,1), bot(:,2), bot(:,3), 'g')

% Desired TCP
plot3(E(1), E(2), E(3), 'Oblack')

% Calculated TCP from FK
plot3(E_0(1), E_0(2), E_0(3), '+black')

%% Plot robot different view
figure
hold on 
grid on
axis square
view(2)
xlabel('X')
ylabel('Y')
zlabel('Z')

% Top fixed platform
plot3(F1(1), F1(2), F1(3), '*r')
plot3(F2(1), F2(2), F2(3), '*r')
plot3(F3(1), F3(2), F3(3), '*r')
top = [F1'; F2'; F3'; F1'];
plot3(top(:,1), top(:,2), top(:,3), 'r')

% Three arms connection fixed and moving platforms
plot3(J1(1), J1(2), J1(3), 'ob')
plot3(J2(1), J2(2), J2(3), '*b')
plot3(J3(1), J3(2), J3(3), 'xb')
arm1 = [F1'; J1'; E1'];
plot3(arm1(:,1), arm1(:,2), arm1(:,3), 'b')
arm2 = [F2'; J2'; E2'];
plot3(arm2(:,1), arm2(:,2), arm2(:,3), 'b')
arm3 = [F3'; J3'; E3'];
plot3(arm3(:,1), arm3(:,2), arm3(:,3), 'b')

% Bottom moving platform
plot3(E1(1), E1(2), E1(3), '*g')
plot3(E2(1), E2(2), E2(3), '*g')
plot3(E3(1), E3(2), E3(3), '*g')
bot = [E1'; E2'; E3'; E1'];
plot3(bot(:,1), bot(:,2), bot(:,3), 'g')

% Desired TCP
plot3(E(1), E(2), E(3), 'Oblack')

% Calculated TCP from FK
plot3(E_0(1), E_0(2), E_0(3), '+black')
