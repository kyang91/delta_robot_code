% Delta Robot - Simulation
clc;clear;close all;

%% Robot Parameters of the Delta Robot
f = 109.5; % equilateral triangle side of fixed platform
e = 33; % equilateral triangle side of moving platform
rf = 100; % Upper arm length in mm
re = 300; % Forearm length in mm
alpha = [0 120*pi/180 -120*pi/180]; % Angle exploiting symmetry when computing IK. -120deg/+120deg

%% Inverse Kinematics - returns joint angles given end-effector position
E = [-50; -300; 0];% User desired pose of TCP (Tool Center Point)
[q1, F1, J1, E1] = IK(E, alpha(1), f, e, rf, re);
[q2, F2, J2, E2] = IK(E, alpha(2), f, e, rf, re);
[q3, F3, J3, E3] = IK(E, alpha(3), f, e, rf, re);

%% Forward Kinematics - returns end-effector position given joint angles



%% Plot Delta robot
figure(1)
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
plot3(J1(1), J1(2), J1(3), '*b')
plot3(J2(1), J2(2), J2(3), '*b')
plot3(J3(1), J3(2), J3(3), '*b')
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

%%
figure(2)
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
plot3(J1(1), J1(2), J1(3), '*b')
plot3(J2(1), J2(2), J2(3), '*b')
plot3(J3(1), J3(2), J3(3), '*b')
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
