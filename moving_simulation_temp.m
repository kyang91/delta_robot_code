%% temperary file for testing purposes only.

clc; clear all; close all
%% Robot Parameters
disp('Simulation is now running...')
disp('Simulation will loop through six trajectories:')
f = 380; % equilateral triangle side of fixed platform
e = 116; % equilateral triangle side of moving platform
rf = 154; % Upper arm length in mm
re = 345; % Forearm length in mm
alpha = [0 -120*pi/180 120*pi/180]; % Angle exploiting symmetry when computing IK. -120deg/+120deg
E = [0;-300;0]; % Desired TCP

%% Generate Trajectory Points 
% Points will be generated to follow an arbitrarily chosen trajectory

for j=1:1 % Loop through 6 different trajectories    
    
    [x1,y1,z1] = generate_hypotrochoid_star;    
    [x2,y2,z2] = generate_hypotrochoid;
    [x3,y3,z3] = generate_helix;
    [x4,y4,z4] = generate_slinky;
    [x5,y5,z5] = generate_Lissajous_curve;
    [x6,y6,z6] = generate_spiral;
    
        
    
    trajectory = horzcat([x1; y1; z1], [x2; y2; z2], [x3; y3; z3],[x4; y4; z4],[x5; y5; z5],[x6; y6; z6]);    
    x = trajectory(1,1);
    y = trajectory(2,1);
    z = trajectory(3,1);

    %% Loop Through Points, Compute IK and FK and plot animation
    % Loop through the points in the trajectory calculate joint locaitons and
    % plot them.

    i=1;
    figure    
    set(gcf,'units','normalized','outerposition',[0 0 1 1])
    hold on;
    grid on;
    plot3(x,y,z,':blue','linewidth',1)
    axis([-300 300 -500 50 -300 300])
    axis 'square';
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    title_name2 = strcat('Animation of Delta Robot tracing desired path: ');
    title(title_name2, 'FontSize', 18)
    view(10,-46);
    line_width = 2; % Set width of the drawn lines of robot1

    % Solve IK
    [q1(1,i), F1, J1, E1] = IK(trajectory(:,i), alpha(1), f, e, rf, re);
    [q2(1,i), F2, J2, E2] = IK(trajectory(:,i), alpha(2), f, e, rf, re);
    [q3(1,i), F3, J3, E3] = IK(trajectory(:,i), alpha(3), f, e, rf, re);        
    
    % Solve FK
    E_0(i,:) = FK(q1(1,i), q3(1,i), q2(1,i), alpha, f, e, rf, re)';

    % Top fixed platform
    fixed1_handle = plot3(F1(1), F1(2), F1(3), 'Or');
    fixed2_handle = plot3(F2(1), F2(2), F2(3), 'Or');
    fixed3_handle = plot3(F3(1), F3(2), F3(3), 'Or');
    top = [F1'; F2'; F3'; F1'];
    fixed_lines_handle = plot3(top(:,1), top(:,2), top(:,3), 'r', 'LineWidth', line_width);

    % Three arms connection fixed and moving platforms
    joint1_handle = plot3(J1(1), J1(2), J1(3), 'Ob');
    joint2_handle = plot3(J2(1), J2(2), J2(3), 'Ob');
    joint3_handle = plot3(J3(1), J3(2), J3(3), 'Ob');
    arm1 = [F1'; J1'; E1'];
    joint1_line_handle = plot3(arm1(:,1), arm1(:,2), arm1(:,3), 'b', 'LineWidth', line_width);
    arm2 = [F2'; J2'; E2'];
    joint2_line_handle = plot3(arm2(:,1), arm2(:,2), arm2(:,3), 'b', 'LineWidth', line_width);
    arm3 = [F3'; J3'; E3'];
    joint3_line_handle = plot3(arm3(:,1), arm3(:,2), arm3(:,3), 'b', 'LineWidth', line_width);

    % Bottom moving platform
    moving1_handle = plot3(E1(1), E1(2), E1(3), 'Og');
    moving2_handle = plot3(E2(1), E2(2), E2(3), 'Og');
    moving3_handle = plot3(E3(1), E3(2), E3(3), 'Og');
    bot = [E1'; E2'; E3'; E1'];
    moving_line_handle = plot3(bot(:,1), bot(:,2), bot(:,3), 'g', 'LineWidth', line_width);

    % Desired TCP
    point_handle = plot3(trajectory(1,i), trajectory(2,i), trajectory(3,i), '.red','markersize', 3);

    for i=1:length(trajectory)
        % Solve IK
        [q1(1,i), F1, J1, E1] = IK(trajectory(:,i), alpha(1), f, e, rf, re);
        [q2(1,i), F2, J2, E2] = IK(trajectory(:,i), alpha(2), f, e, rf, re);
        [q3(1,i), F3, J3, E3] = IK(trajectory(:,i), alpha(3), f, e, rf, re);      

        % Check if trajectory pts are out of bounds for robot
        if (abs(q1(1,i))*180/pi) < 10
            disp('out of bound for our real robot')
        end
        if (abs(q2(1,i))*180/pi) < 10
            disp('out of bound for our real robot')
        end
        if (abs(q3(1,i))*180/pi) < 10
            disp('out of bound for our real robot')
        end
        if (abs(q1(1,i))*180/pi) > 80
            disp('out of bound for our real robot')
        end
        if (abs(q2(1,i))*180/pi) > 80
            disp('out of bound for our real robot')
        end
        if (abs(q3(1,i))*180/pi) > 80
            disp('out of bound for our real robot')
        end

        % Fixed platform
        set(fixed1_handle, 'XData', F1(1));
        set(fixed1_handle, 'YData', F1(2));
        set(fixed1_handle, 'ZData', F1(3));   

        set(fixed2_handle, 'XData', F2(1));
        set(fixed2_handle, 'YData', F2(2));
        set(fixed2_handle, 'ZData', F2(3));

        set(fixed3_handle, 'XData', F3(1));
        set(fixed3_handle, 'YData', F3(2));
        set(fixed3_handle, 'ZData', F3(3));

        set(moving1_handle, 'XData', E1(1));
        set(moving1_handle, 'YData', E1(2));
        set(moving1_handle, 'ZData', E1(3));   

        % Moving platform
        set(moving2_handle, 'XData', E2(1));
        set(moving2_handle, 'YData', E2(2));
        set(moving2_handle, 'ZData', E2(3));

        set(moving3_handle, 'XData', E3(1));
        set(moving3_handle, 'YData', E3(2));
        set(moving3_handle, 'ZData', E3(3));  

        % Show the points using in path
        point_handle = plot3(trajectory(1,i), trajectory(2,i), trajectory(3,i), '.red', 'markersize', 3);

        % Moving platform lines
        bot = [E1'; E2'; E3'; E1'];
        set(moving_line_handle, 'XData', bot(:,1));
        set(moving_line_handle, 'YData', bot(:,2));
        set(moving_line_handle, 'ZData', bot(:,3));

        % Elbow joints
        set(joint1_handle, 'XData', J1(1));
        set(joint1_handle, 'YData', J1(2));
        set(joint1_handle, 'ZData', J1(3));

        set(joint2_handle, 'XData', J2(1));
        set(joint2_handle, 'YData', J2(2));
        set(joint2_handle, 'ZData', J2(3));

        set(joint3_handle, 'XData', J3(1));
        set(joint3_handle, 'YData', J3(2));
        set(joint3_handle, 'ZData', J3(3));

        % arm lines
        arm1 = [F1'; J1'; E1'];
        arm2 = [F2'; J2'; E2'];
        arm3 = [F3'; J3'; E3'];

        set(joint1_line_handle, 'XData', arm1(:,1));
        set(joint1_line_handle, 'YData', arm1(:,2));
        set(joint1_line_handle, 'ZData', arm1(:,3));

        set(joint2_line_handle, 'XData', arm2(:,1));
        set(joint2_line_handle, 'YData', arm2(:,2));
        set(joint2_line_handle, 'ZData', arm2(:,3));

        set(joint3_line_handle, 'XData', arm3(:,1));
        set(joint3_line_handle, 'YData', arm3(:,2));
        set(joint3_line_handle, 'ZData', arm3(:,3));

        pause(0.01); 
    end 
end



