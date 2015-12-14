%% Plot robot workspace
function plotWorkspace(alpha, f, e, rf, re)
    E = [0; -300; 0]; % User desired pose of TCP (Tool Center Point)

    % Solve IK
    [q1, F1, J1, E1] = IK(E, alpha(1), f, e, rf, re);
    [q2, F2, J2, E2] = IK(E, alpha(2), f, e, rf, re);
    [q3, F3, J3, E3] = IK(E, alpha(3), f, e, rf, re);

    rad_m = e/2 * tan(30*pi/180); % radius from joint E to Ei
    rad_f = f/2 * tan(30*pi/180); % radius from joint O to Fi
    theta = 60*pi/180;
    angle = linspace(0, 2*pi, 360);
    
    u = [                 0             rad_f 0;
          -rad_f*sin(theta) -rad_f*cos(theta) 0;
           rad_f*sin(theta) -rad_f*cos(theta) 0];

    s = [                 0             rad_m 0;
          -rad_m*sin(theta) -rad_m*cos(theta) 0;
           rad_m*sin(theta) -rad_m*cos(theta) 0];

    new_centerpts = u-s;

    % center pt of joint Ji
    x1=J1(1); y1=J1(2); z1=J1(3);
    x2=J2(1); y2=J2(2); z2=J2(3);
    x3=J3(1); y3=J3(2); z3=J3(3);
    centerptsJi(1,:) = [x1 z1];
    centerptsJi(2,:) = [x2 z2];
    centerptsJi(3,:) = [x3 z3];

    % % center pt of joint Fi
    % x1=F1(1); y1=F1(2); z1=F1(3);
    % x2=F2(1); y2=F2(2); z2=F2(3);
    % x3=F3(1); y3=F3(2); z3=F3(3);
    % centerptsFi(1,:) = [x1 z1];
    % centerptsFi(2,:) = [x2 z2];
    % centerptsFi(3,:) = [x3 z3];
    % 
    % % center pt of joint Fi
    % x1=E1(1); y1=E1(2); z1=E1(3);
    % x2=E2(1); y2=E2(2); z2=E2(3);
    % x3=E3(1); y3=E3(2); z3=E3(3);
    % centerptsEi(1,:) = [x1 z1];
    % centerptsEi(2,:) = [x2 z2];
    % centerptsEi(3,:) = [x3 z3];

    figure    
    hold on;
    title('Robot workspace circles and shifted center points as red *')
    xlabel('X (mm)');
    ylabel('Z (mm)');
    % unshifted circle centers
    % plot(centerptsFi(1,1), centerptsFi(1,2), '*')
    % plot(centerptsFi(2,1), centerptsFi(2,2), '*')
    % plot(centerptsFi(3,1), centerptsFi(3,2), '*')
    for k = 1:3
        y = 0; % max is the radius of the circle. this is the height level of the sphere
        % Joint Fi
        xcenter = new_centerpts(k,1);
        zcenter = new_centerpts(k,2);
        radius_re = sqrt(re^2 - y^2);
        radius_rf = sqrt(rf^2 - y^2);

        x_re = radius_re*cos(angle) + xcenter;
        z_re = radius_re*sin(angle) + zcenter;

        x_rf = radius_rf*cos(angle) + xcenter;
        z_rf = radius_rf*sin(angle) + zcenter;

        % shifted circle centers
        plot(new_centerpts(1,1), new_centerpts(1,2), '*r')
        plot(new_centerpts(2,1), new_centerpts(2,2), '*r')
        plot(new_centerpts(3,1), new_centerpts(3,2), '*r')

        plot(x_re,z_re, 'b');
        plot(x_rf,z_rf, 'r');   
    end

    xlim = 300; % the min and max values for x-axis
    zlim = 300; % the min and max values for z-axis
    ylim = 200; % the min and max values for y-axis which is for height
    points = [];
    points_alt = [];

    for x = -xlim:1:xlim % check all x values at increments of 5mm
        for z = -zlim:1:zlim % then check all z values at increments of 5mm        
            radius_rf = sqrt(rf^2 - y^2);    
            if norm( [new_centerpts(1,1), new_centerpts(1,2)] - [x, z] ) <= radius_rf && ...               
                norm( [new_centerpts(2,1), new_centerpts(2,2)] - [x, z] ) <= radius_rf && ...
                norm( [new_centerpts(3,1), new_centerpts(3,2)] - [x, z] ) <= radius_rf            
                points_alt = [points_alt; [x,y-J1(3),z]];  % add points to total list

            end
        end    
    end

    figure;
    set(gcf,'units','normalized','outerposition',[0 0 1 1])
    sub2 = subplot(2,2,2);
    plot(points_alt(:,1), points_alt(:,3), '.', 'MarkerSize', 3) % plot the generated points
    grid(sub2, 'on');
    axis(sub2, [-300 300 -300 300 -600 0])
    axis(sub2, 'square')
    xlabel(sub2, 'X (mm)')
    zlabel(sub2, 'Y (mm)')
    ylabel(sub2, 'Z (mm)')
    title(sub2, 'Workspace plot filled in with generated points', 'FontSize', 18)
    view(0,90);

    for x = -xlim:10:xlim % check all x values at increments of 5mm
        for z = -zlim:10:zlim % then check all z values at increments of 5mm
            for y = -ylim:10:ylim % Then check for the range of heights            
                radius_rf = sqrt(rf^2 - y^2);    
                if norm( [new_centerpts(1,1), new_centerpts(1,2)] - [x, z] ) <= radius_rf && ...               
                    norm( [new_centerpts(2,1), new_centerpts(2,2)] - [x, z] ) <= radius_rf && ...
                    norm( [new_centerpts(3,1), new_centerpts(3,2)] - [x, z] ) <= radius_rf            
                    points = [points; [x,y-J1(3),z]];  % add points to total list
                end
            end
        end    
    end

    % figure;
    sub1 = subplot(2,2,1);
    scatter3(points(:,1), points(:,3),points(:,2),20,points(:,2),'filled');
    colormap(prism);
    grid(sub1, 'on');
    axis(sub1, [-300 300 -300 300 -600 0])
    axis(sub1, 'square')
    xlabel(sub1, 'X (mm)')
    zlabel(sub1, 'Y (mm)')
    ylabel(sub1, 'Z (mm)')
    title(sub1, 'Workspace plot filled in with generated points', 'FontSize', 18)
    view(-38,30);

    % sub2 = subplot(2,2,2)
    % scatter3(points(:,1), points(:,3),points(:,2),20,points(:,2),'filled');
    % colormap(prism);
    % grid(sub2, 'on');
    % axis(sub2, [-300 300 -300 300 -600 0])
    % axis(sub2, 'square')
    % xlabel(sub2, 'X (mm)')
    % zlabel(sub2, 'Y (mm)')
    % ylabel(sub2, 'Z (mm)')
    % title(sub2, 'Workspace plot filled in with generated points', 'FontSize', 18)
    % view(0,90);

    sub3 = subplot(2,2,3);
    scatter3(points(:,1), points(:,3),points(:,2),20,points(:,2),'filled');
    colormap(prism);
    grid(sub3, 'on');
    axis(sub3, [-300 300 -300 300 -600 0])
    axis(sub3, 'square')
    xlabel(sub3, 'X (mm)')
    zlabel(sub3, 'Y (mm)')
    ylabel(sub3, 'Z (mm)')
    title(sub3, 'Workspace plot filled in with generated points', 'FontSize', 18)
    view(-90,0);

    sub4 = subplot(2,2,4);
    scatter3(points(:,1), points(:,3),points(:,2),20,points(:,2),'filled');
    colormap(prism);
    grid(sub4, 'on');
    axis(sub4, [-300 300 -300 300 -600 0])
    axis(sub4, 'square')
    xlabel(sub4, 'X (mm)')
    zlabel(sub4, 'Y (mm)')
    ylabel(sub4, 'Z (mm)')
    title(sub4, 'Workspace plot filled in with generated points', 'FontSize', 18)
    view(-45,0);
end