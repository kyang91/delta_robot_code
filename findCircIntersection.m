%% Computes the coordinate of the more negative y of two circle intersections
% NOTE: Two methods in below code method 1 and 2. If method 2 doesn't work
% use the slower method 1. Comment and uncomment appropriate section
function [x_sol,y_sol] = findCircIntersection(c1, r1, c2, r2)

    syms x y; % Points we are trying to solve for

    % Extract coordinates
    x1 = c1(1); x2 = c2(1);
    y1 = c1(2); y2 = c2(2);

    %% ---- Calculate the intersection using method #1. Not optimized ----
    % sol = solve([r1==sqrt((x-x1)^2+(y-y1)^2), ...
    %              r2==sqrt((x-x2)^2+(y-y2)^2)]);

    % % Selected the more negative y value (Otherwords the smallest y value pair)
    % if double(sol.x(1)) > double(sol.x(2)) %z value check
    %     x_sol = double(sol.x(1));
    %     y_sol = double(sol.y(1));
    % else
    %     x_sol = double(sol.x(2));
    %     y_sol = double(sol.y(2));
    % end

    %% ---- Calculate the intersection using method #2. Optimized function. ----
    [circx, circy] = circcirc(x1,y1,r1,x2,y2,r2);
    if double(circx(1)) > double(circx(2)) %z value check
        x_sol = double(circx(1));
        y_sol = double(circy(1));
    else
        x_sol = double(circx(2));
        y_sol = double(circy(2));
    end

end