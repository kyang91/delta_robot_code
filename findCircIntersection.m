%% Computes the coordinate of the more negative y of two circle intersections

function [x_sol,y_sol] = findCircIntersection(c1, r1, c2, r2)

syms x y; % Points we are trying to solve for

% Extract coordinates
x1 = c1(1); x2 = c2(1);
y1 = c1(2); y2 = c2(2);

% Calculate the intersection
sol = solve([r1==sqrt((x-x1)^2+(y-y1)^2), ...
             r2==sqrt((x-x2)^2+(y-y2)^2)]);

% Selected the more negative y value (Otherwords the smallest y value pair)
if double(sol.x(1)) > double(sol.x(2)) %z value check
    x_sol = double(sol.x(1));
    y_sol = double(sol.y(1));
else
    x_sol = double(sol.x(2));
    y_sol = double(sol.y(2));
end

end