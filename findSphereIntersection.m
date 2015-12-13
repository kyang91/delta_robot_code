%% Computes the coordinate of the more negative of three sphere intersection

function [x_sol, y_sol, z_sol] = findSphereIntersection(c1, r1, c2, r2, c3, r3)
% Extract coordinates
x1 = c1(1); x2 = c2(1); x3 = c3(1);
y1 = c1(2); y2 = c2(2); y3 = c3(2);
z1 = c1(3); z2 = c2(3); z3 = c3(3);

% Method 1
% syms x y z; % Points we are trying to solve for
% 
% % Calculate the intersection
% sol = solve([r1==sqrt((x-x1)^2 + (y-y1)^2 + (z-z1)^2), ... 
%              r2==sqrt((x-x2)^2 + (y-y2)^2 + (z-z2)^2), ...
%              r3==sqrt((x-x3)^2 + (y-y3)^2 + (z-z3)^2)]);
%  
% % Selected the more negative y value (Otherwords the smallest y value pair)
% if double(sol.y(1)) < double(sol.y(2))
%     x_sol = double(sol.x(1));
%     y_sol = double(sol.y(1));
%     z_sol = double(sol.z(1));
% else
%     x_sol = double(sol.x(2));
%     y_sol = double(sol.y(2));
%     z_sol = double(sol.z(2));
% end

%% Method 2
xc = [x1;x2;x3];
yc = [y1;y2;y3];
zc = [z1;z2;z3];
r = [r1; r2; r3];
[Y1 Y2 errflag] = trilateration(xc, yc, zc, r);

% Selected the more negative y value (Otherwords the smallest y value pair)
if double(Y1(2)) < double(Y2(2))
    x_sol = double(Y1(1));
    y_sol = double(Y1(2));
    z_sol = double(Y1(3));
else
    x_sol = double(Y2(1));
    y_sol = double(Y2(2));
    z_sol = double(Y2(3));
end


end