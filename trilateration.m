function [Y1 Y2 errflag] = trilateration(xc, yc, zc, r)
%
% [Y1 Y2 errflag] = trilateration(xc, yc, zc, r)
%
% Purpose: find the two intersections of *three* spheres centered at
% (XC,YC,ZC) with radii R.

% INPUTS:
% XC, YC, ZC: coordinates of the centers, they are (3 x 1) 3D vectors
% RADII: coordinates of the centers, they are (3 x 1) array
% 
% OUTPUTS
% Y1, Y2: coordinates respectively of the first and second solution
% they are two (3 x 1) 3D vectors
% The z coordinates is sorted, i.e., Y1(3)< Y2(3).
% errflag: integer 0 -> OK
% -1 -> r1+r2 < distance center#1 to center#2 
% -2 -> r1+r2 ~= distance center#1 to center#2 
% -3 -> Centers are colinear
% -4 -> Inconsistent radii (radii(3) is suspected)
%
% Note: The results might still be useful for errflag < 0
%
% Author: Bruno Luong
% History: 22-Sep-2010 (original)

warningflag = 'off'; % off

warning(warningflag, 'trilateration:h2neg');
warning(warningflag, 'trilateration:cneg');
warning(warningflag, 'trilateration:cpos');

errflag = 0;

% x y z in three rows
C = [xc(:) yc(:) zc(:)].'; % 3 x 3

%% Sort the distances from smallest to largest, for better stability
[r is] = sort(r,'ascend');
C = C(:,is);

a1 = 0;
% vector from center#1 to center#2
v12 = C(:,2)-C(:,1); % 3 x 1
a2 = norm(v12);
da = a2-a1;
sqrradii = r(:).^2; % 3 x 1
if (da == 0)
    error('Input points must be distincts');
end
% unit vector, pointing from 1 to 2
u12 = v12/a2;
a = 0.5*((a1+a2) + (sqrradii(1)-sqrradii(2))/da);
h2 = sqrradii(1) - (a-a1)^2; % == sqrradii(2) - (a-a2)^2

% The r1+r2 < distance center#1 to center#2 
if h2 < 0
    warning('trilateration:h2neg', 'trilateration: h2 < 0')
    errflag = -1;
    h2 = 0;
end

% Radius of the circle { X : |X-C1|=r1 and |X-C2|=r2 } 
h = sqrt(h2);
% Center of the circle { X : |X-C1|=r1 and |X-C2|=r2 } = { C12 + h*v: v in B} 
C12 = C(:,1) + a*u12;

% two vectors perpendicular to u, i.e., u12'*B == 0
B = orthvec(u12(:)); % 3 x 2
% 3 x 3
Q=[B u12];

% Coordinates of the thirdpoint, translated to the center C12 and in the
% new Cartesian's local axis Q
xyz3 = C(:,3)-C12;
xyz3 = Q.'*xyz3;
%
x3 = xyz3(1);
y3 = xyz3(2);
%
b = sqrt(x3^2+y3^2);

if h < eps(a2)
    Y1 = C12;
    Y2 = C12;
    errflag = -2;
elseif b < eps(a2) % centers are colinear
    mindisp = Inf;
    a3 = dot(C(:,3)-C(:,1),u12);
    % Allocate memory
    Y = zeros(3,1);
    % Find the best combination of sign to minimize the dispersion 
    for i=[-1 1]
        Y(1) = a1 + i*r(1);
        for j=[-1 1]
            Y(2) = a2 + j*r(2);
            for k=[-1 1]
                Y(3) = a3 + k*r(3);
                dispersion = max(Y)-min(Y);
                if dispersion < mindisp
                    Ybest = Y;
                    mindisp = dispersion; 
                end
            end
        end
    end
    Y1 = C(:,1) + mean(Ybest)*u12;
    Y2 = Y1;
    errflag = -3;
else % regular trilateration
    numerator = (-sqrradii(3) + sum(xyz3.^2) + h2);
    denominator = 2*b*h;
    c = numerator / denominator;
    if abs(c) > 1
        % Increase h to compensate overflowed
        warning('trilateration:cpos', 'trilateration: abs(c) > +1')
        hscale = sqrt(abs(c));
        h = h*hscale;
        c = sign(c); % +/-1
        errflag = -4;
    end
    theta1 = atan2(y3,x3);
    theta2 = acos(c);
    
    theta = theta1 + theta2;
    Y1 = h*[cos(theta); sin(theta)]; % 2 x 1
    Y1 = C12 + B*Y1; % 3 x 1
    
    theta = theta1 - theta2;
    Y2 = h*[cos(theta); sin(theta)];
    Y2 = C12 + B*Y2;
    
    % Swap Y1,Y2 so that z coordinates increasing
    if Y1(3) > Y2(3)
        [Y1 Y2] = deal(Y2, Y1);
    end
end % cascade if

end % trilateration

%% scalar product
function res = dot(a,b)
res = a(1)*b(1) + a(2)*b(2) + a(3)*b(3);
end

%% vector norm
function res = norm(v)
res = sqrt(dot(v,v));
end

%% cross product
function c = cross(a, b)
c = [a(2)*b(3)-a(3)*b(2);
     a(3)*b(1)-a(1)*b(3);
     a(1)*b(2)-a(2)*b(1)];
end

%% find the orthogonal basis of the orthogonal space to span of the
% input vector
function B = orthvec(v)
% B = null(reshape(v,1,3));
[~, i] = min(abs(v)); % BUG corrected, was v
ei = zeros(3,1);
ei(i) = 1;
b1 = cross(v,ei);
b1 = b1/norm(b1);
b2 = cross(v,b1);
b2 = b2/norm(b2);
B = [b1 b2];
end