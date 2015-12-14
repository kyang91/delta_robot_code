% generate slinky path
function [x, y, z] = generate_hypotrochoid
    t = linspace(0,6.5,1000);
    R = 50;
    r = 3;
    d = 30;
    offset = -50;
    
    x = (R-r)*cos(t) + d*cos((R-r)*t/r)+offset;
    z = (R-r)*sin(t) - d*sin((R-r)*t/r)+offset;
    y = linspace(-370,-370,1000);
    
    
%     figure
%     view(2)
%     plot3(x,y,z,'blue')
end