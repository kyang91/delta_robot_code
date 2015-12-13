% generate slinky path
function [x, y, z] = generate_hypotrochoid
    t = linspace(0,20,2000);
    R = 50;
    r = 3;
    d = 30;
    offset = 100;
    
    x = (R-r)*cos(t) + d*cos((R-r)*t/r)+offset;
    y = (R-r)*sin(t) - d*sin((R-r)*t/r)+offset;
    z = linspace(-300,-300,2000);
    
    
    figure
    view(2)
    plot3(x,y,z,'blue')
end