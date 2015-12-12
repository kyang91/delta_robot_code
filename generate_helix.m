% generate helix path
function [x, y, z] = generate_helix    
    r = 50; % radius of helix
    t = linspace(1,2*pi,100);
    x = r*cos(6*t);
    y = t-380;
    z = r*sin(6*t);
    figure(1)
    view(2)
    plot3(x,y,z,'--blue')
end