% generate helix path
function [x, y, z] = generate_helix    
    yscale = 8; % stretch helix out in y direction
    r = 50; % radius of helix
    t = linspace(1,2*pi,1000);
    x = r*cos(6*t);
    y = yscale*t-380;
    z = r*sin(6*t);
%     figure
%     view(2)
%     plot3(x,y,z,'--blue')
end