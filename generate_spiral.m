% generate spiral path
function [x, y, z] = generate_spiral
    scale = 5; % scaling factor for spiral
    t = linspace(0,5*pi,100);
    x = scale*t.*cos(t);
    y = scale*t-380; % offset start to be y=-380
    z = scale*t.*sin(t);
    figure(2)
    view(2)
    plot3(x,y,z,'--blue')
end