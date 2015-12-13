% generate slinky path
function [x, y, z] = generate_slinky
    t = linspace(0,2*pi,1000);    
    R = 80; % widest loop of the helix 
    a = 20; % amplitude. the small radius of the coil. slinky radius.
    w = 50; % oscillation. loops 
    h = 8; % pitch of the larger helix
    offset = 350; % height offset in negative direction
    x = [R+a*cos(w*t)].*cos(t);
    z = [R+a*cos(w*t)].*sin(t);
    y = h*t+a*sin(w*t)-offset;

    figure
    view(2)
    plot3(x,y,z,'blue')
end