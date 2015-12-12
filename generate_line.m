% Generate line path
function [x, y, z] = generate_line
    x = linspace(-100,100,10);
    y = linspace(-300,-300,10);
    z = linspace(-50,100,10);
    plot3(x,y,z,'--blue')
end