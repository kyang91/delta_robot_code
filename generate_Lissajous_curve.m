% generate helix path
function [x, y, z] = generate_Lissajous_curve        
    t = linspace(1,8,500);    
    kx = 3;
    ky = 2;
    a = 40;
    b = 40;
    x = a*cos(kx*t);
    z = linspace(50,50,500);
    y = b*sin(ky*t)-340;
    
    figure
    view(2)
    plot3(x,y,z,'--blue')
end