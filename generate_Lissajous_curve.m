% generate helix path
function [x, y, z] = generate_Lissajous_curve        
    t = linspace(1,8,1000);    
    kx = 3;
    ky = 2;
    a = 40;
    b = 40;
    x = a*cos(kx*t);
    z = linspace(50,50,1000);
    y = b*sin(ky*t)-360;
    
%     figure
%     view(2)
%     plot3(x,y,z,'--blue')
end