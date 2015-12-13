% generate spiral path
function [xr, yr, zr] = generate_spiral
    scale = 5; % scaling factor for spiral
    t = linspace(0,5*pi,1000);
    x = scale*t.*cos(t);
    y = 2*t-330; % offset start to be y=-380
    z = scale*t.*sin(t);
%     figure
%     view(2)
%     plot3(x,y,z,'--blue')
    
    xr = fliplr(x);
    yr = y;
    zr = fliplr(z);
    
%     figure
%     view(2)
%     plot3(xr,yr,zr,'--blue')
    
%     xt = [x, xr];
%     yt = [y, yr];
%     zt = [z, zr];
%     
%     figure
%     view(2)
%     plot3(xt,yt,zt,'--blue')
end