% generate slinky path
function [x, y, z] = generate_hypotrochoid_star
    t = linspace(0,40,1000);    
    scale = 10;
    R = scale*5;
    r = scale*3;
    d = scale*5;
    theta = 45*pi/180;
    
    Rx = [1 0 0;
          0 cos(theta) -sin(theta);
          0 sin(theta) cos(theta)];
    
    Ry = [cos(theta) 0 sin(theta);
          0 1 0;
          -sin(theta) 0 cos(theta)];
      
    Rot = Rx*Ry;
    
    for i=1:500       
        x1 = (R-r)*cos(t(i)) + d*cos((R-r)*t(i)/r);        
        z1 = (R-r)*sin(t(i)) - d*sin((R-r)*t(i)/r);
        y1 = -420;
    
        val(i,:) = Rot * [x1;y1;z1];        
    end
    
    x=val(:,1)';
    y=val(:,2)';
    z=val(:,3)';
    
%     figure
%     view(2)
%     plot3(x,y,z,'blue')
end