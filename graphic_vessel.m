function [vessel_x, vessel_y] = graphic_vessel(x_c,y_c, angle, scale)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
angle = -angle;
rotArray =[cos(angle), -sin(angle); sin(angle), cos(angle)];

vessel=(scale*[0.5 -1;
            0.5 1;
            0   1.4
            -0.5 1;
            -0.5 -1;
            0.5 -1]')';

dpos = zeros(size(vessel));
dpos(:,1) = x_c;
dpos(:,2) = y_c;
vessel = (rotArray*vessel')';
vessel = vessel+dpos;

vessel_x = vessel(:,1);
vessel_y = vessel(:,2);
end

