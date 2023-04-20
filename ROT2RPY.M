function [roll,pitch,yaw]=rot2rpy(R)

roll=atan2(R(3,2),R(3,3));
if roll<0
    roll=roll+2*pi;
end
pitch=atan2(-R(3,1),sqrt(R(1,1)^2+R(2,1)^2));
yaw=atan2(-R(2,1),R(1,1));

roll=roll*180/pi;
pitch=pitch*180/pi;
yaw=yaw*180/pi;