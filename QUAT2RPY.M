function [roll,pitch,yaw]=quat2rpy(q)

q0=q(1);
q1=q(2);
q2=q(3);
q3=q(4);


R=[1-2*q2^2-2*q3^2 2*q1*q2+2*q0*q3 2*q1*q2-2*q0*q2;
   2*q1*q2-2*q0*q3 1-2*q1^2-2*q3^2 2*q2*q3+2*q0*q1;
	2*q1*q3+2*q0*q2 2*q2*q3-2*q0*q1 1-2*q1^2-2*q2^2];

roll=atan2(R(3,2),R(3,3));
if roll<0
    roll=roll+2*pi;
end
pitch=atan2(-R(3,1),sqrt(R(1,1)^2+R(2,1)^2));
yaw=atan2(-R(2,1),R(1,1));

roll=roll*180/pi;
pitch=pitch*180/pi;
yaw=yaw*180/pi;

