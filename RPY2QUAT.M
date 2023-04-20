function [q]=rpy2quat(roll,pitch,yaw)

roll=pi*roll/180;
pitch=pi*pitch/180;
yaw=pi*yaw/180;

Rx=[1 0 0;0 cos(roll) -sin(roll);0 sin(roll) cos(roll)];
Ry=[cos(pitch) 0 sin(pitch);0 1 0;-sin(pitch) 0 cos(pitch)];
Rz=[cos(yaw) -sin(yaw) 0;sin(yaw) cos(yaw) 0;0 0 1];

R=Rz*Ry*Rx;


if (roll==0)&&(pitch==0)&&(yaw==0)
    q0=1;
    q1=0;
    q2=0;
    q3=0;
elseif (roll>0)
  if roll<=pi
     q0=sqrt(trace(R)+1)/2;
  else
     q0=-sqrt(trace(R)+1)/2;
  end
else
  if roll>=-pi
     q0=sqrt(trace(R)+1)/2;
  else
     q0=-sqrt(trace(R)+1)/2;
  end
end


      q1=-(R(3,2)-R(2,3))/4/q0;
      q2=(R(1,3)-R(3,1))/4/q0;
      q3=(R(2,1)-R(1,2))/4/q0;

q=[q0;q1;q2;q3];

q=q/norm(q);