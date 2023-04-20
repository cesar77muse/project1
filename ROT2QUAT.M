function [q]=rot2quat(R)

q0=sqrt(trace(R)+1)/2;

if (q0==0)
        
else
    q1=(R(3,2)-R(2,3))/4/q0;

    q2=(R(1,3)-R(3,1))/4/q0;

    q3=(R(2,1)-R(1,2))/4/q0;
end

q=[q0;q1;q2;q3];

q=q/norm(q);