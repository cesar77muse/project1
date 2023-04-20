function [hk,Hk]=GenerateOutputPartialDeriv(x)

g=[0;0;-9.81];

p=x(1:3,1);
a=x(7:9,1);
b=x(17:19,1);
q=x(10:13,1);
w=x(14:16,1);

anew=a+b+g;

q0=q(1);
q1=q(2);
q2=q(3);
q3=q(4);

C=[q0^2+q1^2-q2^2-q3^2  2*(q1*q2+q0*q3)     2*(q1*q3-q0*q2);
   2*(q1*q2-q0*q3)      q0^2+q2^2-q1^2-q3^2 2*(q2*q3+q0*q1);
   2*(q1*q3+q0*q2)      2*(q2*q3-q0*q1)     q0^2+q3^2-q1^2-q2^2];


Q0=[q0 q3 -q2;-q3 q0 q1;q2 -q1 q0];
Q1=[q1 q2 q3;q2 -q1 q0;q3 -q0 -q1];
Q2=[-q2 q1 -q0;q1 q2 q3;q0 q3 -q2];
Q3=[-q3 q0 q1;-q0 -q3 q2;q1 q2 q3];
  
Haa=C;
Hab=C;
Haq=[2*Q0*anew 2*Q1*anew 2*Q2*anew 2*Q3*anew];
Hwq=[2*Q0*w 2*Q1*w 2*Q2*w 2*Q3*w];
Hww=C;

hk=[C*anew;C*w;p;q];
Hk=[zeros(3)    zeros(3)    Haa         Haq         zeros(3)    Hab;
    zeros(3)    zeros(3)    zeros(3)    Hwq         Hww         zeros(3);
    eye(3)      zeros(3)    zeros(3)    zeros(3,4)  zeros(3)    zeros(3);
    zeros(4,3)  zeros(4,3)  zeros(4,3)  eye(4)      zeros(4,3)  zeros(4,3)];