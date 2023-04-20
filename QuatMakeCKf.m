function [C] = QuatMakeCKf(x)

% [C] = QuatMakeCKf(x)
% Generation of output matrix for an estimated prediction


C = zeros(4,8);

q0 = x(1);
q1 = x(2);
q2 = x(3);
q3 = x(4);

dq0 = x(5);
dq1 = x(6);
dq2 = x(7);
dq3 = x(8);

C(1,1) = dq0;
C(1,2) = dq1;
C(1,3) = dq2;
C(1,4) = dq3;
C(1,5) =  q0;
C(1,6) = -q1;
C(1,7) = -q2;
C(1,8) = -q3;

C(2,1) = dq1;
C(2,2) =-dq0;
C(2,3) = dq3;
C(2,4) =-dq2;
C(2,5) =  q1;
C(2,6) =  q0;
C(2,7) = -q3;
C(2,8) =  q2;

C(3,1) = dq2;
C(3,2) =-dq3;
C(3,3) =-dq0;
C(3,4) = dq1;
C(3,5) =  q2;
C(3,6) =  q3;
C(3,7) =  q0;
C(3,8) = -q1;

C(4,1) = dq3;
C(4,2) = dq2;
C(4,3) =-dq1;
C(4,4) =-dq0;
C(4,5) =  q3;
C(4,6) = -q2;
C(4,7) =  q1;
C(4,8) =  q0;

C = 2*C;