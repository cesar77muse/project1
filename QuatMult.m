function [qres] = QuatMult(a,b)

% [qres] = QuatMult(a,b)
% multiplication of two quaternions a,b
%
% used functions: QuatAdd
%                 SkewMatrix


a0 = a(1);
b0 = b(1);

a1 = a(2:4);
b1 = b(2:4);

q1 = a0*b0;
q2 = a0*b1;
q3 = b0*a1;
q4 = cross(a1,b1);
q5 = -a1'*b1;

qres = QuatAdd(q1+q5,q2+q3+q4);
E = eye(3,3);
q = [a0*b0 - a1'*b1
     a1*b0 + (a0*E+SkewMatrix(a1))*b1];
 
q = [a0*b0 - b1'*a1
     a0*b1 + (b0*E-SkewMatrix(b1))*a1];