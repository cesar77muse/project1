function [S] = SkewMatrix(a)

% [S] = SkewMatrix(a)
% genertion of a skew-symmetric matrix out of a

S = zeros(3,3);

S(1,2) = -a(3);
S(2,1) =  a(3);
S(1,3) =  a(2);
S(3,1) = -a(2);
S(2,3) = -a(1);
S(3,2) =  a(1);
