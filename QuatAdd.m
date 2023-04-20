function [qres] = QuatAdd(q0,q)

% [qres] = QuatAdd(q0,q)
% Combines tge real and the imaginary part of a quaternion
% qres = [q0 ; q ]'

qres = zeros(4,1);

qres(1) = q0;
qres(2:4) = q;