function [NormQ] = QuatNorm(q)

% [NormQ] = QuatNorm(q)
% calcualtion of the norm of a quaternion
% |q| = sqrt ( q0^2+q1^2+q2^2+q3^2 )

NormQ = sqrt(sum(q.^2));