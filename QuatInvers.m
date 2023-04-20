function [qres] = QuatInvers(q)

% [qres] = QuatInvers(q)
% Inverse of a Quaternion
% q^(-1) = q^* / |q|   
%    q^* .. conjugate complex quaternion
%
% Used function: QuatKonjugiert

qres = QuatKonjugiert(q)/QuatNorm(q);