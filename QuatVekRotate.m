function [prot] = QuatVekRotate(q,p)

% [vrot] = QuatVekRotate(q,p)
% Rotation of a "point" around a quaternion q
%
% prot = q * p * q^*
%      q^* ... conjugat complex quaternion
%
% used functions: QuatMult
%                 QuatKonjugiert

dim = length(p);
if dim==3
    p = [0;p];
end


prot=QuatMult(q,QuatMult(p,QuatKonjugiert(q)));