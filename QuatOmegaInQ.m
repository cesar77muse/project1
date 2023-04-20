function [qdot] = QuatOmegaInQ(q,omega)

% [qdot] = QuatOmegaInQ(q,omega)
% Calculation of a "quaternion velocity" dot(q)
% qdot = 0.5 * q*Qomega
%    Qomega = [0;omega]'
%
% used functions: QuatMult

qomega = [0,omega];
qdot = 0.5 * QuatMult(q,qomega);

