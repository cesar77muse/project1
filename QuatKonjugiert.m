function [qres] = QuatKonjugiert(q)

% [qres] = QuatKonjugiert(q)
% conjugate complex of quaternion
% q^* = [q0 ; -q]'

qres = QuatAdd(q(1),-q(2:4));