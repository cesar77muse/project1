function [q] = QuatExpFunct(r)

% [q] = QuatExpFunct(r)
% Exponential Mapping of a quaternion size(r)=3x1
% q = [cos(|r|) ; sin(|r|)*r

AbsR = sqrt(sum(r.^2));
q = zeros(4,1);

if (AbsR) > 1e-10
    NormR = r/AbsR; 

    q(1) = cos(AbsR);
    q(2:4) = sin(AbsR)*NormR;
else
    q(1) = 1;
    q(2:4) = 0;
end

