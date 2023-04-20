function [xpred] = QuatPrediction(x,dt);

% [xpred] = QuatPrediction(x,dt);
% prediction of a quaternion based on the solution of the differential equation
%
% x = [q0; q1; q2; q3; w1; w2; w3]'
%
% q(k+1|k) = ( cos(|w|*dt/2)*I + 2/|w|*sin(|w|*dt/2) * OmegaM) * q(k)
% q(k)     actual quaternion
% q(k+1|k) quaternion at k = k+1
% OmegaM   skew symmetric matrix of w
%
% w(k+1|k) = w(k)
% if dt ~ 0: xpred = x


q  = x(1:4);
q0 = x(1);
q1 = x(2);
q2 = x(3);
q3 = x(4);
w1 = x(5);
w2 = x(6);
w3 = x(7);
AbsW = sqrt(w1^2+w2^2+w3^2);

if (AbsW > 1e-10)

    OmMat = zeros(4,4);
    OmMat(1,1) =   0; OmMat(1,2) = -w1; OmMat(1,3) = -w2; OmMat(1,4) = -w3;
    OmMat(2,1) =  w1; OmMat(2,2) =   0; OmMat(2,3) = -w3; OmMat(2,4) =  w2;
    OmMat(3,1) =  w2; OmMat(3,2) =  w3; OmMat(3,3) =   0; OmMat(3,4) = -w1;
    OmMat(4,1) =  w3; OmMat(4,2) = -w2; OmMat(4,3) =  w1; OmMat(4,4) =   0;
    
    OmMat = OmMat/2;
    
    cw = cos(AbsW*dt/2);
    sw = sin(AbsW*dt/2);
    qpred = ( cw*eye(4,4) + 2/AbsW*sw*OmMat) * q;
    
    xpred = [qpred ; w1 ; w2 ; w3 ];
else
    xpred = x;
end
