function [Qtran, Qw1,Gq,Gw] = GeneratePartialDeriv(q, w, dt)

% [Qtran, Qw1] = GeneratePartialDeriv(q, w, dt)
% calculates the linearised transfer function for the quaternions and 
% the angular velocity based on the exponential mapping
%
% Input: q   actual quaternion
%        w   actual angular velocity
%        dt  time step
%
% Output: Qtran  linearised transfer function for q(k+1) = f(q(k),w(k))
%         Qw1    transfer function of w(k+1) = h(w(k))

q0 = q(1);
q1 = q(2);
q2 = q(3);
q3 = q(4);

w1 = w(1);
w2 = w(2);
w3 = w(3);

AbsW = sqrt(w1^2+w2^2+w3^2);

Qw1 = zeros(4,3);
Qtran = zeros(4,4);

cw = cos(AbsW*dt/2);
sw = sin(AbsW*dt/2);

if (AbsW > 1e-5)
    OmMat = zeros(4,4);
    OmMat(1,1) =   0; OmMat(1,2) =  -w1; OmMat(1,3) = -w2; OmMat(1,4) =  -w3;
    OmMat(2,1) = w1; OmMat(2,2) =   0; OmMat(2,3) =  -w3; OmMat(2,4) =  w2;
    OmMat(3,1) =  w2; OmMat(3,2) = w3; OmMat(3,3) =   0; OmMat(3,4) =  -w1;
    OmMat(4,1) = w3; OmMat(4,2) = -w2; OmMat(4,3) = w1; OmMat(4,4) =   0;
    
    Qtran = cos(AbsW*dt/2)*eye(4,4) + 1/AbsW*sin(AbsW*dt/2)*OmMat;
    %dOmMatdw1 = [0 0 0 1;0 0 1 0;0 -1 0 0;-1 0 0 0]


    
    %qw0 = q1*w3 - q2*w2 + q3*w1;
    qw0 = -q1*w1 - q2*w2 - q3*w3;
    Qw1(1,1) = 1/AbsW * (-q1-w1/AbsW^2*qw0-q0*w1*dt/2 ) * sw + qw0*w1*dt/(2*AbsW^2)*cw; %q3
    Qw1(1,2) = 1/AbsW * (-q2-w2/AbsW^2*qw0-q0*w2*dt/2 ) * sw + qw0*w2*dt/(2*AbsW^2)*cw; %-q2
    Qw1(1,3) = 1/AbsW * (-q3-w3/AbsW^2*qw0-q0*w3*dt/2 ) * sw + qw0*w3*dt/(2*AbsW^2)*cw; %q1
    
    %qw1 = -q0*w3 + q2*w1 + q3*w2;
    qw1 =  q0*w1 - q2*w3 + q3*w2;
    Qw1(2,1) = 1/AbsW * ( q0-w1/AbsW^2*qw1-q1*w1*dt/2 ) * sw + qw1*w1*dt/(2*AbsW^2)*cw;%q2
    Qw1(2,2) = 1/AbsW * ( q3-w2/AbsW^2*qw1-q1*w2*dt/2 ) * sw + qw1*w2*dt/(2*AbsW^2)*cw;%q3
    Qw1(2,3) = 1/AbsW * (-q2-w3/AbsW^2*qw1-q1*w3*dt/2 ) * sw + qw1*w3*dt/(2*AbsW^2)*cw;%-q0
    
    %qw2 = q0*w2 - q1*w1 + q3*w3;
    qw2 = q0*w2 + q1*w3 - q3*w1;
    Qw1(3,1) = 1/AbsW * (-q3-w1/AbsW^2*qw2-q2*w1*dt/2 ) * sw + qw2*w1*dt/(2*AbsW^2)*cw;%-q1
    Qw1(3,2) = 1/AbsW * ( q0-w2/AbsW^2*qw2-q2*w2*dt/2 ) * sw + qw2*w2*dt/(2*AbsW^2)*cw;%q0
    Qw1(3,3) = 1/AbsW * ( q1-w3/AbsW^2*qw2-q2*w3*dt/2 ) * sw + qw2*w3*dt/(2*AbsW^2)*cw;%q3
    
    %qw3 = -q0*w1 - q1*w2 - q2*w3;
    qw3 =  q0*w3 - q1*w2 + q2*w1;
    Qw1(4,1) = 1/AbsW * ( q2-w1/AbsW^2*qw3-q3*w1*dt/2 ) * sw + qw3*w1*dt/(2*AbsW^2)*cw;%-q0
    Qw1(4,2) = 1/AbsW * (-q1-w2/AbsW^2*qw3-q3*w2*dt/2 ) * sw + qw3*w2*dt/(2*AbsW^2)*cw;%-q1
    Qw1(4,3) = 1/AbsW * ( q0-w3/AbsW^2*qw3-q3*w3*dt/2 ) * sw + qw3*w3*dt/(2*AbsW^2)*cw;%-q2
else
    Qtran = eye(4,4);
    Qw1 = zeros(4,3);
end

Gq=[-q1 -q2 -q3;
    q0  q3  -q2;
    -q3 q0  q1;
    q2  -q1 q0]*dt*dt/4*sw;
    
Gw=dt*eye(3);