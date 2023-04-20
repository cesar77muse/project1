function [xhat, xpred, Phat, ypred, dqTest, K ,Flag] = InertialEKFExt(xhat, Phat, y, T)

% [xhat, xpred, Phat, ypred, dqTest, K ,Flag] = CompleteEKFExt(xhat, Phat, y, T)
% Extended Kalman Filter to calculate the "state" out of cartesian accelerations
% and angular velocities. The states includes an offset of the accelerometers
%
% state x = [px py pz vx vy vz ax ay az q0 q1 q2 q3 w1 w2 w3 bax bay baz]
%       pi    Position
%       vi    Velocity
%       ai    Acceleration
%       qi    Quaternions
%       wi    Angular velocities
%       bai   Offset of the acclerometers
%
% Input: xhat   last state estimation xhat == x(k-1|k-1); size: 19x1
%        Phat   covariance matrix of xhat; size: 19x20
%        y      meausrements y = [ax; ay; az; w1; w2; w3]; size: 6x1
%        T     Time difference between the last estimation (xhat) and the actual time
%
% Output: xhat  new estimation
%         xpred one step prediction from (k-1) to (k) only for tests
%         Phat  new covariance of xhat
%         upred prediction of the measurements (for tests)
%         dqTest for test (sum of the Quaternions)
%         K     actual gain of the Kalman Filter
%         Flag  if S = (C*Pbar*C'+R)^(-1) is bad condition Flag == 1 
%
% Used functions: GeneratePartialDeriv
%                 QuatPrediction
%                 PositionPrediction

q0 = xhat(10);
q1 = xhat(11);
q2 = xhat(12);
q3 = xhat(13);
w1 = xhat(14);
w2 = xhat(15);
w3 = xhat(16);
q = [q0;q1;q2;q3];
w = [w1; w2; w3];

% Linearised Transfer Function for the orientation
[Qtran, Qw1,Gqkk1,Gwkk1] = GeneratePartialDeriv(q, w, T);

%Dynamic linearised model
if (T ~= 0)
    Atilde = [  eye(3,3) T*eye(3,3) 0.5*T^2*eye(3,3) zeros(3,4) zeros(3,3) zeros(3,3)
            zeros(3,3)    eye(3,3)    T*eye(3,3)    zeros(3,4) zeros(3,3) zeros(3,3)
            zeros(3,3)  zeros(3,3)     eye(3,3)      zeros(3,4) zeros(3,3) zeros(3,3)
            zeros(4,3)  zeros(4,3)    zeros(4,3)       Qtran      Qw1       zeros(4,3)
            zeros(3,3)  zeros(3,3)    zeros(3,3)     zeros(3,4)  eye(3,3) zeros(3,3)
            zeros(3,3)  zeros(3,3)    zeros(3,3)     zeros(3,4) zeros(3,3)  eye(3)]; 
else
    Atilde = eye(19,19);
end

Aakk1=[0 -xhat(16) xhat(15);
               xhat(16) 0 -xhat(14);
               -xhat(15) xhat(14) 0]*T;
        Aawkk1=[0 xhat(9) -xhat(8);
               -xhat(9) 0 xhat(7);
               xhat(8) -xhat(7) 0]*T;
        Atilde(7:9,7:9)=Atilde(7:9,7:9)+Aakk1;
        Atilde(7:9,14:16)=Aawkk1;

% Output equation
C = zeros(6,19);
C = [ zeros(1,3) zeros(1,3)   1 0 0    zeros(1,4) zeros(1,3) 1 0 0 
      zeros(1,3) zeros(1,3)   0 1 0    zeros(1,4) zeros(1,3) 0 1 0
      zeros(1,3) zeros(1,3)   0 0 1    zeros(1,4) zeros(1,3) 0 0 1
	  zeros(1,3) zeros(1,3) zeros(1,3) zeros(1,4)   1 0 0    0 0 0
      zeros(1,3) zeros(1,3) zeros(1,3) zeros(1,4)   0 1 0    0 0 0
      zeros(1,3) zeros(1,3) zeros(1,3) zeros(1,4)   0 0 1    0 0 0];  

% system noise
Q=[0.7447*eye(3) zeros(3,6);zeros(3) 0.0038*eye(3) zeros(3);zeros(3,6) 1.9074e-6*eye(3)];

Gakk1=[0 xhat(6) -xhat(5);
               -xhat(6) 0 xhat(4);
               xhat(5) -xhat(4) 0]*T;

Gamma_cart=[T*T*T/6*eye(3);T*T/2*eye(3);T*eye(3)];

Gamma=[Gamma_cart [zeros(6,3);Gakk1] zeros(9,3);
        zeros(4,3) Gqkk1 zeros(4,3)
        zeros(3)   Gwkk1 zeros(3)
        zeros(3)   zeros(3) eye(3)]; 

Q1=Gamma*Q*Gamma';

% measurement noise	 
R = [7.4486e-5*eye(3,3) zeros(3,3)
        zeros(3,3) 3.8149e-5*eye(3,3)];

% Kalman Filter equations
Pbar = Atilde*Phat*Atilde' + Q1;
Pbar;
[qpred] = QuatPrediction(xhat(10:16), T);
[Kartpred] = PositionPrediction(xhat(1:9),T);

xpred = [Kartpred ; qpred;xhat(17:19)];

I1919 = eye(19,19);

d1 = rcond((C*Pbar*C'+R)^(-1));
if (d1 < 1e-10)
    Flag  = 1;
    ['HELP']
    Pbar
    C
    R
    Atilde
    Gamma
    Qmeas
    dt
else
    Flag = 0;
end
K = Pbar*C'*(C*Pbar*C'+R)^(-1);
Phat = [I1919 - K*C] * Pbar;
ypred = zeros(6,1);
ypred = C * xpred;
xhat = xpred + K * [y - ypred];
dqTest = sqrt(sum(xhat(10:13).^2));
% normalisation of the quaternion
xhat(10:13) = xhat(10:13)/dqTest;


