function [xpred] = PositionPrediction(xhat,dt);

% function [xpred] = PositionPrediction(xhat,dt)
% Prediction of an cartesian motion with a consant acceleration model
%
% Input: xhat  Last state
%        dt    Time to predict into the future
%
% Output: xpred  Predicted state

A = [ eye(3,3)  dt*eye(3,3) 0.5*dt^2*eye(3,3)
     zeros(3,3)  eye(3,3)     dt*eye(3,3)
     zeros(3,3) zeros(3,3)   eye(3,3)];

 xpred = A*xhat;