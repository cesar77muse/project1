function [q] = QuatMake(Winkel,Achse);

% [q] = QuatMake(Winkel,Achse);
% Generation of a quaternion
% q = [cos(Winkel/2) ; sin(Winkel(2)*Achse]'
% 
% Achse is normalized in the function

q=zeros(4,1);

Achse = Achse/sqrt(sum(Achse.^2));

q(1) = cos(Winkel/2);
q(2:4) = sin(Winkel/2)*Achse;