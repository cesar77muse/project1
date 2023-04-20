function B = QuatMeasMatrix(Omega)

% QuatMeasMatrix(Omega)
% Generates a measurement matrix for an estimated prediction

B = zeros(4,4);
B(1,2) = -Omega(1+1);
B(1,3) = -Omega(2+1);
B(1,4) = -Omega(3+1);

B(2,2) =  Omega(0+1);
B(2,3) = -Omega(3+1);
B(2,4) =  Omega(2+1);

B(3,2) =  Omega(3+1);
B(3,3) =  Omega(0+1);
B(3,4) = -Omega(1+1);

B(4,2) = -Omega(2+1);
B(4,3) =  Omega(1+1);
B(4,4) =  Omega(0+1);