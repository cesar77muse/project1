function [a_new] = RotateMeasurement(a,qrot);

dim = length(a);
a_new = zeros(3,dim);

for i=1:dim
	dummy = QuatVekRotate(qrot,a(1:3,i));
	a_new(:,i) = dummy(2:4);
end
