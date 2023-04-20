tdim = length(t);

aDummy = zeros(4,1);
aRot = zeros(3,tdim);
for i = 1:tdim
	aDummy = -cross(InertiaReal_a0.w(:,i),InertiaReal_w0.v(:,i));
	aDummy = QuatVekRotate(InertiaReal_a0.q(:,i),aDummy);
	aRot(:,i) = aDummy(2:4);
end
