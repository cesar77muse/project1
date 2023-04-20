function [wRes,aRes,qRes,vRes,pRes,aResNG,ain,win,tsim] = GenerateInertiaData1(tend,aInArray,wInArray,qstart,q_vi,dt)

tsim = 0:dt:tend;
dim = length(tsim);

ain = zeros(3,dim);
win = zeros(3,dim);

[AnzA dummy] = size(aInArray);
clear dummy

aInArrayX=zeros(0,4);
aInArrayY=zeros(0,4);
aInArrayZ=zeros(0,4);

AnzAX = 0;
AnzAY = 0;
AnzAZ = 0;

t0 = cputime;
for i = 1:AnzA
	if (aInArray(i,1)==1)
		aInArrayX = [aInArrayX ; aInArray(i,2:5)];
		AnzAX = AnzAX + 1;
	elseif (aInArray(i,1)==2)
		aInArrayY = [aInArrayY ; aInArray(i,2:5)];
		AnzAY = AnzAY + 1;
	elseif (aInArray(i,1)==3)
		aInArrayZ = [aInArrayZ ; aInArray(i,2:5)];
		AnzAZ = AnzAZ + 1;
	end
end

[AnzW dummy] = size(wInArray);
clear dummy

wInArrayX=zeros(0,4);
wInArrayY=zeros(0,4);
wInArrayZ=zeros(0,4);

%Decomposition of the input data
AnzWX = 0;
AnzWY = 0;
AnzWZ = 0;
for i = 1:AnzW
	if (wInArray(i,1)==1)
		wInArrayX = [wInArrayX ; wInArray(i,2:5)];
		AnzWX = AnzWX + 1;
	elseif (wInArray(i,1)==2)
		wInArrayY = [wInArrayY ; wInArray(i,2:5)];
		AnzWY = AnzWY + 1;
	elseif (wInArray(i,1)==3)
		wInArrayZ = [wInArrayZ ; wInArray(i,2:5)];
		AnzWZ = AnzWZ + 1;
	end
end
t1 = cputime;

% Construction of the "fine" motion data
[AnzAX AnzAY AnzAZ];
aInArrayY;
if (AnzAX>0)
	[aXDt, tsim] = GenerateFineInput(aInArrayX,dt,tend);
else
	aXDt = zeros(size(tsim));
end
if (AnzAY>0)
	[aYDt, tsim] = GenerateFineInput(aInArrayY,dt,tend);
else
	aYDt = zeros(size(tsim));
end
if (AnzAZ>0)
	[aZDt, tsim] = GenerateFineInput(aInArrayZ,dt,tend);
else
	aZDt = zeros(size(tsim));
end

if (AnzWX>0)
	[wXDt, tsim] = GenerateFineInput(wInArrayX,dt,tend);
else
	wXDt = zeros(size(tsim));
end
if (AnzWY>0)
	[wYDt, tsim] = GenerateFineInput(wInArrayY,dt,tend);
else
	wYDt = zeros(size(tsim));
end
if (AnzWZ>0)
	[wZDt, tsim] = GenerateFineInput(wInArrayZ,dt,tend);
else
	wZDt = zeros(size(tsim));
end
t2 = cputime;

ain = [aXDt ; aYDt; aZDt];
win = [wXDt ; wYDt; wZDt];

wRes = zeros(3,dim);
aRes = zeros(3,dim);
vRes = zeros(3,dim);
pRes = zeros(3,dim);
aResNG = zeros(3,dim);
qRes = zeros(4,dim);
qI = qstart;
vI = zeros(3,1);
pI = zeros(3,1);
aI = zeros(3,1);
gI = [0;0;9.81];
q_iv = QuatInvers(q_vi);
gV = QuatVekRotate(q_iv,gI);
for i=1:dim
	wI = win(:,i);
    pI = pI + dt*vI + dt^2/2*aI;
	vI = vI + dt*aI;
    %vIDummy=QuatVekRotate(qI,[0;vI]);
    aDummy = QuatVekRotate(qI,ain(:,i)) + [0;cross(wI,vI)];
    %if (abs(aDummy(2))<=0.1)
    %    aDummy(2)=0;
    %end
    %if (abs(aDummy(3))<=0.1)
    %    aDummy(3)=0;
    %end
    %if (abs(aDummy(4))<=0.1)
    %    aDummy(4)=0;
    %end    
    vDummy = [0;vI];
	vI = vDummy(2:4);

	aG = QuatVekRotate(qI,gI);
    aI = aDummy(2:4);
	wDummy = QuatVekRotate(qI,wI);
	wI = wDummy(2:4);
	qDummy = [qI;wI];

	qpred = QuatPrediction(qDummy,dt);
	qI = qpred(1:4);
    qI=qI/norm(qI);
	
	aRes(:,i) = aI;
	wRes(:,i) = wI;
	qRes(:,i) = qI;
	vRes(:,i) = vI;
	pRes(:,i) = pI;
	aResNG(:,i) = aI + aG(2:4);
end
t3 = cputime;
['Time for inertia data: ' num2str(t3-t2)];

	

