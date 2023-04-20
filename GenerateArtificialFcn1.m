function [VisionData, InertialDataG, InertialData] = GenerateArtificialFcn1(AInArray, WInArray, tInertial, tVision);

q_vi = [-0.5;-0.5;0.5;-0.5];
q_iv = QuatInvers(q_vi);
t_iv = [0;-0.04;0.04];
tBase_I = QuatVekRotate(q_vi,t_iv);
tShift = [0;0;0];%[0.0091;0.0068;0.0052];
tBase_I = tBase_I + QuatVekRotate(q_vi,tShift);

tEnd=max(tInertial(length(tInertial)), tVision(length(tVision)));

dt = 0.001;
t = 0:dt:tEnd;
dim = length(t);

ax_org = zeros(size(t));
ay_org = zeros(size(t));
az_org = zeros(size(t));

wis = 50/180*pi;

dtsim = 0.001;
t0 = cputime;

% Construction of the Motion Data in Inertial Frame
[wResI,aResI,qResI,vResI,pResI,aResNGI,aDt,wDt,tsim] = GenerateInertiaData1(tEnd,AInArray,WInArray,[1;0;0;0],q_vi,dtsim);
dim = length(wResI);
t1 = cputime;

% Construction of the Motion Data in the Vision Frame
qResV = zeros(size(qResI));
pResV = zeros(size(pResI));
aResV = zeros(size(aResI));
wResV = zeros(size(wResI));
vResV = zeros(size(vResI));
aResNGV = zeros(size(aResNGI));
for i=1:dim
	qResV(:,i) = QuatVekRotate(q_vi,qResI(:,i));
	d = QuatVekRotate(q_vi,pResI(:,i));
	pResV(:,i) = d(2:4);
	d = QuatVekRotate(q_vi,vResI(:,i));
	vResV(:,i) = d(2:4);
	d = QuatVekRotate(q_vi,aResI(:,i));
	aResV(:,i) = d(2:4);
	d = QuatVekRotate(q_vi,aResNGI(:,i));
	aResNGV(:,i) = d(2:4);
	d = QuatVekRotate(q_vi,wResI(:,i));
	wResV(:,i) = d(2:4);
end
clear d;
t2 = cputime;

['Time needed for Inertia Data t = ' num2str(t1-t0)];
['Time needed for Vision Data t = ' num2str(t2-t1)];

tIn=(round((tInertial/1e-3))+1)';
tVi=(round((tVision/1e-3))+1)';

VisionData.t = tsim(tVi)';
VisionData.px = pResV(1,tVi)';
VisionData.py = pResV(2,tVi)';
VisionData.pz = pResV(3,tVi)';
VisionData.q0 = qResV(1,tVi)';
VisionData.q1 = qResV(2,tVi)';
VisionData.q2 = qResV(3,tVi)';
VisionData.q3 = qResV(4,tVi)';

InertialData.t = tsim(tIn)';
InertialData.ax = aResI(1,tIn)';
InertialData.ay = aResI(2,tIn)';
InertialData.az = aResI(3,tIn)';
InertialData.wx = wResI(1,tIn)';
InertialData.wy = wResI(2,tIn)';
InertialData.wz = wResI(3,tIn)';

InertialDataG.t = tsim(tIn)';
InertialDataG.ax = aResNGI(1,tIn)';
InertialDataG.ay = aResNGI(2,tIn)';
InertialDataG.az = aResNGI(3,tIn)';
InertialDataG.wx = wResI(1,tIn)';
InertialDataG.wy = wResI(2,tIn)';
InertialDataG.wz = wResI(3,tIn)';