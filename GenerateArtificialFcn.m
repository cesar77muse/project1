function [VisionData, InertialDataG, InertialData, VisionReal,InertialReal] = GenerateArtificialFcn(AInArray, WInArray, tEnd, NoiseFak);

if nargin == 3
	NoiseFak = 0;
end

q_vi = [-0.5;-0.5;0.5;-0.5];
q_iv = QuatInvers(q_vi);
t_iv = [0;-0.04;0.04];
tBase_I = QuatVekRotate(q_vi,t_iv);
tShift = [0;0;0];%[0.0091;0.0068;0.0052];
tBase_I = tBase_I + QuatVekRotate(q_vi,tShift);

[NoiseW, NoiseA, NoiseP, NoiseQ] = NoiseInit(NoiseFak,10000);

dt = 0.005;
t = 0:dt:tEnd;
dim = length(t);

ax_org = zeros(size(t));
ay_org = zeros(size(t));
az_org = zeros(size(t));

wis = 50/180*pi;

dtsim = 0.005;
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

aRes = aResNGI;
wRes = wResI;
pRes = pResV;
qRes = qResV;

wx = wRes(1,:);
wy = wRes(2,:);
wz = wRes(3,:);

ax = aResNGI(1,:);
ay = aResNGI(2,:);
az = aResNGI(3,:);

['Time needed for Inertia Data t = ' num2str(t1-t0)];
['Time needed for Vision Data t = ' num2str(t2-t1)];

% Settings for the sampling time
dtInertia = 0.005;
dtVision  = 0.025;

InertStart = 0;
VisionStart = 0;

AnzInert = floor((max(tsim)-InertStart)/dtInertia)+1;
AnzVision = floor((max(tsim)-VisionStart)/dtVision)+1;

% Measurement Data
a_InertMeas = zeros(3,AnzInert);
ag_InertMeas = zeros(3,AnzInert);
w_InertMeas = zeros(3,AnzInert);
q_VisionMeas = zeros(4,AnzVision);
p_VisionMeas = zeros(3,AnzVision);
w_VisionMeas = zeros(3,AnzVision);
v_VisionMeas = zeros(3,AnzVision);

tVision = VisionStart:dtVision:tEnd;
tInert = InertStart:dtInertia:tEnd;

for i=1:AnzInert
	IndInert = round(tInert(i)/dtsim +1);
	a_InertMeas(:,i) = aResI(:,IndInert) + NoiseA(:,IndInert);
	ag_InertMeas(:,i) = aResNGI(:,IndInert) + NoiseA(:,IndInert);
	w_InertMeas(:,i) = wResI(:,IndInert) + NoiseW(:,IndInert);
    q_InertMeas(:,i) = qResI(:,IndInert);
    v_InertMeas(:,i) = vResI(:,IndInert);
    p_InertMeas(:,i) = pResI(:,IndInert);
end
	
for i=1:AnzVision
	IndInert = round(tVision(i)/dtsim +1);
	p_VisionMeas(:,i) = pResV(:,IndInert) + NoiseP(:,IndInert);
	q_VisionMeas(:,i) = qResV(:,IndInert) + NoiseQ(:,IndInert);
	w_VisionMeas(:,i) = wResV(:,IndInert);
	v_VisionMeas(:,i) = vResV(:,IndInert);
	a_VisionMeas(:,i) = aResV(:,IndInert);
    ag_VisionMeas(:,i) = aResNGV(:,IndInert);
end

VisionData.t = tVision';
VisionData.px = p_VisionMeas(1,:)';
VisionData.py = p_VisionMeas(2,:)';
VisionData.pz = p_VisionMeas(3,:)';
VisionData.q0 = q_VisionMeas(1,:)';
VisionData.q1 = q_VisionMeas(2,:)';
VisionData.q2 = q_VisionMeas(3,:)';
VisionData.q3 = q_VisionMeas(4,:)';
%VisionData.ax = ag_VisionMeas(1,:)';
%VisionData.ay = ag_VisionMeas(2,:)';
%VisionData.az = ag_VisionMeas(3,:)';
%VisionData.wx = w_VisionMeas(1,:)';
%VisionData.wy = w_VisionMeas(2,:)';
%VisionData.wz = w_VisionMeas(3,:)';


InertialData.t = tInert';
InertialData.ax = a_InertMeas(1,:)';
InertialData.ay = a_InertMeas(2,:)';
InertialData.az = a_InertMeas(3,:)';
InertialData.wx = w_InertMeas(1,:)';
InertialData.wy = w_InertMeas(2,:)';
InertialData.wz = w_InertMeas(3,:)';

InertialDataG.t = tInert';
InertialDataG.ax = ag_InertMeas(1,:)';
InertialDataG.ay = ag_InertMeas(2,:)';
InertialDataG.az = ag_InertMeas(3,:)';
InertialDataG.wx = w_InertMeas(1,:)';
InertialDataG.wy = w_InertMeas(2,:)';
InertialDataG.wz = w_InertMeas(3,:)';

VisionReal.t = tsim;
VisionReal.ax = aResV(1,:);
VisionReal.ay = aResV(2,:);
VisionReal.az = aResV(3,:);
VisionReal.vx = vResV(1,:);
VisionReal.vy = vResV(2,:);
VisionReal.vz = vResV(3,:);
VisionReal.px = pResV(1,:);
VisionReal.py = pResV(2,:);
VisionReal.pz = pResV(3,:);
VisionReal.q0 = qResV(1,:);
VisionReal.q1 = qResV(2,:);
VisionReal.q2 = qResV(3,:);
VisionReal.q3 = qResV(4,:);
VisionReal.w1 = wResV(1,:);
VisionReal.w2 = wResV(2,:);
VisionReal.w3 = wResV(3,:);

InertialReal.t = tsim;
InertialReal.ax = aResI(1,:);
InertialReal.ay = aResI(2,:);
InertialReal.az = aResI(3,:);
InertialReal.vx = vResI(1,:);
InertialReal.vy = vResI(2,:);
InertialReal.vz = vResI(3,:);
InertialReal.px = pResI(1,:);
InertialReal.py = pResI(2,:);
InertialReal.pz = pResI(3,:);
InertialReal.q0 = qResI(1,:);
InertialReal.q1 = qResI(2,:);
InertialReal.q2 = qResI(3,:);
InertialReal.q3 = qResI(4,:);
InertialReal.w1 = wResI(1,:);
InertialReal.w2 = wResI(2,:);
InertialReal.w3 = wResI(3,:);
InertialReal.wx = wResI(1,:);
InertialReal.wy = wResI(2,:);
InertialReal.wz = wResI(3,:);