function [VisionData, InertialData, VisionReal] = GenerateArtificialFcn(AInArray, WInArray, tEnd, HeadRotArray);

if nargin == 3
	HeadRotArray = [];
	HeadRotOk = 0;
else
	HeadRotOk = 1;
end

q_vi = [-0.5;-0.5;0.5;-0.5];
%q_vi = [1;0;0;0];
q_iv = QuatInvers(q_vi);
t_iv = [0;-0.04;-0.04];
%t_iv = [0.0091;0.0068;0.0051];
tBase_I = QuatVekRotate(q_vi,t_iv);
tShift = [0.0091;0.0068;0.0052];
tBase_I = tBase_I + QuatVekRotate(q_vi,tShift);

NoiseInit

dt = 0.005;
t = 0:dt:tEnd;
dim = length(t);

ax_org = zeros(size(t));
ay_org = zeros(size(t));
az_org = zeros(size(t));

wis = 50/180*pi;
% Messinput

dtsim = dt;
t0 = cputime;
%[wResI,aResI,qResI,vResI,pResI,aResNGI] = GenerateInertiaData([ax_org;ay_org;az_org],[wx_org;wy_org;wz_org],[1;0;0;0],q_vi,0.01);
[wResI,aResI,qResI,vResI,pResI,aResNGI,aDt,wDt,tsim] = GenerateInertiaData1(tEnd,AInArray,WInArray,[1;0;0;0],q_vi,dtsim);
%dim = length(tsim);
dim = length(wResI);
t1 = cputime;

%HeadInput
if (HeadRotOk==1)
	[wResI_h,aResI_h,qResI_h,vResI_h,pResI_h,aResNGI_h,aDt_h,wDt_h,tsim] = GenerateInertiaData1(tEnd,[],HeadRotArray,[1;0;0;0],q_vi,dtsim);
else
	pResI_h = zeros(size(pResI));
	vResI_h = zeros(size(vResI));
	aResI_h = zeros(size(aResI));
	aResNGI_h = zeros(size(aResI));
	qResI_h = zeros(size(qResI));
	qResI_h(1,:) = 1;
	wResI_h = zeros(size(wResI));
	
end

qResV = zeros(size(qResI));
pResV = zeros(size(pResI));
aResV = zeros(size(aResI));
wResV = zeros(size(wResI));
vResV = zeros(size(vResI));
aResNGV = zeros(size(aResNGI));

qResV_pm = zeros(size(qResI));
pResV_pm = zeros(size(pResI));
aResV_pm = zeros(size(aResI));
wResV_pm = zeros(size(wResI));
vResV_pm = zeros(size(vResI));
aResNGV_pm = zeros(size(aResNGI));

qResI_1 = qResI;

for i=1:dim
	qSum = qResI_1(:,i)+qResI_h(:,i);
	qSum = qSum / QuatNorm(qSum);
	qResV(:,i) = QuatVekRotate(q_vi,qSum);
	qResV_pm(:,i) = QuatVekRotate(q_vi,qResI_1(:,i));
	qRotHead = QuatVekRotate(q_vi,qResI_h(:,i));
	qRotHeadTime(:,i) = qRotHead;
	d = QuatVekRotate(q_vi,wResI(:,i)+wResI_h(:,i));
	if (HeadRotOk==1)
		wResV_pm(:,i) = d(2:4);
		d = QuatVekRotate(qRotHead,d);
	end
	wResV(:,i) = d(2:4);
	
	d = QuatVekRotate(q_vi,pResI(:,i)+pResI_h(:,i));
	if (HeadRotOk==1)
		pResV_pm(:,i) = d(2:4);
		d = QuatVekRotate(qRotHead,d);
	end
	pResV(:,i) = d(2:4);
	d = QuatVekRotate(q_vi,vResI(:,i)+vResI_h(:,i));
	if (HeadRotOk==1)
		vResV_pm(:,i) = d(2:4);
		d = QuatVekRotate(qRotHead,d);
	end
	vResV(:,i) = d(2:4);
	d = QuatVekRotate(q_vi,aResI(:,i)+aResI_h(:,i));
	da = 0;
	if (HeadRotOk==1)
		aResV_pm(:,i) = d(2:4);
		d = QuatVekRotate(qRotHead,d);
		da = cross(wResV(:,i),vResV(:,i));
	end
	aResV(:,i) = d(2:4) + da;
	d = QuatVekRotate(q_vi,aResNGI(:,i)+aResNGI_h(:,i));
	if (HeadRotOk==1)
		aResNGV_pm(:,i) = d(2:4);
		d = QuatVekRotate(qRotHead,d);		
	end
	aResNGV(:,i) = d(2:4) + da;
	
	qResI(:,i) = qSum;
end
if (HeadRotOk==0)
	qResV_pm=qResV;
	wResV_pm=wResV;
	pResV_pm=pResV;
	vResV_pm=vResV;
	aResV_pm=aResV;
	aResNGV_pm=aResNGV;
end
clear d;
t2 = cputime;

['Time needed for Inertia Data t = ' num2str(t1-t0)];
['Time needed for Vision Data t = ' num2str(t2-t1)];

dtInertia = 0.005;
dtVision  = 0.04;

InertStart = 0;
VisionStart = 0.0;

AnzInert = floor((max(tsim)-InertStart)/dtInertia)+1;
AnzVision = floor((max(tsim)-VisionStart)/dtVision)+1;

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
	a_InertMeas(:,i) = aResI(:,IndInert);
	ag_InertMeas(:,i) = aResNGI(:,IndInert);
	w_InertMeas(:,i) = wResI(:,IndInert);
	q_InertMeas(:,i) = qResI(:,IndInert);
	p_InertMeas(:,i) = pResI(:,IndInert);
	v_InertMeas(:,i) = vResI(:,IndInert);
end
	
for i=1:AnzVision
	IndInert = round(tVision(i)/dtsim +1);
	p_VisionMeas(:,i) = pResV(:,IndInert);
	q_VisionMeas(:,i) = qResV(:,IndInert);
	w_VisionMeas(:,i) = wResV(:,IndInert);
	v_VisionMeas(:,i) = vResV(:,IndInert);
	a_VisionMeas(:,i) = aResV(:,IndInert);
end

VisionData.t = tVision;
VisionData.q = q_VisionMeas;
VisionData.p = p_VisionMeas;
VisionData.w = w_VisionMeas;
VisionData.v = v_VisionMeas;
VisionData.a = a_VisionMeas;

InertialData.t = tInert;
InertialData.aNoG = a_InertMeas;
InertialData.a = ag_InertMeas;
InertialData.w = w_InertMeas;
InertialData.q = q_InertMeas;
InertialData.p = p_InertMeas;
InertialData.v = v_InertMeas;

VisionReal.t = tsim;
VisionReal.a = aResV;
VisionReal.v = vResV;
VisionReal.p = pResV;
VisionReal.q = qResV;
VisionReal.w = wResV;
VisionReal.vI = vResI;
VisionReal.pI = pResI;
VisionReal.qI = qResI;
VisionReal.wI = wResI;
VisionReal.a_pm = aResV_pm;
VisionReal.v_pm = vResV_pm;
VisionReal.p_pm = pResV_pm;
VisionReal.q_pm = qResV_pm;
VisionReal.w_pm = wResV_pm;
VisionReal.qrt = qRotHeadTime;
VisionReal.qh = qResI_h;