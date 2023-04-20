function [Output] = FUsionSimulationExt(InertiaData, VisionData, dtSim, FlagInterSampling);

t1 = cputime;

if (nargin==3)
	FlagInterSampling=1;
end

t_length=max(InertiaData.t(length(InertiaData.t)),VisionData.t(length(VisionData.t)));

num_iter=round(t_length/dtSim);

% Initialisation
t_Vmeas = VisionData.t;
px_vmeas = VisionData.px;
py_vmeas = VisionData.py;
pz_vmeas = VisionData.pz;

qw_vmeas = VisionData.q0;
qx_vmeas = VisionData.q1;
qy_vmeas = VisionData.q2;
qz_vmeas = VisionData.q3;

t_Imeas = InertiaData.t;
ax_meas = InertiaData.ax;
ay_meas = InertiaData.ay;
az_meas = InertiaData.az;

wx_meas = InertiaData.wx;
wy_meas = InertiaData.wy;
wz_meas = InertiaData.wz;

ViDim = length(t_Vmeas);
InDim = length(t_Imeas);

tEnd = max(max(t_Vmeas),max(t_Imeas));

%dtSim = 1e-4;
t = 0:dtSim:tEnd;
dim = length(t);

% Position Messungen
px = zeros(size(t));
py = zeros(size(t));
pz = zeros(size(t));

px = px_vmeas;
py = py_vmeas;
pz = pz_vmeas;

% Orientation Messungen
qw = zeros(size(t));
qx = zeros(size(t));
qy = zeros(size(t));
qz = zeros(size(t));

qw = qw_vmeas;
qx = qx_vmeas;
qy = qy_vmeas;
qz = qz_vmeas;

% Beschleunigung Messungen
ax = zeros(size(t));
ay = zeros(size(t));
ay = zeros(size(t));

ax = ax_meas;
ay = ay_meas;
az = az_meas;

% Winkelgeschwindigkeit Messungen
wx = zeros(size(t));
wy = zeros(size(t));
wz = zeros(size(t));

wx = wx_meas;
wy = wy_meas;
wz = wz_meas;

PHatTime = zeros(3,0);
VHatTime = zeros(3,0);
AHatTime = zeros(3,0);

qHatTime = zeros(4,0);
WHatTime = zeros(3,0);

BAxTime = zeros(3,0);

UPInTime = zeros(6,InDim);
UPVTime = zeros(7,ViDim);
QNormTime = zeros(1,0);

PNiTime = zeros(3,0);
ANTime = zeros(3,0);
WNTime = zeros(3,0);

UIin = zeros(6,0);
UVin = zeros(7,0);

InCountTime = zeros(1,0);

tTime =[];

azClear = [];

xhat = zeros(19,1);
xhat(10) = 1;
Phat = 1e5*eye(19,19);

InCount = 1;
ViCount = 1;

Tlast = 0;

% Sensor Setup
TimeCount = 1;
q_vi = [-0.5;-0.5;0.5;-0.5];
q_iv = QuatInvers(q_vi);
t_iv = [0;-0.04;-0.04];
%t_iv = [0.0091;0.0068;0.0051];
tBase_I = QuatVekRotate(q_vi,t_iv);
tShift = [0.0091;0.0068;0.0052];
tBase_I = tBase_I + QuatVekRotate(q_vi,tShift);
gI = [0;0;9.81];
gV = QuatVekRotate(q_vi,gI);

['Start of Loop']
for TimeCount=1:num_iter
    DataOIk = 0;
    DataVOk = 0;
%    [t(TimeCount) t_Imeas(InCount) t_Vmeas(ViCount)];
    time=(TimeCount-1)*dtSim;
    if (InCount<length(InertiaData.t))
    %if there is an inertial data use it
	if ( InertiaData.t(InCount)<=time)  %(InCount<=InDim) & (round(t_Imeas(InCount)*1e4) == (round(t(TimeCount)*1e4))) )
	    % Inertial Measurement 
%        ['Inertial Measurements: ' num2str(t(TimeCount))]
		aI = [ ax(InCount); ay(InCount); az(InCount)];  
        wI = [ wx(InCount); wy(InCount); wz(InCount)];
		
     	% Rotate the measurements and the gravity
		q_rot = xhat(10:13);
		aDummy = QuatVekRotate(q_rot,gV);
		azClear = [azClear aDummy(2:4)];
		aIN = QuatVekRotate(q_vi,aI); aIN = aIN(2:4);
		wIN = QuatVekRotate(q_vi,wI); wIN = wIN(2:4);			
		aI = aIN - aDummy(2:4);  
        
		% Use the rotatet input
        uIn = [aI ; wIN];
		UIin = [UIin uIn];
        dt = time - Tlast;
        
		% EKF for inertial data
        [xhat, xpred, Phat, upred, dqTest, K, Flag] = InertialEKFExt(xhat, Phat, uIn, dt);
        Tlast = time;
        UPInTime(:,InCount) = upred;
       
		% output generation
        DataIOk = 1;
        InCount = InCount+1;		
    
    end
    end
	if (ViCount<length(VisionData.t))
	%if there is a vision data use it
    if (VisionData.t(ViCount)<=time)  %(ViCount<=ViDim) & (round(t_Vmeas(ViCount)*1e4) == (round(t(TimeCount)*1e4)))) %
        dt = time - Tlast;
		% generate vision measurements
 %       ['Vision Measurements']
        pN = [ px(ViCount);py(ViCount);pz(ViCount)];  
        qV = [qw(ViCount) ; qx(ViCount) ; qy(ViCount); qz(ViCount)];
        uV = [pN ; qV];
		
		UVin = [UVin uV];
        
		% EKF for vision data
        [xhat, xpred, Phat, upred, dqTest] = VisionKFExt(xhat, Phat, uV, dt);
        
        Tlast = time;
        UPVTime(:,ViCount) = upred;
        
        DataVOk = 1;        
        ViCount = ViCount+1;
        
    end
    end
    
    if (FlagInterSampling==0)
    
        dt = time - Tlast;
        
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
        [Qtran, Qw1,Gqkk1,Gwkk1] = GeneratePartialDeriv(q, w, dt);

                %Dynamic linearised model
        if (dt ~= 0)
         Atilde = [  eye(3,3) dt*eye(3,3) 0.5*dt^2*eye(3,3) zeros(3,4) zeros(3,3) zeros(3,3)
            zeros(3,3)    eye(3,3)    dt*eye(3,3)    zeros(3,4) zeros(3,3) zeros(3,3)
            zeros(3,3)  zeros(3,3)     eye(3,3)      zeros(3,4) zeros(3,3) zeros(3,3)
            zeros(4,3)  zeros(4,3)    zeros(4,3)       Qtran      Qw1       zeros(4,3)
            zeros(3,3)  zeros(3,3)    zeros(3,3)     zeros(3,4)  eye(3,3) zeros(3,3)
            zeros(3,3)  zeros(3,3)    zeros(3,3)     zeros(3,4) zeros(3,3)  eye(3)]; 
        else
         Atilde = eye(19,19);
        end
        
        Aakk1=[0 -xhat(16) xhat(15);
               xhat(16) 0 -xhat(14);
               -xhat(15) xhat(14) 0]*dt;
        Aawkk1=[0 xhat(9) -xhat(8);
               -xhat(9) 0 xhat(7);
               xhat(8) -xhat(7) 0]*dt;
        Atilde(7:9,7:9)=Atilde(7:9,7:9)+Aakk1;
        Atilde(7:9,14:16)=Aawkk1;
        
        Q=[10*eye(3) zeros(3,6);zeros(3) 10*eye(3) zeros(3);zeros(3,6) 0.01*eye(3)];

        Gakk1=[0 xhat(6) -xhat(5);
               -xhat(6) 0 xhat(4);
               xhat(5) -xhat(4) 0]*dt;
        
        Gamma_cart=[dt*dt*dt/6*eye(3);dt*dt/2*eye(3);dt*eye(3)];

        Gamma=[Gamma_cart [zeros(6,3);Gakk1] zeros(9,3);
        zeros(4,3) Gqkk1 zeros(4,3)
        zeros(3)   Gwkk1 zeros(3)
        zeros(3)   zeros(3) eye(3)]; 

        Q1=Gamma*Q*Gamma';
        Phat = Atilde*Phat*Atilde' + Q1;
        [qpred] = QuatPrediction(xhat(10:16), dt);
        [Kartpred] = PositionPrediction(xhat(1:9),dt);
        
        %Add cross product between omega and acceleration
        Kartpred(7:9)=Kartpred(7:9)+cross(xhat(14:16),xhat(7:9))*dt;
        
        xhat = [Kartpred ; qpred;xhat(17:19)];
        
        dqTest = sqrt(sum(xhat(10:13).^2));
        xhat(10:13) = xhat(10:13)/dqTest;
        
        Tlast = time;
    end

    if (DataIOk==1)|(DataVOk==1)
        % output generation
        PHatTime = [PHatTime xhat(1:3)];
        VHatTime = [VHatTime xhat(4:6)];
        AHatTime = [AHatTime xhat(7:9)];
        qHatTime = [qHatTime xhat(10:13)];
        WHatTime = [WHatTime xhat(14:16)];
        QNormTime = [QNormTime dqTest];
	    BAxTime = [BAxTime xhat(17:19)];
        tTime = [tTime t(TimeCount)];
    end
end

Output.t     = tTime';
Output.px    = PHatTime(1,:)';
Output.py    = PHatTime(2,:)';
Output.pz    = PHatTime(3,:)';
Output.vx    = VHatTime(1,:)';
Output.vy    = VHatTime(2,:)';
Output.vz    = VHatTime(3,:)';
Output.ax    = AHatTime(1,:)';
Output.ay    = AHatTime(2,:)';
Output.az    = AHatTime(3,:)';
Output.q0    = qHatTime(1,:)';
Output.q1    = qHatTime(2,:)';
Output.q2    = qHatTime(3,:)';
Output.q3    = qHatTime(4,:)';
Output.w1    = WHatTime(1,:)';
Output.w2    = WHatTime(2,:)';
Output.w3    = WHatTime(3,:)';
Output.bx    = BAxTime(1,:)';
Output.by    = BAxTime(2,:)';
Output.bz    = BAxTime(3,:)';

t2 = cputime;
['Time needed = ' num2str(t2-t1)]