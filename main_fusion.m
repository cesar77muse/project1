close all
clear 
clc

% add the sub-libs' path
addpath('calibration_filess');
addpath(genpath('vo_functions'));
addpath(genpath('load_files'));
addpath(genpath('imu_and_ground_truth_processing'));

%gg : the gravity correction
global idt gg Tic;          % global variables for IMU params

idt       = 1/10;           % IMU    @10Hz
cdt       = 1/10;           % Camera @10Hz
sampleRatio = cdt/idt;

%set the defauls variance, file paths, etc
setupParams;

% set plot boundary
gt_x_min= -3;
gt_x_max = 20;
gt_z_max = 120;
gt_z_min = -2;

% obtain the transformation between IMU and Camera and ground truth
% position
camCalib = loadCalibrationCamToCam( [ calibDir 'calib_cam_to_cam.txt' ] );
IMU_to_vel_Calib = loadCalibrationRigid( [ calibDir 'calib_imu_to_velo.txt' ] );
vel_to_cam_Calib = loadCalibrationRigid( [ calibDir 'calib_velo_to_cam.txt' ] );
oxtsTab = loadOxtsliteData( BaseDir, imInit+1:imInit+M );
pose = convertOxtsToPose( oxtsTab );


imSize = camCalib.S_rect{1};
K = camCalib.P_rect{1}*[ camCalib.R_rect{1} zeros(3,1); zeros(1,3) 1 ];
K = K(1:3,1:3);

IMU_to_cam_Calib = vel_to_cam_Calib*IMU_to_vel_Calib;
Rci = IMU_to_cam_Calib(1:3,1:3);
pci = IMU_to_cam_Calib(1:3,4);

Ric = Rci';
pic = -Ric * pci;

%% load ground truth data
RgiTab = zeros( 3, 3, M );
pgiTab = zeros( 3, M );

ground_truth_rotation = zeros( 3, 3, M );
ground_truth_position = zeros( 3, M );

for k = 1:M
    RgiTab(:,:,k) = pose{k}(1:3,1:3);
    pgiTab(:,k) = pose{k}(1:3,4);
    ground_truth_rotation(:,:,k) = RgiTab(:,:,k)*Ric;
    ground_truth_position(:,k) = pgiTab(:,k)+RgiTab(:,:,k)*pic;
end

%plot(-ground_truth_position(2,:), ground_truth_position(1,:));
%% load IMU Data
vel_index = 9:11; % FLU frame
am_index = 12:14; % 12:14 body frame, 15:17 FLU frame
wm_index = 18:20; % 18:20 body frame, 21:23 FLU frame

IMUTab = zeros(6,M);
for k = 1:M
    IMUTab(1:3,k) = oxtsTab{k}(wm_index);
    IMUTab(4:6,k) = oxtsTab{k}(am_index);
end

% figure();
% subplot( 3, 1, 1 ); plot( 1:M, IMUTab(1,:) ); title( 'x gyro' );
% subplot( 3, 1, 2 ); plot( 1:M, IMUTab(2,:) ); title( 'y gyro' ); 
% subplot( 3, 1, 3 ); plot( 1:M, IMUTab(3,:) ); title( 'z gyro' ); 
% figure();
% subplot( 3, 1, 1 ); plot( 1:M, IMUTab(4,:) ); title( 'x acc' ); 
% subplot( 3, 1, 2 ); plot( 1:M, IMUTab(5,:) ); title( 'y acc' ); 
% subplot( 3, 1, 3 ); plot( 1:M, IMUTab(6,:) ); title( 'z acc' ); 

%%  only use IMU to calculate pose
% nomial state 
% x = [   pgi',   qgi',   vgi',   ba',   bg', ... 
%         1~3     4~7    8~10   11~13  14~16 
%         pgi1,   qgi1,   pgi2,   qgi2   ];
%        17~19   20~23   24~26   27~30    
pgi_nom_index = 1:3; qgi_nom_index = 4:7; vgi_nom_index = 8:10;
ba_nom_index = 11:13; bg_nom_index = 14:16;
pgi1_nom_index = 17:19; qgi1_nom_index = 20:23;
pgi2_nom_index = 24:26; qgi2_nom_index = 27:30;

% error state
% dx = [   pgi',   thgi',   vgi',   ba',   bg', ... 
%          1~3     4~6      7~9    10~12  13~15 
%          pgi1,   thgi1,   pgi2,   thgi2   ];
%         16~18   19~21    22~24   25~27
pgi_index = 1:3; thgi_index = 4:6; vgi_index = 7:9;
ba_index = 10:12; bg_index = 13:15;
pgi1_index = 16:18; thgi1_index = 19:21;
pgi2_index = 22:24; thgi2_index = 25:27;

gg = getRnb(oxtsTab{1})'*[ 0; 0; 9.81; ]; % gravity in body frame
Tic = [ Ric, pic ];

% nominal state
x_nom = zeros( 30, 1 );
x_nom(vgi_nom_index) = getRnb(oxtsTab{1})'*[ oxtsTab{1}(8); oxtsTab{1}(7); 0; ];
x_nom(qgi_nom_index) = [ 1; 0; 0; 0; ];

pgiIMURec = zeros( 3, M );               
vgiIMURec = zeros( 3, M );
vgiIMURec(:,1) = x_nom(vgi_nom_index);
qgiIMURec = zeros( 4, M );
qgiIMURec(:,1) = x_nom(qgi_nom_index); 
for k = 2:M
    x_nom = nomial_state_prediction( x_nom, IMUTab(:,k-1) );
    pgiIMURec(:,k) = x_nom(pgi_nom_index);
    vgiIMURec(:,k) = x_nom(vgi_nom_index);
    qgiIMURec(:,k) = x_nom(qgi_nom_index);
end
%plot3(pgiIMURec(1,:), pgiIMURec(2,:), pgiIMURec(3,:));

%plot(-pgiIMURec(2,:), pgiIMURec(1,:));
%% only stereo visual odometry 
%start parallel processing
if (isempty(gcp) && data_params.use_multithreads)
   parpool("LocalProfile1",12)

end


% Read directories containing images
img_files1 = dir(strcat(data_params.path1,'*.png'));
img_files2 = dir(strcat(data_params.path2,'*.png'));
num_of_images = length(img_files1);
[P1, P2] = createCamProjectionMatrices(cam_params);
pos = [0;0;0];
Rpos = eye(3);

r_pose = [];
start = 0;
for t = 1 : num_of_images - 1
    %% Read images for time instant t
    I2_l = imread([img_files1(t+1).folder, '/', img_files1(t).name]);
    I2_r = imread([img_files2(t+1).folder, '/', img_files2(t).name]);
    fprintf('Frame: %i\n', t);

    %% Bootstraping for initialization
    if (start == 0)
        vo_previous.pts1_l = computeFeatures(I2_l, vo_params.feature);
        vo_previous.pts1_r = computeFeatures(I2_r, vo_params.feature);
        start = 1;
        I1_l = I2_l;
        I1_r = I2_r;
        fprintf('\n---------------------------------\n');
        continue;
    end

    %% Implement SOFT for time instant t+1
    [R, tr, vo_previous] = visualSOFT(t, I1_l, I2_l, I1_r, I2_r, P1, P2, vo_params, vo_previous);

    %% Estimated pose relative to global frame at t = 0
    % pos = pos + Rpos * tr';
    % r_pose = [r_pose;pos' ];
    % Rpos = R * Rpos;
    pos = pos + Rpos * tr';
    r_pose = [r_pose;pos' ];
    Rpos = R * Rpos;
    Vision(t,2:4) = pos';
    Vision(t,8:11) =dcm2quat(Rpos);
    Vision(t,5:7) = dcm2angle(Rpos);

    %% Prepare frames for next iteration
    I1_l = I2_l;
    I1_r = I2_r;

    %% Plot the odometry transformed data
    subplot(2, 3, [3, 6]);

    % Read ground truth pose if flag is true
    if data_params.show_gt_flag
      axis([gt_x_min gt_x_max gt_z_min gt_z_max])
      %T = reshape(ground_truth(t, :), 4, 3)';
      %pos_gt = T(:, 4);
      scatter(-ground_truth_position(2,t), ground_truth_position(1,t), 'g', 'filled');
      hold on;
      scatter(-pgiIMURec(2, t), pgiIMURec(1, t), 'r', 'filled');
    end
    scatter( - pos(1),pos(3), 'b', 'filled');
    title(sprintf('Odometry plot at frame %d', t))
    xlabel('x-axis (in meters)');
    ylabel('z-axis (in meters)');

    if data_params.show_gt_flag
        legend('Ground Truth Pose', 'IMU Pose', 'Stereo VO Pose')
    else
        legend('Estimated Pose')
    end

    %% Pause to visualize the plot
    pause(0.0001);
    fprintf('\n---------------------------------\n');
end

%% visual inertial odometry

%%% you need follow these steps to perform fusion: 
%%% 1. get the IMU pose at time K
%%% 2. Derive the F, G matrix according to thia paper:
%%% https://journals.sagepub.com/doi/pdf/10.1177/0278364907079283 
%%% 3. Calculate Pk|k-1 basedon Fk-1, Pk-1, and Gk-1
%%% 4. get the Stereo VO at time K
%%% 5. base on the setup parameters to obat camera variance : R_delta=R(sampled_outputs,sampled_outputs);
%%% calculate the residual covariance: Sk_delta=(Hk_delta*Pkk1*Hk_delta'+R_delta);
%%% calcualte the kalman gain : Kk_delta=Pkk1*Hk_delta'*inv(Sk_delta);
%%% calculate the residual: rho = (stereo_VO - IMU);
%%% fuse the pose, this is the fused output: xk=xkk1+Kk_delta*rho;
%%% update the covariance: Pk=Pkk1-Kk_delta*Hk_delta*Pkk1;

%% please read the example VI fusion code, and implement your ideas!!! 
%% The code is the demo of paper: https://journals.sagepub.com/doi/pdf/10.1177/0278364907079283 
%% Fast Ego-motion Estimation with Multi-rate Fusion of Inertial and Vision

syms dt 
%% IMU data
imu_pose = zeros(107,7);
imu_pose(:,1) = 1;
tmp_imu = IMUTab'; %% here we have a matrix with 6 data ( x gyro, y gyro, z gyro, x acc, y acc, z acc)
imu_pose(:,2:7) = [tmp_imu(:,4:6) tmp_imu(:,1:3)]; % IMU estimation





F= zeros(21,21); % initializing F matrix
G= zeros(21,12); % intializing G matrix
% according to sate propagation:
F=[eye(3)     dt*eye(3)    0.5*dt*dt*eye(3)
   zeros(3)   eye(3)      dt*eye(3)
   zeros(3)   zeros(3)    eye(3)]

G= [dt*dt*dt/6*eye(3);dt*dt/2*eye(3);dt*eye(3)]



%% quaternions deduction
q_vi = [-0.5;-0.5;0.5;-0.5];

gI = [0;0;9.81]; % gravity 
gV = QuatVekRotate(q_vi,gI); % result quaternion

%% acceleration and angular matrix
aI = [ imu_pose(:,2) imu_pose(:,3) imu_pose(:,4)];  
wI = [ imu_pose(:,5) imu_pose(:,6) imu_pose(:,7)];

%% vision data

Vision(1:106, 2:4) = r_pose; % VO estimation
Vision(:,1) = 1;

pV = [Vision(:,2) Vision(:,3) Vision(:,4)];
qV = [Vision(:,8) Vision(:,9) Vision(:,10) Vision(:,11)];

%% timing
T=gcd(idt*1000,cdt*1000)/1000
t_length=max((length(imu_pose(:,1)))*idt,(length(Vision(:,1)))*cdt);
num_iter=round(t_length/T);
dt=T;

%% counter initializer and state vector
count_I=1;
count_V=1;

%Initial State Vector
xk1=[0;0;0;0;0;0;0;0;0;1;0;0;0;0;0;0;0;0;0];

Pk1=eye(length(xk1));

X=[];
TIME=[];
RHO=[];

%% fki gki cart

fk1_cart=[eye(3)     T*eye(3)    (1/2)*T*T*eye(3)
          zeros(3)   eye(3)      T*eye(3)
          zeros(3)   zeros(3)    eye(3)];
            
gk1_cart=[T*T*T/6*eye(3);T*T/2*eye(3);T*eye(3)];

%% H matrix

Hk = [zeros(3,3) zeros(3,3) eye(3,3)    zeros(3,4)  zeros(3,3) eye(3)
      zeros(3,3) zeros(3,3) zeros(3,3)  zeros(3,4)  eye(3,3)   zeros(3)
      eye(3,3)   zeros(3,3) zeros(3,3)  zeros(3,4)  zeros(3,3) zeros(3)
      zeros(4,3) zeros(4,3) zeros(4,3)  eye(4,4)    zeros(4,3) zeros(4,3)];

AINew=[];
WINew=[];

%% q and r matrix

Q = [7.447e-1*eye(3) zeros(3,6);zeros(3) 3.8e-2*eye(3) zeros(3);zeros(3,6) 1.9074e-7*eye(3)];
R = [7.4486e-1*eye(3,3) zeros(3,10)
    zeros(3,3) 1.8149e-2*eye(3,3) zeros(3,7)
    zeros(3,6) 1.7066e-3*eye(3,3) zeros(3,4)
    zeros(4,9) 9.9441e-5*eye(4)];



sensor = 0
for i=1:num_iter
    %Read Inertial measurements
    time=(i-1)*T;
    sampled_outputs=[];
    yk_delta=[];
    if ((count_I<=length(imu_pose(:,1)))&&((sensor==0)||(sensor==2)))
    if (rem(time,idt)==0)
    if (imu_pose(count_I,1)==1)
        %Rotate Inertial Sensor Measurements and cancels the effect of the gravity
        %aINew   = QuatVekRotate(q_vi,[-9.81;0;0]); aINew = aINew(2:4);
        GravityQuat   = QuatVekRotate(xk1(10:13),gI);
        GravityQuat = QuatVekRotate(QuatKonjugiert(q_vi),GravityQuat(2:4));
        aINew   = aI(count_I,:)'+GravityQuat(2:4);
        aINew   = QuatVekRotate(q_vi,aINew); aINew = aINew(2:4);
        wINew   = QuatVekRotate(q_vi,wI(count_I,:)'); wINew = wINew(2:4);
        %wINew   = QuatVekRotate(q_vi,[-0.5;0;0]); wINew = wINew(2:4);
        %wINew   = QuatVekRotate(QuatKonjugiert(xk1(10:13)),wINew); wINew = wINew(2:4);        
        count_I = count_I +1;
        dt_I    = 1;  %Outputs from the inertial sensors are measured
        sampled_outputs=[1:6];
        yk_delta=[aINew;wINew];
        AINew=[AINew;aINew'];
        WINew=[WINew;wINew'];
    else
        count_I=count_I+1;
        dt_I  = 0;  %Outputs from the inertial sensors are not measured
        AINew=[AINew;aINew'];
        WINew=[WINew;wINew'];
    end
    else
        dt_I = 0; %Outputs from the inertial sensors are not measured
    end
    else
        dt_I=0; %Outputs from the inertial sensors are not measured
    end
    
    if ((count_V<=length(Vision(:,1)))&&((sensor==0)||(sensor==1)))
        if (rem(time,cdt)==0)
            if (Vision(count_V,1)==1)
                pVNew   = pV(count_V,:)';
                qVNew   = qV(count_V,:)';
                count_V = count_V +1;
                dt_V    = 1;
                sampled_outputs=[sampled_outputs 7:13];
                yk_delta=[yk_delta;pVNew;qVNew];
            else
                count_V=count_V+1;
                dt_V  = 0; %Outputs from the vision sensors are not measured
            end
        else
            dt_V=0; %Outputs from the vision sensors are not measured
        end
    else
        dt_V=0; %Outputs from the vision sensors are not measured
    end
    
    %Prediction of the state
    %Cartesian states. fk1_cart is computed out of the loop
    xkk1_cart=fk1_cart*xk1(1:9);
    
    %Add cross product between omega and acceleration
    xkk1_cart(7:9)=xkk1_cart(7:9)+cross(xk1(14:16),xk1(7:9))*T;
    
    %Rotation states
    q0=xk1(10);
    q1=xk1(11);
    q2=xk1(12);
    q3=xk1(13);
    w1=xk1(14);
    w2=xk1(15);
    w3=xk1(16);
    wnorm=norm(xk1(14:16));

    Qk1 = [q0;q1;q2;q3];
    Wk1 = [w1; w2; w3];

    % Linearised Transfer Function for the orientation
    [Fqkk1, Fwkk1,Gqkk1,Gwkk1] = GeneratePartialDeriv(Qk1, Wk1, T);
    
    
    if wnorm>=1e-5
        cw=cos(wnorm*T/2);
        sw=sin(wnorm*T/2);
        xkk1_rot=[q0*cw+(1/wnorm)*(-q1*w1-q2*w2-q3*w3)*sw
                q1*cw+(1/wnorm)*(q0*w1+q3*w2-q2*w3)*sw
                q2*cw+(1/wnorm)*(-q3*w1+q0*w2+q1*w3)*sw
                q3*cw+(1/wnorm)*(q2*w1-q1*w2+q0*w3)*sw
                w1
                w2
                w3];
    else
            xkk1_rot=[q0;q1;q2;q3;w1;w2;w3];
    end
    
    xkk1_bias=xk1(17:19);
    xkk1=[xkk1_cart;xkk1_rot;xkk1_bias];
   
    %Dynamic linearised model
        Fk1=[fk1_cart zeros(9,10)
             zeros(4,9) Fqkk1 Fwkk1 zeros(4,3)
            zeros(3,13) eye(3,3)    zeros(3,3)
            zeros(3,16) eye(3)];
        %Partial derivates of cartesian accelerations with respect to
        %accelerations and angular velocities
        Fakk1=[0 -xk1(16) xk1(15);
               xk1(16) 0 -xk1(14);
               -xk1(15) xk1(14) 0]*T;
        Fawkk1=[0 xk1(9) -xk1(8);
               -xk1(9) 0 xk1(7);
               xk1(8) -xk1(7) 0]*T;
%         Fakk1=zeros(3);
%         Fawkk1=zeros(3);
        Fk1(7:9,7:9)=Fk1(7:9,7:9)+Fakk1;
        Fk1(7:9,14:16)=Fawkk1;
        Gakk1=[0 xk1(6) -xk1(5);
               -xk1(6) 0 xk1(4);
               xk1(5) -xk1(4) 0]*T;
           
%         Gakk1=zeros(3);
                  
        Gk1=[gk1_cart [zeros(6,3);Gakk1] zeros(9,3);
            zeros(4,3) Gqkk1 zeros(4,3);
            zeros(3)   Gwkk1 zeros(3);
            zeros(3)   zeros(3) eye(3)]; 
    %Covariance matrix prediction
    Pkk1=Fk1*Pk1*Fk1'+Gk1*Q*Gk1';
    
    %Kalman filter
    if (isempty(yk_delta)~=1)
        R_delta=R(sampled_outputs,sampled_outputs);
        %Hk is computed out of the loop
        Hk_delta=Hk(sampled_outputs,:);
        try
         hk_delta=Hk_delta*xkk1;
        catch
            size(Hk_delta)
            size(xkk1_cart)
            disp(xkk1_rot)
            size(xkk1_bias)
            size(xkk1)
        end
        Sk_delta=(Hk_delta*Pkk1*Hk_delta'+R_delta);
        Kk_delta=Pkk1*Hk_delta'*inv(Sk_delta);
        %rho = (yk_delta-hk_delta);
        xk=xkk1+Kk_delta*(yk_delta-hk_delta);
        Pk=Pkk1-Kk_delta*Hk_delta*Pkk1;
    else
        xk=xkk1;
        Pk=Pkk1;
    end
   
    %Normalize Quaternions
    qnorm = norm(xk(10:13));
    % normalisation of the quaternion
    xk(10:13) = xk(10:13)/qnorm;
        
    %Prepare for the next iteration
    Pk1=Pk;
    xk1=xk;
    X=[X;xk'];
    TIME=[TIME;time];
    %if length(rho)==6
    %    RHO=[RHO;[rho;zeros(7,1)]'];
    %elseif length(rho)== 7
    %    RHO=[RHO;[zeros(6,1);rho]'];
    %else
    %    RHO=[RHO;rho'];
    %end

end
%% results
IMU_pose     = pgiIMURec;
ground_truth = ground_truth_position;
VO_pose      = r_pose;
VIO_pose     = X(:,1:3);

%% Ground Truth and the prediction plot
figure
scatter(-ground_truth(2,:), ground_truth(1,:), 'g', 'filled');
hold on
scatter(-IMU_pose(2, :), IMU_pose(1, :), 'r', 'filled');
hold on
scatter( - VO_pose(:, 1), VO_pose(:, 3), 'b', 'filled');
hold on
scatter(-VIO_pose(1:106,1), VIO_pose(1:106,3), 'm');
hold on
legend('Ground Truth Pose', 'IMU Pose', 'Stereo VO Pose', 'VIO Pose')

%% Error between the Ground Truth and the prediction
figure
IMU_error =[-ground_truth(2,:) + IMU_pose(2, :); ground_truth(1,:)- IMU_pose(1, :)];
VO_error  =[-ground_truth(2,1:106)' + VO_pose(1:106, 1) ground_truth(1,1:106)' - VO_pose(1:106, 3)];
VIO_error = [-ground_truth(2,1:106)' + VIO_pose(1:106,1) ground_truth(1,1:106)' - VIO_pose(1:106, 3)];

plot(1:106, IMU_error(2,1:106), 'r');
hold on
plot(1:106, VO_error(:, 2), 'b');
hold on
plot(1:106, VIO_error(:, 2), 'm');
hold on

plot(1:106, IMU_error(1,1:106), '-.r');
hold on
plot(1:106, VO_error(:, 1), '-.b');
hold on
plot(1:106, VIO_error(:, 1), '-.m');
hold on

legend('IMU Error Y', 'VO error Y', 'VIO error Y', 'IMU Error X', 'VO error X', 'VIO error X')
%% please read the example VI fusion code, and implement your ideas!!! 
%% The code is the demo of paper: https://journals.sagepub.com/doi/pdf/10.1177/0278364907079283 
%% Fast Ego-motion Estimation with Multi-rate Fusion of Inertial and Vision

