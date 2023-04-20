% the dataset can be downloaded from:
% http://www.cvlibs.net/datasets/kitti/raw_data.php
% pleas select the [synced data] version

BaseDir = 'D:\CCNY\mobile robotics\2011_09_26_drive_0001_sync\2011_09_26\2011_09_26_drive_0001_sync';
calibDir = 'calibration_filess\';

%set image start and end index here
imInit      = 0;   % initial image number, 0-base index
M           = 107; % number of image

% init state variance
var_pgi  = 10^-2; var_thgi  = 10^-6; var_vgi  = (10)^0;
var_ba   = 10^-2; var_bg    = 10^-2;
var_pgi1 = 10^-2; var_thgi1 = 10^-6;
var_pgi2 = 10^-2; var_thgi2 = 10^-6;

% process noise
na_var   = 2*10^-1;  % acc noise (m/s^2)
ng_var   = 5*10^-4;	 % gyro noise (rad/s)
nba_var  = 10^-2;    % acc bias noise (m/s^2)
nbg_var  = 10^-5;    % gyro bias noise (rad/s)

% measure noise of camera
camera_var_pgi_x  = 10^-2; 
camera_var_pgi_y  = 15^-2;
camera_var_pgi_z  = 12^-2; 
camera_var_thgi_pitch  = 14^-6;
camera_var_thgi_yaw  = 14^-6;
camera_var_thgi_roll  = 10^-6;
% Path to the directories containing images
data_params.path1 = 'D:\CCNY\mobile robotics\2011_09_26_drive_0001_sync\2011_09_26\2011_09_26_drive_0001_sync\image_00\data\';
data_params.path2 = 'D:\CCNY\mobile robotics\2011_09_26_drive_0001_sync\2011_09_26\2011_09_26_drive_0001_sync\image_01\data\';



% Path to groundtruth poses. Set flag to 1 to plot groundtruth as well
data_params.gt_file = 'all_pose.mat';
data_params.show_gt_flag = 1;

% Use parallel threads (requires Parallel Processing Toolbox)
% !! TO-DO: fix parfor and for loops for this functionality!
data_params.use_multithreads = 1;                % 0: disabled, 1: enabled

%% Read the calibration file to find parameters of the cameras
% !! TO-DO: Read from the calib_file instead

% calibration parameters for sequence 2010_03_09_drive_0000
cam_params.fx = 7.188560000000e+02;               % focal length (u-coordinate) in pixels
cam_params.cx = 6.071928000000e+02;               % principal point (u-coordinate) in pixels
cam_params.fy = 7.188560000000e+02;               % focal length (v-coordinate) in pixels
cam_params.cy = 1.852157000000e+02;               % principal point (v-coordinate) in pixels
% cam_params.fx = 9.8952670000000e+02;               % focal length (u-coordinate) in pixels
% cam_params.cx = 7.02000000e+02;               % principal point (u-coordinate) in pixels
% cam_params.fy = 9.8783860000000e+02;               % focal length (v-coordinate) in pixels
% cam_params.cy = 2.45559000000e+02;               % principal point (v-coordinate) in pixels
cam_params.base = 3.861448000000e+02;             % baseline in meters (absolute value)

%% Parameters for Feature Extraction
vo_params.feature.nms_n = 8;                      % non-max-suppression: min. distance between maxima (in pixels)
vo_params.feature.nms_tau = 50;                   % non-max-suppression: interest point peakiness threshold
vo_params.feature.margin = 21;                    % leaving margin for safety while computing features ( >= 25)

%% Parameters for Feature Matching
vo_params.matcher.match_binsize = 50;             % matching bin width/height (affects efficiency only)
vo_params.matcher.match_radius = 200;             % matching radius (du/dv in pixels)
vo_params.matcher.match_disp_tolerance = 1;       % dx tolerance for stereo matches (in pixels)
vo_params.matcher.match_ncc_window = 21;          % window size of the patch for normalized cross-correlation
vo_params.matcher.match_ncc_tolerance = 0.3;      % threshold for normalized cross-correlation
% !! TO-DO: add subpixel-refinement using parabolic fitting
vo_params.matcher.refinement = 2;                 % refinement (0=none,1=pixel,2=subpixel)

%% Paramters for Feature Selection using bucketing
vo_params.bucketing.max_features = 1;             % maximal number of features per bucket
vo_params.bucketing.bucket_width = 50;            % width of bucket
vo_params.bucketing.bucket_height = 50;           % height of bucket
% !! TO-DO: add feature selection based on feature tracking
vo_params.bucketing.age_threshold = 10;           % age threshold while feature selection

%% Paramters for motion estimation
vo_params.estim.ransac_iters = 200;              % number of RANSAC iterations
vo_params.estim.inlier_threshold = 2.0;          % fundamental matrix inlier threshold
vo_params.estim.reweighing = 1;                  % lower border weights (more robust to calibration errors)
