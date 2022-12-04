%% five_link_walking_sim
% 2022.11.11 - Melih Pelit
% Blind underactuated walking based on optimized trajectory on rough
% terrain

clear all
close all
addpath("C:\Matlab Workspace\Biarticular Robustness\Controlled System\BiarticularModelRepo")
%% Reference OpenOCL traj

load('..\OpenOCLTraj\BA_landing_traj_v12022-07-06-17-15'); % loads the landing_traj variable
ocl_traj = landing_traj.ocl_traj;
%% uneven ground input

terrain_name = '..\terrain data\unevenground_0_1_v2_13.mat'; % single seed
load(terrain_name)

% try catch is because I previously didn't have "seed" fields for some terrain
try
    uneven_terrain.seed
catch
    uneven_terrain.seed = 10 * uneven_terrain.y_g(101,:); % TODO
end
uneven_terrain.y_g_curr = 0.001 * uneven_terrain.seed; % set this temporarily for BUS setting
%% Model parameters
params.m1 = 5;
params.m2 = 5;

params.m5 = 60; % 5,1,1
params.m3 = params.m2;
params.m4 = params.m1;

params.l1 = 0.48;
params.l2 = 0.48;
params.l3 = params.l2;
params.l4 = params.l1;
params.l5 = 0.48;

params.g = 9.81;
params.I1 = 0.0960;
params.I2 = 0.0960;
params.I3 = params.I1;
params.I4 = params.I2;
params.I5 = 1.1520;

% biarticular muscle parameters

params.r = ocl_traj.biart.r; % dimensionless lever arm ratio (found from optimizing wrt SR) r = r_h / r_k
params.k_bar_ba = ocl_traj.biart.k_bi; % [Nm] k_bar_ba = k_ba * r_k^2

params.r_k = 0.02; % [m] we choose this value and rest of the parameters are defined according to it and r, k_bar_ba
params.r_h = params.r*params.r_k; % [m]
params.k_ba = params.k_bar_ba/(params.r_k^2); % [N/m]
% params.k_ba = 0; % [N/m]
params.phi_h0 = pi; % [rad] free angle of springs at hip
params.phi_k0 = pi; % [rad] free angle of springs at knee

param = [params.m1; params.m2; params.m5; params.l1; params.l2; params.l5; params.g; params.I1; params.I2; params.I5; params.r_k; params.r_h; params.k_ba; params.phi_h0; params.phi_k0];

%% gain test and recording
gains_KP = 100:200:20000;
gains_KD = 0:20:500;

start_i = 1;

% loading an old result and continuing from its last row
display("***Loading From an Old Result***")
load('gain_test_results\5LinkWalkingOpenOCL2022-11-20-12-37.mat')
PASS = gain_test_result.PASS;
size_PASS = size(PASS);
start_i = size_PASS(1);
date_str = ocl_traj.date_str;

% recording
% gain_test_result.ocl_traj_name = ocl_traj.date_str;
% gain_test_result.landing_traj_name = landing_traj.date_str;
% gain_test_result.gains_KP = gains_KP;
% gain_test_result.gains_KD = gains_KD;
% gain_test_result.terrain_name = terrain_name;
% gain_test_result.params = params;
% date_str = datestr(now,'yyyy-mm-dd-HH-MM');

filename = sprintf('5LinkWalkingOpenOCL%s.mat', date_str);
subfolder = 'gain_test_results';
save(fullfile(subfolder,filename),'gain_test_result')

%% 

Tf = 10;
open_system('model_5LinkWalking_NODS')

for i=start_i:length(gains_KP)
    for j=1:length(gains_KD)
        K_p = gains_KP(i);
        K_d = gains_KD(j);
        gains = [K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d];
        fprintf ('-----KP=%.4f, KD=%.4f----- \n', K_p, K_d);

        % Search for the failing point (delta bar) by skipping  and searching in minus direction
        [simout, inputTorque, des_theta_alpha, flag, time, PASS(i,j), k] = search_delta_bar(landing_traj, uneven_terrain, params, Tf, gains);
        gain_test_result.PASS = PASS;
        fprintf ('delta_bar = %.4f [m] \n', PASS(i,j))
        save(fullfile(subfolder,filename),'gain_test_result')
    end
end
%% Trajectory Tracking Plots

f_print = 0;
time_start = 0;
time_end = 2;
trackingPlots(simout, inputTorque, des_theta_alpha, param, flag, time, f_print, time_start, time_end) % for unnamed conference 2023

%% Animation
f_animation = 1;
if f_animation == 1
    f_video = 0; % flag for recording video
    f_pause = 0;
    frame_leap = 5;
    animation(f_video, simout, param, f_pause, frame_leap, flag, uneven_terrain, time, k)
pause
end