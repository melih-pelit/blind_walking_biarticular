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

terrain_name = '..\terrain data\unevenground_v3_1.mat'; % single seed
load(terrain_name)

% try catch is because I previously didn't have "seed" fields for some terrain
uneven_terrain.y_g_curr = 0.001 * uneven_terrain.y_g_seed; % set this temporarily for BUS setting
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

params.r_k = 0.02; % [m] we choose this value and rest of the parameters are defined according to it and r, k_bar_ba
params.phi_h0 = pi; % [rad] free angle of springs at hip
params.phi_k0 = pi; % [rad] free angle of springs at knee

%% BA param test and recording
K_p = 9700;
K_d = 220;
gains = [K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d];

r_search = 1:0.2:5;
k_bar_ba_search = 0:5:200;

% r_search = 1:0.2:1.4;
% k_bar_ba_search = 0:10:50;

start_i = 1;
delta_increment = 0.001;

% loading an old result and continuing from its last row
% display("***Loading From an Old Result***")
% load('gain_test_results\5LinkWalkingOpenOCL2022-11-20-12-37.mat')
% PASS = BA_test_result.PASS;
% size_PASS = size(PASS);
% start_i = size_PASS(1);
% date_str = BA_test_result.date_str;

% recording
BA_test_result.ocl_traj_name = ocl_traj.date_str;
BA_test_result.landing_traj_name = landing_traj.date_str;
BA_test_result.r_search = r_search;
BA_test_result.k_bar_ba_search = k_bar_ba_search;
BA_test_result.terrain_name = terrain_name;
BA_test_result.params = params;
BA_test_result.K_p = K_p;
BA_test_result.K_d = K_d;
BA_test_result.delta_increment = delta_increment;
date_str = datestr(now,'yyyy-mm-dd-HH-MM');
BA_test_result.date_str = date_str;

filename = sprintf('BA_param_result_%s_terrain_%s.mat', date_str, terrain_name(31:33));
subfolder = 'BA_test_results';
save(fullfile(subfolder,filename),'BA_test_result')

%% 
Tf = 10;

load_system('model_5LinkWalking_NODS')

tic
for i=start_i:length(r_search)
    for j=1:length(k_bar_ba_search)
        % biarticular muscle parameters

        params.r = r_search(i); % dimensionless lever arm ratio (found from optimizing wrt SR) r = r_h / r_k
        params.k_bar_ba = k_bar_ba_search(j); % [Nm] k_bar_ba = k_ba * r_k^2

        params.r_h = params.r*params.r_k; % [m]
        params.k_ba = params.k_bar_ba/(params.r_k^2); % [N/m]
        param = [params.m1; params.m2; params.m5; params.l1; params.l2; params.l5; params.g; params.I1; params.I2; params.I5; params.r_k; params.r_h; params.k_ba; params.phi_h0; params.phi_k0];

        fprintf ('-----r=%.4f, k_bar_ba=%.4f----- \n', params.r, params.k_bar_ba);

        % Search for the failing point (delta bar) by skipping  and searching in minus direction
        % skip_amount = 10;
        % [simout, inputTorque, des_theta_alpha, flag, time, PASS(i,j), k] = search_delta_bar(landing_traj, uneven_terrain, params, Tf, gains, skip_amount);
        [PASS(i,j), k] = search_delta_bar_parallel( ...
            landing_traj, uneven_terrain, params, Tf, K_p, K_d, delta_increment);
        BA_test_result.PASS = PASS;
        fprintf ('delta_bar = %.4f [m] \n', PASS(i,j))
        save(fullfile(subfolder,filename),'BA_test_result')
    end
end
elapsed_time = toc;

fprintf("Test Completed Successfully for r = " + num2str(r_search(1)) + ...
    ":" + num2str(r_search(end)) +", k_bar_ba = " + num2str(k_bar_ba_search(1)) + ...
    ":" + num2str(k_bar_ba_search(end))+ "\n")

fprintf("Elapsed time was " + num2str(elapsed_time/60) + " mins \n")