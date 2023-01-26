%% five_link_walking_sim
% 2022.11.11 - Melih Pelit
% Blind underactuated walking based on optimized trajectory on rough
% terrain

clear all
close all
addpath("..\") % add above directory to path
%% Reference OpenOCL traj

load('..\OpenOCLTraj\BA_landing_traj_v12022-07-06-17-15'); % loads the landing_traj variable
ocl_traj = landing_traj.ocl_traj;
%% uneven ground input

terrain_name = '..\terrain data\unevenground_v3_2.mat'; % single seed
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
 % TODO

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

filename = sprintf('BA_param_result_%s_terrain_%s.mat', date_str, terrain_name(31:end-4));
subfolder = 'BA_test_results';
save(fullfile(subfolder,filename),'BA_test_result')

%% 
Tf = 10;

load_system('model_5LinkWalking_NODS')

tic

% creating the search list
delta_search = 0:delta_increment:0.2; % [m]
search_list_length = length(r_search)*length(k_bar_ba_search)*length(delta_search);
[r_search_tmp, k_bar_ba_search_tmp, delta_search_tmp] = ndgrid(r_search, k_bar_ba_search, delta_search);
search_list = [r_search_tmp(:), k_bar_ba_search_tmp(:), K_p*ones(search_list_length, 1), K_d*ones(search_list_length, 1), delta_search_tmp(:), NaN(search_list_length, 1)];
% clear r_search_tmp k_bar_ba_search_tmp delta_search_tmp search_list_length % TODO

search_size = 8*8; % making it a multiple of 8 since this PC has 8 cores

iter_cur = 1;
tic
while(1)

    % append to the current search list
    search_list_cur = search_list(iter_cur:min((iter_cur + search_size - 1), length(search_list)), :);

    % do the search
    search_list_cur = run_walking_simulation_parallel(landing_traj, uneven_terrain, params, Tf, search_list_cur);
    search_list(iter_cur:min((iter_cur + search_size - 1), length(search_list)), :) = search_list_cur;

    % remove unnecessary queries from the search_list
    removed_entries = 0;
    for i = 1:length(search_list_cur)
        if search_list_cur(i, 6) == 0
            search_list( ...
                search_list(:,1) == search_list_cur(i, 1) & ...
                search_list(:,2) == search_list_cur(i, 2) & ...
                isnan(search_list(:,6)), :) = [];
            removed_entries = removed_entries + 1;
        end
    end
    display(removed_entries)

    % save the results
    elapsed_time = toc;
    fprintf("Elapsed time = %f mins \n", elapsed_time/60)
    BA_test_result.elapsed_time = elapsed_time;
    BA_test_result.search_list = search_list;
    save(fullfile(subfolder,filename), 'BA_test_result')

    % end the loop if search is complete
    if isnan(search_list(end, 6)) ~= true
        break
    end

    % increase the iteration
    iter_cur = iter_cur + search_size;
    remaining_iterations = length(search_list) - iter_cur + 1;
    fprintf("Remaining iteration number = %d \n", remaining_iterations)
end

%% Extract the PASS from search_list

for i = 1:length(r_search)
    for j = 1:length(k_bar_ba_search)
        fail_points = find(search_list(:,1) == r_search(i)& search_list(:,2) == k_bar_ba_search(j) & search_list(:,6) == 0);
        PASS(i,j) = search_list(fail_points(1), 5) - delta_increment;
    end
end

% save the results
elapsed_time = toc;
fprintf("Elapsed final time = %f mins \n", elapsed_time/60)
BA_test_result.elapsed_time = elapsed_time;
BA_test_result.PASS = PASS;
save(fullfile(subfolder,filename), 'BA_test_result')

% PASS_w_header(1, 1) = NaN;
% PASS_w_header(2:(1+length(r_search)), 1) = r_search';
% PASS_w_header(1, 2:(1+length(k_bar_ba_search))) = k_bar_ba_search;
% PASS_w_header(2:(1+length(r_search)), 2:(1+length(k_bar_ba_search))) = PASS;