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
KP_search = 100:200:20000;
KD_search = 0:20:500;

% KP_search = 100:50:500;
% KD_search = 0:5:50;

r = 1;
k_bar_ba = 0;

start_i = 1;
delta_increment = 0.001;
subfolder = 'gain_test_results';

% loading an old result and continuing from its last row
% display("***Loading From an Old Result***")
% TODO

%%
Tf = 10;

load_system('model_5LinkWalking_NODS')

tic

% Search Parameters
delta_search = 0:delta_increment:0.2; % [m]
search_list_length = length(KP_search)*length(KD_search)*length(delta_search);
[KP_search_tmp, KD_search_tmp, delta_search_tmp] = ndgrid(KP_search, KD_search, delta_search);
% clear r_search_tmp k_bar_ba_search_tmp delta_search_tmp search_list_length % TODO

search_size = 8*8; % making it a multiple of 8 since this PC has 8 cores
terrain_search_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
% terrain_search_list = [1, 2];

for j = 1:length(terrain_search_list)
    % iteratate through terrains in terrain_search_list

    % clear data from the prev. search
    clear search_list gain_result PASS uneven_terrain

    % load the uneven ground
    terrain_name =num2str("..\terrain data\unevenground_v3_" + num2str(terrain_search_list(j)) + ".mat"); % single seed
    load(terrain_name)

    % initialize the recording
    [gain_result, filename] = initialize_recording(ocl_traj, landing_traj, r, k_bar_ba, terrain_name, params, KP_search, KD_search, delta_increment, subfolder);
    fprintf("Current filename is %s \n", filename)
    
    % create the search_list
    search_list = [r*ones(search_list_length, 1), k_bar_ba*ones(search_list_length, 1), KP_search_tmp(:), KD_search_tmp(:), delta_search_tmp(:), NaN(search_list_length, 1)];

    uneven_terrain.y_g_curr = 0.001 * uneven_terrain.y_g_seed; % set this temporarily for BUS setting

    % start the search
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
                removed_entries = removed_entries + length(search_list( ...
                    search_list(:,3) == search_list_cur(i, 3) & ...
                    search_list(:,4) == search_list_cur(i, 4) & ...
                    isnan(search_list(:,6)), :));
                search_list( ...
                    search_list(:,3) == search_list_cur(i, 3) & ...
                    search_list(:,4) == search_list_cur(i, 4) & ...
                    isnan(search_list(:,6)), :) = [];
            end
        end
        display(removed_entries)

        % save the results
        elapsed_time = toc;
        fprintf("Elapsed time = %f mins \n", elapsed_time/60)
        gain_result.elapsed_time = elapsed_time;
        gain_result.search_list = search_list;
        save(fullfile(subfolder,filename), 'gain_result')

        % end the loop if search is complete
        if isnan(search_list(end, 6)) ~= true
            break
        end

        % increase the iteration
        iter_cur = iter_cur + search_size;
        remaining_iterations = length(search_list) - iter_cur + 1;
        fprintf("Remaining iteration number = %d, current terrain is 3v%d \n", remaining_iterations, terrain_search_list(j))
    end

    %% Extract the PASS from search_list

    for i = 1:length(KP_search)
        for j = 1:length(KD_search)
            fail_points = find(search_list(:,3) == KP_search(i)& search_list(:,4) == KD_search(j) & search_list(:,6) == 0);
            PASS(i,j) = search_list(fail_points(1), 5) - delta_increment;
        end
    end

    % save the results
    elapsed_time = toc;
    fprintf("Elapsed final time = %f mins \n", elapsed_time/60)
    gain_result.elapsed_time = elapsed_time;
    gain_result.PASS = PASS;
    save(fullfile(subfolder,filename), 'gain_result')

end
%%

PASS_w_header(1, 1) = NaN;
PASS_w_header(2:(1+length(KP_search)), 1) = KP_search';
PASS_w_header(1, 2:(1+length(KD_search))) = KD_search;
PASS_w_header(2:(1+length(KP_search)), 2:(1+length(KD_search))) = PASS;

%%
function [gain_result, filename] = initialize_recording(ocl_traj, landing_traj, r_search, k_bar_ba_search, terrain_name, params, K_p_search, K_d_search, delta_increment, subfolder)
gain_result.ocl_traj_name = ocl_traj.date_str;
gain_result.landing_traj_name = landing_traj.date_str;
gain_result.r_search = r_search;
gain_result.k_bar_ba_search = k_bar_ba_search;
gain_result.terrain_name = terrain_name;
gain_result.params = params;
gain_result.K_p = K_p_search;
gain_result.K_d = K_d_search;
gain_result.delta_increment = delta_increment;
date_str = datestr(now,'yyyy-mm-dd-HH-MM');
gain_result.date_str = date_str;

filename = sprintf('gain_test_result_%s_terrain_%s.mat', date_str, terrain_name(31:end-4));
save(fullfile(subfolder,filename),'gain_result')
end