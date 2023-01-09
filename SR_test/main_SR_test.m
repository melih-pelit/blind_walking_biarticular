%% five_link_walking_sim
% 2023.01.09 - Melih Pelit
% Searching SR values for different terrain heights for different BA parameters

clear all
close all
addpath("C:\Matlab Workspace\Biarticular Robustness\Controlled System\BiarticularModelRepo")


load('..\OpenOCLTraj\BA_landing_traj_v12022-07-06-17-15'); % loads the landing_traj variable
ocl_traj = landing_traj.ocl_traj;
terrain_name = '..\terrain data\unevenground_v3_1.mat'; % single seed
load(terrain_name)

Tf = 10;
K_p = 9700;
K_d = 220;

% BA params
r_search = 1:0.2:5;
k_bar_ba_search = 0:5:200;

% r_search = 1:0.2:1.4;
% k_bar_ba_search = 0:5:10;

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

% constant biarticular muscle parameters
params.r_k = 0.02; % [m] we choose this value and rest of the parameters are defined according to it and r, k_bar_ba
params.phi_h0 = pi; % [rad] free angle of springs at hip
params.phi_k0 = pi; % [rad] free angle of springs at knee


%%
% Reference OpenOCL traj
ocl_traj = landing_traj.ocl_traj;
alpha_ref = pi - (2*ocl_traj.simout(:,1)+ocl_traj.simout(:,2))/2;
joint_angles_ref = [ocl_traj.time, ocl_traj.simout, alpha_ref];
alpha_ref_landing = pi - (2*landing_traj.simout(:,1)+landing_traj.simout(:,2))/2;
landing_ref = [landing_traj.time, landing_traj.simout, alpha_ref_landing];

% uneven ground input
uneven_terrain.y_g_curr = 0.001 * uneven_terrain.y_g_seed; % set this temporarily for BUS setting
uneven_terrain_bus_info = Simulink.Bus.createObject(uneven_terrain);
uneven_terrain_bus = evalin('base', uneven_terrain_bus_info.busName);

%
sample_time = 0.001;

% Initial conditions
N = 3; % desired starting position of the simulation
q0 =  [ocl_traj.simout(N,1); ocl_traj.simout(N,2); ocl_traj.simout(N,3); ocl_traj.simout(N,4); ocl_traj.simout(N,5)];
dq0 = [ocl_traj.simout(N,6); ocl_traj.simout(N,7); ocl_traj.simout(N,8); ocl_traj.simout(N,9); ocl_traj.simout(N,10)];

Initial_state = [q0;dq0];

% init flag
init_t_mode_change = -ocl_traj.time(N); % if I start the simulation for the mid point of the SS phase, i need to set this so that controller starts from the correct spot

% open the simulink model
load_system('model_5LinkWalking_NODS')

fprintf ('-----KP=%.4f, KD=%.4f----- \n', K_p, K_d);

%% Saving
start_i = 1;
% loading an old result and continuing from its last row

% display("***Loading From an Old Result***")
% load('SR_test_results\5LinkWalkingOpenOCL2022-11-20-12-37.mat')
% SR = SR_test_result.SR;
% size_SR = size(SR);
% start_i = size_SR(1);
% date_str = SR_test_result.date_str;

% recording
SR_test_result.ocl_traj_name = ocl_traj.date_str;
SR_test_result.landing_traj_name = landing_traj.date_str;
SR_test_result.gains_KP = K_p;
SR_test_result.gains_KD = K_d;
SR_test_result.terrain_name = terrain_name;
SR_test_result.params = params;
SR_test_result.r_search = r_search;
SR_test_result.k_bar_ba_search = k_bar_ba_search;
date_str = datestr(now,'yyyy-mm-dd-HH-MM');
SR_test_result.date_str = date_str;

filename = sprintf('SR_test_results%s.mat', date_str);
subfolder = 'SR_test_results';
save(fullfile(subfolder,filename),'SR_test_result')

%%  parallel simulation
delta = 0.025;

for i=start_i:length(r_search)
    fprintf ('-----r=%.4f----- \n', r_search(i))
    k = 1;
    for j=1:length(k_bar_ba_search)

        % biarticular muscle parameters

        params.r = r_search(i); % dimensionless lever arm ratio (found from optimizing wrt SR) r = r_h / r_k
        params.k_bar_ba = k_bar_ba_search(j); % [Nm] k_bar_ba = k_ba * r_k^2
        params.r_h = params.r*params.r_k; % [m]
        params.k_ba = params.k_bar_ba/(params.r_k^2); % [N/m]

        in(k) = Simulink.SimulationInput('model_5LinkWalking_NODS');
        x_g = uneven_terrain.x_g;
        y_g = delta.*uneven_terrain.y_g_seed;
        y_sw_init = interp1(x_g, y_g, 0);
        in(k) = in(k).setVariable('init_flag', ...
            [-1; 0; ocl_traj.x_sw(1); init_t_mode_change; alpha_ref(1); 0; y_sw_init]);
        in(k) = in(k).setVariable('uneven_terrain.y_g_curr', y_g);
        in(k) = in(k).setVariable('param', ...
            [params.m1; params.m2; params.m5; params.l1; params.l2; params.l5; params.g; ...
            params.I1; params.I2; params.I5; params.r_k; params.r_h; params.k_ba; params.phi_h0; params.phi_k0]);
        in(k) = in(k).setVariable('gains', [K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d]);
        in(k) = in(k).setVariable('Tf', Tf);
        in(k) = in(k).setVariable('sample_time', sample_time);
        in(k) = in(k).setVariable('joint_angles_ref', joint_angles_ref);
        in(k) = in(k).setVariable('landing_ref', landing_ref);
        in(k) = in(k).setVariable('Initial_state', Initial_state);
        in(k) = in(k).setVariable('q0', q0);
        in(k) = in(k).setVariable('dq0', dq0);
        in(k) = in(k).setVariable('uneven_terrain_bus', uneven_terrain_bus);

        k = k + 1;
    end

    out = parsim(in, ...
        'ShowSimulationManager', 'on', ...
        'TransferBaseWorkspaceVariables','on');
    
    % calculate SR values
    for j=1:length(k_bar_ba_search)
        if out(j).time < Tf
            specific_resistance(i,j) = NaN;
            avg_vel(i,j) = NaN;
        else
            params.r = r_search(i); % dimensionless lever arm ratio (found from optimizing wrt SR) r = r_h / r_k
            params.k_bar_ba = k_bar_ba_search(j); % [Nm] k_bar_ba = k_ba * r_k^2
            params.r_k = 0.02; % [m] we choose this value and rest of the parameters are defined according to it and r, k_bar_ba
            params.r_h = params.r*params.r_k; % [m]
            params.k_ba = params.k_bar_ba/(params.r_k^2); % [N/m]
            [specific_resistance(i,j), avg_vel(i,j)] = calc_sr( ...
                out(j).simout, out(j).inputTorque, out(j).flag, params, out(j).time);
        end
    end

    % save the results (after each row such that less progress is lost in
    % the event of a crash)
    SR_test_result.SR = specific_resistance;
    SR_test_result.avg_vel = avg_vel;
    save(fullfile(subfolder,filename),'SR_test_result')

    clear in
end

%% Animation
i = 1;
j = 1;
f_animation = 0;
if f_animation == 1
    f_video = 0; % flag for recording video
    f_pause = 0;
    frame_leap = 10;
    param =  [params.m1; params.m2; params.m5; params.l1; params.l2; params.l5; params.g; ...
            params.I1; params.I2; params.I5; params.r_k; r_search(i)*params.r_k; k_bar_ba_search(j)/(params.r_k^2); params.phi_h0; params.phi_k0];
    k = delta/0.001 + 1;
    uneven_terrain.deltaY_inc = 0.001;
    uneven_terrain.y_g = delta.*uneven_terrain.y_g_seed;
    animation(f_video, out(i,j).simout, param, f_pause, frame_leap, out(i,j).flag, uneven_terrain, out(i,j).time, k)
pause
end
