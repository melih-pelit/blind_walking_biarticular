%% five_link_walking_sim
% 2023.07.27 - Melih Pelit
% Blind underactuated walking based on optimized trajectory on soft ground 
% - rough terrain 

clear all
close all

addpath("..\")

%% Reference OpenOCL traj

load('..\OpenOCLTraj\BA_landing_traj_v12022-07-06-17-15'); % loads the landing_traj variable
ocl_traj = landing_traj.ocl_traj;
%% uneven ground input

terrain_name = '..\terrain data\unevenground_v3_1.mat'; % single seed
load(terrain_name)

% try catch is because I previously didn't have "seed" fields for some terrain
try
    uneven_terrain.y_g_seed;
catch
    uneven_terrain.y_g_seed= 10 * uneven_terrain.y_g(101,:); % TODO
end
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

% biarticular muscle parameters

% params.r = ocl_traj.biart.r; % dimensionless lever arm ratio (found from optimizing wrt SR) r = r_h / r_k
% params.k_bar_ba = ocl_traj.biart.k_bi; % [Nm] k_bar_ba = k_ba * r_k^2

params.r = 1.6; % dimensionless lever arm ratio (found from optimizing wrt SR) r = r_h / r_k
params.k_bar_ba = 200; % [Nm] k_bar_ba = k_ba * r_k^2

params.r_k = 0.02; % [m] we choose this value and rest of the parameters are defined according to it and r, k_bar_ba
params.r_h = params.r*params.r_k; % [m]
% params.k_ba = params.k_bar_ba/(params.r_k^2); % [N/m]
params.k_ba = 0; % [N/m]
params.phi_h0 = pi; % [rad] free angle of springs at hip
params.phi_k0 = pi; % [rad] free angle of springs at knee

% ground parameters
params.k_ground = 90000;
params.d_ground = 10;

%% 

Tf = 10;
K_p = 9700;
K_d = 220;
% K_p = 2700;
% K_d = 80;
gains = [K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d];
open_system('model_5LinkWalk_NODS_soft_ground')
% load_system('model_5LinkWalk_NODS_soft_ground')

k = 1; % delta = 0 [m]

%% Run walking simulation

%%
ocl_traj = landing_traj.ocl_traj;

alpha_ref = pi - (2*ocl_traj.simout(:,1)+ocl_traj.simout(:,2))/2;
[step_alpha_ref, step_inputs_ref] = calc_step_input_ref(ocl_traj);

% inputs_ref = ocl_traj.inputTorques;
inputs_ref = [step_alpha_ref, step_inputs_ref];
joint_angles_ref = [ocl_traj.time, ocl_traj.simout, alpha_ref];

alpha_ref_landing = pi - (2*landing_traj.simout(:,1)+landing_traj.simout(:,2))/2;
landing_ref = [landing_traj.time, landing_traj.simout, alpha_ref_landing];

%% Set the uneven_terrain BUS
uneven_terrain_bus_info = Simulink.Bus.createObject(uneven_terrain);
uneven_terrain_bus = evalin('base', uneven_terrain_bus_info.busName);

%%
sample_time = 0.001;

%%
param = [
    params.m1; 
    params.m2; 
    params.m5; 
    params.l1; 
    params.l2; 
    params.l5; 
    params.g; 
    params.I1; 
    params.I2; 
    params.I5; 
    params.r_k; 
    params.r_h; 
    params.k_ba; 
    params.phi_h0; 
    params.phi_k0;
    params.k_ground;
    params.d_ground];
%%
N = 3; % desired starting position of the simulation
% TODO: initial state stance foot can start on rough terrain
q0 =  [0; 0; ocl_traj.simout(N,1); ocl_traj.simout(N,2); ocl_traj.simout(N,3); ocl_traj.simout(N,4); ocl_traj.simout(N,5)];
dq0 = [0; 0; ocl_traj.simout(N,6); ocl_traj.simout(N,7); ocl_traj.simout(N,8); ocl_traj.simout(N,9); ocl_traj.simout(N,10)];
X = [q0; dq0];

Initial_state = [q0;dq0];
init_t_mode_change = -ocl_traj.time(N); % if I start the simulation for the mid point of the SS phase, i need to set this so that controller starts from the correct spot

delta = (k - 1)*0.001;
uneven_terrain.y_g_curr = delta * uneven_terrain.y_g_seed;

x_g = uneven_terrain.track_start:uneven_terrain.dist_step_size:uneven_terrain.track_end;
y_g = uneven_terrain.y_g_curr;
y_g_curr = interp1(x_g, y_g, 0);
init_flag = [-1; 0; ocl_traj.x_sw(1); init_t_mode_change; alpha_ref(1); 0; y_g_curr]; % change the stance foot to start on the uneven ground

%%
options = simset('SrcWorkspace','current');
sim('model_5LinkWalk_NODS_soft_ground', [], options)

simout_non_floating = [simout(:, 3:7), simout(:, 10:14)];

%% GRF plots
figure()
plot(time, lambda_SS)
xlabel("Time [sec]")
ylabel("\lambda_{ss} [N]")
legend("x", "y")
xlim([0,2])
%% Trajectory Tracking Plots

f_print = 1;
time_start = 5;
time_end = 8;
trackingPlots(simout_non_floating, inputTorque, des_theta_alpha, param, flag, time, f_print, time_start, time_end) % for unnamed conference 2023

%% y_st traj
figure()
plot(time, simout(:,2))
xlim([time_start, time_end])
xlabel("Time [sec]")
ylabel("y_{st} [m]")
grid on

if f_print == 1
    set(gcf, 'color', 'none');
    set(gca, 'color', 'none');
    print(gcf,'y_st_trajectory.png','-dpng','-r300');
end


%% Animation
f_animation = 1;
if f_animation == 1
    f_video = 1; % flag for recording video
    f_pause = 0;
    frame_leap = 5;
    deltaY = 0.001;
    start_end_time = [5, 10];
    animation_floating(f_video, simout, param, f_pause, frame_leap, flag, uneven_terrain, time, k, deltaY, start_end_time)
    pause
end