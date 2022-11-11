%% five_link_walking_sim
% 2021.02.03 - Melih Pelit
% Tracking individiual reference joint angles instead of the CoM and swing
% foot trajectories.

tic
clear all
close all

%% Reference OpenOCL traj
%%%% BA trajectory %%%% (before I just use default model trajectory with BA added)

% gain = (7500,100), stable, 0.0400 [m] PASS
load('OpenOCLTraj\BA_landing_traj_v12022-07-06-17-15')
ocl_traj = landing_traj.ocl_traj;

alpha_ref = pi - (2*ocl_traj.simout(:,1)+ocl_traj.simout(:,2))/2;
[step_alpha_ref, step_inputs_ref] = calc_step_input_ref(ocl_traj);

% inputs_ref = ocl_traj.inputTorques;
inputs_ref = [step_alpha_ref, step_inputs_ref];
joint_angles_ref = [ocl_traj.time, ocl_traj.simout, alpha_ref];

alpha_ref_landing = pi - (2*landing_traj.simout(:,1)+landing_traj.simout(:,2))/2;
landing_ref = [landing_traj.time, landing_traj.simout, alpha_ref_landing];

%% uneven ground input
terrain_name = 'terrain data\unevenground_0_1_v2_15.mat'; % single seed

load(terrain_name)

uneven_terrain.y_g_curr = uneven_terrain.y_g(1,:);

uneven_terrain_bus_info = Simulink.Bus.createObject(uneven_terrain);
uneven_terrain_bus = evalin('base', uneven_terrain_bus_info.busName);

%%
sample_time = 0.001;

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

% SLIP Params
L_slip = 1; % [m]
alpha0 = 1.2; % angle at which swing leg touches the ground
k_slip = 15696; % nominal leg stiffness [N/m]
m_slip = 80; % hip mass [kg]

param = [params.m1; params.m2; params.m5; params.l1; params.l2; params.l5; params.g; params.I1; params.I2; params.I5; params.r_k; params.r_h; params.k_ba; params.phi_h0; params.phi_k0];

%% Initial conditions
N = 3; % desired starting position of the simulation
q0 =  [ocl_traj.simout(N,1); ocl_traj.simout(N,2); ocl_traj.simout(N,3); ocl_traj.simout(N,4); ocl_traj.simout(N,5)];
dq0 = [ocl_traj.simout(N,6); ocl_traj.simout(N,7); ocl_traj.simout(N,8); ocl_traj.simout(N,9); ocl_traj.simout(N,10)];
X = [q0; dq0];

Initial_state = [q0;dq0];

%% init flag
init_t_mode_change = -ocl_traj.time(N); % if I start the simulation for the mid point of the SS phase, i need to set this so that controller starts from the correct spot

% init_flag = [-1; 0; ocl_traj.x_sw(1); init_t_mode_change; alpha_ref(1); 0; 0];

%%
Tf = 10;

K_p = 9700;
K_d = 220;

% open the simulink model
open_system('model_5LinkWalking_NODS')

skip_amount = 5; % skip how many 0.001 m to make it faster
flag_break = 0; % to break out of the outer for loop

size_y_g = size(uneven_terrain.y_g);


gains = [K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d];

fprintf ('-----KP=%.4f, KD=%.4f----- \n', K_p, K_d);

% uneven ground test
nt = size(uneven_terrain.y_g);
% for k=1:skip_amount:nt(1)
for k=1:1
    
    if k > size_y_g(1)
        % if k is beyond the generated terrain delta
        PASS(i,j) = k*size_y_g(1);
        gain_test_result.PASS = PASS;
        save(fullfile(subfolder,filename),'gain_test_result')
        break
    end
    
    uneven_terrain.y_g_curr = uneven_terrain.y_g(k,:);
    
    x_g = uneven_terrain.track_start:uneven_terrain.dist_step_size:uneven_terrain.track_end;
    y_g = uneven_terrain.y_g_curr;
    y_g_curr = interp1(x_g, y_g, 0);
    init_flag = [-1; 0; ocl_traj.x_sw(1); init_t_mode_change; alpha_ref(1); 0; y_g_curr]; % change the stance foot to start on the uneven ground
    
    % Run the simulation
    
    sim('model_5LinkWalking_NODS')
    
    terrain_height = (k*uneven_terrain.deltaY_inc - uneven_terrain.deltaY_inc);
    if time(end)<10 && terrain_height ~= 0
        % Searching in minus direction
        fprintf ('deltaY = %.4f [m] FAIL \n', terrain_height)
        
        fprintf('Searching in minus direction \n')
        
        k_prev = k;
        
        for k=(k_prev-1):-1:(k_prev-skip_amount)
            
            uneven_terrain.y_g_curr = uneven_terrain.y_g(k,:);
            
            x_g = uneven_terrain.track_start:uneven_terrain.dist_step_size:uneven_terrain.track_end;
            y_g = uneven_terrain.y_g_curr;
            y_g_curr = interp1(x_g, y_g, 0);
            init_flag = [-1; 0; ocl_traj.x_sw(1); init_t_mode_change; alpha_ref(1); 0; y_g_curr]; % change the stance foot to start on the uneven ground
            
            % Run the simulation
            
            sim('model_5LinkWalking_NODS')
            
            terrain_height = (k*uneven_terrain.deltaY_inc - uneven_terrain.deltaY_inc);
            
            if time(end)<10
                fprintf ('deltaY = %.4f [m] FAIL \n', terrain_height)
                
                if k == k_prev-skip_amount+1
                    % if k_prev is the PASS point
                    fprintf ('deltaY = %.4f [m] PASS \n', ((k_prev-skip_amount)*uneven_terrain.deltaY_inc - uneven_terrain.deltaY_inc))
                    PASS(i,j) = (k_prev-skip_amount)*uneven_terrain.deltaY_inc - uneven_terrain.deltaY_inc;
                    gain_test_result.PASS = PASS;
                    save(fullfile(subfolder,filename),'gain_test_result')
                    flag_break = 1; % to break out of the outer for loop
                    break
                end
                
            else
                fprintf ('deltaY = %.4f [m] PASS \n', terrain_height)
                PASS(i,j) = terrain_height - uneven_terrain.deltaY_inc;
                gain_test_result.PASS = PASS;
                save(fullfile(subfolder,filename),'gain_test_result')
                flag_break = 1; % to break out of the outer for loop
                break
            end
            
        end
        
    elseif time(end)<10 && terrain_height == 0
        % if it fails at delta = 0.000 m (k=1)
        fprintf ('deltaY = %.4f [m] FAIL \n', terrain_height)
        PASS(i,j) = terrain_height - uneven_terrain.deltaY_inc;
        gain_test_result.PASS = PASS;
        save(fullfile(subfolder,filename),'gain_test_result')
        break
    else
        fprintf ('deltaY = %.4f [m] PASS \n', terrain_height)
    end
    
    if flag_break == 1
        flag_break = 0;
        break
    end
end


%% Trajectory Tracking Plots

f_print = 0;
time_start = 0;
time_end = 2;
trackingPlots(simout, inputTorque, des_theta_alpha, param, flag, time, f_print, time_start, time_end) % for unnamed conference 2023
% trackingPlots_tasks(simout, inputTorque, des_theta, param, flag, time)

%% Animation
f_animation = 1;
if f_animation == 1
    f_video = 0; % flag for recording video
    f_pause = 0;
    frame_leap = 5;
    animation(f_video, simout, param, f_pause, frame_leap, flag, uneven_terrain, time, k, sample_time)
pause
end

toc