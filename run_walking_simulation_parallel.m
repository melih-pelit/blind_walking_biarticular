function search_list = run_walking_simulation_parallel(landing_traj, uneven_terrain, params, Tf, search_list)

% Reference OpenOCL traj
ocl_traj = landing_traj.ocl_traj;
alpha_ref = pi - (2*ocl_traj.simout(:,1)+ocl_traj.simout(:,2))/2;
joint_angles_ref = [ocl_traj.time, ocl_traj.simout, alpha_ref];
alpha_ref_landing = pi - (2*landing_traj.simout(:,1)+landing_traj.simout(:,2))/2;
landing_ref = [landing_traj.time, landing_traj.simout, alpha_ref_landing];

% uneven ground input
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

%%  parallel simulation
sim_batch_amount = length(search_list);

% Do parallel simulation in batches of sim_batch_amount
for k = 1:sim_batch_amount
    in(k) = Simulink.SimulationInput('model_5LinkWalking_NODS');
    x_g = uneven_terrain.track_start:uneven_terrain.dist_step_size:uneven_terrain.track_end;
    delta = search_list(k, 5);
    y_g = delta * uneven_terrain.y_g_seed;
    y_sw_init = interp1(x_g, y_g, 0);
    in(k) = in(k).setVariable('init_flag', ...
        [-1; 0; ocl_traj.x_sw(1); init_t_mode_change; alpha_ref(1); 0; y_sw_init]);
    in(k) = in(k).setVariable('uneven_terrain.y_g_curr', y_g);

    % biarticular muscle parameters

    params.r = search_list(k, 1); % dimensionless lever arm ratio (found from optimizing wrt SR) r = r_h / r_k
    params.k_bar_ba = search_list(k, 2); % [Nm] k_bar_ba = k_ba * r_k^2

    params.r_h = params.r*params.r_k; % [m]
    params.k_ba = params.k_bar_ba/(params.r_k^2); % [N/m]
    param = [params.m1; params.m2; params.m5; params.l1; params.l2; params.l5; params.g; params.I1; params.I2; params.I5; params.r_k; params.r_h; params.k_ba; params.phi_h0; params.phi_k0];

    in(k) = in(k).setVariable('param', param);

    K_p = search_list(k, 3);
    K_d = search_list(k, 4);
    in(k) = in(k).setVariable('gains', [K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d,K_p,K_d]);
    in(k) = in(k).setVariable('Tf', Tf);
    in(k) = in(k).setVariable('sample_time', sample_time);
    in(k) = in(k).setVariable('joint_angles_ref', joint_angles_ref);
    in(k) = in(k).setVariable('landing_ref', landing_ref);
    in(k) = in(k).setVariable('Initial_state', Initial_state);
    in(k) = in(k).setVariable('q0', q0);
    in(k) = in(k).setVariable('dq0', dq0);
    in(k) = in(k).setVariable('uneven_terrain_bus', uneven_terrain_bus);
end
out = parsim(in, ...
    'ShowSimulationManager', 'off', ...
    'TransferBaseWorkspaceVariables','on');

% Determine pass or fail
for k = 1:sim_batch_amount
    time_end(k) = out(k).time(end);
    if time_end(k) < Tf
        search_list(k, 6) = 0;
    else
        search_list(k, 6) = 1;
    end
end

end