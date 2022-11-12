function [simout, inputTorque, des_theta_alpha, flag, time, PASS, k] = search_delta_bar(landing_traj, terrain_name, params, Tf, gains)

%%

load(terrain_name)
uneven_terrain.y_g_curr = uneven_terrain.y_g(1,:);
uneven_terrain_bus_info = Simulink.Bus.createObject(uneven_terrain);
uneven_terrain_bus = evalin('base', uneven_terrain_bus_info.busName);

%%
skip_amount = 10; % skip how many 0.001 m to make it faster
size_y_g = size(uneven_terrain.y_g);
flag_break = 0;

% uneven ground test
nt = size(uneven_terrain.y_g);
for k=1:skip_amount:nt(1)

    if k > size_y_g(1)
        % if k is beyond the generated terrain delta
        PASS = k*size_y_g(1);
        break
    end

    % Run the simulation
    [simout, inputTorque, des_theta_alpha, flag, time] = run_walking_simulation(landing_traj, terrain_name, params, Tf, gains, k);

    terrain_height = (k*uneven_terrain.deltaY_inc - uneven_terrain.deltaY_inc);
    if time(end)<10 && terrain_height ~= 0
        % Searching in minus direction
        fprintf ('deltaY = %.4f [m] FAIL \n', terrain_height)

        fprintf('Searching in minus direction \n')

        k_prev = k;

        for k=(k_prev-1):-1:(k_prev-skip_amount)

            % Run the simulation

            [simout, inputTorque, des_theta_alpha, flag, time] = run_walking_simulation(landing_traj, terrain_name, params, Tf, gains, k);

            terrain_height = (k*uneven_terrain.deltaY_inc - uneven_terrain.deltaY_inc);

            if time(end)<10
                fprintf ('deltaY = %.4f [m] FAIL \n', terrain_height)

                if k == k_prev-skip_amount+1
                    % if k_prev is the PASS point
                    fprintf ('deltaY = %.4f [m] PASS \n', ((k_prev-skip_amount)*uneven_terrain.deltaY_inc - uneven_terrain.deltaY_inc))
                    PASS = (k_prev-skip_amount)*uneven_terrain.deltaY_inc - uneven_terrain.deltaY_inc;
                    flag_break = 1; % to break out of the outer for loop
                    break
                end

            else
                fprintf ('deltaY = %.4f [m] PASS \n', terrain_height)
                PASS = terrain_height - uneven_terrain.deltaY_inc;
                flag_break = 1; % to break out of the outer for loop
                break
            end

        end

    elseif time(end)<10 && terrain_height == 0
        % if it fails at delta = 0.000 m (k=1)
        fprintf ('deltaY = %.4f [m] FAIL \n', terrain_height)
        PASS = terrain_height - uneven_terrain.deltaY_inc;
        break
    else
        fprintf ('deltaY = %.4f [m] PASS \n', terrain_height)
    end

    if flag_break == 1
        flag_break = 0;
        break
    end
end
end