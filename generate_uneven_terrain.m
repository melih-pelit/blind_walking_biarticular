function generate_uneven_terrain(terrain_name_start, terrain_name_end)

dist_step_size = 0.1; % [m]
track_start = -2; % [m]
track_end = 30; % [m]

deltaY_inc = 0.001; % [m] dist increment
deltaY_max = 0.100; % max dist
deltaY = 0:deltaY_inc:deltaY_max;

x_g=track_start:dist_step_size:track_end;

for j = terrain_name_start:terrain_name_end
    for i = 1:length(x_g)
        y_g_seed(j,i) = rand();
    end

    uneven_terrain.dist_step_size = dist_step_size;
    uneven_terrain.track_start = track_start;
    uneven_terrain.track_end = track_end;
    uneven_terrain.x_g = x_g;
    uneven_terrain.y_g_seed = y_g_seed(j,:);

    % save terrain
    filename = "unevenground_v3_" + int2str(j) + ".mat";
    subfolder = 'terrain data';
    save(fullfile(subfolder,filename),'uneven_terrain')
end

end