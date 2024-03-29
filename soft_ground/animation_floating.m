function animation_floating(f_video, simout, param, f_pause, frame_leap, flag, uneven_terrain, time, k, deltaY, start_end_time)
pause
if f_video == 1
    video_v = VideoWriter('5link_SLIP_walking.avi');
    open(video_v);
end
%%%%% Model Parameters %%%%%
m1 = param(1);
m2 = param(2);
m3 = m2;
m4 = m1;
m5 = param(3);

l1 = param(4);
l2 = param(5);
l5 = param(6);
l3 = l2;
l4 = l1;

% biarticular parameters
r_k = param(11);
r_h = param(12);
k_ba = param(13);
phi_h0 = param(14);
phi_k0 = param(15);

nt = length(simout);

%%%%%%%%%%%
start_sec = start_end_time(1); % start second for the animation
end_sec = start_end_time(2); % ending second for the animation

sample_time = time(2) - time(1);
start_frame = start_sec/sample_time;
if start_sec == 0
    start_frame = 1;
end
end_frame = min(nt, end_sec/sample_time);

f_ba_springs = 1;
if k_ba == 0
    f_ba_springs = 0;
end

figure('units','pixels','position',[0 0 720 720])

for i = start_frame:frame_leap:end_frame
    
    current_X = simout(i,:);
    current_X_non_floating = [current_X(:,3:7), current_X(:,10:14)];

    x_st = current_X(1);
    y_st = current_X(2);
    
    plot_5_links_robot(current_X_non_floating, x_st, y_st, param, f_ba_springs)
    hold on
    
    th1 = current_X_non_floating(1);
    th2 = current_X_non_floating(2);
    th3 = current_X_non_floating(3);
    th4 = current_X_non_floating(4);
    th5 = current_X_non_floating(5);
    
    % Hip Location
    P_hip = [x_st + l1*cos(th1) + l2*cos(th1 + th2);
        y_st + l1*sin(th1) + l2*sin(th1 + th2)];  

    delta = (k - 1)*deltaY;
    title(['t = ' num2str(time(i)) 'sec \delta = ', num2str(delta) 'm'])
    
    % plotting CoM----------------------------------------------
    CoM= calculate_com(current_X, param, flag(i,:), [0;0;0;0;0]);
    xG = CoM(1);
    zG = CoM(2);
    % plot(xG, zG, 'o', 'MarkerSize', 7, 'MarkerFaceColor', 'b');
    %-----------------------------------------------------------
    
    % plotting uneven terrain ----------------------------------------------
    x_g = uneven_terrain.track_start:uneven_terrain.dist_step_size:uneven_terrain.track_end;
    try
        uneven_terrain.y_g_seed;
    catch
        uneven_terrain.y_g_seed = 10 * uneven_terrain.y_g(101,:); % TODO
    end
    plot(x_g, delta * uneven_terrain.y_g_seed, 'k', 'LineWidth', 1.5);
    %-----------------------------------------------------------
    
    % plotting desired location of swing foot and CoM--------------
    %     plot(sw_ft_des(i, 1), sw_ft_des(i, 2), 'o', 'MarkerSize', 3, 'MarkerFaceColor', 'r');
    %     plot(des_z_dz_dx(i,1), des_z_dz_dx(i,2), 'o', 'MarkerSize', 3, 'MarkerFaceColor', 'g');
    %-----------------------------------------------------------
    
    hold off
    
    %     xlim([-2 + P_hip(1), 2 + P_hip(1)])
    xlim([-0.8 + P_hip(1), 0.8 + P_hip(1)])
    ylim([-0.1, 1.5])
    
    if f_video == 1
        frame = getframe(gcf);
        writeVideo(video_v,frame);
    end
    
    if f_pause == 1
        pause
    else
        pause(0.01)
    end
    
end
end