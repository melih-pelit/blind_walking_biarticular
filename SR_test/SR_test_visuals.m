clear
% clc

SR_lim = [0, 1.5];
% mean_SR = average_SR('SR_test_results\SR_results2023-01-10-18-59_delta_0.mat', SR_lim);
mean_SR = average_SR('SR_test_results\SR_results2023-01-10-23-16_delta_0_025.mat', SR_lim);
% mean_SR = average_SR('SR_test_results\SR_results2023-01-11-03-17_delta_0_05.mat', SR_lim);

%% BA param test results
% (v2 terrain results) using the BA Param. Test results from: C:\Matlab Workspace\Biarticular Robustness\Controlled System\Analysis\multi data analysis BAparams
folder_location_results = "C:\Matlab Workspace\Biarticular Robustness\Controlled System\ba model\20220815_ba param testing v2\";
% unevenground_0_1_v2_1.mat
result(1) = load(folder_location_results + "BA_params_test_results\5LinkWalkingOpenOCL2022-08-24-22-17.mat");
% unevenground_0_1_v2_2.mat
result(2) = load(folder_location_results + "BA_params_test_results\5LinkWalkingOpenOCL2022-08-23-20-54.mat");
% unevenground_0_1_v2_4.mat'
result(3) = load(folder_location_results + "BA_params_test_results\5LinkWalkingOpenOCL2022-09-01-21-54.mat");
% unevenground_0_1_v2_17
result(4) = load(folder_location_results + "BA_params_test_results\5LinkWalkingOpenOCL2022-10-16-09-48.mat");
size_PASS = size(result(1).BA_params_test_result.PASS);
sum_BAparam = zeros(size_PASS(1), size_PASS(2));
for i = 1:length(result)
    sum_BAparam = sum_BAparam + result(i).BA_params_test_result.PASS;
end
def_x = result(1).BA_params_test_result.r_search;
def_y = result(1).BA_params_test_result.k_bar_ba_search;

%%
% % v3 terrain results
% folder_location_results = "C:\Matlab Workspace\Biarticular Robustness\Controlled System\BiarticularModelRepo\BA_param_test\";
% result(1) = load(folder_location_results + "BA_test_results\BA_param_result_2023-01-23-12-09_terrain_3_1.mat");
% result(2) = load(folder_location_results + "BA_test_results\BA_param_result_2023-01-25-09-52_terrain_3_2.mat");
% result(3) = load(folder_location_results + "BA_test_results\BA_param_result_2023-01-26-23-54_terrain_3_3.mat");
% result(4) = load(folder_location_results + "BA_test_results\BA_param_result_2023-01-27-13-35_terrain_3_4.mat");
% result(5) = load(folder_location_results + "BA_test_results\BA_param_result_2023-01-28-05-24_terrain_3_5.mat");
% result(6) = load(folder_location_results + "BA_test_results\BA_param_result_2023-01-28-17-03_terrain_3_6.mat");
% result(7) = load(folder_location_results + "BA_test_results\BA_param_result_2023-01-29-09-55_terrain_3_7.mat");
% result(8) = load(folder_location_results + "BA_test_results\BA_param_result_2023-01-29-21-08_terrain_3_8.mat");
% result(9) = load(folder_location_results + "BA_test_results\BA_param_result_2023-01-30-09-43_terrain_3_9.mat");
% result(10) = load(folder_location_results + "BA_test_results\BA_param_result_2023-01-30-21-14_terrain_3_10.mat");
% size_PASS = size(result(1).BA_test_result.PASS);
% 
% % calculate the average result
% sum_BAparam = zeros(size_PASS(1), size_PASS(2));
% for i = 1:length(result)
%     sum_BAparam = sum_BAparam + result(i).BA_test_result.PASS;
% end
% def_x = result(1).BA_test_result.r_search;
% def_y = result(1).BA_test_result.k_bar_ba_search;

%%
mean_BAparam = sum_BAparam./length(result);

% max values of BA param test
% [max_val_BA, idx] = max(mean_BAparam(:));
% [row_BA, col_BA] = ind2sub(size(mean_BAparam),idx);
% fprintf("Max value of delta_bar is " + num2str(max_val_BA) + ...
%     " where r = " + num2str(def_x(row_BA)) + ", k_bar_ba = " + num2str(def_y(col_BA)) + "\n")

% Merging SR and robustness criterions
% crit = mean_BAparam ./ mean_SR;
% crit = (mean_BAparam./mean_BAparam(1, 1)) ./ (mean_SR./mean_SR(1, 1));

mean_BAparam_normalized = mean_BAparam./mean_BAparam(1, 1);
mean_SR_normalized = (mean_SR(1, 1)./mean_SR);

% max values of normalized BA param test
[max_val_BA, idx] = max(mean_BAparam_normalized(:));
[row_BA, col_BA] = ind2sub(size(mean_BAparam_normalized),idx);
fprintf("Max value of delta_bar is " + num2str(max_val_BA) + ...
    " where r = " + num2str(def_x(row_BA)) + ", k_bar_ba = " + num2str(def_y(col_BA)) + "\n")

figure()
def_z = mean_BAparam_normalized;
surf(def_x, def_y, def_z')
colormap jet
colorbar
font_size = 14;
xlabel('r', "Interpreter", "latex", "FontSize", font_size)
ylabel("$\bar{\kappa}$ [N/m]", "Interpreter", "latex", "FontSize", font_size)
zlabel("$\bar{\delta}^*$", 'Interpreter', "latex", "FontSize", font_size)
% view([0 90])
% view([30 60])
% figure_title = "$\delta$ = " + num2str(0.025) + "[m], $\alpha_1 = $" ...
%     + num2str(alpha_1) + ", $\alpha_2 = $" + num2str(alpha_2);
% title(figure_title, "Interpreter", "latex", "FontSize", font_size)

% figure_title = "$min\(\bar{\delta}$ - 1, 1/SR^* -1\)$";
figure_title = "$\bar{\delta}^*$";
title(figure_title, "Interpreter", "latex", "FontSize", font_size)

% alpha_1 = 2;
% alpha_2 = 1;
% crit = alpha_1*mean_BAparam_normalized + alpha_2*mean_SR_normalized;

% crit = mean_BAparam_normalized.*mean_SR_normalized;

crit = min(mean_BAparam_normalized - 1, mean_SR_normalized - 1);

figure()
def_z = crit;
surf(def_x, def_y, def_z')
colormap jet
colorbar
font_size = 14;
xlabel('r', "Interpreter", "latex", "FontSize", font_size)
ylabel("$\bar{\kappa}$ [N/m]", "Interpreter", "latex", "FontSize", font_size)
% zlabel('$\alpha_1 \bar{\delta}^* + \alpha_2 / SR^*$', 'Interpreter', "latex", "FontSize", font_size)
% view([0 90])
% view([30 60])
% figure_title = "$\delta$ = " + num2str(0.025) + "[m], $\alpha_1 = $" ...
%     + num2str(alpha_1) + ", $\alpha_2 = $" + num2str(alpha_2);
% title(figure_title, "Interpreter", "latex", "FontSize", font_size)

% figure_title = "$min\(\bar{\delta}$ - 1, 1/SR^* -1\)$";
figure_title = "$min( \bar{\delta}^* - 1, 1/SR^* - 1)$";
title(figure_title, "Interpreter", "latex", "FontSize", font_size)

% max values of Criterion
[max_val, idx] = max(def_z(:));
[row, col] = ind2sub(size(def_z),idx);

criterion_string = "min(1 - delta_bar^*star, 1 - 1/SR^*)";

fprintf("Max value of " + criterion_string + " is " + num2str(max_val) + ...
    " where r = " + num2str(def_x(row)) + ", k_bar_ba = " + num2str(def_y(col)) + "\n")

fprintf("For max value of " + criterion_string + " is delta^* is " + num2str(mean_BAparam_normalized(row, col)) + ...
    " and 1/SR^* = " + num2str(mean_SR_normalized(row, col)) + "\n")

function results_mean = average_SR(file_name, SR_lim)

load(file_name)
% load('SR_test_results\SR_results2023-01-10-23-16_delta_0_025.mat')

% averaging the results
results_length = length(SR_test_result.result);

results_size = size(SR_test_result.result(1).SR);
results_sum = zeros(results_size);
for i = 1:results_length
    results_sum = results_sum + SR_test_result.result(i).SR;
    result_matrices(i, :, :) = SR_test_result.result(i).SR;
end
% results_mean = results_sum./results_length;
results_mean_temp = mean(result_matrices, 'omitnan');
results_mean(:, :) = results_mean_temp(1, :, :);

def_x = SR_test_result.r_search;
def_y = SR_test_result.k_bar_ba_search;
def_z = results_mean;

% figure()
% 
% surf(def_x, def_y, 1./def_z')
% colormap jet
% colorbar
% 
% % caxis([-0.001 max(max(gain_test_results_BA.gain_test_result.PASS))])
% 
% font_size = 14;
% figure_title = "1/SR where $\delta$ = " + num2str(SR_test_result.delta) + "[m]";
% title(figure_title, "Interpreter", "latex", "FontSize", font_size)
% xlabel('r', "Interpreter", "latex", "FontSize", font_size)
% ylabel("$\bar{\kappa}$ [N/m]", "Interpreter", "latex", "FontSize", font_size)
% zlabel('1/SR', 'Interpreter', "latex", "FontSize", font_size)
% % zlim(SR_lim)
% % xlim([0,500])
% view([0 90])
% 
% figure()
% hold on
% grid on
% figure_title = "$\delta$ = " + num2str(SR_test_result.delta) + "[m]";
% title(figure_title, "Interpreter", "latex", "FontSize", font_size)
% grey_color = linspace(0.8, 0.1, length(SR_test_result.r_search));
% for i = 1:length(SR_test_result.r_search)
%     display_name = "r = " + num2str(SR_test_result.r_search(i));
%     plot(SR_test_result.k_bar_ba_search, results_mean(i,:), ...
%         'DisplayName', display_name, ...
%         'Color', [grey_color(i), grey_color(i), grey_color(i)], ...
%         'LineWidth', 2);
%     legend
% end
% 
% xlabel("$\bar{\kappa}$ [N/m]", "Interpreter", "latex", "FontSize", font_size)
% ylabel('SR', 'Interpreter', "latex", "FontSize", font_size)
% ylim(SR_lim)

% min values of SR
% [min_val, idx] = min(def_z(:));
% [row, col] = ind2sub(size(def_z),idx);
% 
% fprintf("Max value of the SR is " + num2str(min_val) + ...
%     " where r = " + num2str(def_x(row)) + ", k_bar_ba = " + num2str(def_y(col)) + "\n")

% relative plot (SR^*)
SR_star = (def_z./def_z(1,1));
one_over_SR_star = 1 ./ SR_star;

% min value of 1 / SR^*
[max_val, idx] = max(one_over_SR_star(:));
[row, col] = ind2sub(size(SR_star),idx);

fprintf("Max value of the SR^* is " + num2str(max_val) + ...
    " where r = " + num2str(def_x(row)) + ", k_bar_ba = " + num2str(def_y(col)) + "\n")

figure()

surf(def_x, def_y, one_over_SR_star')
colormap jet
colorbar

font_size = 14;
figure_title = "$1/SR^*,$ $\delta$ = " + num2str(SR_test_result.delta) + "[m]";
title(figure_title, "Interpreter", "latex", "FontSize", font_size)
xlabel('r', "Interpreter", "latex", "FontSize", font_size)
ylabel("$\bar{\kappa}$ [N/m]", "Interpreter", "latex", "FontSize", font_size)
zlabel('1/$SR^*$', 'Interpreter', "latex", "FontSize", font_size)
% zlim(SR_lim)
% xlim([0,500])
% view([0 90])

end