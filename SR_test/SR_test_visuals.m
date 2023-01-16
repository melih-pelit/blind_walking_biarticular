clear
clc

SR_lim = [0, 1.5];
results_mean = average_SR('SR_test_results\SR_results2023-01-10-18-59_delta_0.mat', SR_lim);
results_mean = average_SR('SR_test_results\SR_results2023-01-10-23-16_delta_0_025.mat', SR_lim);
results_mean = average_SR('SR_test_results\SR_results2023-01-11-03-17_delta_0_05.mat', SR_lim);

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

figure()

surf(def_x, def_y, def_z')
colormap jet
colorbar

% caxis([-0.001 max(max(gain_test_results_BA.gain_test_result.PASS))])

font_size = 14;
figure_title = "$\delta$ = " + num2str(SR_test_result.delta) + "[m]";
title(figure_title, "Interpreter", "latex", "FontSize", font_size)
xlabel("$\bar{\kappa}$ [N/m]", "Interpreter", "latex", "FontSize", font_size)
ylabel('r', "Interpreter", "latex", "FontSize", font_size)
zlabel('SR', 'Interpreter', "latex", "FontSize", font_size)
zlim(SR_lim)
% xlim([0,500])

figure()
hold on
grid on
title(figure_title, "Interpreter", "latex", "FontSize", font_size)
grey_color = linspace(0.8, 0.1, length(SR_test_result.r_search));
for i = 1:length(SR_test_result.r_search)
    display_name = "r = " + num2str(SR_test_result.r_search(i));
    plot(SR_test_result.k_bar_ba_search, results_mean(i,:), ...
        'DisplayName', display_name, ...
        'Color', [grey_color(i), grey_color(i), grey_color(i)], ...
        'LineWidth', 2);
    legend
end

xlabel("$\bar{\kappa}$ [N/m]", "Interpreter", "latex", "FontSize", font_size)
ylabel('SR', 'Interpreter', "latex", "FontSize", font_size)
ylim(SR_lim)

end