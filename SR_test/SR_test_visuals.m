clear
clc

load('SR_test_results\SR_results2023-01-10-15-48_delta_0_025.mat')

% averaging the results
results_length = length(SR_test_result.result);

results_size = size(SR_test_result.result(1).SR);
results_sum = zeros(results_size);
for i = 1:results_length
    results_sum = results_sum + SR_test_result.result(i).SR;
end
results_mean = results_sum./results_length;

def_x = SR_test_result.r_search;
def_y = SR_test_result.k_bar_ba_search;
def_z = results_mean;

figure()

surf(def_x, def_y, def_z)
colormap jet
colorbar

% caxis([-0.001 max(max(gain_test_results_BA.gain_test_result.PASS))])

font_size = 14;
figure_title = "$\delta$ = " + num2str(SR_test_result.delta) + "[m]";
title(figure_title, "Interpreter", "latex", "FontSize", font_size)
xlabel("$\bar{\kappa}$ [N/m]", "Interpreter", "latex", "FontSize", font_size)
ylabel('r', "Interpreter", "latex", "FontSize", font_size)
zlabel('$\bar{\delta}$ [m]', 'Interpreter', "latex", "FontSize", font_size)
% zlim([-0.001,0.1])
% xlim([0,500])