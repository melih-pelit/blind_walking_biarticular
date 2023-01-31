result(1) = load('BA_test_results\BA_param_result_2023-01-23-12-09_terrain_3_1.mat');
result(2) = load('BA_test_results\BA_param_result_2023-01-25-09-52_terrain_3_2.mat');
result(3) = load('BA_test_results\BA_param_result_2023-01-26-23-54_terrain_3_3.mat');
result(4) = load('BA_test_results\BA_param_result_2023-01-27-13-35_terrain_3_4.mat');
result(5) = load('BA_test_results\BA_param_result_2023-01-28-05-24_terrain_3_5.mat');
result(6) = load('BA_test_results\BA_param_result_2023-01-28-17-03_terrain_3_6.mat');
result(7) = load('BA_test_results\BA_param_result_2023-01-29-09-55_terrain_3_7.mat');
result(8) = load('BA_test_results\BA_param_result_2023-01-29-21-08_terrain_3_8.mat');
result(9) = load('BA_test_results\BA_param_result_2023-01-30-09-43_terrain_3_9.mat');
result(10) = load('BA_test_results\BA_param_result_2023-01-30-21-14_terrain_3_10.mat');

i = 5;
y_BA = result(i).BA_test_result.r_search;
x_BA = result(i).BA_test_result.k_bar_ba_search;
PASS_BA = result(i).BA_test_result.PASS;

size_BA = size(PASS_BA);

figure()
surf(x_BA, y_BA(1:size_BA(1)), PASS_BA)
colormap jet
colorbar

font_size = 14;
xlabel("$\bar{\kappa}$ [N/m]", "Interpreter", "latex", "FontSize", font_size)
ylabel('r', "Interpreter", "latex", "FontSize", font_size)
zlabel('$\bar{\delta}$ [m]', 'Interpreter', "latex", "FontSize", font_size)
% zlim([-0.001,0.05])
xlim([x_BA(1),x_BA(end)])
ylim([y_BA(1),y_BA(end)])

%% Average result
sum_delta_bar = zeros(size_BA);
for i = 1:length(result)
    sum_delta_bar = sum_delta_bar + result(i).BA_test_result.PASS;
end

mean_delta_bar = sum_delta_bar./length(result);

figure()
surf(x_BA, y_BA(1:size_BA(1)), mean_delta_bar)
colormap jet
colorbar

font_size = 14;
xlabel("$\bar{\kappa}$ [N/m]", "Interpreter", "latex", "FontSize", font_size)
ylabel('r', "Interpreter", "latex", "FontSize", font_size)
zlabel('$\bar{\delta}$ [m]', 'Interpreter', "latex", "FontSize", font_size)
% zlim([-0.001,0.05])
xlim([x_BA(1),x_BA(end)])
ylim([y_BA(1),y_BA(end)])

%% Percentage-wise result
mean_delta_bar_star = mean_delta_bar./mean_delta_bar(1,1);

figure()
surf(x_BA, y_BA(1:size_BA(1)), mean_delta_bar_star)
colormap jet
colorbar

font_size = 14;
xlabel("$\bar{\kappa}$ [N/m]", "Interpreter", "latex", "FontSize", font_size)
ylabel('r', "Interpreter", "latex", "FontSize", font_size)
zlabel('$\bar{\delta}$ [m]', 'Interpreter', "latex", "FontSize", font_size)
% zlim([-0.001,0.05])
xlim([x_BA(1),x_BA(end)])
ylim([y_BA(1),y_BA(end)])