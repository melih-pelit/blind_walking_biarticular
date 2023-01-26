result(1) = load('BA_test_results\BA_param_result_2023-01-25-09-52_terrain_3_2.mat');

y_BA = result(1).BA_test_result.r_search;
x_BA = result(1).BA_test_result.k_bar_ba_search;
PASS_BA = result(1).BA_test_result.PASS;

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
