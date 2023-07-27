clear
clc

load("gain_test_results\gain_test_result_2023-02-07-11-15_terrain_3_1.mat")
% load("gain_test_results\gain_test_result_2023-02-12-00-08_terrain_3_2.mat")

def_x = gain_result.K_d;
def_y = gain_result.K_p;
def_PASS = gain_result.PASS;

size_def = size(def_PASS);

figure()
surf(def_x, def_y(1:size_def(1)), def_PASS)
colormap jet
colorbar

% caxis([-0.001 max(max(gain_test_results_BA.gain_test_result.PASS))])

xlabel('K_D')
ylabel('K_P')
zlabel('max pass \delta')
zlim([-0.001,0.1])
xlim([0,500])