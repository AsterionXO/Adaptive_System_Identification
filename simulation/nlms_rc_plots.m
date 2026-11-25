%% NLMS adaptive FIR identification of an RC plant - result plots
% Figure 1: 12-tap weights. Figure 2: 24-tap weights.
% Each figure shows, against the delay k:
%   - expected weights using the Arduino DC calibration
%   - expected weights using the multimeter DC calibration
%   - weights learned by the Arduino (breadboard)
%   - weights learned by the MATLAB simulation (nlms_rc_sim.m)
% Expected weights: w[k] = G(1 - a)a^k, a = exp(-T/tau), with tau taken from the
% Arduino's own estimate for the same run.

clear; clc; close all;

%% ===================== Data =====================
T = 0.2;                                   % sample interval (s)

% DC calibration (V): duty 40 and 200
dArd   = [0.7872 2.5099];                  % Arduino readings
dMeter = [0.784  2.490];                   % multimeter readings
G_ard   = (dArd(2)   - dArd(1))  /160*80;  % gain, V per unit u
G_meter = (dMeter(2) - dMeter(1))/160*80;

% Weights learned by the Arduino (breadboard)
hw12 = [0.1757 0.1402 0.1119 0.0888 0.0707 0.0549 0.0417 0.0329 0.0267 0.0219 0.0181 0.0165];
hw24 = [0.1751 0.1385 0.1097 0.0872 0.0700 0.0557 0.0448 0.0360 0.0293 0.0235 0.0188 0.0156 ...
        0.0125 0.0104 0.0082 0.0066 0.0057 0.0049 0.0042 0.0036 0.0030 0.0026 0.0021 0.0017];
tau12 = 0.859;                             % Arduino tau estimate, 12-tap run (s)
tau24 = 0.873;                             % Arduino tau estimate, 24-tap run (s)

% Weights learned by the MATLAB simulation (nlms_rc_sim.m, seed 1)
sim12 = [0.1776 0.1410 0.1132 0.0901 0.0729 0.0591 0.0483 0.0379 0.0326 0.0270 0.0228 0.0181];
sim24 = [0.1775 0.1417 0.1128 0.0899 0.0716 0.0575 0.0458 0.0371 0.0294 0.0237 0.0189 0.0154 ...
         0.0124 0.0101 0.0080 0.0065 0.0056 0.0045 0.0042 0.0038 0.0036 0.0033 0.0032 0.0030];

% Use the saved simulation results instead, if available
useSavedSim = true;
if useSavedSim && exist('nlms_sim_results.mat', 'file')
    S = load('nlms_sim_results.mat');
    for i = 1:numel(S.simRes)
        if S.simRes(i).M == 12, sim12 = S.simRes(i).w(:)'; end
        if S.simRes(i).M == 24, sim24 = S.simRes(i).w(:)'; end
    end
    fprintf('Simulation weights loaded from nlms_sim_results.mat\n');
end

%% ===================== Figures =====================
plotWeights(12, hw12, sim12, tau12, G_ard, G_meter, T);
print(gcf, 'weights_12tap.png', '-dpng', '-r300');

plotWeights(24, hw24, sim24, tau24, G_ard, G_meter, T);
print(gcf, 'weights_24tap.png', '-dpng', '-r300');

fprintf('Saved weights_12tap.png and weights_24tap.png\n');

%% ===================== Local functions =====================
function plotWeights(M, wHW, wSim, tau, G_ard, G_meter, T)
% Plot expected, breadboard and simulated weights for an M-tap filter.
    k = 0:M-1;
    a = exp(-T/tau);
    wExpArd   = G_ard  *(1-a)*a.^k;
    wExpMeter = G_meter*(1-a)*a.^k;

    figure('Color', 'w', 'Position', [100 100 760 480]);
    hold on; grid on; box on;
    plot(k, wExpArd,   '--s', 'Color', [0.00 0.45 0.74], 'LineWidth', 1.2, 'MarkerSize', 6);
    plot(k, wExpMeter, ':^',  'Color', [0.85 0.33 0.10], 'LineWidth', 1.2, 'MarkerSize', 6);
    plot(k, wHW,       '-o',  'Color', [0.00 0.00 0.00], 'LineWidth', 1.5, 'MarkerSize', 6, ...
         'MarkerFaceColor', [0.00 0.00 0.00]);
    plot(k, wSim,      '-d',  'Color', [0.64 0.08 0.18], 'LineWidth', 1.2, 'MarkerSize', 6);
    hold off;

    xlim([-0.5, M-0.5]);
    xticks(0:(1 + (M > 12)):M-1);
    xlabel('Delay k (samples, T = 0.2 s)');
    ylabel('Weight w[k] (V per unit u)');
    title(sprintf('%d-tap NLMS filter: learned and expected weights (\\tau = %.3f s)', M, tau));
    legend({sprintf('Expected, Arduino calibration (G = %.4f)', G_ard), ...
            sprintf('Expected, multimeter calibration (G = %.4f)', G_meter), ...
            'Learned by Arduino', 'Learned by simulation'}, 'Location', 'northeast');
    set(gca, 'FontSize', 11);
end
