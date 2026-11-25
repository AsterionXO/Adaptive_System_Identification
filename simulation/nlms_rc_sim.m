%% NLMS adaptive FIR identification of an RC plant - MATLAB simulation
% Replicates the Arduino experiment step by step:
%   1. DC calibration: hold duty 40, 120 and 200, read the output, compute G.
%   2. Training: random duty 40-200 every 0.2 s, NLMS update, early stop when the
%      100-step window RMS error is below 15 mV for 3 consecutive windows.
%   3. Test: 200 new random steps with the weights frozen.
%   4. Tables: simulated weights against the expected RC response and against
%      the breadboard results, plus a test-error summary.
%
% Plant model (time-domain, PWM switching included):
%   D9 PWM (490 Hz, 0 V / Vhigh) -> R3/R4 divider with capacitor C -> buffer -> amplifier
%   dx/dt = (k_div * v_pin(t) - x) / tau,     d = K_amp * x + V_off
% ADC model: 10-bit, code = floor(V * 1024 / VREF), 16 conversions averaged after
% one discarded conversion, additive Gaussian noise before quantisation.
%
% Runs as a plain MATLAB script (R2016b or later, no toolboxes).

clear; clc;

%% ===================== Settings =====================
cfg.Mlist     = [12 24];      % tap counts to simulate
cfg.seed      = 1;            % random seed (the Arduino sequence itself cannot be reproduced)
cfg.usePWM    = true;         % true: simulate PWM switching; false: ideal average input

% Plant parameters, set from the breadboard measurements
cfg.VREF      = 5.14;         % supply rail and ADC reference (V)
cfg.Vhigh     = 5.14;         % PWM high level (V)
cfg.G         = 0.8614;       % DC gain, V per unit u (Arduino DC calibration)
cfg.tau       = 0.873;        % time constant (s), Arduino estimate from the 24-tap run
cfg.d120      = 1.6489;       % output at duty 120 as reported by the Arduino (V)
cfg.K_amp     = 2.85;         % amplifier gain from x to d: (1 + Rf/Rg) / 2
cfg.noiseLSB  = 0.7;          % ADC input noise, in LSB (assumed value)

% Arduino sketch settings
cfg.T         = 0.200;        % sample interval (s), T_MS
cfg.mu        = 0.2;          % NLMS step size
cfg.epsReg    = 0.001;        % regularisation constant in the NLMS step
cfg.Nmax      = 1000;         % maximum training steps
cfg.window    = 100;          % steps per RMS window
cfg.rmsStop   = [12 0.020;    % early-stop threshold (V) per tap count:
                 24 0.015];    % 20 mV for 12 taps, 15 mV for 24 taps
cfg.nWinStop  = 3;            % consecutive windows below threshold
cfg.Ntest     = 200;          % test steps
cfg.tSettle   = 5;            % settle time at duty 120 before training and test (s)
cfg.tHold     = 10;           % hold time before each DC calibration reading (s)
cfg.nDCreads  = 3;            % readings averaged per DC calibration point
cfg.tConv     = 112e-6;       % ADC conversion time (s)
cfg.fPWM      = 490;          % PWM frequency on pin D9 (Hz)

%% ===================== Breadboard results (latest session) =====================
hw = struct();
hw(12).w     = [0.1757 0.1402 0.1119 0.0888 0.0707 0.0549 0.0417 0.0329 0.0267 0.0219 0.0181 0.0165];
hw(12).bias  = 1.6417;  hw(12).tau = 0.859;  hw(12).stop = 600;
hw(12).rms   = 13.5;    hw(12).max = 35.9;
hw(24).w     = [0.1751 0.1385 0.1097 0.0872 0.0700 0.0557 0.0448 0.0360 0.0293 0.0235 0.0188 0.0156 ...
                0.0125 0.0104 0.0082 0.0066 0.0057 0.0049 0.0042 0.0036 0.0030 0.0026 0.0021 0.0017];
hw(24).bias  = 1.6496;  hw(24).tau = 0.873;  hw(24).stop = 600;
hw(24).rms   = 2.4;     hw(24).max = 9.9;

%% ===================== Derived plant constants =====================
P.lsb    = cfg.VREF/1024;
P.Vhigh  = cfg.Vhigh;
P.VREF   = cfg.VREF;
P.K_amp  = cfg.K_amp;
P.k_div  = cfg.G / (cfg.K_amp * cfg.Vhigh * 80/255);          % divider ratio giving the measured G
P.V_off  = (cfg.d120 + 0.5*P.lsb) - cfg.K_amp*P.k_div*cfg.Vhigh*120/255;  % offset (ADC floor corrected)
P.Tpwm   = 1/cfg.fPWM;
P.dt     = P.Tpwm/40;                                         % 40 sub-samples per PWM period
P.alpha  = exp(-P.dt/cfg.tau);                                % exact per-sub-sample RC decay
P.usePWM = cfg.usePWM;
P.noise  = cfg.noiseLSB * P.lsb;
P.tConv  = cfg.tConv;

if exist('rng','file') || exist('rng','builtin'), rng(cfg.seed); else, rand('seed',cfg.seed); randn('seed',cfg.seed); end %#ok<RAND>

fprintf('Plant: G = %.4f V/unit u, tau = %.3f s, divider ratio = %.4f, offset = %.4f V\n', ...
        cfg.G, cfg.tau, P.k_div, P.V_off);
fprintf('Simulation: PWM switching %s, ADC noise %.1f LSB\n\n', mat2str(cfg.usePWM), cfg.noiseLSB);

%% ===================== 1. DC calibration =====================
st.x = P.k_div*cfg.Vhigh*120/255;  st.t = 0;                  % start settled at duty 120
dcDuty = [40 120 200];  dcVal = zeros(1,3);
for i = 1:3
    st = advance(st, dcDuty(i), cfg.tHold, P);
    r = zeros(1,cfg.nDCreads);
    for j = 1:cfg.nDCreads
        [st, r(j)] = readD(st, dcDuty(i), P);
    end
    dcVal(i) = mean(r);
end
Gsim = (dcVal(3) - dcVal(1))/160*80;
fprintf('DC calibration (simulated ADC): d40 = %.4f V, d120 = %.4f V, d200 = %.4f V, G = %.4f\n\n', ...
        dcVal(1), dcVal(2), dcVal(3), Gsim);

%% ===================== 2-4. Train, test and tabulate for each tap count =====================
summary = zeros(numel(cfg.Mlist), 8);
for iM = 1:numel(cfg.Mlist)
    M = cfg.Mlist(iM);
    fprintf('==================== %d taps ====================\n', M);

    % ---- training ----
    rmsStop = cfg.rmsStop(cfg.rmsStop(:,1) == M, 2);         % threshold for this tap count
    w = zeros(M,1);  bias = 0;  u = zeros(M,1);
    st = advance(st, 120, cfg.tSettle, P);                     % settle at mid-scale
    fprintf('step, d (V), y (V), e (mV), window RMS error (mV)\n');
    acc = 0;  below = 0;  nStop = cfg.Nmax;  stoppedEarly = false;
    for n = 1:cfg.Nmax
        duty = randi([40 200]);
        u = [(duty-120)/80; u(1:end-1)];                        % shift input history
        st = advance(st, duty, cfg.T, P);                       % hold input for one interval
        [st, d] = readD(st, duty, P);                           % measured output
        y = bias + w'*u;                                        % filter prediction
        e = d - y;                                              % prediction error
        g = cfg.mu*e/(cfg.epsReg + 1 + u'*u);                   % normalised step
        bias = bias + g;  w = w + g*u;                          % NLMS update
        acc = acc + e^2;
        if mod(n, cfg.window) == 0
            rmsW = sqrt(acc/cfg.window);  acc = 0;
            fprintf('%d, %.4f, %.4f, %.1f, %.1f\n', n, d, y, 1000*e, 1000*rmsW);
            if rmsW < rmsStop, below = below + 1; else, below = 0; end
            if below >= cfg.nWinStop, nStop = n; stoppedEarly = true; break; end
        end
    end
    if stoppedEarly
        fprintf('Training stopped at step %d: window RMS below threshold for %d consecutive windows.\n', nStop, cfg.nWinStop);
    else
        fprintf('Training stopped: maximum step count reached.\n');
    end

    % ---- tau estimate (same method as the sketch) and expected weights ----
    aEst   = (w(6)/w(1))^(1/5);
    tauEst = -cfg.T/log(aEst);
    k      = (0:M-1)';
    wExp   = Gsim*(1-aEst)*aEst.^k;

    % ---- test with frozen weights ----
    st = advance(st, 120, cfg.tSettle, P);
    u = zeros(M,1);  et = zeros(cfg.Ntest,1);
    for n = 1:cfg.Ntest
        duty = randi([40 200]);
        u = [(duty-120)/80; u(1:end-1)];
        st = advance(st, duty, cfg.T, P);
        [st, d] = readD(st, duty, P);
        y = bias + w'*u;
        et(n) = d - y;
    end
    testRMS = 1000*sqrt(mean(et.^2));  testMax = 1000*max(abs(et));
    fprintf('TEST rms error (mV): %.1f\nTEST max error (mV): %.1f\n', testRMS, testMax);
    fprintf('Last test step: d (V) = %.4f, y (V) = %.4f, e (mV) = %.1f\n\n', d, y, 1000*et(end));

    % ---- table: simulated weights vs expected vs breadboard ----
    wHW = hw(M).w(:);
    fprintf('Expected weights: w[k] = G(1 - a)a^k with G = %.4f (simulated DC calibration), tau = %.3f s (simulated estimate)\n', Gsim, tauEst);
    fprintf('%-6s %9s %9s %9s %8s %10s %10s %8s\n', 'k', 'Sim', 'Expected', 'Err (mV)', 'Err (%)', 'Breadboard', 'Sim-BB(mV)', 'Diff (%)');
    for i = 1:M
        fprintf('%-6d %9.4f %9.4f %+9.1f %+8.1f %10.4f %+10.1f %+8.1f\n', k(i), w(i), wExp(i), ...
            1000*(w(i)-wExp(i)), 100*(w(i)-wExp(i))/wExp(i), wHW(i), 1000*(w(i)-wHW(i)), 100*(w(i)-wHW(i))/wHW(i));
    end
    fprintf('%-6s %9.4f %9.4f %+9.1f %+8.2f %10.4f %+10.1f %+8.2f\n', 'Bias', bias, dcVal(2), ...
        1000*(bias-dcVal(2)), 100*(bias-dcVal(2))/dcVal(2), hw(M).bias, 1000*(bias-hw(M).bias), 100*(bias-hw(M).bias)/hw(M).bias);
    fprintf('%-6s %9.4f %9.4f %+9.1f %+8.1f %10.4f %+10.1f %+8.1f\n', 'Sum', sum(w), sum(wExp), ...
        1000*(sum(w)-sum(wExp)), 100*(sum(w)-sum(wExp))/sum(wExp), sum(wHW), 1000*(sum(w)-sum(wHW)), 100*(sum(w)-sum(wHW))/sum(wHW));
    fprintf('%-6s %9s %9s %9.1f %8s %10s %10.1f %8s\n\n', 'RMS', '', '', ...
        1000*sqrt(mean((w-wExp).^2)), '', '', 1000*sqrt(mean((w-wHW).^2)), '');

    summary(iM,:) = [M, nStop, tauEst, testRMS, testMax, hw(M).stop, hw(M).tau, hw(M).rms];
    simRes(iM).M = M; simRes(iM).w = w; simRes(iM).bias = bias; simRes(iM).wExp = wExp; simRes(iM).tau = tauEst; %#ok<SAGROW>
    simRes(iM).et = et; simRes(iM).testRMS = testRMS; simRes(iM).testMax = testMax; simRes(iM).nStop = nStop; %#ok<SAGROW>
end

%% ===================== Save results for plotting =====================
save('nlms_sim_results.mat', 'simRes', 'cfg', 'dcVal', 'Gsim');
fprintf('Simulation results saved to nlms_sim_results.mat\n\n');

%% ===================== Test-error summary =====================
fprintf('==================== Summary: simulation vs breadboard ====================\n');
fprintf('%-6s %10s %10s %10s %10s %10s %10s %10s %10s\n', 'Taps', 'Sim stop', 'HW stop', 'Sim tau', 'HW tau', ...
        'Sim RMS', 'HW RMS', 'Sim max', 'HW max');
for iM = 1:numel(cfg.Mlist)
    M = cfg.Mlist(iM);
    fprintf('%-6d %10d %10d %10.3f %10.3f %10.1f %10.1f %10.1f %10.1f\n', M, summary(iM,2), hw(M).stop, ...
        summary(iM,3), hw(M).tau, summary(iM,4), hw(M).rms, summary(iM,5), hw(M).max);
end
fprintf('(HW = breadboard; stop = step at which training ended; tau in s; RMS and max test errors in mV)\n');

%% ===================== Local functions =====================
function st = advance(st, duty, dur, P)
% Advance the RC plant by dur seconds with the given PWM duty (0-255).
    n = max(1, round(dur/P.dt));
    t = st.t + (1:n)'*P.dt;
    if P.usePWM
        % Exact fraction of each sub-interval [t-dt, t] during which the PWM output is high
        Th = duty/255*P.Tpwm;
        v  = P.Vhigh * (highTime(t, P.Tpwm, Th) - highTime(t - P.dt, P.Tpwm, Th)) / P.dt;
    else
        v = P.Vhigh*duty/255*ones(n,1);                     % ideal average
    end
    xs = filter(1-P.alpha, [1 -P.alpha], P.k_div*v, P.alpha*st.x);
    st.x = xs(end);  st.t = t(end);
end

function [st, d] = readD(st, duty, P)
% One readD() call: one discarded conversion, then 16 conversions averaged.
    codes = zeros(16,1);
    for i = 1:17
        st = advance(st, duty, P.tConv, P);
        v = P.K_amp*st.x + P.V_off + P.noise*randn;
        c = min(max(floor(v/P.VREF*1024), 0), 1023);       % 10-bit ADC transfer
        if i > 1, codes(i-1) = c; end
    end
    d = mean(codes)*P.VREF/1024;
end

function h = highTime(s, Tp, Th)
% Cumulative time the PWM output has been high from t = 0 to t = s.
    q = s/Tp;
    k = floor(q);
    h = k*Th + min((q - k)*Tp, Th);
end