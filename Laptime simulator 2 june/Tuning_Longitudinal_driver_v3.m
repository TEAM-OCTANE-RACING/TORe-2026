% =========================================================================
% LONGITUDINAL DRIVER PID OPTIMIZATION
% ADAM GRADIENT DESCENT  (replaces brute-force grid search)
% =========================================================================
% HOW IT WORKS:
%   Each iteration runs 3 sims:
%     f0 = RMSE at current (Kp, Ki)
%     f1 = RMSE at (Kp + small step, Ki)   <- measures Kp gradient
%     f2 = RMSE at (Kp, Ki + small step)   <- measures Ki gradient
%   Adam then takes an adaptive step downhill and repeats.
%   ~14x fewer simulations than a full grid sweep.
% =========================================================================

%% =========================================================================
% USER SETTINGS
% =========================================================================
model_name     = 'Vehicle_Model_999_Modular_roll_gradient';   % <-- CHANGE THIS
stop_time      = 50;
Kff_controller = 0.15;                          % Fixed feedforward gain

%% =========================================================================
% INITIAL GUESS
% Change these if you have a rough idea of good values.
% A poor guess only means more iterations — it will still converge.
% =========================================================================
Kp_init = 10;
Ki_init = 5;

%% =========================================================================
% ADAM OPTIMIZER SETTINGS
% =========================================================================
lr       = 1.0;     % Overall step size. Increase if converging too slowly.
                    % Decrease if RMSE is jumping up and down wildly.
beta1    = 0.90;    % How much "memory" the gradient direction has (0–1).
beta2    = 0.999;   % How much memory the step-size scaling has. Keep as-is.
adam_eps = 1e-8;    % Tiny number to prevent divide-by-zero. Keep as-is.

%% =========================================================================
% FINITE DIFFERENCE STEP SIZES
% These control how much we perturb Kp/Ki to estimate the gradient.
% Rule of thumb: ~2–5% of the expected parameter range.
% =========================================================================
h_kp = 1.0;         % Perturbation for Kp  (Kp lives in [1, 50])
h_ki = 0.05;        % Perturbation for Ki  (Ki lives in [0,  5])

%% =========================================================================
% TERMINATION
% =========================================================================
max_iter = 1000;      % Hard cap on iterations
tol_rmse = 1e-5;    % Stop early if RMSE barely changes between iterations

%% =========================================================================
% PARAMETER BOUNDS  (same as your original sweep range)
% =========================================================================
Kp_min = 1;   Kp_max = 50;
Ki_min = 0;   Ki_max = 50;

%% =========================================================================
% STORAGE  (minimal — only the convergence trace, no giant results matrix)
% =========================================================================
hist_Kp   = nan(max_iter, 1);
hist_Ki   = nan(max_iter, 1);
hist_rmse = nan(max_iter, 1);

%% =========================================================================
% LOAD & CONFIGURE MODEL
% =========================================================================
load_system(model_name);
set_param(model_name, 'StopTime',       num2str(stop_time));
set_param(model_name, 'SimulationMode', 'accelerator');
set_param(model_name, 'FastRestart',    'on');

%% =========================================================================
% BANNER
% =========================================================================
sep = repmat('=', 1, 56);
fprintf('\n%s\n', sep);
fprintf('  PID OPTIMIZER  —  ADAM GRADIENT DESCENT\n');
fprintf('  Model       : %s\n', model_name);
fprintf('  Max iters   : %d    (~%d simulations max)\n', max_iter, max_iter*3);
fprintf('  Init  Kp    : %.2f  |  Ki : %.2f\n', Kp_init, Ki_init);
fprintf('  Bounds  Kp  : [%.0f, %.0f]  |  Ki : [%.0f, %.0f]\n', ...
        Kp_min, Kp_max, Ki_min, Ki_max);
fprintf('%s\n\n', sep);

%% =========================================================================
% INITIALIZE ADAM STATE
% =========================================================================
Kp = Kp_init;
Ki = Ki_init;

m_kp = 0;  m_ki = 0;   % First moment  (running mean of gradient)
v_kp = 0;  v_ki = 0;   % Second moment (running mean of gradient^2)

best_rmse = inf;
best_Kp   = Kp;
best_Ki   = Ki;
prev_rmse = inf;

%% =========================================================================
% MAIN OPTIMIZATION LOOP
% =========================================================================
for iter = 1 : max_iter

    fprintf('\n%s\n', repmat('-',1,56));
    fprintf('  ITERATION  %d / %d\n', iter, max_iter);
    fprintf('  Current  Kp = %.4f  |  Ki = %.4f\n', Kp, Ki);

    % Save the params being evaluated this iteration
    % (Kp, Ki will be updated AFTER gradient step; we need the pre-step
    %  values to correctly log which point gave f0)
    eval_Kp = Kp;
    eval_Ki = Ki;

    % -------------------------------------------------------
    % SIM 1 of 3:  f0 at current (Kp, Ki)
    % -------------------------------------------------------
    fprintf('  [Sim 1/3]  f0 at Kp=%.4f, Ki=%.4f ...\n', eval_Kp, eval_Ki);
    f0 = run_sim(model_name, eval_Kp, eval_Ki, Kff_controller, vel_ts);

    % -------------------------------------------------------
    % SIM 2 of 3:  f1 at (Kp + h_kp, Ki)
    % -------------------------------------------------------
    Kp_p  = min(eval_Kp + h_kp, Kp_max);   % clamp to upper bound
    dKp   = Kp_p - eval_Kp;
    fprintf('  [Sim 2/3]  f1 at Kp=%.4f (+%.2f), Ki=%.4f ...\n', ...
            Kp_p, dKp, eval_Ki);
    f1 = run_sim(model_name, Kp_p, eval_Ki, Kff_controller, vel_ts);

    % -------------------------------------------------------
    % SIM 3 of 3:  f2 at (Kp, Ki + h_ki)
    % -------------------------------------------------------
    Ki_p  = min(eval_Ki + h_ki, Ki_max);    % clamp to upper bound
    dKi   = Ki_p - eval_Ki;
    fprintf('  [Sim 3/3]  f2 at Kp=%.4f, Ki=%.4f (+%.3f) ...\n', ...
            eval_Kp, Ki_p, dKi);
    f2 = run_sim(model_name, eval_Kp, Ki_p, Kff_controller, vel_ts);

    % -------------------------------------------------------
    % Numerical gradients  (forward difference)
    %   grad ≈ (f_perturbed - f_base) / step_size
    % -------------------------------------------------------
    grad_kp = 0;
    grad_ki = 0;
    if dKp > 0,  grad_kp = (f1 - f0) / dKp;  end
    if dKi > 0,  grad_ki = (f2 - f0) / dKi;  end

    fprintf('\n  RMSE   f0=%.6f | f1=%.6f | f2=%.6f  (m/s)\n', f0, f1, f2);
    fprintf('  Grad   dRMSE/dKp=%.6f | dRMSE/dKi=%.6f\n', grad_kp, grad_ki);

    % -------------------------------------------------------
    % Adam parameter update
    % Equations follow the original Adam paper (Kingma & Ba, 2015)
    % -------------------------------------------------------
    % Update biased first and second moment estimates
    m_kp = beta1*m_kp + (1-beta1)*grad_kp;
    m_ki = beta1*m_ki + (1-beta1)*grad_ki;
    v_kp = beta2*v_kp + (1-beta2)*grad_kp^2;
    v_ki = beta2*v_ki + (1-beta2)*grad_ki^2;

    % Bias-corrected estimates (important in early iterations)
    mh_kp = m_kp / (1 - beta1^iter);
    mh_ki = m_ki / (1 - beta1^iter);
    vh_kp = v_kp / (1 - beta2^iter);
    vh_ki = v_ki / (1 - beta2^iter);

    % Compute steps (Adam normalizes by sqrt of variance)
    step_kp = lr * mh_kp / (sqrt(vh_kp) + adam_eps);
    step_ki = lr * mh_ki / (sqrt(vh_ki) + adam_eps);

    fprintf('  Step   Kp: %+.5f | Ki: %+.6f\n', step_kp, step_ki);

    % Move downhill and clamp inside bounds
    Kp = max(Kp_min, min(Kp_max, eval_Kp - step_kp));
    Ki = max(Ki_min, min(Ki_max, eval_Ki - step_ki));

    fprintf('  Next   Kp = %.4f  |  Ki = %.4f\n', Kp, Ki);

    % -------------------------------------------------------
    % Store convergence history (use eval_Kp/Ki + f0)
    % -------------------------------------------------------
    hist_Kp(iter)   = eval_Kp;
    hist_Ki(iter)   = eval_Ki;
    hist_rmse(iter) = f0;

    % -------------------------------------------------------
    % Track global best
    % -------------------------------------------------------
    if f0 < best_rmse
        best_rmse = f0;
        best_Kp   = eval_Kp;
        best_Ki   = eval_Ki;
        fprintf('\n  *** NEW BEST  Kp=%.4f  Ki=%.4f  RMSE=%.5f m/s ***\n', ...
                best_Kp, best_Ki, best_rmse);
    end

    % -------------------------------------------------------
    % Convergence check  (only after a few warmup iterations)
    % -------------------------------------------------------
    if iter > 4
        delta = abs(prev_rmse - f0);
        if delta < tol_rmse
            fprintf('\n  CONVERGED: RMSE change = %.2e < tol %.2e\n', ...
                    delta, tol_rmse);
            break;
        end
    end
    prev_rmse = f0;
end

%% =========================================================================
% FINAL REPORT
% =========================================================================
fprintf('\n\n%s\n', sep);
fprintf('  OPTIMIZATION COMPLETE\n');
fprintf('%s\n', sep);
fprintf('  Best Kp         = %.4f\n', best_Kp);
fprintf('  Best Ki         = %.4f\n', best_Ki);
fprintf('  Min RMSE        = %.5f m/s\n', best_rmse);
fprintf('  Iterations run  = %d\n', iter);
fprintf('  Simulations run ≈ %d  (brute-force would have been %d)\n', ...
        iter*3, 50*51);
fprintf('%s\n\n', sep);

%% =========================================================================
% RESULTS TABLE  (top 10 iterations by RMSE)
% =========================================================================
valid = ~isnan(hist_rmse);
T = table(hist_Kp(valid), hist_Ki(valid), hist_rmse(valid), ...
          'VariableNames', {'Kp','Ki','RMSE_m_s'});
T = sortrows(T, 'RMSE_m_s');
fprintf('===== TOP ITERATIONS BY RMSE =====\n');
disp(T(1:min(10,height(T)), :));

%% =========================================================================
% PLOTS
% =========================================================================
iters_valid = find(valid);
rmse_valid  = hist_rmse(valid);

figure('Color','w','Name','Adam PID Optimizer Results');

% -- Convergence curve --
subplot(2,1,1);
plot(iters_valid, rmse_valid, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 5);
xlabel('Iteration');
ylabel('RMSE (m/s)');
title('RMSE Convergence');
grid on;

% -- Parameter trajectory --
subplot(2,1,2);
scatter(hist_Kp(valid), hist_Ki(valid), 60, rmse_valid, 'filled');
hold on;
plot(hist_Kp(valid), hist_Ki(valid), 'k-', 'LineWidth', 0.8);
plot(hist_Kp(1), hist_Ki(1), 'gs', 'MarkerSize', 10, 'LineWidth', 2);  % start
plot(best_Kp,    best_Ki,    'r*', 'MarkerSize', 12, 'LineWidth', 2);  % best
colorbar;  xlabel('Kp');  ylabel('Ki');
title('Parameter Space Trajectory  (green=start, red=best)');
grid on;

%% =========================================================================
% CLEANUP
% =========================================================================
set_param(model_name, 'FastRestart', 'off');


%% =========================================================================
% LOCAL FUNCTION  —  run one simulation and return RMSE
% Placed at the bottom so the script above can call it.
% Requires MATLAB R2016b or newer.
% =========================================================================
function rmse_val = run_sim(model_name, Kp_val, Ki_val, Kff_val, vel_ts)

    % Push gains into base workspace (where Simulink reads them from)
    assignin('base', 'Kp_controller',  Kp_val);
    assignin('base', 'Ki_controller',  Ki_val);
    assignin('base', 'Kff_controller', Kff_val);

    try
        simOut = sim(model_name);

        % ---- EXTRACT SIGNAL  (3 methods, same as original) ----
        sim_time = [];
        sim_vel  = [];

        % Method 1: Dataset object named 'out'
        if isempty(sim_time)
            try
                out_data   = simOut.get('out');
                vel_signal = out_data.get('Longitudinal_vel');
                sim_time   = vel_signal.Values.Time(:);
                sim_vel    = squeeze(vel_signal.Values.Data);
                sim_vel    = sim_vel(:);
            catch
            end
        end

        % Method 2: Signal directly inside simOut
        if isempty(sim_time)
            try
                vel_signal = simOut.get('Longitudinal_vel');
                sim_time   = vel_signal.Time(:);
                sim_vel    = squeeze(vel_signal.Data);
                sim_vel    = sim_vel(:);
            catch
            end
        end

        % Method 3: Variable saved to workspace by Simulink
        % (uses evalin so this works inside a function scope too)
        if isempty(sim_time)
            try
                lv       = evalin('base', 'Longitudinal_vel');
                sim_time = lv.Time(:);
                sim_vel  = squeeze(lv.Data);
                sim_vel  = sim_vel(:);
            catch
            end
        end

        if isempty(sim_time)
            error('Could not extract Longitudinal_vel by any of the 3 methods.');
        end

        % ---- SYNC TARGET TO SIM TIMEBASE ----
        tgt_time = vel_ts.Time(:);
        tgt_vel  = vel_ts.Data(:);
        vel_ref  = interp1(tgt_time, tgt_vel, sim_time, 'linear', 'extrap');

        % ---- RMSE ----
        err      = vel_ref - sim_vel;
        rmse_val = sqrt(mean(err.^2, 'omitnan'));

        % Free simOut from memory immediately
        clear simOut;

    catch ME
        fprintf('    [!] Simulation failed: %s\n', ME.message);
        rmse_val = inf;
    end
end