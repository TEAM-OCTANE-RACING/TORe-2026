% =========================================================================
% SPATIAL LONGITUDINAL DRIVER PID OPTIMIZATION (ADAM)
% Minimizes velocity error mapped against Distance (S), avoiding lockup.
% =========================================================================

%% =========================================================================
% USER SETTINGS
% =========================================================================
model_name     = 'Vehicle_Model_999_Modular_roll_gradient';   % <-- CHANGE THIS
stop_time      = 27.5;
Kff_controller = 0.15;              % Fixed feedforward gain

% --- NEW: THE KICKSTART FLOOR ---
% Prevents the "Zero-Velocity Lockup" at the starting line.
min_start_vel  = 1.5;               % m/s (approx 5.4 km/h)

%% =========================================================================
% INITIAL GUESS
% =========================================================================
Kp_init = 10;
Ki_init = 5;

%% =========================================================================
% ADAM OPTIMIZER SETTINGS & FINITE DIFFERENCES
% =========================================================================
lr       = 1.0;     
beta1    = 0.90;    
beta2    = 0.999;   
adam_eps = 1e-8;    

h_kp = 1.0;         % Perturbation for Kp
h_ki = 0.05;        % Perturbation for Ki

max_iter = 1000;    
tol_rmse = 1e-5;    

Kp_min = 1;   Kp_max = 50;
Ki_min = 0;   Ki_max = 50;

%% =========================================================================
% PREPARE SPATIAL TARGET DATA
% =========================================================================
% Pull the target arrays directly from the timeseries objects in base workspace
tgt_dist = evalin('base', 'dist_ts.Data(:)');
tgt_vel  = evalin('base', 'vel_ts.Data(:)');

% APPLY KICKSTART FLOOR: Force all velocities to be at least min_start_vel
tgt_vel = max(tgt_vel, min_start_vel);

%% =========================================================================
% STORAGE & MODEL CONFIG
% =========================================================================
hist_Kp   = nan(max_iter, 1);
hist_Ki   = nan(max_iter, 1);
hist_rmse = nan(max_iter, 1);

load_system(model_name);
set_param(model_name, 'StopTime',       num2str(stop_time));
set_param(model_name, 'SimulationMode', 'accelerator');
set_param(model_name, 'FastRestart',    'on');

%% =========================================================================
% BANNER
% =========================================================================
sep = repmat('=', 1, 56);
fprintf('\n%s\n', sep);
fprintf('  SPATIAL PID OPTIMIZER  —  ADAM GRADIENT DESCENT\n');
fprintf('  Model       : %s\n', model_name);
fprintf('  Init  Kp    : %.2f  |  Ki : %.2f\n', Kp_init, Ki_init);
fprintf('  Kickstart V : %.2f m/s\n', min_start_vel);
fprintf('%s\n\n', sep);

%% =========================================================================
% INITIALIZE ADAM STATE
% =========================================================================
Kp = Kp_init;
Ki = Ki_init;
m_kp = 0;  m_ki = 0;   
v_kp = 0;  v_ki = 0;   
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
    
    eval_Kp = Kp;
    eval_Ki = Ki;

    % SIM 1 of 3:  f0
    fprintf('  [Sim 1/3]  f0 at Kp=%.4f, Ki=%.4f ...\n', eval_Kp, eval_Ki);
    f0 = run_spatial_sim(model_name, eval_Kp, eval_Ki, Kff_controller, tgt_dist, tgt_vel);

    % SIM 2 of 3:  f1 (Kp gradient)
    Kp_p  = min(eval_Kp + h_kp, Kp_max);   
    dKp   = Kp_p - eval_Kp;
    fprintf('  [Sim 2/3]  f1 at Kp=%.4f (+%.2f), Ki=%.4f ...\n', Kp_p, dKp, eval_Ki);
    f1 = run_spatial_sim(model_name, Kp_p, eval_Ki, Kff_controller, tgt_dist, tgt_vel);

    % SIM 3 of 3:  f2 (Ki gradient)
    Ki_p  = min(eval_Ki + h_ki, Ki_max);    
    dKi   = Ki_p - eval_Ki;
    fprintf('  [Sim 3/3]  f2 at Kp=%.4f, Ki=%.4f (+%.3f) ...\n', eval_Kp, Ki_p, dKi);
    f2 = run_spatial_sim(model_name, eval_Kp, Ki_p, Kff_controller, tgt_dist, tgt_vel);

    % Numerical gradients
    grad_kp = 0;
    grad_ki = 0;
    if dKp > 0,  grad_kp = (f1 - f0) / dKp;  end
    if dKi > 0,  grad_ki = (f2 - f0) / dKi;  end
    
    fprintf('\n  RMSE   f0=%.6f | f1=%.6f | f2=%.6f  (m/s)\n', f0, f1, f2);
    fprintf('  Grad   dRMSE/dKp=%.6f | dRMSE/dKi=%.6f\n', grad_kp, grad_ki);

    % Adam update
    m_kp = beta1*m_kp + (1-beta1)*grad_kp;
    m_ki = beta1*m_ki + (1-beta1)*grad_ki;
    v_kp = beta2*v_kp + (1-beta2)*grad_kp^2;
    v_ki = beta2*v_ki + (1-beta2)*grad_ki^2;
    
    mh_kp = m_kp / (1 - beta1^iter);
    mh_ki = m_ki / (1 - beta1^iter);
    vh_kp = v_kp / (1 - beta2^iter);
    vh_ki = v_ki / (1 - beta2^iter);
    
    step_kp = lr * mh_kp / (sqrt(vh_kp) + adam_eps);
    step_ki = lr * mh_ki / (sqrt(vh_ki) + adam_eps);
    
    Kp = max(Kp_min, min(Kp_max, eval_Kp - step_kp));
    Ki = max(Ki_min, min(Ki_max, eval_Ki - step_ki));
    
    hist_Kp(iter)   = eval_Kp;
    hist_Ki(iter)   = eval_Ki;
    hist_rmse(iter) = f0;

    % Track global best
    if f0 < best_rmse
        best_rmse = f0;
        best_Kp   = eval_Kp;
        best_Ki   = eval_Ki;
        fprintf('\n  *** NEW BEST  Kp=%.4f  Ki=%.4f  RMSE=%.5f m/s ***\n', best_Kp, best_Ki, best_rmse);
    end

    % Convergence check
    if iter > 4
        delta = abs(prev_rmse - f0);
        if delta < tol_rmse
            fprintf('\n  CONVERGED: RMSE change = %.2e < tol %.2e\n', delta, tol_rmse);
            break;
        end
    end
    prev_rmse = f0;
end

%% =========================================================================
% CLEANUP & FINAL REPORT
% =========================================================================
set_param(model_name, 'FastRestart', 'off');

fprintf('\n\n%s\n', sep);
fprintf('  SPATIAL OPTIMIZATION COMPLETE\n');
fprintf('%s\n', sep);
fprintf('  Best Kp         = %.4f\n', best_Kp);
fprintf('  Best Ki         = %.4f\n', best_Ki);
fprintf('  Min RMSE        = %.5f m/s\n', best_rmse);
fprintf('%s\n\n', sep);

%% =========================================================================
% PLOTS
% =========================================================================
valid = ~isnan(hist_rmse);
iters_valid = find(valid);
rmse_valid  = hist_rmse(valid);

figure('Color','w','Name','Spatial Adam PID Optimizer Results');
subplot(2,1,1);
plot(iters_valid, rmse_valid, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 5);
xlabel('Iteration'); ylabel('Spatial RMSE (m/s)');
title('RMSE Convergence'); grid on;

subplot(2,1,2);
scatter(hist_Kp(valid), hist_Ki(valid), 60, rmse_valid, 'filled'); hold on;
plot(hist_Kp(valid), hist_Ki(valid), 'k-', 'LineWidth', 0.8);
plot(hist_Kp(1), hist_Ki(1), 'gs', 'MarkerSize', 10, 'LineWidth', 2); 
plot(best_Kp,    best_Ki,    'r*', 'MarkerSize', 12, 'LineWidth', 2);  
colorbar;  xlabel('Kp');  ylabel('Ki');
title('Parameter Space Trajectory (green=start, red=best)'); grid on;

%% =========================================================================
% LOCAL FUNCTION — RUN SIM AND CALCULATE SPATIAL RMSE
% =========================================================================
function rmse_val = run_spatial_sim(model_name, Kp_val, Ki_val, Kff_val, tgt_dist, tgt_vel)
    assignin('base', 'Kp_controller',  Kp_val);
    assignin('base', 'Ki_controller',  Ki_val);
    assignin('base', 'Kff_controller', Kff_val);
    
    try
        simOut = sim(model_name);
        
        sim_time = []; sim_vel  = [];
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
        if isempty(sim_time)
            try
                vel_signal = simOut.get('Longitudinal_vel');
                sim_time   = vel_signal.Time(:);
                sim_vel    = squeeze(vel_signal.Data);
                sim_vel    = sim_vel(:);
            catch
            end
        end
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
            error('Could not extract Longitudinal_vel.');
        end
        
        % ---- NEW: CALCULATE ACTUAL DISTANCE DYNAMICALLY ----
        sim_dist = cumtrapz(sim_time, sim_vel);
        
        % ---- NEW: INTERPOLATE TARGET TO SIMULATED DISTANCE ----
        % The target array already has the kickstart floor applied
        vel_ref_spatial = interp1(tgt_dist, tgt_vel, sim_dist, 'linear', 'extrap');
        
        % ---- CALCULATE RMSE ----
        err = vel_ref_spatial - sim_vel;
        rmse_val = sqrt(mean(err.^2, 'omitnan'));
        
        clear simOut;
    catch ME
        fprintf('    [!] Simulation failed: %s\n', ME.message);
        rmse_val = inf;
    end
end