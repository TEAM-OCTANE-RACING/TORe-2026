% =========================================================================
% MASTER SENSITIVITY ANALYSIS SCRIPT 
% Purpose: OFAT Sensitivity Sweep & Heatmap for 22-DOF Suspension Setup
% Metric: Maximum Roll Angle Sensitivity
% =========================================================================
% clear; clc; close all;
clc
Mode = 0;

%% --- 1. INITIALIZATION & SETUP ---
fprintf('=== INITIALIZING SENSITIVITY ANALYSIS ===\n');

% Load Base Parameters from Workspace
try
    sus_par_base = evalin('base', 'sus_par');
catch
    error('Could not find "sus_par" in the base workspace. Please load it first.');
end
try
    par_base = evalin('base', 'Par');
catch
    error('Could not find "Par" in the base workspace. Please load it first.');
end

modelName  = 'Vehicle_Model_999_Modular_roll_gradient';
track_file = 'C:\Disk_E\TOR_controls\Modular_vehicle_model_23_May\Kari_FB.csv'; 

if ~bdIsLoaded(modelName)
    load_system(modelName); 
end

% Pre-compute racing line ONCE
fprintf('Pre-computing optimal racing line...\n');
[path_s, path_k, path_X, path_Y] = Optimize_Racing_Line(track_file);
fprintf('Racing line optimized.\n\n');

%% --- 2. DEFINE PARAMETER SWEEP SPACE ---
% Parameters: Ks_f, Ks_r, K_arb_f, K_arb_r, Camber_static, C_damp_f, C_damp_r
param_names = {'Ks_f', 'Ks_r', 'K_arb_f', 'K_arb_r', 'Camber_static', 'C_damp_f', 'C_damp_r'};
num_params  = length(param_names);

% Create a cell array holding the 3 test values [Low, Base, High] for each parameter
sweep_vals = cell(num_params, 1);
sweep_vals{1} = [sus_par_base.Ks_f * 0.7,      sus_par_base.Ks_f,      sus_par_base.Ks_f * 1.3];
sweep_vals{2} = [sus_par_base.Ks_r * 0.7,      sus_par_base.Ks_r,      sus_par_base.Ks_r * 1.3];
sweep_vals{3} = [sus_par_base.K_arb_f * 0.7,   sus_par_base.K_arb_f,   sus_par_base.K_arb_f * 1.3];
sweep_vals{4} = [sus_par_base.K_arb_r * 0.7,   sus_par_base.K_arb_r,   sus_par_base.K_arb_r * 1.3];
sweep_vals{5} = [-4.0,                         sus_par_base.Camber_static, 2.0]; % Degrees
% sweep_vals{6} = [sus_par_base.C_damp_f * 0.7,  sus_par_base.C_damp_f,  sus_par_base.C_damp_f * 1.3];
% sweep_vals{7} = [sus_par_base.C_damp_r * 0.7,  sus_par_base.C_damp_r,  sus_par_base.C_damp_r * 1.3];

% Storage for results (7 parameters x 3 levels)
roll_matrix = zeros(num_params, 3);
pct_change_matrix = zeros(num_params, 3);

%% --- 3. ESTABLISH BASELINE ---
fprintf('=== RUNNING BASELINE SETUP ===\n');

% Safely convert baseline static camber to radians for the model
sus_par_base_model = sus_par_base;
par_base_model = par_base;

if isfield(sus_par_base_model, 'Camber_static')
    sus_par_base_model.Camber_static = deg2rad(sus_par_base.Camber_static);
end
if isfield(par_base_model, 'Camber_static')
    par_base_model.Camber_static = deg2rad(par_base.Camber_static);
end

% Run the baseline setup and pass the structs explicitly
baseline_results = Generate_GGV_Map(modelName, sus_par_base_model, par_base_model);

% --- GENERATE 3D SURFACE DATA (OPTIMIZED) ---
theta = linspace(0, 2*pi, 60); 

% 1. Determine dimensions
n_speeds = length(baseline_results.v);
n_angles = length(theta);

% 2. Pre-allocate the grids
V_grid  = zeros(n_speeds, n_angles);
Ax_grid = zeros(n_speeds, n_angles);
Ay_grid = zeros(n_speeds, n_angles);

% 3. Fill the grids
for i = 1:n_speeds
    v_slice = baseline_results.v(i);
    ax_pos = baseline_results.ax_max(i);
    ax_neg = baseline_results.ax_brk(i); 
    ay_lim = baseline_results.ay_max(i);
    
    for k = 1:n_angles
        th = theta(k);
        % Lateral (Cosine)
        y_val = ay_lim * cos(th);
        % Longitudinal (Sine) - Asymmetric blending
        sin_val = sin(th);
        if sin_val >= 0
            x_val = ax_pos * sin_val; % Forward Accel
        else
            x_val = abs(ax_neg) * sin_val; % Braking
        end
        Ax_grid(i, k) = x_val;
        Ay_grid(i, k) = y_val;
        V_grid(i, k)  = v_slice;
    end
end

% --- PLOTTING ---
figure('Name', 'Master GGV Analysis', 'Color', 'w', 'Position', [100 100 1200 600]);
subplot(1, 2, 1);
surf(Ax_grid, Ay_grid, V_grid, 'FaceAlpha', 0.5, 'EdgeColor', 'interp');
colormap jet; colorbar;
xlabel('Longitudinal G'); ylabel('Lateral G'); zlabel('Speed (m/s)');
title('3D Performance Envelope');
grid on; axis equal; view(135, 30);

subplot(1, 2, 2);
plot(baseline_results.v, baseline_results.ay_max, 'b-o', 'LineWidth', 2); hold on;
plot(baseline_results.v, baseline_results.ax_max, 'g-o', 'LineWidth', 2);
plot(baseline_results.v, baseline_results.ax_brk, 'r-o', 'LineWidth', 2);
ylabel('Acceleration (G)'); xlabel('Speed (m/s)');
legend('Max Lateral', 'Max Accel', 'Max Brake');
title('2D Limits vs Speed');
grid on;

baseline_env     = Process_GGV_Envelope(baseline_results);
baseline_lap     = Solve_3Pass(path_s, path_k, baseline_env);

base_time = baseline_lap.laptime;
base_roll = baseline_results.max_roll;
fprintf('-> BASELINE LAP TIME: %.6f seconds\n', base_time);
fprintf('-> BASELINE MAX ROLL: %.6f\n\n', base_roll);

% Pre-fill the baseline column (index 2) in our matrices
roll_matrix(:, 2) = base_roll;
pct_change_matrix(:, 2) = 0.0; % 0% change for baseline

%% --- 4. SENSITIVITY LOOP (OFAT) ---
fprintf('=== STARTING SENSITIVITY SWEEPS ===\n');

for p = 1:num_params
    p_name = param_names{p};
    fprintf('\n--- Sweeping Parameter %d/%d: %s ---\n', p, num_params, p_name);
    
    for level = [1, 3] % Test only Low (1) and High (3) [Base (2) is already done]
        
        % 1. Create a fresh copy of the FULL base parameters
        sus_par_test = sus_par_base;
        Par_test = par_base;
        
        % 2. Get the modification value (Display value)
        test_val = sweep_vals{p}(level);
        
        % 3. Check if parameter is Camber_static, then convert to radians for model
        if strcmp(p_name, 'Camber_static')
            model_test_val = deg2rad(test_val); 
        else
            model_test_val = test_val;
        end
        
        % 4. Update BOTH structs to ensure the parameter actually changes
        if isfield(sus_par_test, p_name)
            sus_par_test.(p_name) = model_test_val;
        end
        if isfield(Par_test, p_name)
            Par_test.(p_name) = model_test_val;
        end
        
        if level == 1
            lvl_str = 'LOW (-30% / -4deg)';
        else
            lvl_str = 'HIGH (+30% / +2deg)';
        end
        fprintf('Testing %s = %.6f (%s)...\n', p_name, test_val, lvl_str);
        
        % 5. Push the FULL updated structs to the base workspace (for workspace clarity/debugging)
        assignin('base', 'sus_par', sus_par_test);
        assignin('base', 'Par', Par_test);
        
        % 6. Generate GGV Map explicitly passing local structs to sandbox
        ggv_results = Generate_GGV_Map(modelName, sus_par_test, Par_test);
        
        % 7. Process Envelope and Calculate Lap Time (Optional, kept for log)
        ggv_env = Process_GGV_Envelope(ggv_results);
        lap_res = Solve_3Pass(path_s, path_k, ggv_env);
        
        % 8. Store & Calculate % Change for Max Roll
        l_roll = ggv_results.max_roll;
        roll_matrix(p, level) = l_roll;
        
        % Negative value means LESS roll (Car is stiffer)
        pct_change = ((l_roll - base_roll) / base_roll) * 100;
        pct_change_matrix(p, level) = pct_change;
        
        fprintf('-> Max Roll: %.6f | Lap Time: %.6f s | Change (Roll): %+.6f %%\n', l_roll, lap_res.laptime, pct_change);
    end
end
fprintf('\n=== SENSITIVITY ANALYSIS COMPLETE ===\n');

%% --- 5. CLEANUP (RESTORE ORIGINAL WORKSPACE) ---
assignin('base', 'sus_par', sus_par_base);
assignin('base', 'Par', par_base);
fprintf('Restored original base workspace parameters.\n');

%% --- 6. GENERATE HEATMAP VISUALIZATION ---
figure('Name', 'Suspension Sensitivity Heatmap', 'Color', 'w', 'Position', [200 150 900 650]);

% Format labels
x_labels = {'Low (-30% / -4°)', 'Baseline', 'High (+30% / +2°)'};
y_labels = {'Front Spring (Ks_f)', 'Rear Spring (Ks_r)', ...
            'Front ARB (K_arb_f)', 'Rear ARB (K_arb_r)', ...
            'Static Camber', 'Front Damper (C_damp_f)', 'Rear Damper (C_damp_r)'};

% Create the heatmap
h = heatmap(x_labels, y_labels, pct_change_matrix);

% Styling the Heatmap to look cooler
h.Title = sprintf('Max Roll Angle Sensitivity\n(Negative %% = Less Roll | Base Roll: %.6f)', base_roll);
h.XLabel = 'Parameter Variation';
h.YLabel = 'Suspension Parameter';
h.CellLabelFormat = '%+.4f %%';

% Visual Polish: Turbo is highly vibrant, good for distinguishing structural variations
h.Colormap = turbo; 
h.FontSize = 11;
h.FontName = 'Helvetica';

% Equalize the color limits around zero so diverging changes stand out
max_val = max(abs(pct_change_matrix(:)));
if max_val > 0
    h.ColorLimits = [-max_val-0.1, max_val+0.1];
end

% Optional: Save the matrix data to base workspace for future review
assignin('base', 'Sensitivity_Pct_Matrix', pct_change_matrix);
assignin('base', 'Sensitivity_Roll_Matrix', roll_matrix);

% =========================================================================
% LOCAL FUNCTIONS 
% =========================================================================
function results = Generate_GGV_Map(modelName, sus_par_local, Par_local)
    % Runs the core GGV generation without plotting
    velocities = 1:5:21; 
    results = struct('v', [], 'ay_max', [], 'ax_max', [], 'ax_brk', [], 'max_roll', 0);
    overall_max_roll = 0;
    
    % Fetch current wheelbase explicitly from passed struct
    try
        current_WB = Par_local.WB;
    catch
        current_WB = 1.532; % Fallback
    end
    
    for i = 1:length(velocities)
        v_target = velocities(i);
        baseSimIn = Simulink.SimulationInput(modelName);
        
        baseSimIn = baseSimIn.setModelParameter('SimulationMode', 'accelerator');
        baseSimIn = baseSimIn.setModelParameter('StopTime', '2.5'); 
        baseSimIn = baseSimIn.setModelParameter('Solver', 'ode23s'); 
        
        % CRITICAL FIX: Push the local structs into the simulation sandbox
        baseSimIn = baseSimIn.setVariable('sus_par', sus_par_local);
        baseSimIn = baseSimIn.setVariable('Par', Par_local);
        
        baseSimIn = baseSimIn.setVariable('init_vx', v_target);
        baseSimIn = baseSimIn.setVariable('init_vy', 0);
        baseSimIn = baseSimIn.setVariable('init_r', 0);
        
        % Braking
        simIn = baseSimIn.setVariable('Steer_Val', 0).setVariable('Throttle_Val', 0).setVariable('Brake_Val', 1.0); 
        [ax_brk, roll_b] = Get_Peak_Explicit(simIn, v_target, 'brake', 0, current_WB);
        overall_max_roll = max(overall_max_roll, roll_b);
        
        % Accel
        simIn = baseSimIn.setVariable('Steer_Val', 0).setVariable('Throttle_Val', 1.0).setVariable('Brake_Val', 0);
        [ax_acc, roll_a] = Get_Peak_Explicit(simIn, v_target, 'accel', 0, current_WB);
        overall_max_roll = max(overall_max_roll, roll_a);
        
        % Lateral Sweep
        best_ay = 0; drop_count = 0; 
        for steer = 0.04:0.04:0.6
            simIn = baseSimIn.setVariable('Steer_Val', steer).setVariable('Throttle_Val', 0.5).setVariable('Brake_Val', 0);
            [ay_curr, roll_l] = Get_Peak_Explicit(simIn, v_target, 'lat', steer, current_WB);
            overall_max_roll = max(overall_max_roll, roll_l);
            
            if ay_curr > best_ay
                best_ay = ay_curr;
                drop_count = 0;
            elseif ay_curr < (best_ay * 0.95)
                drop_count = drop_count + 1;
                if drop_count >= 2, break; end
            end
        end
        
        % Store Results in Gs
        results.v(i)      = v_target;
        results.ax_max(i) = ax_acc / 9.81;
        results.ax_brk(i) = ax_brk / 9.81; 
        results.ay_max(i) = best_ay / 9.81;
    end
    
    results.max_roll = overall_max_roll;
end

function [peak_val, peak_roll] = Get_Peak_Explicit(simIn, target_v, mode, steer_angle, wheelbase)
    peak_val = 0; peak_yaw = 0; peak_roll = 0;
    try
        warning('off', 'Simulink:Commands:SimulatingWithAccelMode');
        out = sim(simIn);
        
        try t = out.tout; catch, t = out.get('tout'); end
        try vx = out.Longitudinal_vel.Data; catch, try vx = out.get('vx').Data; catch, vx = target_v*ones(size(t)); end, end
        try yaw = out.Yaw_rate.Data; catch, try yaw = out.yawrate.Data; catch, yaw = zeros(size(t)); end, end
        
        % Extract Roll Angle explicitly
        try 
            roll = out.roll_angle.Data; 
        catch 
            try 
                roll = out.get('roll_angle').Data; 
            catch 
                try 
                    roll = out.roll_angle; % Fallback if it is not inside a Dataset
                catch 
                    roll = zeros(size(t)); 
                end
            end
        end
        
        if strcmp(mode, 'lat')
            try acc = abs(out.Lateral_accln_sensor.Data); catch, try acc = abs(out.ay.Data); catch, acc = zeros(size(t)); end, end
        else
            try acc = out.Longitudinal_accln_sensor.Data; catch, try acc = out.ax.Data; catch, acc = zeros(size(t)); end, end
        end
        
        switch mode
            case 'accel', peak_val = -999; 
            case 'brake', peak_val = 999;  
            case 'lat',   peak_val = 0;    
        end
        
        valid_points_found = false;
        peak_roll_temp = 0;
        
        for k = 1:length(t)
            if vx(k) < (target_v * 0.3), continue; end 
            r_theo = (vx(k) * tan(steer_angle)) / wheelbase;
            r_limit = max(1.0, abs(r_theo)*4.0 + 0.5);
            
            if abs(yaw(k)) > r_limit, continue; end
            if abs(acc(k)) > 45, continue; end 
            
            val = acc(k);
            valid_points_found = true;
            
            % Track Max Roll on valid points
            curr_roll = abs(roll(k));
            if curr_roll > peak_roll_temp
                peak_roll_temp = curr_roll;
            end
            
            switch mode
                case 'accel', if val > peak_val, peak_val = val; end
                case 'brake', if val < peak_val, peak_val = val; end
                case 'lat',   if val > peak_val, peak_val = val; end
            end
        end
        
        peak_roll = peak_roll_temp; % Pass temporary max back to function output
        
        if ~valid_points_found || peak_val == -999 || peak_val == 999
            peak_val = 0; 
        end
    catch ME
        fprintf('\n!!! SIMULATION FAILED !!!\nError: %s\n', ME.message);
        peak_val = 0;
        peak_roll = 0;
    end
end

function [s_final, k_final, x_final, y_final] = Optimize_Racing_Line(filename)
    try
        data = readmatrix(filename);
        if size(data, 2) < 4
             opts = detectImportOptions(filename);
             opts.DataLines = [2 Inf];
             data = readmatrix(filename, opts);
        end
    catch
        error('Could not read track file. Make sure it is the 4-column CSV format.');
    end
    
    xc = data(:,1); yc = data(:,2); wl = data(:,3); wr = data(:,4);
    if norm([xc(1)-xc(end), yc(1)-yc(end)]) > 0.1
        xc(end+1)=xc(1); yc(end+1)=yc(1); wl(end+1)=wl(1); wr(end+1)=wr(1);
    end
    N = length(xc); x = xc; y = yc; 
    Iterations = 200; Safety_Margin = 0.5;
    
    for iter = 1:Iterations
        xt = 0.5 * ([x(end-1); x(1:end-1)] + [x(2:end); x(2)]);
        yt = 0.5 * ([y(end-1); y(1:end-1)] + [y(2:end); y(2)]);
        for i = 1:N
            dx = xt(i) - xc(i); dy = yt(i) - yc(i); dist = sqrt(dx^2 + dy^2);
            if i < N, tx = xc(i+1)-xc(i); ty = yc(i+1)-yc(i); else, tx = xc(2)-xc(1); ty = yc(2)-yc(1); end
            if (tx*dy - ty*dx) > 0, max_w = wl(i)-Safety_Margin; else, max_w = wr(i)-Safety_Margin; end
            if max_w < 0.1, max_w = 0.1; end
            if dist > max_w, scale = max_w / dist; x(i) = xc(i) + dx * scale; y(i) = yc(i) + dy * scale;
            else, x(i) = xt(i); y(i) = yt(i); end
        end
    end
    dx = gradient(x); dy = gradient(y); ddx = gradient(dx); ddy = gradient(dy);
    k_raw = (dx .* ddy - dy .* ddx) ./ ((dx.^2 + dy.^2).^1.5);
    k_final = smoothdata(k_raw, 'gaussian', 15);
    ds = sqrt(dx.^2 + dy.^2); s_final = [0; cumsum(ds(2:end))];
    x_final = x; y_final = y;
end

function Env = Process_GGV_Envelope(results)
    Env.v = results.v(:); 
    Env.ay_max = abs(results.ay_max(:)) * 9.81;
    Env.ax_acc = results.ax_max(:) * 9.81;
    Env.ax_brk = abs(results.ax_brk(:)) * 9.81; 
    [Env.v, idx] = sort(Env.v);
    Env.ay_max = Env.ay_max(idx); Env.ax_acc = Env.ax_acc(idx); Env.ax_brk = Env.ax_brk(idx);
end

function R = Solve_3Pass(s, k, Env)
    N = length(s);
    v_apex = zeros(N, 1); v_acc = zeros(N, 1); ax_acc_log = zeros(N, 1);
    v_brk = zeros(N, 1); ax_brk_log = zeros(N, 1);
    
    for i = 1:N
        curv = abs(k(i));
        if curv < 1e-4, v_apex(i) = max(Env.v);
        else
            v_g = 12; 
            for iter=1:15
                ay_lim = interp1(real(Env.v), Env.ay_max, real(v_g), 'linear', 'extrap');
                v_new = sqrt(ay_lim / curv);
                v_g = 0.6*v_g + 0.4*v_new;
            end
            v_apex(i) = min(21.4, v_g);
        end
    end
    
    v_acc(1) = 0;
    for i = 1:N-1
        ds = s(i+1) - s(i); v_curr = v_acc(i);
        ay_req = v_curr^2 * abs(k(i));
        ay_max_v = interp1(Env.v, Env.ay_max, v_curr, 'linear', 'extrap');
        ax_max_v = interp1(Env.v, Env.ax_acc, v_curr, 'linear', 'extrap');
        if ay_req >= ay_max_v, ax_avail = 0; else, ax_avail = ax_max_v * sqrt(1 - (ay_req/ay_max_v)^2); end
        ax_acc_log(i) = ax_avail; 
        v_next = sqrt(v_curr^2 + 2 * ax_avail * ds);
        v_acc(i+1) = min(v_next, v_apex(i+1));
    end
    ax_acc_log(N) = 0; 
    
    v_brk(N) = v_apex(N);
    for i = N:-1:2
        ds = s(i) - s(i-1); v_curr = v_brk(i);
        ay_req = v_curr^2 * abs(k(i));
        ay_max_v = interp1(Env.v, Env.ay_max, v_curr, 'linear', 'extrap');
        ax_brk_v = abs(interp1(Env.v, Env.ax_brk, v_curr, 'linear', 'extrap'));
        if ay_req >= ay_max_v, ax_avail = 0; else, ax_avail = ax_brk_v * sqrt(1 - (ay_req/ay_max_v)^2); end
        ax_brk_log(i) = -ax_avail;
        v_prev = sqrt(v_curr^2 + 2 * ax_avail * ds);
        v_brk(i-1) = min(v_prev, v_apex(i-1));
    end
    ax_brk_log(1) = -abs(interp1(Env.v, Env.ax_brk, v_brk(1), 'linear', 'extrap'));
    
    v_final = zeros(N, 1); ax_final = zeros(N, 1);
    for i = 1:N
        [v_lim, idx] = min([v_acc(i), v_brk(i), v_apex(i)]);
        v_final(i) = v_lim;
        if idx == 1, ax_final(i) = ax_acc_log(i);
        elseif idx == 2, ax_final(i) = ax_brk_log(i);
        else, ax_final(i) = 0; end
    end
    
    v_avg = max((v_final(1:end-1) + v_final(2:end)) / 2, 0.001); 
    dt = diff(s) ./ v_avg;
    time_array = [0; cumsum(dt)]; 
    
    R.s = s; R.time = time_array; R.v = v_final; 
    R.ay_used = v_final.^2 .* k; R.ax_used = ax_final; R.laptime = time_array(end);
end
Mode = 1;