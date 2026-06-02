% SCRIPT: RunGGV_Master_3D.m
% Purpose: Final GGV Generator with 3D Surface & Explicit Peak Logic.
% Features:
%   - Uses 'ax_sensor' and 'ay_sensor' (Physically Correct).
%   - Captures Transients (Turn-in/Braking spikes).
%   - Explicit Mode: Prevents Drag from confusing Accel/Brake detection.
%   - Generates 3D Performance Tube.
%   - OPTIMIZED FOR SPEED (Accelerator Mode & Early Break)
Mode = 0;
% clc;
% --- 1. SETUP ---
modelName = 'Vehicle_Model_999_Modular_roll_gradient'; 
if ~bdIsLoaded(modelName), load_system(modelName); end

% Velocity Sweep (m/s)
velocities = 1:2:21; 

% Initialize Storage
results = struct('v', [], 'ay_max', [], 'ax_max', [], 'ax_brk', []);

fprintf('=== STARTING GGV SIMULATION (MASTER MODE - FAST) ===\n');

for i = 1:length(velocities)
    v_target = velocities(i);
    fprintf('\n--- Testing V = %.1f m/s ---\n', v_target);
    
    baseSimIn = Simulink.SimulationInput(modelName);
    
    % SPEED UP 1 & 2: Accelerator mode and reduced StopTime
    baseSimIn = baseSimIn.setModelParameter('SimulationMode', 'accelerator');
    baseSimIn = baseSimIn.setModelParameter('StopTime', '5'); 
    baseSimIn = baseSimIn.setModelParameter('Solver', 'ode23s'); 
    
    baseSimIn = baseSimIn.setVariable('init_vx', v_target);
    baseSimIn = baseSimIn.setVariable('init_vy', 0);
    baseSimIn = baseSimIn.setVariable('init_r', 0);
    
    % Load Parameters
    if exist('Par', 'var')
        baseSimIn = baseSimIn.setVariable('Par', Par);
        current_WB = Par.WB;
    else
        error('Par struct not found. Please load parameters.');
    end
    
    % --- TEST 1: MAX BRAKING (Explicit 'brake' mode) ---
    simIn = baseSimIn;
    simIn = simIn.setVariable('Steer_Val', 0);
    simIn = simIn.setVariable('Throttle_Val', 0);
    simIn = simIn.setVariable('Brake_Val', 1.0); 
    
    [ax_brk, ~] = Get_Peak_Explicit(simIn, v_target, 'brake', 0, current_WB);
    
    % --- TEST 2: MAX ACCEL (Explicit 'accel' mode) ---
    simIn = baseSimIn;
    simIn = simIn.setVariable('Steer_Val', 0);
    simIn = simIn.setVariable('Throttle_Val', 1.0); 
    simIn = simIn.setVariable('Brake_Val', 0);
    
    [ax_acc, ~] = Get_Peak_Explicit(simIn, v_target, 'accel', 0, current_WB);
    
    % --- TEST 3: MAX LATERAL (Explicit 'lat' mode) ---
    best_ay = 0;
    drop_count = 0; % Track consecutive grip drops
    
    % Optimization: Ramp steer to find limit 
    for steer = 0.04:0.04:0.6
        simIn = baseSimIn;
        simIn = simIn.setVariable('Steer_Val', steer);
        simIn = simIn.setVariable('Throttle_Val', 0.5); 
        simIn = simIn.setVariable('Brake_Val', 0);
        
        [ay_curr, ~] = Get_Peak_Explicit(simIn, v_target, 'lat', steer, current_WB);
        
        % SPEED UP 3: Robust Early Break Logic
        if ay_curr > best_ay
            best_ay = ay_curr;
            drop_count = 0; % Reset counter since we found a new peak
        elseif ay_curr < (best_ay * 0.95) % If grip drops by more than 5%
            drop_count = drop_count + 1;
            if drop_count >= 2 % Require two consecutive drops to prevent false stops
                fprintf('    (Peak Lat found, breaking early at steer = %.2f)\n', steer);
                break; 
            end
        end
    end
    
    % Store Results (Convert to Gs)
    results.v(i)      = v_target;
    results.ax_max(i) = ax_acc / 9.81;
    results.ax_brk(i) = ax_brk / 9.81; % Should be negative
    results.ay_max(i) = best_ay / 9.81;
    
    fprintf('  Peak Limits: %.2fG (Acc) / %.2fG (Brk) / %.2fG (Lat)\n', ...
        results.ax_max(i), results.ax_brk(i), results.ay_max(i));
end

% --- 2. GENERATE 3D SURFACE DATA (OPTIMIZED) ---
theta = linspace(0, 2*pi, 60); 

% 1. Determine dimensions
n_speeds = length(results.v);
n_angles = length(theta);

% 2. Pre-allocate the grids (Zeros fill)
V_grid  = zeros(n_speeds, n_angles);
Ax_grid = zeros(n_speeds, n_angles);
Ay_grid = zeros(n_speeds, n_angles);

% 3. Fill the grids
for i = 1:n_speeds
    v_slice = results.v(i);
    ax_pos = results.ax_max(i);
    ax_neg = results.ax_brk(i); 
    ay_lim = results.ay_max(i);
    
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
        
        % Assign directly to the matrix
        Ax_grid(i, k) = x_val;
        Ay_grid(i, k) = y_val;
        V_grid(i, k)  = v_slice;
    end
end

% --- 3. PLOTTING ---
figure('Name', 'Master GGV Analysis', 'Color', 'w', 'Position', [100 100 1200 600]);

% Subplot 1: 3D Surface
subplot(1, 2, 1);
surf(Ax_grid, Ay_grid, V_grid, 'FaceAlpha', 0.5, 'EdgeColor', 'interp');
colormap jet; colorbar;
xlabel('Longitudinal G'); ylabel('Lateral G'); zlabel('Speed (m/s)');
title('3D Performance Envelope');
grid on; axis equal; view(135, 30);

% Subplot 2: 2D Limits
subplot(1, 2, 2);
plot(results.v, results.ay_max, 'b-o', 'LineWidth', 2); hold on;
plot(results.v, results.ax_max, 'g-o', 'LineWidth', 2);
plot(results.v, results.ax_brk, 'r-o', 'LineWidth', 2);
ylabel('Acceleration (G)'); xlabel('Speed (m/s)');
legend('Max Lateral', 'Max Accel', 'Max Brake');
title('2D Limits vs Speed');
grid on;

save('GGV_Final_Data.mat', 'results');
fprintf('\nDone. Master GGV generated.\n');

Mode = 1;
% =========================================================================
% FUNCTION: EXPLICIT PEAK FINDER (STRICT PEAK ONLY - NO MEAN)
% =========================================================================
function [peak_val, peak_yaw] = Get_Peak_Explicit(simIn, target_v, mode, steer_angle, wheelbase)
    peak_val = 0; 
    peak_yaw = 0;
    
    try
        % Suppress Simulink warnings to clean up the command window
        warning('off', 'Simulink:Commands:SimulatingWithAccelMode');
        
        out = sim(simIn);
        
        % Robust Data Fetch
        try t = out.tout; catch, t = out.get('tout'); end
        try vx = out.Longitudinal_vel.Data; catch, try vx = out.get('vx').Data; catch, vx = target_v*ones(size(t)); end, end
        try yaw = out.Yaw_rate.Data; catch, try yaw = out.yawrate.Data; catch, yaw = zeros(size(t)); end, end
        
        % Fallbacks added to ensure we don't return 0 if exact block name mismatches
        if strcmp(mode, 'lat')
            try acc = abs(out.Lateral_accln_sensor.Data); catch, try acc = abs(out.ay.Data); catch, acc = zeros(size(t)); end, end
        else
            try acc = out.Longitudinal_accln_sensor.Data; catch, try acc = out.ax.Data; catch, acc = zeros(size(t)); end, end
        end
        
        % INITIALIZE EXTREMES
        switch mode
            case 'accel'
                peak_val = -999; % Look for highest positive
            case 'brake'
                peak_val = 999;  % Look for lowest negative
            case 'lat'
                peak_val = 0;    % Look for highest absolute
        end
        
        % POINT-BY-POINT SCAN
        valid_points_found = false;
        
        for k = 1:length(t)
            % 1. Velocity Guard (Relaxed: Allows heavy cornering tire scrub)
            if vx(k) < (target_v * 0.3), continue; end 
            
            % 2. Stability Guard (Relaxed: Allows peak limit transient before spin)
            r_theo = (vx(k) * tan(steer_angle)) / wheelbase;
            r_limit = max(1.0, abs(r_theo)*4.0 + 0.5);
            if abs(yaw(k)) > r_limit, continue; end
            
            % 3. Sanity Guard (Ignore simulation explosions)
            if abs(acc(k)) > 45, continue; end 
            
            val = acc(k);
            valid_points_found = true;
            
            % 4. STRICT PEAK LOGIC (No Averaging)
            switch mode
                case 'accel'
                    if val > peak_val, peak_val = val; end
                case 'brake'
                    if val < peak_val, peak_val = val; end
                case 'lat'
                    if val > peak_val, peak_val = val; end
            end
        end
        
        % Handle cases where no valid data was found
        if ~valid_points_found
            peak_val = 0;
        elseif peak_val == -999 || peak_val == 999
            peak_val = 0; 
        end
        
    catch ME
        fprintf('Error during sim: %s\n', ME.message);
        peak_val = 0;
    end
end