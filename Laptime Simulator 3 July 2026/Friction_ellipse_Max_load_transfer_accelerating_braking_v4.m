% Combined GGV and Dynamic Load Calculator (Final Version)
clear; clc; close all;
fprintf('=== STARTING COMBINED GGV & DYNAMIC LOAD ANALYSIS ===\n');

%% 1. Vehicle Parameters
% Mass and Geometry
m_total = 270;      % Total mass (kg)
h_cg = 0.21;        % CG height (m)
wheelbase = 1.532;  % Wheelbase (m)
track_width = 1.235;% Track width (m)
g = 9.81;           % Gravity (m/s^2)

% Roll and Load Transfer parameters
m_roll = 210;       % Sprung mass or mass causing roll (kg)
h_roll = 0.13;      % Roll moment arm (m)
k_roll_rear = 0.55; % Rear roll stiffness distribution
k_roll_front = 0.45;% Front roll stiffness distribution

% Aerodynamic and Weight Distribution Parameters
rho = 1.225;        % Air density (kg/m^3)
ClA = 3.5526;       % Downforce coefficient * Area
wd_front = 0.45;    % Front weight distribution
wd_rear = 0.55;     % Rear weight distribution

%% 2. Load GGV Data
filename = 'GGV_Final_Data.mat';
if exist(filename, 'file')
    load(filename, 'results');
    fprintf('Successfully loaded data from %s\n', filename);
else
    if evalin('base', 'exist(''results'', ''var'')')
        results = evalin('base', 'results');
        fprintf('Loaded ''results'' directly from the active MATLAB workspace.\n');
    else
        error('GGV data not found. Please run the master simulation script first.');
    end
end

% Extract variables mapped to Load Inputs
v_data        = results.v;
ax_accel_data = results.ax_max;
ax_brake_data = results.ax_brk; 
ay_max_data   = results.ay_max;
num_vel = length(v_data);

%% 3. Preallocate Storage for Max Values and Angles
max_Load_rear_accel = zeros(num_vel, 1);
max_Load_front_brake = zeros(num_vel, 1);
theta_max_accel_rad = zeros(num_vel, 1); % Store angle of max accel load
theta_max_brake_rad = zeros(num_vel, 1); % Store angle of max brake load

%% 4. Setup Plot
figure('Name', 'Dynamic Load over Sweep Angle', 'Color', 'w', 'Position', [200 200 900 600]);
hold on;
colors = jet(num_vel); % Generate colors for the plot based on velocity

%% 5. Loop Through Each Velocity Slice
theta = linspace(0, pi, 180)'; 
accel_idx = cos(theta) >= 0;  % Acceleration phase
brake_idx = cos(theta) < 0;   % Braking phase

for i = 1:num_vel
    % Extract limits for current velocity
    v_current = v_data(i);
    ay_max = ay_max_data(i);
    ax_accel = ax_accel_data(i);
    ax_brake = abs(ax_brake_data(i)); % Use absolute value
    
    % Calculate Downforce for current velocity (per wheel)
    DF_total = 0.5 * ClA * rho * v_current^2;
    DF_front = (DF_total * wd_front) / 2;
    DF_rear  = (DF_total * wd_rear) / 2;
    
    % Calculate Accelerations
    ay = ay_max * sin(theta);
    ax = zeros(size(theta));
    
    ax(accel_idx) = ax_accel * cos(theta(accel_idx));
    ax(brake_idx) = ax_brake * cos(theta(brake_idx)); 
    
    Load_dynamic = zeros(size(theta)); 
    
    % --- Acceleration Phase (Rear Outer Wheel Dynamic Load) ---
    LT_long_accel = m_total * h_cg * g .* ax(accel_idx) / (wheelbase * 2);
    LT_lat_accel  = m_roll .* ay(accel_idx) * g * h_roll * k_roll_rear / track_width;
    % Dynamic Load = Aero + Longitudinal Transfer + Lateral Transfer
    Load_dynamic_accel = DF_rear + LT_long_accel + LT_lat_accel;
    Load_dynamic(accel_idx) = Load_dynamic_accel;
    
    % --- Braking Phase (Front Outer Wheel Dynamic Load) ---
    LT_long_brake = m_total * h_cg * g .* abs(ax(brake_idx)) / (wheelbase * 2);
    LT_lat_brake  = m_roll .* ay(brake_idx) * g * h_roll * k_roll_front / track_width;
    % Dynamic Load = Aero + Longitudinal Transfer + Lateral Transfer
    Load_dynamic_brake = DF_front + LT_long_brake + LT_lat_brake;
    Load_dynamic(brake_idx) = Load_dynamic_brake;
    
    % Save Maximums AND their locations for this velocity
    [val_accel, idx_accel] = max(Load_dynamic_accel);
    [val_brake, idx_brake] = max(Load_dynamic_brake);
    
    max_Load_rear_accel(i) = val_accel;
    max_Load_front_brake(i) = val_brake;
    
    % Map indices back to global theta array
    global_accel_indices = find(accel_idx);
    global_brake_indices = find(brake_idx);
    
    theta_max_accel_rad(i) = theta(global_accel_indices(idx_accel));
    theta_max_brake_rad(i) = theta(global_brake_indices(idx_brake));
    
    % Plot the curve for this velocity
    plot(rad2deg(theta), Load_dynamic, 'Color', colors(i,:), 'LineWidth', 1.5);
end

%% 6. Find and Plot Global Maximums
[global_max_accel, max_accel_idx] = max(max_Load_rear_accel);
[global_max_brake, max_brake_idx] = max(max_Load_front_brake);

% Convert the angles to degrees for plotting
global_theta_accel_deg = rad2deg(theta_max_accel_rad(max_accel_idx));
global_theta_brake_deg = rad2deg(theta_max_brake_rad(max_brake_idx));

% Plot the Global Max Markers
plot(global_theta_accel_deg, global_max_accel, 'p', 'MarkerSize', 16, ...
    'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
plot(global_theta_brake_deg, global_max_brake, 's', 'MarkerSize', 12, ...
    'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);

% Add coordinate text labels next to the markers
text(global_theta_accel_deg, global_max_accel, sprintf('  Max Accel Load\n  (%.1f\\circ, %.3f N)', ...
    global_theta_accel_deg, global_max_accel), 'VerticalAlignment', 'bottom', 'FontWeight', 'bold');
text(global_theta_brake_deg, global_max_brake, sprintf('  Max Brake Load\n  (%.1f\\circ, %.3f N)', ...
    global_theta_brake_deg, global_max_brake), 'VerticalAlignment', 'bottom', 'FontWeight', 'bold');

%% 7. Formatting the Plot
title('Combined Dynamic Load over Sweep Angle for All Velocities');
xlabel('Sweep Angle \theta (Degrees)');
ylabel('Dynamic Load on Outer Wheel (N)');
grid on;

% Add Colorbar to indicate velocity accurately
colormap(jet);
c = colorbar;
caxis([min(v_data) max(v_data)]);
c.Label.String = 'Velocity (m/s)';

% Vertical line separating Accel and Braking phases
xline(90, '--k', 'Transition (Accel to Brake)', 'LabelVerticalAlignment', 'bottom');

% Dummy plots to ensure markers show up nicely in the legend
h1 = plot(NaN,NaN,'p', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k');
h2 = plot(NaN,NaN,'s', 'MarkerSize', 10, 'MarkerFaceColor', 'c', 'MarkerEdgeColor', 'k');
legend([h1, h2], {'Global Max Dynamic (Rear Outer Accel)', 'Global Max Dynamic (Front Outer Brake)'}, 'Location', 'best');
hold off;

%% 8. Output Global Maximums to Command Window
% --- Calculate Ax and Ay for Max Acceleration Phase ---
theta_accel_rad_val = theta_max_accel_rad(max_accel_idx);
Ax_accel_val = ax_accel_data(max_accel_idx) * cos(theta_accel_rad_val);
Ay_accel_val = ay_max_data(max_accel_idx) * sin(theta_accel_rad_val);

% --- Calculate Ax and Ay for Max Braking Phase ---
theta_brake_rad_val = theta_max_brake_rad(max_brake_idx);
Ax_brake_val = abs(ax_brake_data(max_brake_idx)) * cos(theta_brake_rad_val); 
Ay_brake_val = ay_max_data(max_brake_idx) * sin(theta_brake_rad_val);

fprintf('\n--- GLOBAL MAXIMUM DYNAMIC LOAD RESULTS ---\n\n');

fprintf('1. Acceleration + Cornering (Rear Outer Wheel):\n');
fprintf('   Max Dynamic Load   : %.3f N at v = %.2f m/s\n', ...
    global_max_accel, v_data(max_accel_idx));
fprintf('   Longitudinal Accel : %.3f g\n', Ax_accel_val);
fprintf('   Lateral Accel      : %.3f g\n\n', Ay_accel_val);

fprintf('2. Braking + Cornering (Front Outer Wheel):\n');
fprintf('   Max Dynamic Load   : %.3f N at v = %.2f m/s\n', ...
    global_max_brake, v_data(max_brake_idx));
fprintf('   Longitudinal Accel : %.3f g\n', Ax_brake_val);
fprintf('   Lateral Accel      : %.3f g\n\n', Ay_brake_val);