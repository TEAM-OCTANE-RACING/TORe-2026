% SCRIPT: Plot_GGV_Results.m
% Purpose: Loads previously generated GGV data and visualizes the 3D 
%          performance envelope and 2D limits.

clc; 

fprintf('=== STARTING GGV VISUALIZATION ===\n');

% --- 1. LOAD DATA ---
filename = 'GGV_Final_Data.mat';

if exist(filename, 'file')
    load(filename, 'results');
    fprintf('Successfully loaded data from %s\n', filename);
else
    % Fallback: Check if 'results' already exists in the base workspace
    if evalin('base', 'exist(''results'', ''var'')')
        results = evalin('base', 'results');
        fprintf('Loaded ''results'' directly from the active MATLAB workspace.\n');
    else
        error('GGV data not found. Please run the master simulation script first.');
    end
end

% Extract variables for easier reading
v      = results.v;
ax_max = results.ax_max;
ax_brk = results.ax_brk;
ay_max = results.ay_max;

% --- 2. RECONSTRUCT 3D SURFACE DATA ---
% We need to rebuild the grid mapping since only the 1D limits were saved.
theta = linspace(0, 2*pi, 60); 

n_speeds = length(v);
n_angles = length(theta);

V_grid  = zeros(n_speeds, n_angles);
Ax_grid = zeros(n_speeds, n_angles);
Ay_grid = zeros(n_speeds, n_angles);

for i = 1:n_speeds
    v_slice = v(i);
    ax_pos  = ax_max(i);
    ax_neg  = ax_brk(i); 
    ay_lim  = ay_max(i);
    
    for k = 1:n_angles
        th = theta(k);
        
        % Lateral (Cosine)
        y_val = ay_lim * cos(th);
        
        % Longitudinal (Sine) - Asymmetric blending
        sin_val = sin(th);
        if sin_val >= 0
            x_val = ax_pos * sin_val;      % Forward Accel
        else
            x_val = abs(ax_neg) * sin_val; % Braking
        end
        
        % Assign to grids
        Ax_grid(i, k) = x_val;
        Ay_grid(i, k) = y_val;
        V_grid(i, k)  = v_slice;
    end
end

% --- 3. PLOTTING ---
figure('Name', 'GGV Data Visualization', 'Color', 'w', 'Position', [100 100 1200 600]);

% Subplot 1: 3D Surface
subplot(1, 2, 1);
surf(Ax_grid, Ay_grid, V_grid, 'FaceAlpha', 0.7, 'EdgeColor', 'interp');
colormap jet; 
c = colorbar;
c.Label.String = 'Speed (m/s)';
xlabel('Longitudinal G (Ax)'); 
ylabel('Lateral G (Ay)'); 
zlabel('Speed (m/s)');
title('3D Performance Envelope');
grid on; 
axis equal; 
view(135, 30);

% Subplot 2: 2D Limits vs Speed
subplot(1, 2, 2);
plot(v, ay_max, 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b'); hold on;
plot(v, ax_max, 'g-o', 'LineWidth', 2, 'MarkerFaceColor', 'g');
plot(v, ax_brk, 'r-o', 'LineWidth', 2, 'MarkerFaceColor', 'r');

% Aesthetics
ylabel('Acceleration (G)'); 
xlabel('Speed (m/s)');
legend('Max Lateral (Ay)', 'Max Accel (Ax+)', 'Max Brake (Ax-)', 'Location', 'best');
title('2D Acceleration Limits vs. Speed');
grid on;

fprintf('Plotting complete.\n');