% Friction Ellipse (g-g diagram) Generator
% clear; clc; close all;

% 1. Define Parameters (in g's)
ay_max = 1.7;     % Max lateral acceleration
ax_accel = 1;   % Max longitudinal acceleration
ax_brake = 1.5;   % Max longitudinal braking (magnitude)

% 2. Generate Theta
% Use 360 points for a smooth curve, transpose to make it a column vector
theta = linspace(0, pi, 180)'; 

% 3. Calculate Accelerations
ay = ay_max * sin(theta);

% Preallocate ax vector
ax = zeros(size(theta));

% Apply asymmetric conditions for acceleration and braking
% Vectorized approach for efficiency
accel_idx = cos(theta) >= 0;
brake_idx = cos(theta) < 0;

ax(accel_idx) = ax_accel * cos(theta(accel_idx));
ax(brake_idx) = ax_brake * cos(theta(brake_idx)); % cos is negative here

% 4. Store Data to CSV
% Create a table with clear headers
dataTable = table(theta, ax, ay, 'VariableNames', {'Theta_rad', 'ax_g', 'ay_g'});

% Write to a CSV file in the current working directory
filename = 'friction_ellipse.csv';
writetable(dataTable, filename);
fprintf('Data successfully saved to %s\n', filename);

% 5. Plot the Friction Ellipse
figure('Name', 'Friction Ellipse (g-g diagram)', 'Color', 'w');

% Standard vehicle dynamics plots put lateral (ay) on the X-axis 
% and longitudinal (ax) on the Y-axis.
plot(ay, ax, 'b-', 'LineWidth', 2);
hold on;

% Add origin lines for reference
xline(0, 'k--', 'HandleVisibility', 'off');
yline(0, 'k--', 'HandleVisibility', 'off');

% Formatting
title('Vehicle Friction Ellipse (g-g Diagram)');
xlabel('Lateral Acceleration, a_y (g)');
ylabel('Longitudinal Acceleration, a_x (g)');
grid on;

% axis equal ensures the scale of X and Y are identical so the ellipse isn't distorted
axis equal; 
xlim([-2, 2]);
ylim([-1.5, 1.5]);