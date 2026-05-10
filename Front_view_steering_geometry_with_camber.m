% Front View Suspension Kinematics - Full Visualization (Toe & Camber)
clc; clear; close all;

% --- 1. GEOMETRY INPUTS (Static Constants) ---
L = 330.714;       % Lower arm length (Chassis Pivot A to Ball Joint C)
U = 289.019;       % Upper arm length (Chassis Pivot B to Ball Joint D)
K = 150;           % Knuckle height (Vertical distance between C and D)
H_chassis = 150;   % Vertical spread between chassis mounts A and B
theta_L0 = -1.522; % Static angle of lower arm (degrees)
Rw = 228;          % Wheel radius in mm

% Steering / Tie-Rod Inputs
d = 60;            % Vertical distance from C up to Tie-rod point P
Sx = -20;          % Steering Rack Pivot X (Chassis mount for tie-rod)
Sy = 40;           % Steering Rack Pivot Y (Chassis mount for tie-rod)

% Travel range for simulation (Wheel moving up/down from static)
h_array = linspace(-30, 30, 100); 

% --- 2. STATIC CALCULATIONS ---
Ax = 0; Ay = 0;             % Point A: Lower Chassis Pivot
Bx = 0; By = H_chassis;     % Point B: Upper Chassis Pivot

% Locate Static Lower Ball Joint (C)
C0x = L * cosd(theta_L0);
C0y = L * sind(theta_L0);

% Solve for Static Upper Ball Joint (D) using the "Invisible Triangle" B-C-D
dist_BC0 = sqrt((C0x-Bx)^2 + (C0y-By)^2); 
phi0 = atan2(C0y-By, C0x-Bx);             
alpha0 = acos((U^2 + dist_BC0^2 - K^2) / (2 * U * dist_BC0)); 
D0x = Bx + U * cos(phi0 + alpha0);
D0y = By + U * sin(phi0 + alpha0);

% Calculate Static Camber (Angle of knuckle CD relative to vertical)
% atan2d(dx, dy) where dy is vertical gives angle from vertical
camber_static = atan2d(D0x - C0x, D0y - C0y);

% Point P: The tie-rod pickup point on the knuckle
P0x = C0x + (d/K)*(D0x-C0x);
P0y = C0y + (d/K)*(D0y-C0y);

% R_rod: The fixed physical length of your tie-rod
R_rod = sqrt((P0x - Sx)^2 + (P0y - Sy)^2);

% --- 3. DYNAMIC SWEEP ---
Px_knuckle = []; Py_knuckle = []; 
Px_arc = [];     bump_steer = [];
Cx_all = []; Cy_all = []; Dx_all = []; Dy_all = [];
camber_change = []; % Storage for camber values

for h = h_array
    % A. Move Lower Ball Joint C along its arc
    Cy = C0y + h;
    Cx = sqrt(L^2 - Cy^2);
    
    % B. Update Upper Ball Joint D
    d_BC = sqrt((Cx-Bx)^2 + (Cy-By)^2);
    if d_BC > (U + K) || d_BC < abs(U - K), continue; end
    phi_n = atan2(Cy-By, Cx-Bx);
    alpha_n = acos((U^2 + d_BC^2 - K^2) / (2 * U * d_BC));
    Dx = Bx + U * cos(phi_n + alpha_n);
    Dy = By + U * sin(phi_n + alpha_n);
    
    % NEW: Calculate Camber at current height
    current_camber = atan2d(Dx - Cx, Dy - Cy);
    camber_change(end+1) = current_camber - camber_static; % Relative to static
    
    % C. P_kx: Where the knuckle MOVES point P
    P_kx = Cx + (d/K)*(Dx-Cx);
    P_ky = Cy + (d/K)*(Dy-Cy);
    
    % D. P_ax: Where the tie-rod LENGTH constrains point P
    dy_rod = P_ky - Sy;
    if abs(dy_rod) < R_rod
        P_ax = Sx + sqrt(R_rod^2 - dy_rod^2);
    else
        continue; 
    end
    
    % Store values for plotting
    Px_knuckle(end+1) = P_kx; Py_knuckle(end+1) = P_ky;
    Px_arc(end+1) = P_ax;
    bump_steer1 = P_kx - P_ax; 
    bump_steer(end+1) = rad2deg(bump_steer1/Rw);
    Cx_all(end+1) = Cx; Cy_all(end+1) = Cy;
    Dx_all(end+1) = Dx; Dy_all(end+1) = Dy;
end

% --- 4. VISUALIZATION (Dynamic Scaling & Large Left Plot) ---
fig = figure('Color', [0.1 0.1 0.1], 'Name', 'Refined Suspension Analysis');
set(fig, 'Units', 'Normalized', 'Position', [0.05, 0.1, 0.9, 0.8]); 

% --- PLOT 1: Main Assembly (Large Left Side) ---
ax1 = axes('Position', [0.07, 0.15, 0.55, 0.75]); 
set(ax1, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
hold on; axis equal; grid on;

% Plot Paths
plot(Px_knuckle, Py_knuckle, 'cyan', 'LineWidth', 1, 'DisplayName', 'Knuckle P-Path');
plot(Px_arc, Py_knuckle, 'g--', 'LineWidth', 1, 'DisplayName', 'Tie-Rod Arc');

% Plot Static Geometry
idx = round(length(Cx_all)/2);
lw = 1.2; 
plot([Ax Cx_all(idx)], [Ay Cy_all(idx)], 'Color', [0.2 0.6 1], 'LineWidth', lw, 'DisplayName', 'Lower Arm (L)');
plot([Bx Dx_all(idx)], [By Dy_all(idx)], 'Color', [1 0.6 0 ], 'LineWidth', lw, 'DisplayName', 'Upper Arm (U)');
plot([Cx_all(idx) Dx_all(idx)], [Cy_all(idx) Dy_all(idx)], 'Color', [1 0.2 0.2], 'LineWidth', 1.8, 'DisplayName', 'Knuckle (K)');
plot([Sx Px_arc(idx)], [Sy Py_knuckle(idx)], 'Color', [0.2 1 0.2], 'LineWidth', lw, 'DisplayName', 'Actual Tie-Rod');
plot([Ax Bx], [Ay By], 'w', 'LineWidth', 2, 'DisplayName', 'Chassis Wall');

% Annotations
text(Ax-30, Ay, 'A', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
text(Bx-30, By, 'B', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
text(Cx_all(idx)+12, Cy_all(idx), 'C', 'Color', [0.2 0.6 1], 'FontSize', 11);
text(Dx_all(idx)+12, Dy_all(idx), 'D', 'Color', [1 0.6 0], 'FontSize', 11);
text(Px_arc(idx)+12, Py_knuckle(idx), 'P', 'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold');
text(Sx-25, Sy-15, 'S', 'Color', [0.2 1 0.2], 'FontSize', 11);

% DYNAMIC SCALE (Assembly) - Adds 15% padding automatically
x_data = [Ax, Bx, Sx, Cx_all, Dx_all];
y_data = [Ay, By, Sy, Cy_all, Dy_all];
xlim([min(x_data)-50, max(x_data)+50]);
ylim([min(y_data)-50, max(y_data)+50]);

title('Assembly Geometry', 'Color', 'w', 'FontSize', 15);
legend('TextColor', 'w', 'Location', 'southoutside', 'Orientation', 'horizontal', 'FontSize', 8);

% --- PLOT 2: Bump Steer Plot (Top Right) ---
ax2 = axes('Position', [0.70, 0.58, 0.25, 0.32]); 
set(ax2, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
hold on; grid on;

travel = Py_knuckle - P0y;
plot(travel, bump_steer, 'y', 'LineWidth', 2.5);
plot([min(travel) max(travel)], [0 0], 'w--', 'HandleVisibility', 'off'); 

% DYNAMIC SCALE (Bump Steer) - Adds 10% offset
pad_t = (max(travel) - min(travel)) * 0.1;
pad_b = (max(bump_steer) - min(bump_steer)) * 0.1;
xlim([min(travel)-pad_t, max(travel)+pad_t]);
ylim([min(bump_steer)-pad_b, max(bump_steer)+pad_b]);

title('Bump Steer', 'Color', 'w', 'FontSize', 13);
xlabel('Wheel Travel (mm)'); ylabel('Toe (deg)');

% --- PLOT 3: Camber Change Plot (Bottom Right) ---
ax3 = axes('Position', [0.70, 0.15, 0.25, 0.32]); 
set(ax3, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
hold on; grid on;

plot(travel, camber_change, 'm', 'LineWidth', 2.5);
plot([min(travel) max(travel)], [0 0], 'w--', 'HandleVisibility', 'off'); 

% DYNAMIC SCALE (Camber) - Adds 10% offset
pad_c = (max(camber_change) - min(camber_change)) * 0.1;
xlim([min(travel)-pad_t, max(travel)+pad_t]);
ylim([min(camber_change)-pad_c, max(camber_change)+pad_c]);

title('Camber Change', 'Color', 'w', 'FontSize', 13);
xlabel('Wheel Travel (mm)'); ylabel('Camber (deg)');