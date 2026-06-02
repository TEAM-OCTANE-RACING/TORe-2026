% =========================================================================
% MASTER LAP TIME SIMULATION SCRIPT (Compatible with RunGGV_Master_3D)
% 1. Loads Track Data
% 2. Loads 'GGV_Final_Data.mat' (Result from 3D GGV Script)
% 3. Optimizes Racing Line
% 4. Runs 3-Pass Solver (Fixed Unit Mismatch)
% 5. INVERSE DYNAMICS: Generates Throttle, Brake, Steer commands
% 6. Visualizes: Speed, Controls, and 3D GGV Usage
% =========================================================================

% clear; clc; close all;

% --- 1. CHECK & LOAD INPUTS ---
ggv_file = 'GGV_Final_Data.mat';

if ~exist(ggv_file, 'file')
    fprintf('Warning: "%s" not found. Searching for other GGV data...\n', ggv_file);
    potential_files = {'GGV_Sensor_Data.mat', 'GGV_Safe_Data.mat', 'GGV_Map.mat'};
    found = false;
    for k = 1:length(potential_files)
        if exist(potential_files{k}, 'file')
            ggv_file = potential_files{k};
            found = true;
            fprintf('Found alternative file: "%s"\n', ggv_file);
            break;
        end
    end
    if ~found
        error('Please run "RunGGV_Master_3D.m" and check where it saves the .mat file.');
    end
end

load(ggv_file, 'results'); 

if ~exist('results', 'var')
    error('File "%s" loaded, but it does not contain the "results" structure.', ggv_file);
end

% Set your Track File here
track_file = 'Kari_FB.csv'; 
if ~exist(track_file, 'file')
    error('Track file "%s" not found in current folder.', track_file);
end

% --- 2. OPTIMIZE RACING LINE ---
fprintf('=== STEP 1: LOAD & OPTIMIZE RACING LINE ===\n');
[path_s, path_k, path_X, path_Y] = Optimize_Racing_Line(track_file);

% --- 3. PRE-PROCESS GGV MAP ---
fprintf('\n=== STEP 2: PRE-PROCESS GGV MAP ===\n');
GGV_Env = Process_GGV_Envelope(results);

% --- 4. RUN 3-PASS SOLVER ---
fprintf('\n=== STEP 3: RUN 3-PASS SOLVER ===\n');
Results = Solve_3Pass(path_s, path_k, GGV_Env);

% Calculate basic kinematic steering angle
Results.steer_deg = rad2deg(atan(1.53 * path_k)); % Default WB = 1.53m

fprintf('Estimated Lap Time: %.3f seconds\n', Results.laptime);
fprintf('Max Speed:          %.1f km/h\n', max(Results.v)*3.6);


% --- 5. INVERSE DYNAMICS GENERATION ---
fprintf('\n=== STEP 4: GENERATE CONTROLS (INVERSE DYNAMICS) ===\n');

% Update these with your vehicle parameters!
Veh_Par.Mass     = 270;   % kg (Car + Driver)
Veh_Par.CdA      = 0.99;   % Drag Area [m^2]
Veh_Par.Cr       = 0.015; % Rolling Resistance Coeff
Veh_Par.rho      = 1.225; % Air Density
Veh_Par.Rw       = 0.228;  % Tire Radius [m]
Veh_Par.GR       = 10.05;   % Gear Ratio
Veh_Par.WB       = 1.532;  % Wheelbase [m]
Veh_Par.K_us     = -7.143e-7;  % Understeer Gradient [rad/g]

% Motor & Powertrain limits
Veh_Par.T_max_Nm = 29.1;   % Peak Motor Torque
Veh_Par.P_max_kW = 80;    % Peak Electrical Power limit

Controls = Generate_Control_Inputs(Results, Veh_Par,path_k);

% Map outputs back to Results struct for plotting
Results.Throttle = Controls.Throttle;
Results.Brake    = Controls.Brake;
Results.Steer_Dyn= Controls.Steer_Angle_Deg;

fprintf('Controls Generated successfully.\n');


% =========================================================================
% --- VISUALIZATION SECTION ---
% =========================================================================
figure('Name', 'Race Analysis', 'Color', 'k', 'Position', [100 100 1400 900]);
t = tiledlayout(2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

v_min_kph = min(Results.v) * 3.6;
v_max_kph = max(Results.v) * 3.6;
c_limits  = [v_min_kph, v_max_kph]; 

% --- PLOT 1: TRACK MAP (LEFT LARGE) ---
nexttile([2 1]);
hold on; axis equal; grid on;
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);

z = zeros(size(path_X));
col = Results.v * 3.6; 
surface([path_X'; path_X'], [path_Y'; path_Y'], [z'; z'], ...
        [col'; col'], ...
        'FaceColor', 'no', 'EdgeColor', 'interp', 'LineWidth', 3);

colormap(gca, 'turbo'); clim(c_limits);         
c = colorbar('southoutside'); 
c.Color = 'w'; c.Label.Color = 'w'; c.Label.String = 'Speed [km/h]';
title('Optimal Racing Line', 'Color', 'w', 'FontSize', 14);
xlabel('X [m]'); ylabel('Y [m]');

% --- PLOT 2: SPEED TRACE ---
nexttile;
plot(Results.s, Results.v * 3.6, 'LineWidth', 2, 'Color', '#4DBEEE'); 
grid on; xlim([0 max(Results.s)]);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
ylabel('Speed [km/h]', 'Color', 'w', 'FontWeight', 'bold'); 
title(['Lap Time: ' num2str(Results.laptime, '%.2f') 's'], 'Color', 'w');

% --- PLOT 3: INVERSE DYNAMICS (THROTTLE / BRAKE) ---
nexttile;
hold on; grid on; xlim([0 max(Results.s)]); ylim([-0.1 1.1]);
plot(Results.s, Results.Throttle, 'LineWidth', 1.5, 'Color', '#77AC30', 'DisplayName', 'Throttle');
plot(Results.s, Results.Brake, 'LineWidth', 1.5, 'Color', '#D95319', 'DisplayName', 'Brake');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
ylabel('Pedal Input [0 to 1]', 'Color', 'w', 'FontWeight', 'bold');
title('Inverse Dynamics: Pedals', 'Color', 'w');
legend('TextColor', 'w', 'Color', 'none', 'EdgeColor', 'none');

% --- PLOT 4: INVERSE DYNAMICS (STEERING) ---
nexttile;
plot(Results.s, Results.Steer_Dyn, 'LineWidth', 1.5, 'Color', '#EDB120');
grid on; xlim([0 max(Results.s)]);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
ylabel('Steer [deg]', 'Color', 'w', 'FontWeight', 'bold');
xlabel('Distance [m]', 'Color', 'w'); 
title('Inverse Dynamics: Steering', 'Color', 'w');

% --- PLOT 5: 3D PERFORMANCE TUBE ---
nexttile;
hold on; grid on;

[V_mesh, Theta_mesh] = meshgrid(linspace(min(GGV_Env.v), max(GGV_Env.v), 40), linspace(0, 2*pi, 40));
Ax_surf = zeros(size(V_mesh)); Ay_surf = zeros(size(V_mesh));

for r = 1:size(V_mesh, 1)
    for c = 1:size(V_mesh, 2)
        v_val = V_mesh(r,c); th = Theta_mesh(r,c);
        ay_lim = interp1(GGV_Env.v, GGV_Env.ay_max, v_val, 'linear', 'extrap') / 9.81;
        ax_acc = interp1(GGV_Env.v, GGV_Env.ax_acc, v_val, 'linear', 'extrap') / 9.81;
        ax_brk = abs(interp1(GGV_Env.v, GGV_Env.ax_brk, v_val, 'linear', 'extrap') / 9.81);
        
        if sin(th) >= 0, r_long = ax_acc; else, r_long = ax_brk; end
        rad = (r_long * ay_lim) / sqrt((ay_lim * sin(th))^2 + (r_long * cos(th))^2);
        
        Ay_surf(r,c) = rad * cos(th); Ax_surf(r,c) = rad * sin(th); 
    end
end

surf(Ay_surf, Ax_surf, V_mesh * 3.6, 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'FaceColor', 'w');
scatter3(Results.ay_used/9.81, Results.ax_used/9.81, Results.v*3.6, 25, Results.v*3.6, 'filled');

view(135, 30);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w', 'GridColor', 'w', 'GridAlpha', 0.2);
xlabel('Lat G'); ylabel('Long G'); zlabel('Speed [km/h]');
title('3D Performance Envelope', 'Color', 'w');
colormap(gca, 'turbo'); clim(c_limits);         

% =========================================================================
% FUNCTION 1: RACING LINE OPTIMIZER 
% =========================================================================
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

% =========================================================================
% FUNCTION 2: PROCESS GGV STRUCT 
% =========================================================================
function Env = Process_GGV_Envelope(results)
    Env.v = results.v(:); 
    Env.ay_max = abs(results.ay_max(:)) * 9.81;
    Env.ax_acc = results.ax_max(:) * 9.81;
    Env.ax_brk = abs(results.ax_brk(:)) * 9.81; 
    [Env.v, idx] = sort(Env.v);
    Env.ay_max = Env.ay_max(idx); Env.ax_acc = Env.ax_acc(idx); Env.ax_brk = Env.ax_brk(idx);
end

% =========================================================================
% FUNCTION 3: 3-PASS SOLVER
% =========================================================================
function R = Solve_3Pass(s, k, Env)
    N = length(s);
    v_apex = zeros(N, 1);
    v_acc = zeros(N, 1); ax_acc_log = zeros(N, 1);
    v_brk = zeros(N, 1); ax_brk_log = zeros(N, 1);
    
    for i = 1:N
        curv = abs(k(i));
        if curv < 1e-4, v_apex(i) = max(Env.v);
        else
            v_g = 12; 
            for iter=1:15
                ay_lim = interp1(Env.v, Env.ay_max, v_g, 'linear', 'extrap');
                v_new = sqrt(ay_lim / curv);
                v_g = 0.6*v_g + 0.4*v_new;
            end
            v_apex(i) = min(21.4,v_g);
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
    
    R.s = s; R.v = v_final; R.ay_used = v_final.^2 .*k; R.ax_used = ax_final;
    R.laptime = sum(diff(s) ./ ((v_final(1:end-1) + v_final(2:end))/2));
end

% =========================================================================
% FUNCTION 4: GENERATE CONTROL INPUTS (INVERSE DYNAMICS)
% =========================================================================
function Controls = Generate_Control_Inputs(Results, Par, path_k)
    m    = Par.Mass;     CdA  = Par.CdA;       Cr   = Par.Cr;    
    rho  = Par.rho;      Rw   = Par.Rw;        GR   = Par.GR;    
    L    = Par.WB;       K_us = Par.K_us;  
    
    v  = Results.v;      ax = Results.ax_used; ay = Results.ay_used;  
    k  = path_k;         % Using actual geometric curvature
    N  = length(v);
    
    % --- Lateral Control (Road Wheel Angle) ---
    Steer_Geo = atan(L .* k);            
    Steer_Dyn = K_us .* (ay / 9.81);     
    Controls.Steer_Angle_Deg = rad2deg(Steer_Geo + Steer_Dyn);
    
    % --- Longitudinal Forces ---
    F_aero = 0.5 * rho * CdA * v.^2;
    F_roll = Cr * m * 9.81 * cos(0); 
    F_acc  = m .* ax;
    
    alpha_est = (abs(ay)/9.81) * 0.05; 
    F_corn    = m .* abs(ay) .* sin(alpha_est);
    
    % Lumped resistance for undriven front wheels
    F_front_drag = 500; 
    F_total = F_aero + F_roll + F_acc + F_corn + F_front_drag;
    
    T_max_motor_Nm = Par.T_max_Nm; 
    P_max_W        = Par.P_max_kW * 1000;
    
    Throttle_Cmd = zeros(N, 1); Brake_Cmd = zeros(N, 1); 
    
    for i = 1:N
        % Single motor limits
        Tw_limit_mech = T_max_motor_Nm * GR; 
        w_wheel = max(v(i) / Rw, 0.1); % Prevent divide-by-zero at standstill
        Tw_limit_pwr  = P_max_W / w_wheel;
        
        Tw_max_avail = min(Tw_limit_mech, Tw_limit_pwr) * 0.95; 
        
        % Torque required PER MOTOR (2 driven wheels)
        Tw_req_per_motor = F_total(i) * Rw * 0.5;
        
        if Tw_req_per_motor >= 0
            % Motoring
            Throttle_Cmd(i) = min(max(Tw_req_per_motor / Tw_max_avail, 0), 1.0);
            Brake_Cmd(i)    = 0;
        else
            % Braking
            Throttle_Cmd(i) = 0;
            % Assuming brake distribution uses all wheels, total max braking force:
            F_brake_max = 1 * 9.81 * m; 
            Brake_Cmd(i) = min(max(abs(F_total(i)) / F_brake_max, 0), 1.0);
        end
    end
    
    Controls.Throttle = Throttle_Cmd;
    Controls.Brake    = Brake_Cmd;
    Controls.F_Total  = F_total; 
end