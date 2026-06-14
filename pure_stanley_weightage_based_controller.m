%% 1. Setup Path & Pre-calculate Curvature
race = readmatrix("Austin.csv");
path.x = race(2:end,1);
path.y = race(2:end,2);
dx = gradient(path.x);
dy = gradient(path.y);
path.psi = atan2(dy, dx);
path.length = cumsum([0; sqrt(dx.^2 + dy.^2)]);

% PRE-CALCULATE CURVATURE (Kappa)
Npath = length(path.x);
path.kappa = zeros(Npath,1);
for i = 2:Npath-1
    psi1 = atan2(path.y(i)-path.y(i-1), path.x(i)-path.x(i-1));
    psi2 = atan2(path.y(i+1)-path.y(i), path.x(i+1)-path.x(i));
    path.kappa(i) = atan2(sin(psi2-psi1), cos(psi2-psi1)) / ...
                    max(hypot(path.x(i+1)-path.x(i), path.y(i+1)-path.y(i)), 1e-3);
end

%% 2. Setup Control Parameters
ctrl.L_base = 1.53;
ctrl.k_stanley = 0.6;
ctrl.k_soft = 2.0;
ctrl.delta_max = deg2rad(25);
ctrl.Kp_v = 2.5;
ctrl.Ki_v = 0.05;
ctrl.L_min = 2.5;
ctrl.k_pure = 0.3;

% --- NEW WEIGHTING PARAMETERS ---
% Curvature bounds
ctrl.kappa_low = 0.005;   % Below this curvature -> 100% Pure Pursuit preference
ctrl.kappa_high = 0.02;   % Above this curvature -> 100% Stanley preference
% Speed bounds
ctrl.v_low = 10.0;        % Below this speed -> 100% Stanley preference
ctrl.v_high = 25.0;       % Above this speed -> 100% Pure Pursuit preference
% Dominance distribution (How much does curvature matter vs speed?)
ctrl.weight_kappa = 0.7;  % 70% of the decision is based on Curvature
ctrl.weight_v = 0.3;      % 30% of the decision is based on Speed

%% 3. INITIAL STATE
est.x = race(2,1); est.y = race(2,2); est.psi = path.psi(1);
est.vx = 2; dt = 0.01;
target_laps = input('Enter laps: '); 
if isempty(target_laps), target_laps = 1; end
k = 0;
passed_halfway = false;
half_track = round(Npath / 2);

%% 4. SETUP PLOT HANDLES
fig = figure('Name','Fuzzy Blended Control Sim','Color','k','Units','normalized','Position',[0.05 0.05 0.9 0.85]);

% Main Track Map
ax1 = subplot(4,4,[1 2 5 6 9 10 13 14]); 
plot(ax1, path.x, path.y, 'w--', 'LineWidth', 1); hold on;
h_trail = animatedline(ax1, 'Color', 'c', 'LineWidth', 1.5);
h_car   = plot(ax1, est.x, est.y, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
h_ld    = plot(ax1, 0, 0, 'gp', 'MarkerSize', 10, 'LineWidth', 1.5);
axis(ax1, 'equal'); grid(ax1, 'on'); title(ax1, 'Track Map', 'Color', 'w');

% Speed Telemetry
ax2 = subplot(4,4,[3 4]); h_vel = animatedline(ax2, 'Color', 'g', 'LineWidth', 2);
ylabel(ax2, 'Speed [m/s]'); grid(ax2, 'on');

% Steering Telemetry
ax3 = subplot(4,4,[7 8]); h_delta = animatedline(ax3, 'Color', 'm', 'LineWidth', 1.5);
ylabel(ax3, 'Delta [deg]'); grid(ax3, 'on');

% Controller Weight Telemetry (Replaced Curvature Threshold)
ax4 = subplot(4,4,[11 12]); 
yyaxis(ax4, 'left');
h_kappa = animatedline(ax4, 'Color', 'y', 'LineWidth', 1.5);
ylabel(ax4, 'Curvature [1/m]');
yyaxis(ax4, 'right');
h_weight = animatedline(ax4, 'Color', 'c', 'LineWidth', 1.5);
ylabel(ax4, 'Stanley Weight [0-1]'); ylim(ax4, [-0.1 1.1]);
xlabel(ax4, 'Time [s]'); grid(ax4, 'on');

h_timer = annotation('textbox', [0.05, 0.05, 0.2, 0.05], 'String', 'Time: 0.00s', 'Color', 'g');

%% 5. THE SIMULATION LOOP
do_reset = true; 
prev_idx = 1; 
lap_count = 0; 
target_laps = floor(target_laps); 

while true 
    k = k + 1;
    
    if ~ishandle(h_car), break; end
    
    % 1. Call Controller
    [delta, acc, dbg] = blended_hybrid_controller(est, path, ctrl, dt, do_reset);
    do_reset = false; 
    
    % 2. Physics Update
    est.x   = est.x + est.vx * cos(est.psi) * dt;
    est.y   = est.y + est.vx * sin(est.psi) * dt;
    est.psi = est.psi + (est.vx/ctrl.L_base) * tan(delta) * dt;
    est.vx  = est.vx + acc * dt; 
    
    % 3. Update Visuals
    if mod(k, 10) == 0
        set(h_car, 'XData', est.x, 'YData', est.y);
        set(h_ld, 'XData', path.x(dbg.idx_ld), 'YData', path.y(dbg.idx_ld));
        addpoints(h_trail, est.x, est.y);
        addpoints(h_vel, k*dt, est.vx);
        addpoints(h_delta, k*dt, rad2deg(delta)); 
        
        yyaxis(ax4, 'left');  addpoints(h_kappa, k*dt, abs(path.kappa(dbg.idx_closest)));
        yyaxis(ax4, 'right'); addpoints(h_weight, k*dt, dbg.w_stanley);
        
        h_timer.String = sprintf('Lap: %d/%d | Time: %.2fs | Mode: %s', ...
            lap_count + 1, target_laps, k*dt, dbg.mode);
        drawnow; 
        pause(5 * dt); 
    end
    
    % 4. LAP COUNTING
    if (prev_idx - dbg.idx_closest) > (length(path.x) / 2)
        lap_count = lap_count + 1;
        fprintf('\n[SYSTEM] Lap %d detected! (Current Total: %d)\n', lap_count, lap_count);
    end
    prev_idx = dbg.idx_closest;
    
    % 5. EXIT CONDITION
    if lap_count >= target_laps
        fprintf('\n>>> TARGET REACHED: %d Laps. Terminating Simulation. <<<\n', target_laps);
        break; 
    end
end
clear blended_hybrid_controller; 
disp('Simulation complete. Functions cleared.');

%% Controller Function
function [delta, acc, dbg] = blended_hybrid_controller(est, path, ctrl, dt, do_reset)
    persistent idx_closest vel_int
    N = length(path.x);
    if isempty(idx_closest) || do_reset, idx_closest = 1; vel_int = 0; end
    
    % 1. Find Nearest Point
    win = mod((idx_closest : idx_closest + 50) - 1, N) + 1;
    [~, kmin] = min((path.x(win) - est.x).^2 + (path.y(win) - est.y).^2);
    idx_closest = win(kmin);
    
    % 2. Lookahead for Pure Pursuit
    L_ld = ctrl.L_min + ctrl.k_pure * est.vx;
    idx_ld = idx_closest;
    while hypot(path.x(idx_ld) - est.x, path.y(idx_ld) - est.y) < L_ld
        idx_ld = mod(idx_ld, N) + 1;
        if idx_ld == idx_closest, break; end 
    end
    
    % 3. CALCULATE BOTH CONTROL LAWS
    curv = abs(path.kappa(idx_closest));
    
    % --- Pure Pursuit Calculation ---
    alpha = mod(atan2(path.y(idx_ld)-est.y, path.x(idx_ld)-est.x) - est.psi + pi, 2*pi) - pi;
    delta_pp = atan2(2 * ctrl.L_base * sin(alpha), L_ld);
    
    % --- Stanley Calculation ---
    xf = est.x + ctrl.L_base * cos(est.psi);
    yf = est.y + ctrl.L_base * sin(est.psi);
    
    path_heading = atan2(path.y(idx_ld) - est.y, path.x(idx_ld) - est.x);
    psi_e = mod(path_heading - est.psi + pi, 2*pi) - pi;

    ef = -(path.x(idx_closest)-xf)*sin(path_heading) + (path.y(idx_closest)-yf)*cos(path_heading);
    delta_stanley = psi_e + atan2(ctrl.k_stanley * ef, ctrl.k_soft + est.vx);
    
    % 4. DYNAMIC WEIGHTING LOGIC
    % Curvature factor: 0 at low curvature (favor PP), 1 at high curvature (favor Stanley)
    w_k = min(1, max(0, (curv - ctrl.kappa_low) / (ctrl.kappa_high - ctrl.kappa_low)));
    
    % Speed factor: 1 at low speeds (favor Stanley), 0 at high speeds (favor PP)
    w_v = min(1, max(0, (ctrl.v_high - est.vx) / (ctrl.v_high - ctrl.v_low)));
    
    % Total weights
    w_stanley = (ctrl.weight_kappa * w_k) + (ctrl.weight_v * w_v);
    w_stanley = min(1, max(0, w_stanley)); % Ensure it stays bounded 0-1
    w_pp = 1.0 - w_stanley;
    
    % 5. BLEND STEERING
    delta = (w_stanley * delta_stanley) + (w_pp * delta_pp);
    delta = max(min(delta, ctrl.delta_max), -ctrl.delta_max);
    
    % 6. SPEED CONTROL
    v_target = min(40, sqrt((0.7 * 9.81) / max(curv, 0.001)));
    v_err = v_target - est.vx;
    vel_int = vel_int + v_err * dt;
    acc = max(min(ctrl.Kp_v * v_err + ctrl.Ki_v * vel_int, 3), -5);
    
    % Debug / Telemetry mapping
    dbg.idx_closest = idx_closest; 
    dbg.idx_ld = idx_ld;
    dbg.w_stanley = w_stanley;
    
    % Determine Dominant Mode for Display
    if w_stanley > 0.5
        dbg.mode = sprintf('Stanley (%.0f%%)', w_stanley * 100);
    else
        dbg.mode = sprintf('Pure Pursuit (%.0f%%)', w_pp * 100);
    end
end
