% =========================================================================
% MASTER ENDURANCE SIMULATION SCRIPT (22km + Energy Calculation)
% UPDATED VERSION:
% 1. Stores FIRST LAP separately as timeseries
% 2. Generates control inputs from FIRST LAP itself
% 3. Exports BOTH:
%       - First Lap Inputs
%       - Full 22km Inputs
% =========================================================================

% =========================================================================
% --- 0. SIMULATION PARAMETERS ---
% =========================================================================

TARGET_DISTANCE_M = 22000;

% Vehicle Parameters
Par.m        = 270;
aero_par.Area= 0.99;
aero_par.rho = 1.225;

Par.C_rr     = 0.015;
Par.Rw       = 0.228;
Par.GR       = 10.05;
Par.WB       = 1.532;
Par.K_us     = -7.143e-7;

Par.T_max_Nm = 29.1;
Par.P_max_kW = 80;

MASS_KG      = Par.m;
CdA          = aero_par.Area;
RHO          = aero_par.rho;
ROLLING_RES  = Par.C_rr;

% =========================================================================
% --- 1. LOAD INPUTS ---
% =========================================================================

ggv_file = 'GGV_Final_Data.mat';

if ~exist(ggv_file, 'file')

    fprintf('Searching alternative GGV files...\n');

    potential_files = {
        'GGV_Sensor_Data.mat'
        'GGV_Safe_Data.mat'
        'GGV_Map.mat'
        };

    found = false;

    for k = 1:length(potential_files)

        if exist(potential_files{k}, 'file')

            ggv_file = potential_files{k};
            found = true;

            fprintf('Found: %s\n', ggv_file);
            break;

        end
    end

    if ~found
        error('No GGV file found.');
    end
end

load(ggv_file, 'results');

if ~exist('results', 'var')
    error('GGV file does not contain results structure.');
end

track_file = ...
'C:\Disk_E\TOR_controls\Modular_vehicle_model_23_May\Kari_FB.csv';

if ~exist(track_file, 'file')

    track_file = 'FB Track.csv';

    if ~exist(track_file, 'file')
        error('Track file not found.');
    end
end

% =========================================================================
% --- 2. OPTIMIZE RACING LINE ---
% =========================================================================

fprintf('\n=== STEP 1 : OPTIMIZE RACING LINE ===\n');

[path_s, path_k, path_X, path_Y] = ...
    Optimize_Racing_Line(track_file);

% =========================================================================
% --- 3. PROCESS GGV ---
% =========================================================================

fprintf('\n=== STEP 2 : PROCESS GGV ===\n');

GGV_Env = Process_GGV_Envelope(results);

% =========================================================================
% --- 4. RUN ENDURANCE SIMULATION ---
% =========================================================================

fprintf('\n=== STEP 3 : RUN ENDURANCE ===\n');

Track_Length_m = path_s(end);

Num_Laps = ceil(TARGET_DISTANCE_M / Track_Length_m);

fprintf('Track Length : %.2f m\n', Track_Length_m);
fprintf('Num Laps     : %d\n', Num_Laps);

% --- Standing Start Lap
fprintf('\nRunning Standing Start Lap...\n');

Results_SS = Solve_3Pass(path_s, path_k, GGV_Env, 0);

% --- Flying Lap
fprintf('Running Flying Lap...\n');

v_flying_start = Results_SS.v(end);

Results_FL = Solve_3Pass( ...
    path_s, ...
    path_k, ...
    GGV_Env, ...
    v_flying_start);

Endurance_Time = ...
    Results_SS.laptime + ...
    (Num_Laps - 1) * Results_FL.laptime;

% =========================================================================
% --- 5. GENERATE CONTROLS ---
% =========================================================================

fprintf('\n=== STEP 4 : GENERATE CONTROLS ===\n');

Controls_SS = ...
    Generate_Control_Inputs( ...
    Results_SS, ...
    Par, ...
    aero_par, ...
    path_k);

Controls_FL = ...
    Generate_Control_Inputs( ...
    Results_FL, ...
    Par, ...
    aero_par, ...
    path_k);

% =========================================================================
% --- FIRST LAP DATA ---
% =========================================================================

fprintf('\nGenerating FIRST LAP timeseries...\n');

firstlap_time = Results_SS.time;
firstlap_s    = Results_SS.s;
firstlap_v    = Results_SS.v;

firstlap_thr  = Controls_SS.Throttle;
firstlap_brk  = Controls_SS.Brake;
firstlap_str  = Controls_SS.Steer_Angle_Deg;

% Normalize

if max(firstlap_thr) > 0
    firstlap_thr_norm = ...
        firstlap_thr ./ max(firstlap_thr);
else
    firstlap_thr_norm = firstlap_thr;
end

if max(firstlap_brk) > 0
    firstlap_brk_norm = ...
        firstlap_brk ./ max(firstlap_brk);
else
    firstlap_brk_norm = firstlap_brk;
end

% =========================================================================
% --- CREATE FIRST LAP TIMESERIES ---
% =========================================================================

vel_ts_firstlap = ...
    timeseries(firstlap_v, firstlap_time);

dist_ts_firstlap = ...
    timeseries(firstlap_s, firstlap_time);

thr_ts_firstlap = ...
    timeseries(firstlap_thr, firstlap_time);

brk_ts_firstlap = ...
    timeseries(firstlap_brk, firstlap_time);

str_ts_firstlap = ...
    timeseries(firstlap_str, firstlap_time);

thr_norm_ts_firstlap = ...
    timeseries(firstlap_thr_norm, firstlap_time);

brk_norm_ts_firstlap = ...
    timeseries(firstlap_brk_norm, firstlap_time);

% =========================================================================
% --- EXPORT FIRST LAP TIMESERIES ---
% =========================================================================

assignin('base', 'vel_ts_firstlap', vel_ts_firstlap);
assignin('base', 'dist_ts_firstlap', dist_ts_firstlap);

assignin('base', 'thr_ts_firstlap', thr_ts_firstlap);
assignin('base', 'brk_ts_firstlap', brk_ts_firstlap);
assignin('base', 'str_ts_firstlap', str_ts_firstlap);

assignin('base', ...
    'thr_norm_ts_firstlap', ...
    thr_norm_ts_firstlap);

assignin('base', ...
    'brk_norm_ts_firstlap', ...
    brk_norm_ts_firstlap);

fprintf('FIRST LAP timeseries exported.\n');

% =========================================================================
% --- BUILD COMPLETE 22km PROFILE ---
% =========================================================================

fprintf('\nGenerating 22km profile...\n');

v_22km    = Results_SS.v;
s_22km    = Results_SS.s;
time_22km = Results_SS.time;

thr_22km  = Controls_SS.Throttle;
brk_22km  = Controls_SS.Brake;
str_22km  = Controls_SS.Steer_Angle_Deg;

cum_time = Results_SS.laptime;
cum_dist = Results_SS.s(end);

for lap = 2:Num_Laps

    v_22km = [
        v_22km;
        Results_FL.v(2:end)
        ];

    s_22km = [
        s_22km;
        Results_FL.s(2:end) + cum_dist
        ];

    time_22km = [
        time_22km;
        Results_FL.time(2:end) + cum_time
        ];

    thr_22km = [
        thr_22km;
        Controls_FL.Throttle(2:end)
        ];

    brk_22km = [
        brk_22km;
        Controls_FL.Brake(2:end)
        ];

    str_22km = [
        str_22km;
        Controls_FL.Steer_Angle_Deg(2:end)
        ];

    cum_time = cum_time + Results_FL.laptime;
    cum_dist = cum_dist + Results_FL.s(end);

end

% =========================================================================
% --- NORMALIZE FULL ENDURANCE INPUTS ---
% =========================================================================

if max(thr_22km) > 0
    thr_norm_22km = thr_22km ./ max(thr_22km);
else
    thr_norm_22km = thr_22km;
end

if max(brk_22km) > 0
    brk_norm_22km = brk_22km ./ max(brk_22km);
else
    brk_norm_22km = brk_22km;
end

% =========================================================================
% --- CREATE FULL ENDURANCE TIMESERIES ---
% =========================================================================

vel_ts = timeseries(v_22km, time_22km);

dist_ts = timeseries(s_22km, time_22km);

thr_ts = timeseries(thr_22km, time_22km);

brk_ts = timeseries(brk_22km, time_22km);

str_ts = timeseries(str_22km, time_22km);

thr_norm_ts = timeseries(thr_norm_22km, time_22km);

brk_norm_ts = timeseries(brk_norm_22km, time_22km);

% =========================================================================
% --- EXPORT FULL ENDURANCE TIMESERIES ---
% =========================================================================

assignin('base', 'vel_ts', vel_ts);
assignin('base', 'dist_ts', dist_ts);

assignin('base', 'thr_ts', thr_ts);
assignin('base', 'brk_ts', brk_ts);
assignin('base', 'str_ts', str_ts);

assignin('base', 'thr_norm_ts', thr_norm_ts);
assignin('base', 'brk_norm_ts', brk_norm_ts);

fprintf('22km timeseries exported.\n');

% =========================================================================
% --- ENERGY CALCULATION ---
% =========================================================================

fprintf('\n=== STEP 5 : ENERGY ===\n');

[E_SS_kWh, P_SS_kW] = ...
    Calculate_Energy( ...
    Results_SS, ...
    MASS_KG, ...
    CdA, ...
    RHO, ...
    ROLLING_RES);

[E_FL_kWh, P_FL_kW] = ...
    Calculate_Energy( ...
    Results_FL, ...
    MASS_KG, ...
    CdA, ...
    RHO, ...
    ROLLING_RES);

Total_Energy_kWh = ...
    E_SS_kWh + ...
    (Num_Laps - 1) * E_FL_kWh;

Results = Results_FL;

Results.Power_kW  = P_FL_kW;

Results.steer_deg = ...
    Controls_FL.Steer_Angle_Deg;

% =========================================================================
% --- RESULTS ---
% =========================================================================

fprintf('\n=== SIMULATION COMPLETE ===\n');

fprintf('Standing Start Lap : %.3f s\n', ...
    Results_SS.laptime);

fprintf('Flying Lap         : %.3f s\n', ...
    Results_FL.laptime);

fprintf('Total Endurance    : %.3f s\n', ...
    Endurance_Time);

fprintf('Total Energy       : %.3f kWh\n', ...
    Total_Energy_kWh);

fprintf('Max Speed          : %.1f km/h\n', ...
    max(Results.v)*3.6);


% =========================================================================
% --- VISUALIZATION SECTION ---
% =========================================================================

figure( ...
    'Name', 'Endurance Analysis', ...
    'Color', 'k', ...
    'Position', [50 50 1400 900]);

t = tiledlayout(2,3, ...
    'TileSpacing', 'compact', ...
    'Padding', 'compact');

v_min_kph = min(Results.v) * 3.6;

v_max_kph = max(Results.v) * 3.6;

c_limits = [v_min_kph v_max_kph];

% =========================================================================
% --- TRACK MAP ---
% =========================================================================

nexttile([2 1]);

hold on;

axis equal;

grid on;

set(gca, ...
    'Color', 'k', ...
    'XColor', 'w', ...
    'YColor', 'w', ...
    'GridColor', 'w', ...
    'GridAlpha', 0.2);

z = zeros(size(path_X));

col = Results.v * 3.6;

surface( ...
    [path_X'; path_X'], ...
    [path_Y'; path_Y'], ...
    [z'; z'], ...
    [col'; col'], ...
    'FaceColor', 'no', ...
    'EdgeColor', 'interp', ...
    'LineWidth', 3);

colormap(gca, 'turbo');

clim(c_limits);

c = colorbar('southoutside');

c.Color = 'w';

c.Label.String = 'Speed [km/h]';

c.Label.Color = 'w';

title( ...
    ['Endurance : ' num2str(Num_Laps) ' Laps'], ...
    'Color', 'w');

xlabel('X [m]');

ylabel('Y [m]');

% =========================================================================
% --- SPEED TRACE ---
% =========================================================================

nexttile;

plot( ...
    Results.s, ...
    Results.v * 3.6, ...
    'LineWidth', 2, ...
    'Color', '#4DBEEE');

grid on;

xlim([0 max(Results.s)]);

set(gca, ...
    'Color', 'k', ...
    'XColor', 'w', ...
    'YColor', 'w', ...
    'GridColor', 'w', ...
    'GridAlpha', 0.2);

ylabel('Speed [km/h]', 'Color', 'w');

title( ...
    ['Flying Lap : ' ...
    num2str(Results.laptime,'%.2f') ' s'], ...
    'Color', 'w');

% =========================================================================
% --- POWER TRACE ---
% =========================================================================

nexttile;

area( ...
    Results.s, ...
    Results.Power_kW, ...
    'FaceColor', '#77AC30', ...
    'FaceAlpha', 0.5, ...
    'EdgeColor', 'w');

grid on;

xlim([0 max(Results.s)]);

set(gca, ...
    'Color', 'k', ...
    'XColor', 'w', ...
    'YColor', 'w', ...
    'GridColor', 'w', ...
    'GridAlpha', 0.2);

ylabel('Power [kW]', 'Color', 'w');

title( ...
    ['Total Energy : ' ...
    num2str(Total_Energy_kWh,'%.2f') ...
    ' kWh'], ...
    'Color', 'w');

xlabel('Distance [m]', 'Color', 'w');

% =========================================================================
% --- STEERING TRACE ---
% =========================================================================

nexttile;

plot( ...
    Results.s, ...
    Results.steer_deg, ...
    'LineWidth', 1.5, ...
    'Color', '#D95319');

grid on;

xlim([0 max(Results.s)]);

set(gca, ...
    'Color', 'k', ...
    'XColor', 'w', ...
    'YColor', 'w', ...
    'GridColor', 'w', ...
    'GridAlpha', 0.2);

ylabel('Steer [deg]', 'Color', 'w');

xlabel('Distance [m]', 'Color', 'w');

% =========================================================================
% --- GGV USAGE ---
% =========================================================================

nexttile;

hold on;

grid on;

axis equal;

set(gca, ...
    'Color', 'k', ...
    'XColor', 'w', ...
    'YColor', 'w', ...
    'GridColor', 'w', ...
    'GridAlpha', 0.2);

scatter( ...
    Results.ay_used/9.81, ...
    Results.ax_used/9.81, ...
    15, ...
    Results.v*3.6, ...
    'filled');

xlabel('Lat G');

ylabel('Long G');

title('G-G Diagram', 'Color', 'w');

xlim([-3 3]);

ylim([-3 3]);

colormap(gca, 'turbo');

clim(c_limits);

% =========================================================================
% --- VELOCITY TRACKING ERROR ---
% =========================================================================

if exist('out', 'var') && ...
        isfield(out, 'Longitudinal_vel')

    fprintf('\n=== COMPUTE TRACKING ERROR ===\n');

    try

        sim_time = ...
            double( ...
            squeeze( ...
            out.Longitudinal_vel.Time(:)));

        sim_vel = ...
            double( ...
            squeeze( ...
            out.Longitudinal_vel.Data(:)));

        tgt_time = ...
            double( ...
            squeeze( ...
            vel_ts.Time(:)));

        tgt_vel = ...
            double( ...
            squeeze( ...
            vel_ts.Data(:)));

        [sim_time_clean, unique_idx] = ...
            unique(sim_time, 'stable');

        sim_vel_clean = sim_vel(unique_idx);

        vel_ref_synced = interp1( ...
            tgt_time, ...
            tgt_vel, ...
            sim_time_clean, ...
            'linear', ...
            'extrap');

        error_data = ...
            vel_ref_synced - sim_vel_clean;

        max_error = ...
            max(abs(error_data), [], 'omitnan');

        rmse = ...
            sqrt(mean(error_data.^2, 'omitnan'));

        fprintf( ...
            'Max Velocity Error : %.3f m/s\n', ...
            max_error);

        fprintf( ...
            'RMSE : %.3f m/s\n', ...
            rmse);

        figure( ...
            'Name', 'Velocity Tracking Error', ...
            'Color', 'k', ...
            'Position', [150 150 800 600]);

        t2 = tiledlayout(2,1, ...
            'TileSpacing', 'compact', ...
            'Padding', 'compact');

        % --- Velocity Comparison
        nexttile;

        hold on;

        grid on;

        plot( ...
            tgt_time, ...
            tgt_vel, ...
            'w--', ...
            'LineWidth', 1.5, ...
            'DisplayName', 'Target');

        plot( ...
            sim_time_clean, ...
            sim_vel_clean, ...
            '-', ...
            'Color', '#4DBEEE', ...
            'LineWidth', 1.5, ...
            'DisplayName', 'Actual');

        set(gca, ...
            'Color', 'k', ...
            'XColor', 'w', ...
            'YColor', 'w', ...
            'GridColor', 'w', ...
            'GridAlpha', 0.2);

        ylabel('Velocity [m/s]', ...
            'Color', 'w');

        title( ...
            'Velocity Tracking', ...
            'Color', 'w');

        legend( ...
            'TextColor', 'w', ...
            'Color', 'none', ...
            'EdgeColor', 'none');

        % --- Error Plot
        nexttile;

        plot( ...
            sim_time_clean, ...
            error_data, ...
            '-', ...
            'Color', '#D95319', ...
            'LineWidth', 1.5);

        grid on;

        set(gca, ...
            'Color', 'k', ...
            'XColor', 'w', ...
            'YColor', 'w', ...
            'GridColor', 'w', ...
            'GridAlpha', 0.2);

        xlabel('Time [s]', 'Color', 'w');

        ylabel('Error [m/s]', 'Color', 'w');

        title( ...
            sprintf( ...
            'Tracking Error (RMSE = %.2f m/s)', ...
            rmse), ...
            'Color', 'w');

    catch ME

        fprintf( ...
            'Tracking Error Failed : %s\n', ...
            ME.message);

    end
end

% =========================================================================
% --- HELPER FUNCTIONS ---
% =========================================================================

function [s_final, k_final, x_final, y_final] = ...
    Optimize_Racing_Line(filename)

    data = readmatrix(filename);

    xc = data(:,1);
    yc = data(:,2);

    wl = data(:,3);
    wr = data(:,4);

    if norm([xc(1)-xc(end), yc(1)-yc(end)]) > 0.1

        xc(end+1)=xc(1);
        yc(end+1)=yc(1);

        wl(end+1)=wl(1);
        wr(end+1)=wr(1);

    end

    N = length(xc);

    x = xc;
    y = yc;

    Iterations = 200;

    Safety_Margin = 0.5;

    for iter = 1:Iterations

        xt = 0.5 * ...
            ([x(end-1); x(1:end-1)] + ...
             [x(2:end); x(2)]);

        yt = 0.5 * ...
            ([y(end-1); y(1:end-1)] + ...
             [y(2:end); y(2)]);

        for i = 1:N

            dx = xt(i) - xc(i);
            dy = yt(i) - yc(i);

            dist = sqrt(dx^2 + dy^2);

            if i < N
                tx = xc(i+1)-xc(i);
                ty = yc(i+1)-yc(i);
            else
                tx = xc(2)-xc(1);
                ty = yc(2)-yc(1);
            end

            if (tx*dy - ty*dx) > 0
                max_w = wl(i)-Safety_Margin;
            else
                max_w = wr(i)-Safety_Margin;
            end

            if max_w < 0.1
                max_w = 0.1;
            end

            if dist > max_w

                scale = max_w / dist;

                x(i) = xc(i) + dx * scale;
                y(i) = yc(i) + dy * scale;

            else

                x(i) = xt(i);
                y(i) = yt(i);

            end
        end
    end

    dx = gradient(x);
    dy = gradient(y);

    ddx = gradient(dx);
    ddy = gradient(dy);

    k_raw = ...
        (dx .* ddy - dy .* ddx) ./ ...
        ((dx.^2 + dy.^2).^1.5);

    k_final = smoothdata(k_raw, 'gaussian', 15);

    ds = sqrt(dx.^2 + dy.^2);

    s_final = [0; cumsum(ds(2:end))];

    x_final = x;
    y_final = y;

end

function Env = Process_GGV_Envelope(results)

    Env.v = results.v(:);

    Env.ay_max = abs(results.ay_max(:)) * 9.81;

    Env.ax_acc = results.ax_max(:) * 9.81;

    Env.ax_brk = abs(results.ax_brk(:)) * 9.81;

    [Env.v, idx] = sort(Env.v);

    Env.ay_max = Env.ay_max(idx);

    Env.ax_acc = Env.ax_acc(idx);

    Env.ax_brk = Env.ax_brk(idx);

end

% =========================================================================

function R = Solve_3Pass(s, k, Env, v_start)

    if nargin < 4
        v_start = 0;
    end

    N = length(s);

    v_apex = zeros(N,1);

    v_acc  = zeros(N,1);

    ax_acc_log = zeros(N,1);

    v_brk = zeros(N,1);

    ax_brk_log = zeros(N,1);

    % --- Apex Speed
    for i = 1:N

        curv = abs(k(i));

        if curv < 1e-4

            v_apex(i) = max(Env.v);

        else

            v_g = 12;

            for iter = 1:15

                ay_lim = interp1( ...
                    Env.v, ...
                    Env.ay_max, ...
                    v_g, ...
                    'linear', ...
                    'extrap');

                v_new = sqrt(ay_lim / curv);

                v_g = 0.6*v_g + 0.4*v_new;

            end

            v_apex(i) = min(v_g,21.4);

        end
    end

    % --- Forward Pass
    v_acc(1) = v_start;

    for i = 1:N-1

        ds = s(i+1) - s(i);

        v_curr = v_acc(i);

        ay_req = v_curr^2 * abs(k(i));

        ay_max_v = interp1( ...
            Env.v, ...
            Env.ay_max, ...
            v_curr, ...
            'linear', ...
            'extrap');

        ax_max_v = interp1( ...
            Env.v, ...
            Env.ax_acc, ...
            v_curr, ...
            'linear', ...
            'extrap');

        if ay_req >= ay_max_v

            ax_avail = 0;

        else

            ax_avail = ...
                ax_max_v * ...
                sqrt(1 - (ay_req/ay_max_v)^2);

        end

        ax_acc_log(i) = ax_avail;

        v_next = sqrt(v_curr^2 + 2*ax_avail*ds);

        v_acc(i+1) = min(v_next, v_apex(i+1));

    end

    ax_acc_log(N) = 0;

    % --- Backward Pass
    v_brk(N) = v_apex(N);

    for i = N:-1:2

        ds = s(i) - s(i-1);

        v_curr = v_brk(i);

        ay_req = v_curr^2 * abs(k(i));

        ay_max_v = interp1( ...
            Env.v, ...
            Env.ay_max, ...
            v_curr, ...
            'linear', ...
            'extrap');

        ax_brk_v = abs(interp1( ...
            Env.v, ...
            Env.ax_brk, ...
            v_curr, ...
            'linear', ...
            'extrap'));

        if ay_req >= ay_max_v

            ax_avail = 0;

        else

            ax_avail = ...
                ax_brk_v * ...
                sqrt(1 - (ay_req/ay_max_v)^2);

        end

        ax_brk_log(i) = -ax_avail;

        v_prev = sqrt(v_curr^2 + 2*ax_avail*ds);

        v_brk(i-1) = min(v_prev, v_apex(i-1));

    end

    ax_brk_log(1) = ...
        -abs(interp1( ...
        Env.v, ...
        Env.ax_brk, ...
        v_brk(1), ...
        'linear', ...
        'extrap'));

    % --- Final Velocity
    v_final = zeros(N,1);

    ax_final = zeros(N,1);

    for i = 1:N

        [v_lim, idx] = min([ ...
            v_acc(i), ...
            v_brk(i), ...
            v_apex(i)]);

        v_final(i) = v_lim;

        if idx == 1

            ax_final(i) = ax_acc_log(i);

        elseif idx == 2

            ax_final(i) = ax_brk_log(i);

        else

            ax_final(i) = 0;

        end
    end

    v_avg = ...
        max( ...
        (v_final(1:end-1) + v_final(2:end))/2, ...
        0.001);

    dt = diff(s) ./ v_avg;

    time_array = [0; cumsum(dt)];

    R.s = s;

    R.time = time_array;

    R.v = v_final;

    R.ay_used = v_final.^2 .* k;

    R.ax_used = ax_final;

    R.laptime = time_array(end);

end

% =========================================================================

function [E_kWh, P_kW] = ...
    Calculate_Energy(Results, m, CdA, rho, Cr)

    v = Results.v;

    a = Results.ax_used;

    F_aero = 0.5 * rho * CdA * v.^2;

    F_roll = Cr * m * 9.81;

    F_acc  = m * a;

    F_total = F_aero + F_roll + F_acc;

    P_mech_W = F_total .* v;

    eff_motor = 0.90;

    P_elec_W = zeros(size(P_mech_W));

    for i = 1:length(P_mech_W)

        if P_mech_W(i) > 0

            P_elec_W(i) = ...
                P_mech_W(i) / eff_motor;

        else

            P_elec_W(i) = 0;

        end
    end

    P_kW = P_elec_W / 1000;

    ds = gradient(Results.s);

    dt = ds ./ max(v, 0.1);

    E_joules = sum(P_elec_W .* dt);

    E_kWh = E_joules / 3.6e6;

end

% =========================================================================

function Controls = ...
    Generate_Control_Inputs( ...
    Results, ...
    Par, ...
    aero_par, ...
    path_k)

    m    = Par.m;

    CdA  = aero_par.Area;

    Cr   = Par.C_rr;

    rho  = aero_par.rho;

    Rw   = Par.Rw;

    GR   = Par.GR;

    L    = Par.WB;

    K_us = Par.K_us;

    v  = Results.v;

    ax = Results.ax_used;

    ay = Results.ay_used;

    k  = path_k;

    N = length(v);

    % --- Steering
    Steer_Geo = atan(L .* k);

    Steer_Dyn = K_us .* (ay / 9.81);

    Controls.Steer_Angle_Deg = ...
        rad2deg(Steer_Geo + Steer_Dyn);

    % --- Forces
    F_aero = 0.5 * rho * CdA * v.^2;

    F_roll = Cr * m * 9.81;

    F_acc  = m .* ax;

    alpha_est = (abs(ay)/9.81) * 0.05;

    F_corn = m .* abs(ay) .* sin(alpha_est);

    F_front_drag = 500;

    F_total = ...
        F_aero + ...
        F_roll + ...
        F_acc + ...
        F_corn + ...
        F_front_drag;

    % --- Limits
    T_max_motor_Nm = Par.T_max_Nm;

    P_max_W = Par.P_max_kW * 1000;

    Throttle_Cmd = zeros(N,1);

    Brake_Cmd = zeros(N,1);

    for i = 1:N

        Tw_limit_mech = T_max_motor_Nm * GR;

        w_wheel = max(v(i)/Rw, 0.1);

        Tw_limit_pwr = P_max_W / w_wheel;

        Tw_max_avail = ...
            min(Tw_limit_mech, Tw_limit_pwr) * 0.95;

        Tw_req_per_motor = ...
            F_total(i) * Rw * 0.5;

        if Tw_req_per_motor >= 0

            Throttle_Cmd(i) = ...
                min(max( ...
                Tw_req_per_motor / Tw_max_avail, ...
                0),1);

            Brake_Cmd(i) = 0;

        else

            Throttle_Cmd(i) = 0;

            F_brake_max = 1 * 9.81 * m;

            Brake_Cmd(i) = ...
                min(max( ...
                abs(F_total(i)) / F_brake_max, ...
                0),1);

        end
    end

    Controls.Throttle = Throttle_Cmd;

    Controls.Brake = Brake_Cmd;

    Controls.F_Total = F_total;

end