% =========================================================================
% MASTER SCRIPT : AUTOMATIC PID OPTIMIZATION
% FOR LONGITUDINAL DRIVER
%
% Optimizes:
%   Kp_controller
%   Ki_controller
%
% Uses:
%   fminsearch (gradient-free optimization)
%
% Requirements:
%   1) Longitudinal_vel logged as TIMESERIES
%   2) vel_ts exists in workspace
%   3) Kp_controller and Ki_controller used in model
%
% =========================================================================

%% =========================================================================
% USER SETTINGS
% =========================================================================

model_name = 'Vehicle_Model_999_Modular';

stop_time = 50;

% Initial guess
initial_Kp = 2;
initial_Ki = 0.5;

% Fixed feedforward
Kff_controller = 0.15;

%% =========================================================================
% LOAD MODEL
% =========================================================================

load_system(model_name);

%% =========================================================================
% SIMULATION SETTINGS
% =========================================================================

set_param(model_name, ...
    'SimulationMode', 'accelerator');

set_param(model_name, ...
    'FastRestart', 'on');

set_param(model_name, ...
    'StopTime', num2str(stop_time));

%% =========================================================================
% STORAGE VARIABLES
% =========================================================================

global optimization_history

optimization_history = [];

%% =========================================================================
% INITIAL GUESS VECTOR
% =========================================================================

x0 = [initial_Kp initial_Ki];

%% =========================================================================
% OPTIMIZATION OPTIONS
% =========================================================================

options = optimset( ...
    'Display', 'iter', ...
    'TolX', 1e-3, ...
    'TolFun', 1e-3, ...
    'MaxIter', 200, ...
    'MaxFunEvals', 100);

%% =========================================================================
% RUN OPTIMIZATION
% =========================================================================

fprintf('\n========================================\n');
fprintf('STARTING PID OPTIMIZATION\n');
fprintf('========================================\n');

[x_best, best_rmse] = fminsearch( ...
    @(x) local_cost_function( ...
        x, ...
        model_name, ...
        Kff_controller), ...
    x0, ...
    options);

%% =========================================================================
% BEST RESULTS
% =========================================================================

best_Kp = x_best(1);
best_Ki = x_best(2);

fprintf('\n\n========================================\n');
fprintf('OPTIMIZATION COMPLETE\n');
fprintf('========================================\n');

fprintf('Best Kp   = %.6f\n', best_Kp);
fprintf('Best Ki   = %.6f\n', best_Ki);
fprintf('Best RMSE = %.6f m/s\n', best_rmse);

%% =========================================================================
% CONVERT HISTORY TO TABLE
% =========================================================================

results_table = array2table( ...
    optimization_history, ...
    'VariableNames', ...
    {'Kp','Ki','RMSE'});

results_table = sortrows(results_table, 'RMSE');

%% =========================================================================
% DISPLAY TOP RESULTS
% =========================================================================

disp(' ');
disp('========== TOP RESULTS ==========');

disp(results_table(1:min(10,height(results_table)), :));

%% =========================================================================
% SAVE RESULTS
% =========================================================================

save('PID_Optimization_Results.mat', ...
    'results_table', ...
    'best_Kp', ...
    'best_Ki', ...
    'best_rmse');

%% =========================================================================
% PLOT OPTIMIZATION HISTORY
% =========================================================================

figure('Color','w');

scatter3( ...
    optimization_history(:,1), ...
    optimization_history(:,2), ...
    optimization_history(:,3), ...
    100, ...
    optimization_history(:,3), ...
    'filled');

xlabel('Kp');
ylabel('Ki');
zlabel('RMSE');

title('PID Optimization History');

grid on;
colorbar;

%% =========================================================================
% FINAL VALIDATION RUN
% =========================================================================

fprintf('\nRunning final validation simulation...\n');

assignin('base', 'Kp_controller', best_Kp);
assignin('base', 'Ki_controller', best_Ki);
assignin('base', 'Kff_controller', Kff_controller);

sim(model_name);

%% =========================================================================
% EXTRACT FINAL DATA
% =========================================================================

sim_time = Longitudinal_vel.Time(:);

sim_vel = squeeze(Longitudinal_vel.Data);
sim_vel = sim_vel(:);

tgt_time = vel_ts.Time(:);
tgt_vel  = vel_ts.Data(:);

vel_ref_synced = interp1( ...
    tgt_time, ...
    tgt_vel, ...
    sim_time, ...
    'linear', ...
    'extrap');

error_data = vel_ref_synced - sim_vel;

%% =========================================================================
% FINAL PLOTS
% =========================================================================

figure('Name', 'Final Tracking Result', 'Color', 'w');

subplot(2,1,1);

hold on;
grid on;

plot(tgt_time, tgt_vel, ...
    'k--', ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Target');

plot(sim_time, sim_vel, ...
    'b-', ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Actual');

ylabel('Velocity [m/s]');

title(sprintf( ...
    'Best PID Tracking | Kp=%.3f Ki=%.3f', ...
    best_Kp, ...
    best_Ki));

legend('Location','best');

subplot(2,1,2);

plot(sim_time, error_data, ...
    'r-', ...
    'LineWidth', 1.5);

grid on;

xlabel('Time [s]');
ylabel('Error [m/s]');

title(sprintf('Tracking Error | RMSE = %.4f m/s', ...
    best_rmse));

%% =========================================================================
% TURN OFF FAST RESTART
% =========================================================================

set_param(model_name, 'FastRestart', 'off');

fprintf('\nDone.\n');

%% =========================================================================
% LOCAL COST FUNCTION
% =========================================================================

function rmse = local_cost_function( ...
    x, ...
    model_name, ...
    Kff_controller)

global optimization_history

% =========================================================================
% EXTRACT PARAMETERS
% =========================================================================

Kp = x(1);
Ki = x(2);

% =========================================================================
% LIMITS / CONSTRAINTS
% =========================================================================

if Kp < 0 || Ki < 0

    rmse = 1e6;
    return;

end

if Kp > 10 || Ki > 10

    rmse = 1e6;
    return;

end

% =========================================================================
% ASSIGN PARAMETERS
% =========================================================================

assignin('base', 'Kp_controller', Kp);
assignin('base', 'Ki_controller', Ki);
assignin('base', 'Kff_controller', Kff_controller);

try

    % =====================================================================
    % RUN SIMULATION
    % =====================================================================

    sim(model_name);

    % =====================================================================
    % EXTRACT DATA
    % =====================================================================

    sim_time = Longitudinal_vel.Time(:);

    sim_vel = squeeze(Longitudinal_vel.Data);
    sim_vel = sim_vel(:);

    tgt_time = evalin('base', 'vel_ts.Time(:)');
    tgt_vel  = evalin('base', 'vel_ts.Data(:)');

    % =====================================================================
    % INTERPOLATE TARGET
    % =====================================================================

    vel_ref_synced = interp1( ...
        tgt_time, ...
        tgt_vel, ...
        sim_time, ...
        'linear', ...
        'extrap');

    % =====================================================================
    % COMPUTE ERROR
    % =====================================================================

    error_data = vel_ref_synced - sim_vel;

    rmse = sqrt(mean(error_data.^2, 'omitnan'));

    % =====================================================================
    % HANDLE BAD RUNS
    % =====================================================================

    if isnan(rmse) || isinf(rmse)

        rmse = 1e6;

    end

catch

    rmse = 1e6;

end

% =========================================================================
% STORE HISTORY
% =========================================================================

optimization_history = [ ...
    optimization_history;
    Kp, Ki, rmse];

% =========================================================================
% DISPLAY
% =========================================================================

fprintf('Kp = %.5f | Ki = %.5f | RMSE = %.5f\n', ...
    Kp, Ki, rmse);

end