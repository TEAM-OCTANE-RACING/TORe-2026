% =========================================================================
% LONGITUDINAL DRIVER PID OPTIMIZATION
% =========================================================================

%% =========================================================================
% USER SETTINGS
% =========================================================================

model_name = 'Vehicle_Model_999_Modular';     % <-- CHANGE THIS

stop_time = 50;

% =========================================================================
% PARAMETER SEARCH RANGE
% =========================================================================

Kp_values = 1 : 1 : 50;
Ki_values = 0.00 : 0.1 : 5;

% Fixed feedforward
Kff_controller = 0.15;

% =========================================================================
% PREALLOCATE STORAGE
% =========================================================================

num_tests = length(Kp_values) * length(Ki_values);

results = zeros(num_tests,4);

test_idx = 1;

best_rmse = inf;

best_Kp = NaN;
best_Ki = NaN;

%% =========================================================================
% LOAD MODEL
% =========================================================================

load_system(model_name);

% ============================================================
% SIMULATION SETTINGS
% ============================================================

set_param(model_name, ...
    'StopTime', num2str(stop_time));

set_param(model_name, ...
    'SimulationMode', 'accelerator');

set_param(model_name, ...
    'FastRestart', 'on');

fprintf('\n====================================\n');
fprintf('STARTING PID OPTIMIZATION\n');
fprintf('Total Tests = %d\n', num_tests);
fprintf('====================================\n');

%% =========================================================================
% PARAMETER SWEEP
% =========================================================================

for kp = Kp_values

    for ki = Ki_values

        fprintf('\n------------------------------------\n');
        fprintf('Test %d / %d\n', test_idx, num_tests);
        fprintf('Kp = %.3f | Ki = %.3f\n', kp, ki);

        try

            %% =============================================================
            % ASSIGN PARAMETERS
            % =============================================================

            assignin('base', 'Kp_controller', kp);
            assignin('base', 'Ki_controller', ki);
            assignin('base', 'Kff_controller', Kff_controller);

            %% =============================================================
            % RUN SIMULATION
            % =============================================================

            simOut = sim(model_name);

           %% =============================================================
% EXTRACT LOGGED DATA
% =============================================================

try

    % ---------------------------------------------------------
    % METHOD 1
    % Dataset object named "out"
    % ---------------------------------------------------------

    out_data = simOut.get('out');

    vel_signal = out_data.get('Longitudinal_vel');

    sim_time = vel_signal.Values.Time(:);

    sim_vel = squeeze(vel_signal.Values.Data);
    sim_vel = sim_vel(:);

catch

    try

        % -----------------------------------------------------
        % METHOD 2
        % Signal directly inside simOut
        % -----------------------------------------------------

        vel_signal = simOut.get('Longitudinal_vel');

        sim_time = vel_signal.Time(:);

        sim_vel = squeeze(vel_signal.Data);
        sim_vel = sim_vel(:);

    catch

        try

            % -------------------------------------------------
            % METHOD 3
            % Signal directly in workspace
            % -------------------------------------------------

            sim_time = Longitudinal_vel.Time(:);

            sim_vel = squeeze(Longitudinal_vel.Data);
            sim_vel = sim_vel(:);

        catch

            error('Could not find Longitudinal_vel signal.');

        end
    end
end

            %% =============================================================
            % TARGET DATA
            % =============================================================

            tgt_time = vel_ts.Time(:);
            tgt_vel  = vel_ts.Data(:);

            %% =============================================================
            % INTERPOLATE TARGET
            % =============================================================

            vel_ref_synced = interp1( ...
                tgt_time, ...
                tgt_vel, ...
                sim_time, ...
                'linear', ...
                'extrap');

            %% =============================================================
            % COMPUTE ERROR
            % =============================================================

            error_data = vel_ref_synced - sim_vel;

            rmse = sqrt(mean(error_data.^2, 'omitnan'));

            max_error = max(abs(error_data), [], 'omitnan');

            %% =============================================================
            % STORE RESULTS
            % =============================================================

            results(test_idx,:) = ...
                [kp, ki, rmse, max_error];

            %% =============================================================
            % DISPLAY
            % =============================================================

            fprintf('RMSE      = %.4f m/s\n', rmse);
            fprintf('Max Error = %.4f m/s\n', max_error);

            %% =============================================================
            % UPDATE BEST RESULT
            % =============================================================

            if rmse < best_rmse

                best_rmse = rmse;

                best_Kp = kp;
                best_Ki = ki;

                fprintf('*** NEW BEST RESULT ***\n');

            end

        catch ME

            fprintf('SIMULATION FAILED\n');
            fprintf('%s\n', ME.message);

            results(test_idx,:) = [kp, ki, NaN, NaN];

        end

        test_idx = test_idx + 1;

    end
end

%% =========================================================================
% FINAL RESULTS
% =========================================================================

fprintf('\n\n====================================\n');
fprintf('OPTIMIZATION COMPLETE\n');
fprintf('====================================\n');

fprintf('Best Kp        = %.4f\n', best_Kp);
fprintf('Best Ki        = %.4f\n', best_Ki);
fprintf('Minimum RMSE   = %.4f m/s\n', best_rmse);

%% =========================================================================
% CONVERT TO TABLE
% =========================================================================

results_table = array2table( ...
    results, ...
    'VariableNames', ...
    {'Kp','Ki','RMSE','MaxError'});

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

% save('PID_Optimization_Results.mat', ...
%      'results_table', ...
%      'best_Kp', ...
%      'best_Ki', ...
%      'best_rmse');

%% =========================================================================
% HEATMAP SCATTER
% =========================================================================

figure('Color','w');

scatter3( ...
    results(:,1), ...
    results(:,2), ...
    results(:,3), ...
    80, ...
    results(:,3), ...
    'filled');

xlabel('Kp');
ylabel('Ki');
zlabel('RMSE');

title('PID Optimization Results');

grid on;
colorbar;

%% =========================================================================
% TURN OFF FAST RESTART
% =========================================================================

set_param(model_name, 'FastRestart', 'off');