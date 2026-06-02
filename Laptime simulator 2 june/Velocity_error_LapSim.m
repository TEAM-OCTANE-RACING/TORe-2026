% =========================================================================
% COMPUTE VELOCITY TRACKING ERROR (ROBUST VERSION)
% =========================================================================

% 1. Extract and flatten the Simulink data to guarantee 1D column vectors
sim_time = out.Longitudinal_vel.Time(:);
sim_vel  = squeeze(out.Longitudinal_vel.Data);
sim_vel  = sim_vel(:); % Force into a column vector

% Extract target data from the timeseries
tgt_time = vel_ts.Time(:);
tgt_vel  = vel_ts.Data(:);

% 2. Interpolate the reference velocity onto the Simulink time steps
% We use interp1 with 'extrap' to prevent NaNs if Simulink ran longer than the lap
vel_ref_synced = interp1(tgt_time, tgt_vel, sim_time, 'linear', 'extrap');

% 3. Compute the error (Target Velocity - Actual Simulation Velocity)
error_data = vel_ref_synced - sim_vel;

% 4. Store the error as a timeseries object
vel_error_ts = timeseries(error_data, sim_time);
vel_error_ts.Name = 'Velocity Error (Target - Actual)';
assignin('base', 'vel_error_ts', vel_error_ts);

% 5. Calculate useful metrics
max_error = max(abs(error_data));
rmse = sqrt(mean(error_data.^2));

fprintf('\n=== TRACKING ERROR METRICS ===\n');
fprintf('Max Velocity Error:  %.3f m/s (%.2f km/h)\n', max_error, max_error*3.6);
fprintf('RMS Velocity Error:  %.3f m/s (%.2f km/h)\n', rmse, rmse*3.6);

% 6. Plot the results
figure('Name', 'Velocity Tracking Error', 'Color', 'w');

% Top subplot: Overlay of Target vs Actual
subplot(2,1,1);
hold on; grid on;
plot(tgt_time, tgt_vel, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Target (Solver)');
plot(sim_time, sim_vel, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual (Simulink)');
ylabel('Velocity [m/s]');
title('Velocity Tracking: Target vs. Actual');
legend('Location', 'best');

% Bottom subplot: Tracking Error
subplot(2,1,2);
plot(sim_time, error_data, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Error [m/s]');
title(sprintf('Tracking Error (RMSE = %.2f m/s)', rmse));