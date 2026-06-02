% =========================================================================
% COMPUTE VELOCITY TRACKING ERROR (SPATIAL VERSION)
% =========================================================================

% 1. Extract and flatten the Simulink data to guarantee 1D column vectors
sim_time = out.Longitudinal_vel.Time(:);
sim_vel  = squeeze(out.Longitudinal_vel.Data);
sim_vel  = sim_vel(:); % Force into a column vector

% 2. Calculate Actual Distance Traveled (Numerical Integration)
% This calculates distance without needing to route a new output port in Simulink
sim_dist = cumtrapz(sim_time, sim_vel); 

% 3. Extract target data from the timeseries generated in Step 6
tgt_dist = dist_ts.Data(:);
tgt_vel  = vel_ts.Data(:);

% 4. Interpolate the reference velocity onto the Simulink distance steps
% We use interp1 with 'extrap' to prevent NaNs if the car drifts past the exact finish line
vel_ref_synced = interp1(tgt_dist, tgt_vel, sim_dist, 'linear', 'extrap');

% 5. Compute the spatial error (Target Velocity - Actual Simulation Velocity)
error_data = vel_ref_synced - sim_vel;

% 6. Calculate useful metrics
max_error = max(abs(error_data));
rmse = sqrt(mean(error_data.^2));

fprintf('\n=== SPATIAL TRACKING ERROR METRICS ===\n');
fprintf('Max Velocity Error:  %.3f m/s (%.2f km/h)\n', max_error, max_error*3.6);
fprintf('RMS Velocity Error:  %.3f m/s (%.2f km/h)\n', rmse, rmse*3.6);

% 7. Plot the results
figure('Name', 'Velocity Tracking Error (Spatial)', 'Color', 'w');

% Top subplot: Overlay of Target vs Actual
subplot(2,1,1);
hold on; grid on;
plot(tgt_dist, tgt_vel, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Target (Solver)');
plot(sim_dist, sim_vel, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual (Simulink)');
xlabel('Distance [m]');
ylabel('Velocity [m/s]');
title('Velocity Tracking: Target vs. Actual (Spatial)');
legend('Location', 'best');

% Bottom subplot: Tracking Error
subplot(2,1,2);
plot(sim_dist, error_data, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Distance [m]');
ylabel('Error [m/s]');
title(sprintf('Tracking Error (RMSE = %.2f m/s)', rmse));