%% Brake System Parameters
% Based on vehicle design data
Brake=struct();
% 1. Maximum System Torques (Total per Axle)
% These values assume 100% brake pressure and ideal friction.
Brake.MaxTorque_FrontSystem = 800.68; % Nm
Brake.MaxTorque_RearSystem  = 286.49; % Nm

% 2. Per-Wheel Limits
% Divide system torque by 2 for individual corners
Brake.MaxTorque_Wheel_F = Brake.MaxTorque_FrontSystem / 2;
Brake.MaxTorque_Wheel_R = Brake.MaxTorque_RearSystem / 2;

% 3. Simulation Logic Constants
% Threshold to cut off braking torque to prevent reverse oscillation
Brake.ZeroSpeedThreshold = 0.5; % rad/s (approx 0.1 m/s)

% 4. System Latency (For Driverless/FSD Context)
% Time delay for hydraulic pressure buildup [cite: 2025-12-21]
%Brake.ActuatorDelay = 0.05; % Seconds

fprintf('Brake parameters loaded: Front Max = %.2f Nm, Rear Max = %.2f Nm\n', ...
    Brake.MaxTorque_Wheel_F, Brake.MaxTorque_Wheel_R);