%% Fischer Motor TI085-052-070 Parameters
% Run this script before starting the Simulink simulation.
Motor = struct();
%Motor.Name = "Fischer Motor TI085-052-070"; 
% Mechanical Constants
Motor.gear_ratio = 10.05;
Motor.overall_efficiency = 0.91;
Motor.p = 4;                 % Number of pole pairs 
Motor.J_rotor = 0.33e-3;     % Rotor inertia [kg·m²] [cite: 18, 148]
Motor.n_max = 9000;         % Max mechanical speed [rpm] 
Motor.T_peak_limit = 29.1;   % Absolute peak torque [Nm] [cite: 8, 136]

% Electrical Constants
Motor.L = 0.393e-3;          % Phase inductance [H] 
Motor.kt = 0.492;            % Torque constant [Nm/Arms] 
Motor.I_max_rms = 61;        % Peak current [Arms] [cite: 8, 136]
Motor.tau_el = 0.00311;      % Electrical time constant [s] 

% Derived Parameters
% Flux Linkage: psi_m = kt / (1.5 * p * sqrt(2))
Motor.psi_m = Motor.kt / (1.5 * Motor.p * sqrt(2)); 

fprintf('Motor parameters loaded into workspace.\n');