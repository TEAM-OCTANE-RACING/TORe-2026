%% Molicel P45B 1RC Battery Parameter Database
% This script defines temperature-dependent 2D maps for a 1RC cell model.
% Dimensions: Rows = Temperature (T_vec), Columns = State of Charge (SoC_vec).
Battery = struct();
Battery.Name = 'Molicel_P45B';

%% 1. Dimension Vectors
% Temperature breakpoints in K
Battery.T_vec = [278 293 313]; 
% SoC breakpoints (0 = Empty, 1 = Full)
Battery.SoC_vec = [0 0.1 0.25 0.5 0.75 0.9 1];

%% 2. Open Circuit Voltage (OCV) Map [V]
% OCV is primarily SoC-dependent but varies slightly with Temperature.
% Data points based on P45B discharge curve (2.5V to 4.2V).
Battery.OCV_map = [3.49 3.5 3.51;3.55 3.57 3.56;3.62 3.63 3.64;3.71 3.71 3.72;3.91 3.93 3.94;4.07 4.08 4.08;4.19 4.19 4.19];
%% 3. Ohmic Resistance (R0) Map [Ohms]
% Captures instantaneous voltage sag. Resistance spikes at low temperatures.
% Nominal DCR at 25C is approx. 15 mOhm.
Battery.R0_map = [0.0117 0.0085 0.009;0.011 0.0085 0.009;0.0114 0.0087 0.0092;0.0107 0.0082 0.0088;0.0107 0.0083 0.0091;0.0113 0.0085 0.0089;0.0116 0.0085 0.0089];
%% 4. RC Branch Resistance (R1) Map [Ohms]
% Captures transient voltage drop/charge transfer.
Battery.R1_map = [0.0109 0.0029 0.0013;0.0069 0.0024 0.0012;0.0047 0.0026 0.0013;0.0034 0.0016 0.001;0.0033 0.0023 0.0014;0.0033 0.0018 0.0011;0.0028 0.0017 0.0011];
%% 5. Time Constant (Tau) Map [Seconds]
% Tau = R1 * C1. Defines the speed of voltage recovery.
% Reactions are much slower (higher Tau) at freezing temperatures.
Battery.Tau_map = [20 36 39;31 45 39;109 105 61;36 29 26;59 77 67;40 33 29;25 39 33];

%% 6. Physical and Thermal Constants
Battery.Capacity_Ah = 4.5;                   % Rated capacity
Battery.Mass_kg = 0.070;                     % Cell mass (70g)
Battery.SpecificHeat = 920;                  % J/(kg*K)
Battery.Max_Temp_Limit = 80;                 % Absolute cutoff [Celsius]
Battery.Ns = 144;
Battery.Np = 3;

fprintf('Molicel P45B Thermal-Electrical Struct loaded.\n');