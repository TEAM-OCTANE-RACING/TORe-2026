% Aero Parameters: 
aero_par = struct();

aero_par.Cl = -0.39;  % Lift coefficient.
aero_par.Cd = 0.5;  % Drag coefficient.
aero_par.Area = 0.99;     % Reference area in square meters.
aero_par.CP_rear = 0.5;   % COP rear ratio.
aero_par.rho = 1.225;  % Density of air.
aero_par.C_side = 0; % Sideways_drag.

assignin("base","aero_par",aero_par)
