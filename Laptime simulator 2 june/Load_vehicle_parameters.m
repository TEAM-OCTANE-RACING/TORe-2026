% Loading vehicle parameters: 

%initiliaze structure 
Par = struct();

Par.g = 9.81; % Acceleration due to gravity.
Par.R0 = 0.228; % Wheel radius in m.
Par.m = 270; % Vehicle mass in kg.
Par.unsprung_mass = [12;12;18;18];  % Unsprung mass at each wheel.
Par.sprung_mass = 270 -(60);  % Sprung mass.
Par.d_Cg_Cp = 0; % Distance between COP and CG.
Par.WD_f = 0.45; % Front weight distribution.
Par.WD_r = 1 - Par.WD_f; % Rear weight distribution. 
Par.C_rr = 0.0150; % Rolling resistance coefficient.

% Wheel base and track width
Par.WB = 1.532; % Wheel base of the vehicle.
Par.Lf = Par.WB * Par.WD_r; % Front wheel base.
Par.Lr = Par.WB * Par.WD_f; % Rear wheel base.
% Par.Tf = 1.235; % Front track width.
% Par.Tr = 1.235; % Rear track width.
Par.TW = 1.235;

% CG and RC heights
Par.RC_h = 0.080365; % Roll center height in m.
Par.CG_h = 0.21; % CG height in m.

% Inertia values
Par.Ix = 62; %Inertia about roll axis
Par.Iy = 105; %Inertia about pitch axis
Par.Iz = 47.563; %Inertia about yaw axis
Par.Iw = [0.74;0.74;0.7733;0.7733]; % Wheel inertia



