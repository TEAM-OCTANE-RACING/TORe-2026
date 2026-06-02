% Loading Suspensions parameters: 

% suspension and damper constants.

% Initialize structure
sus_par = struct();

% Spring data
sus_par.Ks_f = 14375.89; % spring stiffness for front spring in N/m.
sus_par.Ks_r = 22280.28; % spring stiffness for rear spring in N/m.

% ARB Data
sus_par.K_arb_f = 168.64; % ARB stiffness_front.
sus_par.K_arb_r = 66.16; % ARB stiffness_rear.

% Dampers data
sus_par.C_damp_f = 14143; % Front damper coefficient in Ns/m.
sus_par.C_damp_r = 16358; % Rear damper coefficient in Ns/m.

% Motion ratio data
sus_par.Motion_ratio_f = 1.03; % Motion ratio_front.
sus_par.Motion_ratio_r = 0.96; % Motion ratio_rear.

% Rolling stiffness
sus_par.K_phi_f = 10021.96; % Front rolling stiffness.
sus_par.K_phi_r = 12876.05; % Rear rolling stiffness.

% Static values of angles(Camber, Caster and Toe)
sus_par.Camber_static = 0; % Static camber front.
sus_par.Acc_pct = 0.6173;
% sus_par.Camber_static_r = 0; % Static camber rear
sus_par.Toe_f_static = 0; % Static toe angle_front.
sus_par.Toe_r_static = 0; % Static toe angle_rear.
sus_par.Caster_static = 0; % Static Caster angle.

% Compliance coefficients for FX,Fy and Mz
sus_par.C_Fx_toe = 0; % Toe compliance for Fx.
sus_par.C_Fy_toe = 0; % Toe compliance for Fy.
sus_par.C_Mz_toe = 0; % Toe compliance for Mz.

% Roll center height
sus_par.RC_h_f = 0.0791; %Front roll center height
sus_par.RC_h_r = 0.0814; % Rear roll center height

% Camber geometry
sus_par.IC_y_f = 0.20; % Front instantenous center horizontal distance
sus_par.IC_y_r = 0.22; % Rear instantenous center horizontal distance

