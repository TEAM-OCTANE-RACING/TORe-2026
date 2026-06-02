function [res_Ay, res_Mz, Steer_Deg, Beta_Deg] = yaw_moment_diagram(Par, sus_par, aero_par, TireData, doPlot)
%--------------------------------------------------------------------------
% FSAE_YMD_Hybrid_v10
%
% Hybrid Yaw Moment Diagram generator
% Combines:
%   - v6 solver stability (relaxed iteration)
%   - v9 tire stiffness shaping
%
% INPUTS:
%   Par       : Vehicle parameters struct
%   sus_par   : Suspension parameters struct
%   aero_par  : Aero parameters struct
%   TireData  : Tire data struct
%   doPlot    : true / false (enable plotting)
%
% OUTPUTS:
%   res_Ay    : Lateral acceleration matrix [g]
%   res_Mz    : Normalized yaw moment matrix
%   Steer_Deg : Steering sweep vector [deg]
%   Beta_Deg  : Sideslip sweep vector [deg]
%--------------------------------------------------------------------------

%% -------------------- 1. PARAMETERS --------------------
m  = Par.m;          % Vehicle mass [kg]
WB = Par.WB;         % Wheelbase [m]
LF = Par.Lf;         % CG to front axle [m]
LR = Par.Lr;         % CG to rear axle [m]
T  = Par.TW;         % Track width [m]
h  = Par.CG_h;       % CG height [m]

V     = 20;          % Vehicle speed [m/s]
rho   = aero_par.rho;
Cl    = aero_par.Cl;
Area  = aero_par.Area;

%% Roll stiffness & LLTD
K_phi_F = (0.5 * sus_par.Ks_f * T^2 * sus_par.Motion_ratio_f^2) ...
          + sus_par.K_arb_f;

K_phi_R = (0.5 * sus_par.Ks_r * T^2 * sus_par.Motion_ratio_r^2) ...
          + sus_par.K_arb_r;

LLTD = K_phi_F / (K_phi_F + K_phi_R);

%% -------------------- 2. SWEEP SETUP --------------------
Steer_Deg = -22:2:22;
Beta_Deg  = -20:0.5:20;

n_s = length(Steer_Deg);
n_b = length(Beta_Deg);

res_Ay = zeros(n_s, n_b);
res_Mz = zeros(n_s, n_b);

%% -------------------- 3. STABLE PHYSICS LOOP --------------------
for i = 1:n_s
    delta = deg2rad(Steer_Deg(i));

    for j = 1:n_b
        beta = deg2rad(Beta_Deg(j));

        % Base vertical loads (static + aero)
        Fz_F_base = ((m*9.81*LR/WB) + ...
            (0.5*rho*V^2*Cl*Area*(LR/WB))) / 2;

        Fz_R_base = ((m*9.81*LF/WB) + ...
            (0.5*rho*V^2*Cl*Area*(LF/WB))) / 2;

        % Relaxed solver (v6-style)
        ay_acc = 0.05;      % initial guess [g]
        relax  = 0.95;

        for k = 1:60
            yaw_rate = (ay_acc * 9.81) / V;
            LT = (m * ay_acc * 9.81 * h) / T;

            % Slip angles
            alpha_F = beta + (LF * yaw_rate / V) - delta;
            alpha_R = beta - (LR * yaw_rate / V);

            % Tire forces (left + right)
            Fy_F = Pacejka_Hybrid(alpha_F, max(10, Fz_F_base + LT*LLTD), TireData) + ...
                   Pacejka_Hybrid(alpha_F, max(10, Fz_F_base - LT*LLTD), TireData);

            Fy_R = Pacejka_Hybrid(alpha_R, max(10, Fz_R_base + LT*(1-LLTD)), TireData) + ...
                   Pacejka_Hybrid(alpha_R, max(10, Fz_R_base - LT*(1-LLTD)), TireData);

            ay_new = (Fy_F + Fy_R) / (m * 9.81);
            ay_acc = relax * ay_acc + (1 - relax) * ay_new;

            if abs(ay_new - ay_acc) < 1e-6
                break;
            end
        end

        res_Ay(i,j) = ay_acc;
        res_Mz(i,j) = (Fy_F * LF - Fy_R * LR) / (m * 9.81 * WB);
    end
end

%% ================== HELPER FUNCTION ==================

    function Fy = Pacejka_Hybrid(alpha, Fz, TD)
    % Hybrid Pacejka lateral force model
    % Stable at low load & high slip
    
    Fz0 = TD.FNOMIN;
    dfz = (Fz - Fz0) / Fz0;
    
    mu = (TD.PDY1 + TD.PDY2 * dfz) * TD.Scales.LMUY;
    D  = mu * Fz;
    C  = TD.PCY1;
    
    % Sinusoidal stiffness with fallback protection
    Ky_sine   = TD.PKY1 * Fz0 * ...
                sin(2 * atan(Fz / (max(0.1, TD.PKY2) * Fz0)));
    
    Ky_linear = 15 * Fz;   % v6 fallback stiffness
    
    Ky = max(0.5 * Ky_linear, Ky_sine);
    
    B  = Ky / (C * D + eps);
    
    Fy = D * sin(C * atan(B * alpha - ...
         TD.PEY1 * (B * alpha - atan(B * alpha))));
    end



end

