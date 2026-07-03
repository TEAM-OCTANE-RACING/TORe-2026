%% Analytical_YMD_Generator.m
% Standalone script to generate a Steady-State Yaw Moment Diagram.
% DOES NOT run any Simulink models. Uses purely analytical math.
% IMPORTANT: Do not add "clear" to the top of this script!

clc; close all;

fprintf('--- GENERATING ANALYTICAL YMD ---\n');

%% 1. Workspace Safety Check
if ~exist('Par', 'var') || ~exist('sus_par', 'var') || ...
   ~exist('aero_par', 'var') || ~exist('TireData', 'var')
    error('Missing required structs! Please load Par, sus_par, aero_par, and TireData into your workspace first.');
end

%% 2. Execute the Analytical Solver
% This calls the function located at the very bottom of this file
fprintf('Calculating kinematic limits...\n');
doPlot = false; 
[ss_Ay, ss_Mz_norm, ss_Steer_Deg, ss_Beta_Deg] = yaw_moment_diagram(Par, sus_par, aero_par, TireData, doPlot);

%% 3.De-Normalize Mz to Raw [Nm]
% Converting the normalized ratio back into actual Newton-meters
normalization_factor = Par.m * 9.81 * Par.WB;
ss_Mz_raw = ss_Mz_norm * normalization_factor;

%% 4. Plot the Yaw Moment Diagram
fprintf('Generating plot...\n');

h_fig = figure('Name', 'Analytical Yaw Moment Diagram', 'Color', 'w', 'Position', [100, 100, 900, 700]);
ax = axes('Parent', h_fig);
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');

% --- Plot the Mesh ---
% Constant Steering Lines (Light Blue)
for i = 1:length(ss_Steer_Deg)
    plot(ax, ss_Ay(i, :), ss_Mz_raw(i, :), 'Color', [0.2 0.6 1.0], 'LineWidth', 1.0); 
end

% Constant Sideslip Lines (Light Red)
for j = 1:length(ss_Beta_Deg)
    plot(ax, ss_Ay(:, j), ss_Mz_raw(:, j), 'Color', [1.0 0.4 0.4], 'LineWidth', 1.0); 
end

% --- Highlight Zero-Lines ---
idx_steer_0 = find(ss_Steer_Deg == 0);
if ~isempty(idx_steer_0)
    plot(ax, ss_Ay(idx_steer_0, :), ss_Mz_raw(idx_steer_0, :), 'b-', 'LineWidth', 2.5);
end

idx_beta_0 = find(ss_Beta_Deg == 0);
if ~isempty(idx_beta_0)
    plot(ax, ss_Ay(:, idx_beta_0), ss_Mz_raw(:, idx_beta_0), 'r-', 'LineWidth', 2.5);
end

% --- Add Origin Axes ---
xline(ax, 0, 'k-', 'LineWidth', 1.5);
yline(ax, 0, 'k-', 'LineWidth', 1.5);

% --- Formatting & Labels ---
title(ax, 'Steady-State Yaw Moment Diagram', 'FontSize', 16, 'FontWeight', 'bold');
xlabel(ax, 'Lateral Acceleration, A_y [g]', 'FontSize', 14, 'FontWeight', 'bold');
ylabel(ax, 'Total Yaw Moment, M_z [Nm]', 'FontSize', 14, 'FontWeight', 'bold');

% --- Clean Legend ---
h1 = plot(ax, nan, nan, 'Color', [0.2 0.6 1.0], 'LineWidth', 1.5);
h2 = plot(ax, nan, nan, 'Color', [1.0 0.4 0.4], 'LineWidth', 1.5);
h3 = plot(ax, nan, nan, 'b-', 'LineWidth', 2.5);
h4 = plot(ax, nan, nan, 'r-', 'LineWidth', 2.5);
legend(ax, [h1, h2, h3, h4], ...
    {'Constant Steer (\delta)', 'Constant Sideslip (\beta)', '\delta = 0^\circ', '\beta = 0^\circ'}, ...
    'Location', 'best', 'FontSize', 11);

% Tag this figure so you can easily overlay data onto it later if you want to
set(h_fig, 'Tag', 'YMD_Master_Figure'); 
hold(ax, 'off');

fprintf('Done! YMD generated successfully.\n');


%% ========================================================================
% INTERNAL ANALYTICAL SOLVER (Must remain at the bottom of the script)
% ========================================================================

function [res_Ay, res_Mz, Steer_Deg, Beta_Deg] = yaw_moment_diagram(Par, sus_par, aero_par, TireData, doPlot)
    % Hybrid Yaw Moment Diagram generator
    % -------------------- 1. PARAMETERS --------------------
    m  = Par.m;          
    WB = Par.WB;         
    LF = Par.Lf;         
    LR = Par.Lr;         
    T  = Par.TW;         
    h  = Par.CG_h;       
    V     = 12;          
    rho   = aero_par.rho;
    Cl    = aero_par.Cl;
    Area  = aero_par.Area;
    
    % Roll stiffness & LLTD
    K_phi_F = (0.5 * sus_par.Ks_f * T^2 * sus_par.Motion_ratio_f^2) + sus_par.K_arb_f;
    K_phi_R = (0.5 * sus_par.Ks_r * T^2 * sus_par.Motion_ratio_r^2) + sus_par.K_arb_r;
    LLTD = K_phi_F / (K_phi_F + K_phi_R);
    
    % -------------------- 2. SWEEP SETUP --------------------
    Steer_Deg = -22:2:22;
    Beta_Deg  = -20:0.5:20;
    n_s = length(Steer_Deg);
    n_b = length(Beta_Deg);
    res_Ay = zeros(n_s, n_b);
    res_Mz = zeros(n_s, n_b);
    
    % -------------------- 3. STABLE PHYSICS LOOP --------------------
    for idx_s = 1:n_s
        delta = deg2rad(Steer_Deg(idx_s));
        for idx_b = 1:n_b
            beta = deg2rad(Beta_Deg(idx_b));
            % Base vertical loads (static + aero)
            Fz_F_base = ((m*9.81*LR/WB) + (0.5*rho*V^2*Cl*Area*(LR/WB))) / 2;
            Fz_R_base = ((m*9.81*LF/WB) + (0.5*rho*V^2*Cl*Area*(LF/WB))) / 2;
            
            % Relaxed solver 
            ay_acc = 0.05;      
            relax  = 0.95;
            for k = 1:60
                yaw_rate = (ay_acc * 9.81) / V;
                LT = (m * ay_acc * 9.81 * h) / T;
                % Slip angles
                alpha_F = beta + (LF * yaw_rate / V) - delta;
                alpha_R = beta - (LR * yaw_rate / V);
                % Tire forces 
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
            res_Ay(idx_s, idx_b) = ay_acc;
            res_Mz(idx_s, idx_b) = (Fy_F * LF - Fy_R * LR) / (m * 9.81 * WB);
        end
    end
    
    % ================== HELPER FUNCTION ==================
    function Fy = Pacejka_Hybrid(alpha, Fz, TD)
        Fz0 = TD.FNOMIN;
        dfz = (Fz - Fz0) / Fz0;
        mu = (TD.PDY1 + TD.PDY2 * dfz) * TD.Scales.LMUY;
        D  = mu * Fz;
        C  = TD.PCY1;
        
        Ky_sine   = TD.PKY1 * Fz0 * sin(2 * atan(Fz / (max(0.1, TD.PKY2) * Fz0)));
        Ky_linear = 15 * Fz;   
        Ky = max(0.5 * Ky_linear, Ky_sine);
        
        B  = Ky / (C * D + eps);
        Fy = D * sin(C * atan(B * alpha - TD.PEY1 * (B * alpha - atan(B * alpha))));
    end
end