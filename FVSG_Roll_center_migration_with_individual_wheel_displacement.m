function suspension_kinematics_interactive()
    % Front View Suspension Kinematics - Full Visualization (Toe, Camber, IC & RC)
    % Includes Independent L/R Displacement, Track Width, Exact Scrub Radius & RC Migration
    clc; close all;
    
    % --- 1. GEOMETRY INPUTS (Static Constants) ---
    L = 361.09;       % Lower arm length (Chassis Pivot A to Ball Joint C)
    U = 333.684;        % Upper arm length (Chassis Pivot B to Ball Joint D)
    K = 150;           % Knuckle height (Vertical distance between C and D)
    H_chassis = 121.58;   % Vertical spread between chassis mounts A and B
    theta_L0 = -1.149; % Static angle of lower arm (degrees)
    
    % Steering / Tie-Rod Inputs
    d = 40.7;       % Vertical distance from C up to Tie-rod point P
    Sx_offset = 12.232;    % Steering Rack Pivot X offset from lower mount (Chassis mount)
    Sy = 32.752;           % Steering Rack Pivot Y (Chassis mount for tie-rod)
    
    % Accurate Steering Inputs for Toe Calculation
    L_steer = 61.3;    % Steering arm length in mm (Lever arm from steering axis to tie rod)
    steer_config = 1;  % 1 for Front-steer (Rack ahead of axle), -1 for Rear-steer
    
    % Travel range for simulation (Lower ball joint moving up/down from static)
    h_array = -30:0.1:30; 
    
    % --- 1.5 TRACK WIDTH & SCRUB RADIUS CALCULATIONS ---
    Track_Width = 1235;   % Target track width (mm) at static disp = 0
    Scrub_Radius = 9.999; % Target scrub radius (mm) from inner side
    tire_W = 177.8;       % Tire width
    tire_H = 457.193;     % Tire height
    
    Ay = 0;             % Point A: Lower Chassis Pivot Y
    By = H_chassis;     % Point B: Upper Chassis Pivot Y
    
    % Pre-compute kinematics to find Kingpin ground intersection relative to chassis
    C0x_rel = L * cosd(theta_L0);
    C0y = Ay + L * sind(theta_L0);
    dist_BC0 = sqrt((C0x_rel - 0)^2 + (C0y - By)^2); 
    phi0 = atan2(C0y - By, C0x_rel - 0);    % -ve angle made by line joining UCA inner pivot and LCR outer pivot with horizontal         
    alpha0 = acos((U^2 + dist_BC0^2 - K^2) / (2 * U * dist_BC0)); % angle between the line mentioned on the line above and UCR(using cos rule)
    D0x_rel = 0 + U * cos(phi0 + alpha0);   % Added because phi0 is -ve.
    D0y = By + U * sin(phi0 + alpha0);
    
    % Ground level calculation
    W0y = (C0y + D0y)/2;       
    CP0y = W0y - tire_H/2;
    
    % Kingpin axis intersection with ground (relative to Ax=0)
    KP_gx_rel = C0x_rel + (CP0y - C0y) * (D0x_rel - C0x_rel) / (D0y - C0y);
    
    % Apply Track Width and Scrub Radius to find required global Chassis Offset (Ax_R)
    CP0x_target = Track_Width / 2;
    KP_gx_target = CP0x_target - Scrub_Radius; % Inner side: kingpin hits ground inside CP
    
    Ax_R = KP_gx_target - KP_gx_rel;
    
    % --- 2. STATIC CALCULATIONS ---
    Bx_R = Ax_R;             % Point B: Upper Chassis Pivot X
    Sx_R = Ax_R + Sx_offset; % Point S: Steering Rack Right X
    
    % Left Side Chassis Mounts (Symmetric)
    Ax_L = -Ax_R;
    Bx_L = -Bx_R;
    Sx_L = -Sx_R;
    
    % Locate Static Lower Ball Joint (C) - Right Side
    C0x = Ax_R + L * cosd(theta_L0);
    C0y = Ay + L * sind(theta_L0);
    
    % Solve for Static Upper Ball Joint (D) - Right Side
    dist_BC0 = sqrt((C0x-Bx_R)^2 + (C0y-By)^2); 
    phi0 = atan2(C0y-By, C0x-Bx_R);             
    alpha0 = acos((U^2 + dist_BC0^2 - K^2) / (2 * U * dist_BC0)); 
    D0x = Bx_R + U * cos(phi0 + alpha0);
    D0y = By + U * sin(phi0 + alpha0);
    
    % Calculate Static Camber
    camber_static =  atan2d(D0x - C0x, D0y - C0y);
    
    % Point P: The tie-rod pickup point on the knuckle
    P0x = C0x + (d/K)*(D0x-C0x);
    P0y = C0y + (d/K)*(D0y-C0y);
    
    % R_rod: The fixed physical length of the tie-rod
    R_rod = sqrt((P0x - Sx_R)^2 + (P0y - Sy)^2);
    
    % --- TIRE STATIC GEOMETRY ---
    W0x = CP0x_target; % Lock tire explicitly to the target track width
    W0y = (C0y + D0y)/2;       % Vertically centered on the knuckle
    
    % 5 points to close the rectangle (Initially vertical)
    TC_static_x = [W0x - tire_W/2, W0x + tire_W/2, W0x + tire_W/2, W0x - tire_W/2, W0x - tire_W/2];
    TC_static_y = [W0y + tire_H/2, W0y + tire_H/2, W0y - tire_H/2, W0y - tire_H/2, W0y + tire_H/2];
    
    % Vector from C0 to each tire corner to maintain rigid connection
    V_tire_x = TC_static_x - C0x;
    V_tire_y = TC_static_y - C0y;
    
    % Contact Patch (Bottom Center of Tire)
    CP0x = W0x;
    CP0y = W0y - tire_H/2;
    V_CP_x = CP0x - C0x;
    V_CP_y = CP0y - C0y;
    % Wheel Center (WC)
    V_WC_x = W0x - C0x;
    V_WC_y = W0y - C0y;
    
    % Tire Centerline (Rotating with Tire)
    CL0_top_x = W0x; CL0_top_y = W0y + tire_H/2 + 50;
    CL0_bot_x = W0x; CL0_bot_y = W0y - tire_H/2 - 50;
    V_CL_top_x = CL0_top_x - C0x; V_CL_top_y = CL0_top_y - C0y;
    V_CL_bot_x = CL0_bot_x - C0x; V_CL_bot_y = CL0_bot_y - C0y;
    
    % --- 3. DYNAMIC SWEEP (Generating Kinematic Library) ---
    Px_knuckle = []; Py_knuckle = []; 
    Px_arc = [];     bump_steer = [];
    Cx_all = []; Cy_all = []; Dx_all = []; Dy_all = [];
    ICx_all = []; ICy_all = []; RC_y_all = [];
    camber_change = []; 
    Tire_X_all = []; Tire_Y_all = []; 
    CP_X_all = []; CP_Y_all = [];     
    WC_X_all = []; WC_Y_all = [];     
    CL_top_X_all = []; CL_top_Y_all = []; 
    CL_bot_X_all = []; CL_bot_Y_all = []; 
    
    for h = h_array
        Cy = C0y + h;
        Cx = Ax_R + sqrt(L^2 - (Cy - Ay)^2);
        
        d_BC = sqrt((Cx-Bx_R)^2 + (Cy-By)^2);
        if d_BC > (U + K) || d_BC < abs(U - K), continue; end
        phi_n = atan2(Cy-By, Cx-Bx_R);
        alpha_n = acos((U^2 + d_BC^2 - K^2) / (2 * U * d_BC));
        Dx = Bx_R + U * cos(phi_n + alpha_n);
        Dy = By + U * sin(phi_n + alpha_n); 
        
        current_camber = atan2d(Dx - Cx, Dy - Cy);
        camber_change(end+1) = current_camber - camber_static; 
        
        m1 = (Cy - Ay) / (Cx - Ax_R);
        m2 = (Dy - By) / (Dx - Bx_R);
        ICx = (m1*Ax_R - Ay - m2*Bx_R + By) / (m1 - m2);
        ICy = m1 * (ICx - Ax_R) + Ay;
        
        P_kx = Cx + (d/K)*(Dx-Cx);
        P_ky = Cy + (d/K)*(Dy-Cy);
        
        dy_rod = P_ky - Sy;
        if abs(dy_rod) < R_rod
            P_ax = Sx_R + sqrt(R_rod^2 - dy_rod^2);
        else
            continue; 
        end
        
        delta_X = P_kx - P_ax; 
        bump_steer_deg = steer_config * asind(delta_X / L_steer);
        bump_steer(end+1) = bump_steer_deg;
        
        theta_K_static = atan2(D0y - C0y, D0x - C0x); 
        theta_K_curr   = atan2(Dy - Cy, Dx - Cx);     
        delta_theta    = theta_K_curr - theta_K_static;
        
        R_mat = [cos(delta_theta), -sin(delta_theta);
                 sin(delta_theta),  cos(delta_theta)];
                 
        TC_curr_x = zeros(1, 5);
        TC_curr_y = zeros(1, 5);
        for pt = 1:5
            vec = [V_tire_x(pt); V_tire_y(pt)];
            rot_vec = R_mat * vec; 
            TC_curr_x(pt) = Cx + rot_vec(1);
            TC_curr_y(pt) = Cy + rot_vec(2);
        end
        Tire_X_all = [Tire_X_all; TC_curr_x];
        Tire_Y_all = [Tire_Y_all; TC_curr_y];
        
        rot_cp = R_mat * [V_CP_x; V_CP_y];
        CP_X_curr = Cx + rot_cp(1);
        CP_Y_curr = Cy + rot_cp(2);
        CP_X_all(end+1) = CP_X_curr;
        CP_Y_all(end+1) = CP_Y_curr;
        
        rot_wc = R_mat * [V_WC_x; V_WC_y];
        WC_X_all(end+1) = Cx + rot_wc(1);
        WC_Y_all(end+1) = Cy + rot_wc(2);
        
        rot_cl_top = R_mat * [V_CL_top_x; V_CL_top_y];
        CL_top_X_all(end+1) = Cx + rot_cl_top(1);
        CL_top_Y_all(end+1) = Cy + rot_cl_top(2);
        
        rot_cl_bot = R_mat * [V_CL_bot_x; V_CL_bot_y];
        CL_bot_X_all(end+1) = Cx + rot_cl_bot(1);
        CL_bot_Y_all(end+1) = Cy + rot_cl_bot(2);
        
        % Store symmetric RC for baseline calculation
        m_sa_R = (ICy - CP_Y_curr) / (ICx - CP_X_curr);
        RC_y = CP_Y_curr - m_sa_R * CP_X_curr; 
        RC_y_all(end+1) = RC_y;
        
        Px_knuckle(end+1) = P_kx; Py_knuckle(end+1) = P_ky;
        Px_arc(end+1) = P_ax;
        Cx_all(end+1) = Cx; Cy_all(end+1) = Cy;
        Dx_all(end+1) = Dx; Dy_all(end+1) = Dy;
        ICx_all(end+1) = ICx; ICy_all(end+1) = ICy;
    end
    
    % Derived Left Side Kinematic Arrays for plotting (Symmetry map)
    Cx_L_all = -Cx_all; Dx_L_all = -Dx_all; 
    ICx_L_all = -ICx_all; Px_arc_L = -Px_arc;
    CP_X_L_all = -CP_X_all; WC_X_L_all = -WC_X_all;
    CL_top_X_L_all = -CL_top_X_all; CL_bot_X_L_all = -CL_bot_X_all;
    Tire_X_L_all = -Tire_X_all;
    
    % Generate the final travel array (vertical displacement from static)
    travel = Py_knuckle - P0y;
    
    % Baseline dynamic zero for static reference
    [~, idx_0] = min(abs(travel));
    RC_y_static_baseline = RC_y_all(idx_0);
    
    % =========================================================================
    % --- FIGURE 1: MAIN SUSPENSION GEOMETRY & DASHBOARD ---
    % =========================================================================
    fig1 = figure('Color', [0.1 0.1 0.1], 'Name', 'Front View Suspension Geometry');
    set(fig1, 'Units', 'Normalized', 'Position', [0.02, 0.15, 0.45, 0.75]); 
    
    ax1 = axes('Position', [0.05, 0.22, 0.90, 0.68]); 
    set(ax1, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    hold on; axis equal; grid on;
    lw = 1.5; 
    
    % Trajectory Paths
    plot(Px_knuckle, Py_knuckle, 'cyan', 'LineWidth', 0.5, 'DisplayName', 'Knuckle Path');
    plot(Px_arc, Py_knuckle, 'g--', 'LineWidth', 0.5, 'DisplayName', 'Tie-Rod Arc');
    plot(-Px_knuckle, Py_knuckle, 'cyan', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    plot(Px_arc_L, Py_knuckle, 'g--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    
    % --- KINGPIN & SCRUB RADIUS PLOT (Right Side) ---
    m_kp = (Dy_all(idx_0) - Cy_all(idx_0)) / (Dx_all(idx_0) - Cx_all(idx_0));
    KP_g_x = Cx_all(idx_0) + (CP_Y_all(idx_0) - Cy_all(idx_0)) / m_kp;
    
    h_kp = plot([Dx_all(idx_0) KP_g_x], [Dy_all(idx_0) CP_Y_all(idx_0)], 'y:', 'LineWidth', 1.5, 'DisplayName', 'Kingpin Axis');
    h_scrub = plot([KP_g_x CP_X_all(idx_0)], [CP_Y_all(idx_0) CP_Y_all(idx_0)], 'm-', 'LineWidth', 3, 'DisplayName', sprintf('Scrub Radius (%.2fmm)', Scrub_Radius));
    
    % Plot IC Lines
    h_ic_line1_R = plot([Ax_R ICx_all(idx_0)], [Ay ICy_all(idx_0)], '--', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
    h_ic_line2_R = plot([Bx_R ICx_all(idx_0)], [By ICy_all(idx_0)], '--', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
    h_ic_pt_R = plot(ICx_all(idx_0), ICy_all(idx_0), 'ro', 'MarkerFaceColor', 'y', 'DisplayName', 'Instantaneous Center (IC)');
    
    h_ic_line1_L = plot([Ax_L ICx_L_all(idx_0)], [Ay ICy_all(idx_0)], '--', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
    h_ic_line2_L = plot([Bx_L ICx_L_all(idx_0)], [By ICy_all(idx_0)], '--', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
    h_ic_pt_L = plot(ICx_L_all(idx_0), ICy_all(idx_0), 'ro', 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
    
    % Swing Arms
    h_cp_ic_line_R = plot([CP_X_all(idx_0) ICx_all(idx_0)], [CP_Y_all(idx_0) ICy_all(idx_0)], '-.', 'Color', 'w', 'LineWidth', 1.5, 'DisplayName', 'Swing Arm (CP to IC)');    
    h_cp_ic_line_L = plot([CP_X_L_all(idx_0) ICx_L_all(idx_0)], [CP_Y_all(idx_0) ICy_all(idx_0)], '-.', 'Color', 'w', 'LineWidth', 1.5, 'HandleVisibility', 'off');    
    
    % Roll Center Point (Absolute coordinates for Fig 1)
    h_rc_pt = plot(0, RC_y_all(idx_0), 'mo', 'MarkerFaceColor', 'm', 'MarkerSize', 8, 'DisplayName', 'Roll Center (RC)');
    t_RC = text(15, RC_y_all(idx_0)-15, 'RC', 'Color', 'm', 'FontSize', 12, 'FontWeight', 'bold');
    
    % Contact Patches
    h_cp_pt_R = plot(CP_X_all(idx_0), CP_Y_all(idx_0), 'wo', 'MarkerFaceColor', 'y', 'MarkerSize', 5, 'DisplayName', 'Right CP');
    h_cp_pt_L = plot(CP_X_L_all(idx_0), CP_Y_all(idx_0), 'wo', 'MarkerFaceColor', 'c', 'MarkerSize', 5, 'DisplayName', 'Left CP');
    
    % Vertical References & Dynamic Centerlines
    h_wc_ref_R = plot([WC_X_all(idx_0) WC_X_all(idx_0)], [WC_Y_all(idx_0)-tire_H/2-50, WC_Y_all(idx_0)+tire_H/2+50], 'w:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    h_wc_ref_L = plot([WC_X_L_all(idx_0) WC_X_L_all(idx_0)], [WC_Y_all(idx_0)-tire_H/2-50, WC_Y_all(idx_0)+tire_H/2+50], 'w:', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    h_wc_dyn_R = plot([CL_bot_X_all(idx_0) CL_top_X_all(idx_0)], [CL_bot_Y_all(idx_0) CL_top_Y_all(idx_0)], 'c--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    h_wc_dyn_L = plot([CL_bot_X_L_all(idx_0) CL_top_X_L_all(idx_0)], [CL_bot_Y_all(idx_0) CL_top_Y_all(idx_0)], 'c--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    t_camber_R = text(WC_X_all(idx_0), WC_Y_all(idx_0) + tire_H/2 + 65, sprintf('Camber R: %+.2f°', camber_change(idx_0)), 'Color', 'y', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    t_camber_L = text(WC_X_L_all(idx_0), WC_Y_all(idx_0) + tire_H/2 + 65, sprintf('Camber L: %+.2f°', camber_change(idx_0)), 'Color', 'c', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    % Tires
    h_tire_R = fill(Tire_X_all(idx_0, :), Tire_Y_all(idx_0, :), [0.5 0.5 0.5], 'FaceAlpha', 0.3, 'EdgeColor', 'y', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    h_tire_L = fill(Tire_X_L_all(idx_0, :), Tire_Y_all(idx_0, :), [0.5 0.5 0.5], 'FaceAlpha', 0.3, 'EdgeColor', 'c', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % Plot Arms
    h_lower_R = plot([Ax_R Cx_all(idx_0)], [Ay Cy_all(idx_0)], 'Color', [0.2 0.6 1], 'LineWidth', lw, 'HandleVisibility', 'off');
    h_upper_R = plot([Bx_R Dx_all(idx_0)], [By Dy_all(idx_0)], 'Color', [1 0.6 0 ], 'LineWidth', lw, 'HandleVisibility', 'off');
    h_knuck_R = plot([Cx_all(idx_0) Dx_all(idx_0)], [Cy_all(idx_0) Dy_all(idx_0)], 'Color', [1 0.2 0.2], 'LineWidth', 2, 'HandleVisibility', 'off');
    h_trod_R  = plot([Sx_R Px_arc(idx_0)], [Sy Py_knuckle(idx_0)], 'Color', [0.2 1 0.2], 'LineWidth', lw, 'HandleVisibility', 'off');
    
    h_lower_L = plot([Ax_L Cx_L_all(idx_0)], [Ay Cy_all(idx_0)], 'Color', [0.2 0.6 1], 'LineWidth', lw, 'HandleVisibility', 'off');
    h_upper_L = plot([Bx_L Dx_L_all(idx_0)], [By Dy_all(idx_0)], 'Color', [1 0.6 0 ], 'LineWidth', lw, 'HandleVisibility', 'off');
    h_knuck_L = plot([Cx_L_all(idx_0) Dx_L_all(idx_0)], [Cy_all(idx_0) Dy_all(idx_0)], 'Color', [1 0.2 0.2], 'LineWidth', 2, 'HandleVisibility', 'off');
    h_trod_L  = plot([Sx_L Px_arc_L(idx_0)], [Sy Py_knuckle(idx_0)], 'Color', [0.2 1 0.2], 'LineWidth', lw, 'HandleVisibility', 'off');
    
    h_chassis = plot([Ax_L Ax_R Bx_R Bx_L Ax_L], [Ay Ay By By Ay], 'w', 'LineWidth', 3, 'DisplayName', 'Chassis Box');
    
    % Annotations
    text(Ax_R-15, Ay-15, 'A_R', 'Color', 'w', 'FontSize', 11, 'FontWeight', 'bold');
    text(Bx_R-15, By+15, 'B_R', 'Color', 'w', 'FontSize', 11, 'FontWeight', 'bold');
    text(Ax_L+5, Ay-15, 'A_L', 'Color', 'w', 'FontSize', 11, 'FontWeight', 'bold');
    text(Bx_L+5, By+15, 'B_L', 'Color', 'w', 'FontSize', 11, 'FontWeight', 'bold');
    
    % Figure 1 formatting
    x_data = [Ax_L, Ax_R, Bx_L, Bx_R, Cx_L_all, Cx_all, ICx_L_all(idx_0), ICx_all(idx_0), Tire_X_all(idx_0, :), Tire_X_L_all(idx_0, :)];
    y_data = [Ay, By, Sy, Cy_all, Dy_all, ICy_all(idx_0), RC_y_all(idx_0), Tire_Y_all(idx_0, :)];
    xlim([min(x_data)-150, max(x_data)+150]);
    ylim([min(y_data)-200, max(y_data)+400]); 
    title(sprintf('Front View Suspension Geometry | Track Width: %.0f mm', Track_Width), 'Color', 'w', 'FontSize', 14);
    legend('TextColor', 'w', 'Location', 'northeast', 'FontSize', 10);
    
    % =========================================================================
    % --- UI DASHBOARD (PHYSICAL DISTANCE MAPPING) ---
    % =========================================================================
    uipanel('Parent', fig1, 'BackgroundColor', [0.15 0.15 0.15], 'Position', [0.02 0.01 0.96 0.18], 'BorderType', 'none');
    
    % THE FIX: Force Slider bounds to map exactly to the maximum physical travel
    % By using -max_t to +max_t, the mathematical center of the slider track is strictly 0.
    max_t = ceil(max(abs(travel))); 
    s_min = -max_t;
    s_max = max_t;
    
    % Define step size (minor step: ~0.1mm, major step: 10% of track)
    slider_step = [0.1 / (s_max - s_min), 0.1]; 
    
    % Left Wheel Controls
    txt_travel_L = uicontrol('Style', 'text', 'Units', 'Normalized', ...
        'Position', [0.05, 0.12, 0.35, 0.04], 'String', sprintf('Left Wheel Travel: %+.1f mm', travel(idx_0)), ...
        'BackgroundColor', [0.15 0.15 0.15], 'ForegroundColor', 'c', 'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    h_slider_L = uicontrol('Style', 'slider', 'Units', 'Normalized', ...
        'Position', [0.05, 0.05, 0.35, 0.05], 'Min', s_min, 'Max', s_max, 'Value', 0, ...
        'SliderStep', slider_step, 'BackgroundColor', [0.3 0.3 0.3]);
        
    % --- RESET BUTTON (Middle) ---
    btn_reset = uicontrol('Style', 'pushbutton', 'Units', 'Normalized', ...
        'Position', [0.45, 0.05, 0.10, 0.08], 'String', 'RESET', ...
        'BackgroundColor', [0.6 0.2 0.2], 'ForegroundColor', 'w', ...
        'FontSize', 12, 'FontWeight', 'bold', 'Callback', @resetSliders);

    % Right Wheel Controls
    txt_travel_R = uicontrol('Style', 'text', 'Units', 'Normalized', ...
        'Position', [0.60, 0.12, 0.35, 0.04], 'String', sprintf('Right Wheel Travel: %+.1f mm', travel(idx_0)), ...
        'BackgroundColor', [0.15 0.15 0.15], 'ForegroundColor', 'y', 'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    h_slider_R = uicontrol('Style', 'slider', 'Units', 'Normalized', ...
        'Position', [0.60, 0.05, 0.35, 0.05], 'Min', s_min, 'Max', s_max, 'Value', 0, ...
        'SliderStep', slider_step, 'BackgroundColor', [0.3 0.3 0.3]);

    % =========================================================================
    % --- FIGURE 2 & 3 CODE (Unchanged) ---
    % =========================================================================
    fig2 = figure('Color', [0.1 0.1 0.1], 'Name', 'Kinematics: Camber & Bump Steer');
    set(fig2, 'Units', 'Normalized', 'Position', [0.48, 0.15, 0.30, 0.75]);
    
    ax_bs_L = subplot(2, 2, 1);
    set(ax_bs_L, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    plot(travel, bump_steer, 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
    hold on; grid on; plot([min(travel) max(travel)], [0 0], 'w--');
    h_bump_pt_L = plot(travel(idx_0), bump_steer(idx_0), 'co', 'MarkerFaceColor', 'c', 'MarkerSize', 6);
    title('Left Bump Steer', 'Color', 'c'); ylabel('Toe (deg)');
    
    ax_bs_R = subplot(2, 2, 2);
    set(ax_bs_R, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    plot(travel, bump_steer, 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
    hold on; grid on; plot([min(travel) max(travel)], [0 0], 'w--');
    h_bump_pt_R = plot(travel(idx_0), bump_steer(idx_0), 'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 6);
    title('Right Bump Steer', 'Color', 'y');
    
    ax_cam_L = subplot(2, 2, 3);
    set(ax_cam_L, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    plot(travel, camber_change, 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
    hold on; grid on; plot([min(travel) max(travel)], [0 0], 'w--');
    h_cam_pt_L = plot(travel(idx_0), camber_change(idx_0), 'co', 'MarkerFaceColor', 'c', 'MarkerSize', 6);
    title('Left Camber', 'Color', 'c'); xlabel('Travel (mm)'); ylabel('Camber (deg)');
    
    ax_cam_R = subplot(2, 2, 4);
    set(ax_cam_R, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    plot(travel, camber_change, 'Color', [0.5 0.5 0.5], 'LineWidth', 2);
    hold on; grid on; plot([min(travel) max(travel)], [0 0], 'w--');
    h_cam_pt_R = plot(travel(idx_0), camber_change(idx_0), 'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 6);
    title('Right Camber', 'Color', 'y'); xlabel('Travel (mm)');
    
    fig3 = figure('Color', [0.1 0.1 0.1], 'Name', 'Roll Center Migration'); 
    set(fig3, 'Units', 'Normalized', 'Position', [0.79, 0.15, 0.20, 0.75]);
    
    ax_rc = axes('Parent', fig3, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    hold on; grid on;
    
    RC_trace_X = [0]; RC_trace_Y = [0];
    plot(0, 0, 'r*', 'MarkerSize', 10, 'DisplayName', 'Static RC (0,0)');
    h_rc_trace = plot(RC_trace_X, RC_trace_Y, 'm-', 'LineWidth', 1.5, 'DisplayName', 'Migration Path');
    h_rc_curr_fig3 = plot(RC_trace_X(end), RC_trace_Y(end), 'wo', 'MarkerFaceColor', 'm', 'MarkerSize', 8, 'DisplayName', 'Current RC');
    
    title('Relative RC Migration', 'Color', 'w', 'FontSize', 13);
    xlabel('Horizontal Shift X (mm)', 'Color', 'w'); ylabel('Vertical Shift Y (mm)', 'Color', 'w');
    legend('TextColor', 'w', 'Location', 'best'); xlim([-100 100]); ylim([-150 150]);
    
    % --- EVENT LISTENERS ---
    % Use both listeners for smooth dragging and explicit callbacks for robust clicks
    addlistener(h_slider_L, 'ContinuousValueChange', @updatePlots);
    addlistener(h_slider_R, 'ContinuousValueChange', @updatePlots);
    set(h_slider_L, 'Callback', @updatePlots);
    set(h_slider_R, 'Callback', @updatePlots);
    
    % =========================================================================
    % --- CALLBACK FUNCTIONS ---
    % =========================================================================
    
    function resetSliders(~, ~)
        % Force physical value to EXACT 0 (Center of track)
        set(h_slider_L, 'Value', 0);
        set(h_slider_R, 'Value', 0);
        drawnow;             
        updatePlots([], []); 
    end
    
    function updatePlots(~, ~)
        % Read target physical travel from the UI sliders
        t_L = get(h_slider_L, 'Value');
        t_R = get(h_slider_R, 'Value');
        
        % Map physical target back to closest array indices
        [~, i_L] = min(abs(travel - t_L));
        [~, i_R] = min(abs(travel - t_R));

        % Prevent out-of-bounds just in case
        i_L = max(1, min(length(travel), i_L));
        i_R = max(1, min(length(travel), i_R));
        
        % Update Left Mechanism
        set(h_lower_L, 'XData', [Ax_L Cx_L_all(i_L)], 'YData', [Ay Cy_all(i_L)]);
        set(h_upper_L, 'XData', [Bx_L Dx_L_all(i_L)], 'YData', [By Dy_all(i_L)]);
        set(h_knuck_L, 'XData', [Cx_L_all(i_L) Dx_L_all(i_L)], 'YData', [Cy_all(i_L) Dy_all(i_L)]);
        set(h_trod_L,  'XData', [Sx_L Px_arc_L(i_L)], 'YData', [Sy Py_knuckle(i_L)]);
        set(h_tire_L,  'XData', Tire_X_L_all(i_L, :), 'YData', Tire_Y_all(i_L, :));
        
        % Update Right Mechanism
        set(h_lower_R, 'XData', [Ax_R Cx_all(i_R)], 'YData', [Ay Cy_all(i_R)]);
        set(h_upper_R, 'XData', [Bx_R Dx_all(i_R)], 'YData', [By Dy_all(i_R)]);
        set(h_knuck_R, 'XData', [Cx_all(i_R) Dx_all(i_R)], 'YData', [Cy_all(i_R) Dy_all(i_R)]);
        set(h_trod_R,  'XData', [Sx_R Px_arc(i_R)], 'YData', [Sy Py_knuckle(i_R)]);
        set(h_tire_R,  'XData', Tire_X_all(i_R, :), 'YData', Tire_Y_all(i_R, :));
        
        % Update IC Right
        set(h_ic_line1_R, 'XData', [Ax_R ICx_all(i_R)], 'YData', [Ay ICy_all(i_R)]);
        set(h_ic_line2_R, 'XData', [Bx_R ICx_all(i_R)], 'YData', [By ICy_all(i_R)]);
        set(h_ic_pt_R, 'XData', ICx_all(i_R), 'YData', ICy_all(i_R));
        
        % Update IC Left
        set(h_ic_line1_L, 'XData', [Ax_L ICx_L_all(i_L)], 'YData', [Ay ICy_all(i_L)]);
        set(h_ic_line2_L, 'XData', [Bx_L ICx_L_all(i_L)], 'YData', [By ICy_all(i_L)]);
        set(h_ic_pt_L, 'XData', ICx_L_all(i_L), 'YData', ICy_all(i_L));
        
        % Dynamically Recalculate Roll Center
        m_R = (ICy_all(i_R) - CP_Y_all(i_R)) / (ICx_all(i_R) - CP_X_all(i_R));
        m_L = (ICy_all(i_L) - CP_Y_all(i_L)) / (ICx_L_all(i_L) - CP_X_L_all(i_L));
        
        if abs(m_R - m_L) > 1e-6
            c_R = CP_Y_all(i_R) - m_R * CP_X_all(i_R);
            c_L = CP_Y_all(i_L) - m_L * CP_X_L_all(i_L);
            RC_x = (c_L - c_R) / (m_R - m_L);
            RC_y = m_R * RC_x + c_R;
        else
            RC_x = 0; RC_y = 1e6; 
        end
        
        % Calculate Relative RC
        RC_x_rel = RC_x;
        RC_y_rel = RC_y - RC_y_static_baseline; 
        
        RC_trace_X(end+1) = RC_x_rel;
        RC_trace_Y(end+1) = RC_y_rel;
        if length(RC_trace_X) > 1000
            RC_trace_X(1) = []; RC_trace_Y(1) = [];
        end
        
        set(h_rc_pt, 'XData', RC_x, 'YData', RC_y);
        set(t_RC, 'Position', [RC_x+15, RC_y-15, 0]);
        set(h_rc_trace, 'XData', RC_trace_X, 'YData', RC_trace_Y);
        set(h_rc_curr_fig3, 'XData', RC_x_rel, 'YData', RC_y_rel);
        
        % Update Arms & Kingpin
        set(h_cp_ic_line_R, 'XData', [CP_X_all(i_R) ICx_all(i_R)], 'YData', [CP_Y_all(i_R) ICy_all(i_R)]);
        set(h_cp_pt_R, 'XData', CP_X_all(i_R), 'YData', CP_Y_all(i_R));
        set(h_cp_ic_line_L, 'XData', [CP_X_L_all(i_L) ICx_L_all(i_L)], 'YData', [CP_Y_all(i_L) ICy_all(i_L)]);
        set(h_cp_pt_L, 'XData', CP_X_L_all(i_L), 'YData', CP_Y_all(i_L));
        
        m_kp_curr = (Dy_all(i_R) - Cy_all(i_R)) / (Dx_all(i_R) - Cx_all(i_R));
        KP_g_x_curr = Cx_all(i_R) + (CP_Y_all(i_R) - Cy_all(i_R)) / m_kp_curr;
        set(h_kp, 'XData', [Dx_all(i_R) KP_g_x_curr], 'YData', [Dy_all(i_R) CP_Y_all(i_R)]);
        set(h_scrub, 'XData', [KP_g_x_curr CP_X_all(i_R)], 'YData', [CP_Y_all(i_R) CP_Y_all(i_R)]);
        
        % Update Centerlines & Camber Text
        set(h_wc_ref_R, 'XData', [WC_X_all(i_R) WC_X_all(i_R)], 'YData', [WC_Y_all(i_R)-tire_H/2-50, WC_Y_all(i_R)+tire_H/2+50]);
        set(h_wc_dyn_R, 'XData', [CL_bot_X_all(i_R) CL_top_X_all(i_R)], 'YData', [CL_bot_Y_all(i_R) CL_top_Y_all(i_R)]);
        set(t_camber_R, 'Position', [WC_X_all(i_R), WC_Y_all(i_R) + tire_H/2 + 65, 0], 'String', sprintf('Camber R: %+.2f°', camber_change(i_R)));
        
        set(h_wc_ref_L, 'XData', [WC_X_L_all(i_L) WC_X_L_all(i_L)], 'YData', [WC_Y_all(i_L)-tire_H/2-50, WC_Y_all(i_L)+tire_H/2+50]);
        set(h_wc_dyn_L, 'XData', [CL_bot_X_L_all(i_L) CL_top_X_L_all(i_L)], 'YData', [CL_bot_Y_all(i_L) CL_top_Y_all(i_L)]);
        set(t_camber_L, 'Position', [WC_X_L_all(i_L), WC_Y_all(i_L) + tire_H/2 + 65, 0], 'String', sprintf('Camber L: %+.2f°', camber_change(i_L)));
        
        % Update Tracking Dots on Graphs
        set(h_bump_pt_L, 'XData', travel(i_L), 'YData', bump_steer(i_L));
        set(h_bump_pt_R, 'XData', travel(i_R), 'YData', bump_steer(i_R));
        set(h_cam_pt_L, 'XData', travel(i_L), 'YData', camber_change(i_L));
        set(h_cam_pt_R, 'XData', travel(i_R), 'YData', camber_change(i_R));
        
        % Update Dashboard Text using actual plotted values for precision
        set(txt_travel_L, 'String', sprintf('Left Wheel Travel: %+.1f mm', travel(i_L)));
        set(txt_travel_R, 'String', sprintf('Right Wheel Travel: %+.1f mm', travel(i_R)));
    end
end