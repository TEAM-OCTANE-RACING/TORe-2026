function suspension_kinematics_kpi()
    % Front View Suspension Kinematics - Visualization with KPI & Slider
    clc; close all;
    
    % --- 1. GEOMETRY INPUTS (Static Constants) ---
    L = 369.619;       % Lower arm length (Chassis Pivot A to Ball Joint C)
    U = 348.72;       % Upper arm length (Chassis Pivot B to Ball Joint D)
    K = 150;           % Knuckle height (Vertical distance between C and D)
    H_chassis = 113.396;   % Vertical spread between chassis mounts A and B
    theta_L0 = -1.522; % Static angle of lower arm (degrees)
    
    % Steering / Tie-Rod Inputs
    d = 45.9184;       % Vertical distance from C up to Tie-rod point P
    Sx = 15;           % Steering Rack Pivot X (Chassis mount for tie-rod)
    Sy = 30;           % Steering Rack Pivot Y (Chassis mount for tie-rod)
    
    % Accurate Steering Inputs for Toe Calculation
    L_steer = 61.3;    % Steering arm length in mm 
    steer_config = 1;  % 1 for Front-steer
    
    % Travel range for simulation
    h_array = linspace(-30, 30, 100); 
    
    % --- 2. STATIC CALCULATIONS ---
    Ax = 0; Ay = 0;             
    Bx = 0; By = H_chassis;     
    
    C0x = L * cosd(theta_L0);
    C0y = L * sind(theta_L0);
    
    dist_BC0 = sqrt((C0x-Bx)^2 + (C0y-By)^2); 
    phi0 = atan2(C0y-By, C0x-Bx);             
    alpha0 = acos((U^2 + dist_BC0^2 - K^2) / (2 * U * dist_BC0)); 
    D0x = Bx + U * cos(phi0 + alpha0);
    D0y = By + U * sin(phi0 + alpha0);
    
    % Static Kingpin Inclination (Angle of line CD from vertical)
    kpi_static = atan2d(D0x - C0x, D0y - C0y);
    
    % Point P: The tie-rod pickup point on the knuckle
    P0x = C0x + (d/K)*(D0x-C0x);
    P0y = C0y + (d/K)*(D0y-C0y);
    R_rod = sqrt((P0x - Sx)^2 + (P0y - Sy)^2);
    
    % --- 3. DYNAMIC SWEEP ---
    Px_knuckle = []; Py_knuckle = []; Px_arc = []; bump_steer = [];
    Cx_all = []; Cy_all = []; Dx_all = []; Dy_all = [];
    ICx_all = []; ICy_all = []; kpi_all = [];
    
    for h = h_array
        Cy = C0y + h;
        Cx = sqrt(L^2 - Cy^2); 
        
        d_BC = sqrt((Cx-Bx)^2 + (Cy-By)^2);
        if d_BC > (U + K) || d_BC < abs(U - K), continue; end
        phi_n = atan2(Cy-By, Cx-Bx);
        alpha_n = acos((U^2 + d_BC^2 - K^2) / (2 * U * d_BC));
        Dx = Bx + U * cos(phi_n + alpha_n);
        Dy = By + U * sin(phi_n + alpha_n); 
        
        % Calculate Current Kingpin Angle
        current_kpi = atan2d(Dx - Cx, Dy - Cy);
        kpi_all(end+1) = current_kpi;
        
        m1 = (Cy - Ay) / (Cx - Ax);
        m2 = (Dy - By) / (Dx - Bx);
        ICx = (m1*Ax - Ay - m2*Bx + By) / (m1 - m2);
        ICy = m1 * (ICx - Ax) + Ay;
        
        P_kx = Cx + (d/K)*(Dx-Cx);
        P_ky = Cy + (d/K)*(Dy-Cy);
        dy_rod = P_ky - Sy;
        if abs(dy_rod) < R_rod
            P_ax = Sx + sqrt(R_rod^2 - dy_rod^2);
            delta_X = P_kx - P_ax;
            bump_steer(end+1) = steer_config * asind(delta_X / L_steer);
            
            Px_knuckle(end+1) = P_kx; Py_knuckle(end+1) = P_ky;
            Px_arc(end+1) = P_ax;
            Cx_all(end+1) = Cx; Cy_all(end+1) = Cy;
            Dx_all(end+1) = Dx; Dy_all(end+1) = Dy;
            ICx_all(end+1) = ICx; ICy_all(end+1) = ICy;
        end
    end
    
    % --- 4. VISUALIZATION ---
    fig = figure('Color', [0.1 0.1 0.1], 'Name', 'Suspension Kinematics: Slider Control');
    set(fig, 'Units', 'Normalized', 'Position', [0.05, 0.1, 0.9, 0.8]); 
    
    ax1 = axes('Position', [0.05, 0.15, 0.60, 0.75]); 
    set(ax1, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    hold on; axis equal; grid on;
    idx = round(length(Cx_all)/2); lw = 1.5; 
    
    plot(Px_knuckle, Py_knuckle, 'cyan', 'LineWidth', 0.5, 'DisplayName', 'Knuckle Path');
    plot(Px_arc, Py_knuckle, 'g--', 'LineWidth', 0.5, 'DisplayName', 'Tie-Rod Arc');
    
    % Vertical Reference for KPI
    h_vert_ref = plot([Cx_all(idx) Cx_all(idx)], [Cy_all(idx) Dy_all(idx)+20], 'w:', 'HandleVisibility', 'off');
    h_ic_line1 = plot([Ax ICx_all(idx)], [Ay ICy_all(idx)], '--', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
    h_ic_line2 = plot([Bx ICx_all(idx)], [By ICy_all(idx)], '--', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
    h_ic_pt = plot(ICx_all(idx), ICy_all(idx), 'ro', 'MarkerFaceColor', 'r', 'DisplayName', 'IC');
    
    h_lower = plot([Ax Cx_all(idx)], [Ay Cy_all(idx)], 'Color', [0.2 0.6 1], 'LineWidth', lw, 'DisplayName', 'Lower Arm');
    h_upper = plot([Bx Dx_all(idx)], [By Dy_all(idx)], 'Color', [1 0.6 0 ], 'LineWidth', lw, 'DisplayName', 'Upper Arm');
    h_knuck = plot([Cx_all(idx) Dx_all(idx)], [Cy_all(idx) Dy_all(idx)], 'Color', [1 0.2 0.2], 'LineWidth', 3, 'DisplayName', 'Kingpin Axis (Knuckle)');
    h_trod  = plot([Sx Px_arc(idx)], [Sy Py_knuckle(idx)], 'Color', [0.2 1 0.2], 'LineWidth', lw, 'DisplayName', 'Tie-Rod');
    plot([Ax Bx], [Ay By], 'w', 'LineWidth', 3, 'DisplayName', 'Chassis');
    
    % Annotations
    text(Ax-30, Ay, 'A', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
    text(Bx-30, By, 'B', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
    t_C = text(Cx_all(idx)+12, Cy_all(idx)-5, 'C', 'Color', [0.2 0.6 1], 'FontSize', 11, 'FontWeight', 'bold');
    t_D = text(Dx_all(idx)+12, Dy_all(idx)+5, 'D', 'Color', [1 0.6 0], 'FontSize', 11, 'FontWeight', 'bold');
    t_P = text(Px_arc(idx)+12, Py_knuckle(idx), 'P', 'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold');
    t_IC = text(ICx_all(idx)+15, ICy_all(idx)+15, 'IC', 'Color', 'r', 'FontSize', 11, 'FontWeight', 'bold');
    t_kpi = text(Dx_all(idx)-20, Dy_all(idx)+30, sprintf('KPI: %.2f°', kpi_all(idx)), 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
    
    % --- REFINED DYNAMIC LIMITS FOR OFFSET ---
    pad = 80; % Offset in mm
    all_x = [Ax, Bx, Sx, Cx_all, Dx_all, ICx_all];
    all_y = [Ay, By, Sy, Cy_all, Dy_all, ICy_all];
    xlim([min(all_x)-pad, max(all_x)+pad]);
    ylim([min(all_y)-pad, max(all_y)+pad]);
    
    title('Kinematic Geometry: Kingpin Axis Tracking', 'Color', 'w', 'FontSize', 15);
    legend('TextColor', 'w', 'Location', 'northwest');
    
    % Plot 2: Bump Steer
    ax2 = axes('Position', [0.72, 0.58, 0.25, 0.32]); 
    set(ax2, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    hold on; grid on;
    travel = Py_knuckle - P0y;
    plot(travel, bump_steer, 'y', 'LineWidth', 2.5);
    h_bump_pt = plot(travel(idx), bump_steer(idx), 'wo', 'MarkerFaceColor', 'y');
    title('Bump Steer (Toe)', 'Color', 'w');
    
    % Plot 3: KPI Change
    ax3 = axes('Position', [0.72, 0.15, 0.25, 0.32]); 
    set(ax3, 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
    hold on; grid on;
    plot(travel, kpi_all, 'm', 'LineWidth', 2.5);
    h_kpi_pt = plot(travel(idx), kpi_all(idx), 'wo', 'MarkerFaceColor', 'm');
    title('Kingpin Inclination Angle', 'Color', 'w');
    xlabel('Travel (mm)'); ylabel('KPI (deg)');
    
    % --- 5. INTERACTIVE DASHBOARD ---
    uipanel('Parent', fig, 'BackgroundColor', [0.15 0.15 0.15], 'Position', [0.05 0.02 0.60 0.10], 'BorderType', 'none');
    txt_travel = uicontrol('Style', 'text', 'Units', 'Normalized', ...
        'Position', [0.07, 0.07, 0.56, 0.03], 'String', sprintf('Travel: %.1f mm', travel(idx)), ...
        'BackgroundColor', [0.15 0.15 0.15], 'ForegroundColor', 'w', 'FontSize', 11, 'FontWeight', 'bold');
    h_slider = uicontrol('Style', 'slider', 'Units', 'Normalized', ...
        'Position', [0.07, 0.035, 0.56, 0.03], 'Min', 1, 'Max', length(travel), 'Value', idx, ...
        'SliderStep', [1/length(travel), 5/length(travel)], 'BackgroundColor', [0.3 0.3 0.3]);
        
    addlistener(h_slider, 'ContinuousValueChange', @updatePlots);
    
    function updatePlots(src, ~)
        i = round(src.Value);
        
        % Update Mechanism Lines
        set(h_lower, 'XData', [Ax Cx_all(i)], 'YData', [Ay Cy_all(i)]);
        set(h_upper, 'XData', [Bx Dx_all(i)], 'YData', [By Dy_all(i)]);
        set(h_knuck, 'XData', [Cx_all(i) Dx_all(i)], 'YData', [Cy_all(i) Dy_all(i)]);
        set(h_trod,  'XData', [Sx Px_arc(i)], 'YData', [Sy Py_knuckle(i)]);
        set(h_vert_ref, 'XData', [Cx_all(i) Cx_all(i)], 'YData', [Cy_all(i) Dy_all(i)+20]);
        
        % Update IC Lines & Point
        set(h_ic_line1, 'XData', [Ax ICx_all(i)], 'YData', [Ay ICy_all(i)]);
        set(h_ic_line2, 'XData', [Bx ICx_all(i)], 'YData', [By ICy_all(i)]);
        set(h_ic_pt, 'XData', ICx_all(i), 'YData', ICy_all(i));
        
        % Update Text Annotations
        set(t_C, 'Position', [Cx_all(i)+12, Cy_all(i)-5, 0]);
        set(t_D, 'Position', [Dx_all(i)+12, Dy_all(i)+5, 0]);
        set(t_P, 'Position', [Px_arc(i)+12, Py_knuckle(i), 0]);
        set(t_IC, 'Position', [ICx_all(i)+15, ICy_all(i)+15, 0]);
        set(t_kpi, 'Position', [Dx_all(i)-20, Dy_all(i)+30, 0], 'String', sprintf('KPI: %.2f°', kpi_all(i)));
        
        % Update Trackers on Plots
        set(h_bump_pt, 'XData', travel(i), 'YData', bump_steer(i));
        set(h_kpi_pt, 'XData', travel(i), 'YData', kpi_all(i));
        set(txt_travel, 'String', sprintf('Travel: %+.1f mm', travel(i)));
        
        drawnow;
    end
end