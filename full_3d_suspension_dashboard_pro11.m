function full_3d_suspension_dashboard_pro11()
    % =========================================================================
    % --- 1. GEOMETRY INPUTS (From Front View & Top View Codes) ---
    % Coordinate System: 
    % X = Forward (Longitudinal)
    % Y = Lateral (Positive is LEFT side of the car)
    % Z = Vertical (Upward relative to the ground)
    % Origin (0,0,0) is located at the ground, dead center of the front axle.
    % =========================================================================
    
    % --- Suspension Arms (Front View Projections) ---
    L_fv = 330.714;          % Lower Control Arm length projected on front view
    U_fv = 289.019;         % Upper Control Arm length projected on front view
    K_knuckle = 150;        % Vertical distance between upper and lower ball joints on the upright
    H_chassis = 121.58;     % Vertical distance between chassis mounting points (Lower to Upper)
    theta_L0 = -1.149;      % Initial static angle of the lower arm (degrees)
    
    % --- Chassis Mounting Point Spreads (Top View Longitudinal) ---
    LCA_X_spread = 305;     % Lower arm chassis mounts are separated by 300mm (+/- 150mm from center)
    UCA_X_spread = 279.341;     % Upper arm chassis mounts are separated by 240mm (+/- 120mm from center)
    
    % --- Steering Geometry ---
    d_z = 40.7;             % Vertical height of the steering arm joint above the lower ball joint
    Sy_offset = 32.752;     % Vertical Z-height offset of the steering rack from chassis mounts
    R_steer = 61.3;         % Length of the steering arm (Kingpin axis to tie rod outer pivot)
    theta1 = 6.375;         % Angle of the steering arm pointing inward (Pro-Ackermann)
    rack_X = -61.543;       % The steering rack is placed 61.5mm behind the front axle (Rear steer)
    rack_length = 458.464;  % True total length of the steering rack housing
    steer_config = 1;       % Direction multiplier for steering mapping
    
    % --- Wheels & Tires ---
    Track_Width = 1235;     % Distance between the center of the left and right tire contact patches
    Scrub_Radius = 9.999;   % Distance from the kingpin ground intersection to the tire centerline
    tire_D = 457.193;       % Tire outer diameter
    tire_W = 177.8;         % Tire width
    
    % =========================================================================
    % --- 2. STATIC 3D COORDINATE CONSTRUCTION ---
    % Build the un-moved, default state of the car based on the inputs.
    % =========================================================================
    
    % Define the Z-heights of the chassis mounting points
    A_z = 118;              % Lower chassis mount is 200mm off the ground
    B_z = A_z + H_chassis;  % Upper chassis mount is directly above it
    
    % -- A. Solve the Front View 2D Geometry --
    % Find where the lower ball joint (C0) sits using basic trigonometry
    C0y_rel = L_fv * cosd(theta_L0);
    C0z_rel = A_z + L_fv * sind(theta_L0);
    
    % Use the Law of Cosines to solve the upper arm angle
    dist_BC0 = sqrt(C0y_rel^2 + (C0z_rel - B_z)^2); % Distance from upper chassis mount to lower ball joint
    phi0 = atan2(C0z_rel - B_z, C0y_rel);           % Base angle of that diagonal line
    % Law of Cosines to find angle between diagonal and upper arm
    alpha0 = acos((U_fv^2 + dist_BC0^2 - K_knuckle^2) / (2 * U_fv * dist_BC0)); 
    
    % Find where the upper ball joint (D0) sits
    D0y_rel = U_fv * cos(phi0 + alpha0);   
    D0z_rel = B_z + U_fv * sin(phi0 + alpha0);
    
    % Calculate exact kingpin ground intersection to determine chassis width (A_y)
    W0z = (C0z_rel + D0z_rel)/2;       
    CP0z = W0z - tire_D/2;
    KP_gy_rel = C0y_rel + (CP0z - C0z_rel) * (D0y_rel - C0y_rel) / (D0z_rel - C0z_rel);
    
    A_y = (Track_Width / 2) - Scrub_Radius - KP_gy_rel; % Base chassis Y coordinate
    B_y = A_y; % Upper chassis mount sits perfectly above lower mount
    
    % -- B. Construct Exact Static 3D Knuckle Core (Left Side) --
    % C0 = Lower Ball Joint | D0 = Upper Ball Joint
    C0 = [0, A_y + C0y_rel, C0z_rel];
    D0 = [0, B_y + D0y_rel, D0z_rel];
    
    % Find points exactly ON the kingpin line based on ratios
    t_P = d_z / K_knuckle; % Fraction representing steering arm height
    t_W = 0.5;             % Fraction representing wheel center height
    
    P_base0 = C0 + t_P * (D0 - C0); % Base of the steering arm on the kingpin
    W_base0 = C0 + t_W * (D0 - C0); % Virtual Wheel Center on the kingpin
    
    % -- C. Define Local Spindle & Steer Arm --
    spindle_L = (Track_Width / 2) - W_base0(2);
    
    % Define the vectors pointing OUT of the kingpin base points
    vec_P_L = [-R_steer * cosd(theta1), R_steer * sind(theta1), 0]; % Left Steering Arm
    vec_W_L = [0, spindle_L, 0];                                    % Left Wheel Spindle
    
    vec_P_R = [-R_steer * cosd(theta1), -R_steer * sind(theta1), 0];% Right Steering Arm
    vec_W_R = [0, -spindle_L, 0];                                   % Right Wheel Spindle
    
    % Define inner tie rod pivots on the steering rack based strictly on rack length
    S0_L = [rack_X, rack_length / 2, A_z + Sy_offset];
    S0_R = [rack_X, -rack_length / 2, A_z + Sy_offset];
    
    % Lock in the true 3D lengths that can NEVER change during simulation (Rigid Bodies)
    LCA_3D = sqrt(L_fv^2 + (LCA_X_spread/2)^2);
    UCA_3D = sqrt(U_fv^2 + (UCA_X_spread/2)^2);
    P0_static = P_base0 + vec_P_L; % Outer tie rod pivot point in space
    TieRod_3D = norm(P0_static - S0_L); % Exact rigid tie rod length calculated from space
    camber_static = atan2d(D0(2) - C0(2), D0(3) - C0(3));
    
    % =========================================================================
    % --- 3. DASHBOARD UI SETUP ---
    % =========================================================================
    fig = uifigure('Name', 'Pro 3D Suspension Visualizer', 'Position', [50 50 1450 850], 'Color', [0.1 0.1 0.1]);
    
    ax3D = uiaxes(fig, 'Position', [10 200 1000 630]);
    hold(ax3D, 'on'); grid(ax3D, 'on'); view(ax3D, 145, 25); 
    set(ax3D, 'Color', [0.12 0.12 0.12], 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
    axis(ax3D, 'equal'); xlim(ax3D, [-600 600]); ylim(ax3D, [-1000 1000]); zlim(ax3D, [0 700]);      
    camlight(ax3D, 'headlight'); material(ax3D, 'dull');
    xlabel(ax3D, 'X (Forward)'); ylabel(ax3D, 'Y (Left)'); zlabel(ax3D, 'Z (Up)');
    
    t_steer_L = text(ax3D, nan, nan, nan, '', 'Color', 'c', 'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    t_steer_R = text(ax3D, nan, nan, nan, '', 'Color', 'y', 'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    legend(ax3D, 'TextColor', 'w', 'Location', 'northeast', 'Color', 'none', 'EdgeColor', 'w', 'FontSize', 10);
    
    ax_cam = uiaxes(fig, 'Position', [1050 520 380 280], 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.4 0.4 0.4]);
    hold(ax_cam, 'on'); grid(ax_cam, 'on'); title(ax_cam, 'Relative Camber Gain vs Travel', 'Color', 'w');
    xlabel(ax_cam, 'Travel (mm)', 'Color', 'w', 'FontWeight', 'bold');
    ylabel(ax_cam, 'Relative Camber (deg)', 'Color', 'w', 'FontWeight', 'bold');
    h_cam_line = plot(ax_cam, nan, nan, 'Color', [0.4 0.8 1], 'LineWidth', 2, 'HandleVisibility', 'off');
    plot(ax_cam, [-35 35], [0 0], 'w--', 'HandleVisibility', 'off'); 
    h_cam_dot_L = plot(ax_cam, nan, nan, 'co', 'MarkerFaceColor', 'c', 'MarkerSize', 8, 'DisplayName', 'Left Wheel');
    h_cam_dot_R = plot(ax_cam, nan, nan, 'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 8, 'DisplayName', 'Right Wheel');
    xlim(ax_cam, [-35 35]); 
    legend(ax_cam, 'TextColor', 'w', 'Location', 'best');
    
    ax_bs = uiaxes(fig, 'Position', [1050 200 380 280], 'Color', [0.15 0.15 0.15], 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.4 0.4 0.4]);
    hold(ax_bs, 'on'); grid(ax_bs, 'on'); title(ax_bs, 'Bump Steer vs Travel', 'Color', 'w');
    xlabel(ax_bs, 'Travel (mm)', 'Color', 'w', 'FontWeight', 'bold');
    ylabel(ax_bs, 'Bump Steer (deg)', 'Color', 'w', 'FontWeight', 'bold');
    h_bs_line = plot(ax_bs, nan, nan, 'Color', [1 0.5 0.5], 'LineWidth', 2, 'HandleVisibility', 'off');
    plot(ax_bs, [-35 35], [0 0], 'w--', 'HandleVisibility', 'off'); 
    h_bs_dot_L = plot(ax_bs, nan, nan, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Left Wheel');
    h_bs_dot_R = plot(ax_bs, nan, nan, 'yo', 'MarkerFaceColor', 'y', 'MarkerSize', 8, 'DisplayName', 'Right Wheel');
    xlim(ax_bs, [-35 35]); 
    legend(ax_bs, 'TextColor', 'w', 'Location', 'best');
    
    % =========================================================================
    % --- 4. PRE-ALLOCATE 3D GEOMETRY MATRICES ---
    % =========================================================================
    [cyl_x, cyl_z, cyl_y] = cylinder(tire_D/2, 40);
    Tire_X_loc = cyl_x; Tire_Z_loc = cyl_z; Tire_Y_loc = (cyl_y - 0.5) * tire_W;
    Cap_X_loc = cyl_x(1,:); Cap_Z_loc = cyl_z(1,:);
    Cap1_Y_loc = ones(size(Cap_X_loc)) * (-tire_W/2); Cap2_Y_loc = ones(size(Cap_X_loc)) * (tire_W/2);
    
    A_f_l = [ LCA_X_spread/2,  A_y, A_z]; A_r_l = [-LCA_X_spread/2,  A_y, A_z];
    B_f_l = [ UCA_X_spread/2,  B_y, B_z]; B_r_l = [-UCA_X_spread/2,  B_y, B_z];
    A_f_r = [ LCA_X_spread/2, -A_y, A_z]; A_r_r = [-LCA_X_spread/2, -A_y, A_z];
    B_f_r = [ UCA_X_spread/2, -B_y, B_z]; B_r_r = [-UCA_X_spread/2, -B_y, B_z];
    
    plot3(ax3D, [A_f_l(1), A_f_r(1), A_r_r(1), A_r_l(1), A_f_l(1)], [A_f_l(2), A_f_r(2), A_r_r(2), A_r_l(2), A_f_l(2)], [A_f_l(3), A_f_r(3), A_r_r(3), A_r_l(3), A_f_l(3)], 'Color', [0.4 0.4 0.4], 'LineWidth', 3, 'DisplayName', 'Chassis');
    plot3(ax3D, [B_f_l(1), B_f_r(1), B_r_r(1), B_r_l(1), B_f_l(1)], [B_f_l(2), B_f_r(2), B_r_r(2), B_r_l(2), B_f_l(2)], [B_f_l(3), B_f_r(3), B_r_r(3), B_r_l(3), B_f_l(3)], 'Color', [0.4 0.4 0.4], 'LineWidth', 3, 'HandleVisibility', 'off');
    
    h_UCA_l = patch(ax3D, nan(1,3), nan(1,3), nan(1,3), [0.8 0.6 0.1], 'FaceAlpha', 0.8, 'EdgeColor', 'w', 'LineWidth', 1.5, 'DisplayName', 'Upper Control Arm');
    h_LCA_l = patch(ax3D, nan(1,3), nan(1,3), nan(1,3), [0.2 0.5 0.8], 'FaceAlpha', 0.8, 'EdgeColor', 'w', 'LineWidth', 1.5, 'DisplayName', 'Lower Control Arm');
    h_UCA_r = patch(ax3D, nan(1,3), nan(1,3), nan(1,3), [0.8 0.6 0.1], 'FaceAlpha', 0.8, 'EdgeColor', 'w', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    h_LCA_r = patch(ax3D, nan(1,3), nan(1,3), nan(1,3), [0.2 0.5 0.8], 'FaceAlpha', 0.8, 'EdgeColor', 'w', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    h_UpL = plot3(ax3D, nan, nan, nan, 'c-', 'LineWidth', 5, 'DisplayName', 'Knuckle / Upright');
    h_UpR = plot3(ax3D, nan, nan, nan, 'c-', 'LineWidth', 5, 'HandleVisibility', 'off');
    h_SpL = plot3(ax3D, nan, nan, nan, 'Color', [0.9 0.4 0.1], 'LineWidth', 5, 'DisplayName', 'Spindle');
    h_SpR = plot3(ax3D, nan, nan, nan, 'Color', [0.9 0.4 0.1], 'LineWidth', 5, 'HandleVisibility', 'off');
    h_SaL = plot3(ax3D, nan, nan, nan, 'm-', 'LineWidth', 4, 'DisplayName', 'Steering Arm');
    h_SaR = plot3(ax3D, nan, nan, nan, 'm-', 'LineWidth', 4, 'HandleVisibility', 'off');
    h_TrL = plot3(ax3D, nan, nan, nan, 'g-', 'LineWidth', 3, 'DisplayName', 'Tie Rod');
    h_TrR = plot3(ax3D, nan, nan, nan, 'g-', 'LineWidth', 3, 'HandleVisibility', 'off');
    h_Rack = plot3(ax3D, nan, nan, nan, 'w-', 'LineWidth', 8, 'DisplayName', 'Steering Rack');
    
    h_TL = surface(ax3D, nan(2,41), nan(2,41), nan(2,41), 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.95, 'DisplayName', 'Tire');
    h_TL_C1 = patch(ax3D, nan, nan, nan, [0.15 0.15 0.15], 'FaceAlpha', 0.95, 'HandleVisibility', 'off'); 
    h_TL_C2 = patch(ax3D, nan, nan, nan, [0.15 0.15 0.15], 'FaceAlpha', 0.95, 'HandleVisibility', 'off');
    h_TR = surface(ax3D, nan(2,41), nan(2,41), nan(2,41), 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'none', 'FaceAlpha', 0.95, 'HandleVisibility', 'off');
    h_TR_C1 = patch(ax3D, nan, nan, nan, [0.15 0.15 0.15], 'FaceAlpha', 0.95, 'HandleVisibility', 'off'); 
    h_TR_C2 = patch(ax3D, nan, nan, nan, [0.15 0.15 0.15], 'FaceAlpha', 0.95, 'HandleVisibility', 'off');
    
    % =========================================================================
    % --- 5. UI CONTROLS ---
    % =========================================================================
    pnl = uipanel(fig, 'Position', [10 10 1430 180], 'BackgroundColor', [0.15 0.15 0.15]);
    lbl_telemetry = uilabel(fig, 'Position', [350 780 600 40], 'Text', 'STANDBY', 'FontColor', 'w', 'FontSize', 18, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    
    uilabel(pnl, 'Position', [20 140 300 22], 'Text', 'Left Travel (mm)', 'FontColor', 'c', 'FontWeight', 'bold');
    sld_L = uislider(pnl, 'Position', [20 120 400 3], 'Limits', [-30, 30], 'Value', 0);
    
    uilabel(pnl, 'Position', [480 140 300 22], 'Text', 'Right Travel (mm)', 'FontColor', 'y', 'FontWeight', 'bold');
    sld_R = uislider(pnl, 'Position', [480 120 400 3], 'Limits', [-30, 30], 'Value', 0);
    
    uilabel(pnl, 'Position', [20 60 300 22], 'Text', 'Steering Rack Displacement (mm)', 'FontColor', 'w', 'FontWeight', 'bold');
    sld_Rack = uislider(pnl, 'Position', [20 40 860 3], 'Limits', [-23, 23], 'Value', 0);
    
    btn_Reset = uibutton(pnl, 'push', 'Position', [900 120 100 35], 'Text', 'RESET', 'BackgroundColor', [0.8 0.2 0.2], 'FontColor', 'w', 'FontWeight', 'bold');
    
    uilabel(pnl, 'Position', [900 80 150 22], 'Text', 'Wheel Opacity', 'FontColor', 'w', 'FontWeight', 'bold');
    sld_Alpha = uislider(pnl, 'Position', [900 60 120 3], 'Limits', [0, 1], 'Value', 0.95);
    
    % NEW ENGINEERING CONTROLS
    uilabel(pnl, 'Position', [1050 140 300 22], 'Text', 'Bump Steer Tune: Rack Z-Height Offset (mm)', 'FontColor', 'w', 'FontWeight', 'bold');
    sld_RackZ = uislider(pnl, 'Position', [1050 120 300 3], 'Limits', [-20, 20], 'Value', 0);
    
    uilabel(pnl, 'Position', [1050 60 300 22], 'Text', 'Static Toe Tune: Tie Rod Length Offset (mm)', 'FontColor', 'w', 'FontWeight', 'bold');
    sld_TieRod = uislider(pnl, 'Position', [1050 40 300 3], 'Limits', [-10, 10], 'Value', 0);
    
    % Update callback signatures to pass the new tuning slider values
    sld_L.ValueChangingFcn = @(~, e) updateSim(e.Value, sld_R.Value, sld_Rack.Value, sld_RackZ.Value, sld_TieRod.Value);
    sld_R.ValueChangingFcn = @(~, e) updateSim(sld_L.Value, e.Value, sld_Rack.Value, sld_RackZ.Value, sld_TieRod.Value);
    sld_Rack.ValueChangingFcn = @(~, e) updateSim(sld_L.Value, sld_R.Value, e.Value, sld_RackZ.Value, sld_TieRod.Value);
    sld_RackZ.ValueChangingFcn = @(~, e) updateSim(sld_L.Value, sld_R.Value, sld_Rack.Value, e.Value, sld_TieRod.Value);
    sld_TieRod.ValueChangingFcn = @(~, e) updateSim(sld_L.Value, sld_R.Value, sld_Rack.Value, sld_RackZ.Value, e.Value);
    
    btn_Reset.ButtonPushedFcn = @(~,~) resetSim();
    sld_Alpha.ValueChangingFcn = @(~, e) set([h_TL, h_TR, h_TL_C1, h_TL_C2, h_TR_C1, h_TR_C2], 'FaceAlpha', e.Value);
    
    h_array = -30:1:30; 
    
    % =========================================================================
    % --- KINEMATIC SOLVERS & GEOMETRY MATHEMATICS ---
    % Completely Analytical - No Numerical Solvers Required
    % =========================================================================
    
    % Solves for the position of the Upright based on vertical wheel travel (h)
    function [C, D, P_base, W_base, kp_axis, d_cam, cam] = solve_upright(h)
        % Add wheel travel 'h' to the static Z height of the lower ball joint
        C_z = h + C0z_rel;
        
        % The lower arm swings in an arc. Find the new Y position using pythagorean theorem
        C_y = A_y + real(sqrt(max(0, L_fv^2 - (C_z - A_z)^2)));
        
        % Distance from the static upper chassis mount (B) to the newly moved lower ball joint (C)
        d_BC = sqrt((C_y - B_y)^2 + (C_z - B_z)^2);
        
        % Use the Law of Cosines to find the angle inside the triangle formed by UCA, Knuckle, and d_BC
        a_val = (U_fv^2 + d_BC^2 - K_knuckle^2) / (2 * U_fv * d_BC);
        alpha = acos(max(-1, min(1, a_val))); % Clamp to prevent mathematical imaginary errors
        
        % Calculate the global angle of the diagonal line (d_BC)
        phi = atan2(C_z - B_z, C_y - B_y);
        
        % The upper ball joint (D) is located at the combined angle (phi + alpha)
        D_y = B_y + U_fv * cos(phi + alpha);
        D_z = B_z + U_fv * sin(phi + alpha);
        
        % Save 3D coordinates (X is always 0 in this 2D front-view projection step)
        C = [0, C_y, C_z];
        D = [0, D_y, D_z];
        
        % Calculate the camber angle (tilt of the line connecting C and D)
        cam = atan2d(D_y - C_y, D_z - C_z);
        d_cam = cam - camber_static; % Delta camber from static
        
        % Define the 3D unit vector for the kingpin axis (points from Lower C to Upper D)
        kp_axis = (D - C) / norm(D - C);
        
        % Place the bases for the steering arm and wheel center along the kingpin line
        P_base = C + t_P * (D - C);
        W_base = C + t_W * (D - C);
    end
    % Rodrigues' Rotation Formula
    % Standard 3D vector mathematics. It rotates ANY point in 3D space around an arbitrary 3D axis by a given angle.
    function V_rot = rodrigues_array(pts, origin, axis, angle)
        v = pts - origin; % Shift point so the axis of rotation passes through (0,0,0)
        k = axis / norm(axis); % Ensure the rotation axis is a unit vector
        
        % K is the cross-product matrix of the axis vector
        K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
        
        % The standard Rodrigues formula using matrix math
        R = eye(3) + sin(angle)*K + (1-cos(angle))*(K^2);
        
        % Rotate the vector, then shift the origin back to where it belongs
        V_rot = (R * v')' + origin;
    end
    % --- NEW ANALYTICAL STEERING SOLVER ---
    % Calculates exactly how far the knuckle must rotate so the tie rod connects.
    % Solves the intersection of a 3D circle (steering arm path) and a 3D sphere (tie rod length).
    function theta_final = solve_steer_angle_analytical(P_uns, C_kp, kp_vec, S_rack, L_tie)
        % 1. Find the exact center of the steering arm's rotation circle. 
        % We project the unsteered arm (P_uns) directly onto the Kingpin Axis (kp_vec)
        v_C_to_P = P_uns - C_kp;
        dist_along_kp = dot(v_C_to_P, kp_vec);
        Center_circle = C_kp + dist_along_kp * kp_vec;
        
        % 2. Define the Local 2D flat plane of the rotation circle.
        r_vec = P_uns - Center_circle; % Vector pointing from circle center to unsteered arm
        R_circle = norm(r_vec);        % Radius of the steering arm's rotation
        u_vec = r_vec / R_circle;      % Local X-axis (points toward unsteered position)
        v_vec = cross(kp_vec, u_vec);  % Local Y-axis (tangent to the circle)
        
        % 3. Define the vector pointing from the steering rack pivot (S) to the circle center
        d_vec = Center_circle - S_rack;
        
        % 4. Set up the geometric intersection formula
        % Any point on the circle is: P(theta) = Center + R*cos(theta)*u + R*sin(theta)*v
        % We need the distance from P(theta) to S_rack to be exactly L_tie.
        % This simplifies to a standard trigonometric equation: A*cos(theta) + B*sin(theta) = -C_eq
        A = 2 * R_circle * dot(d_vec, u_vec);
        B = 2 * R_circle * dot(d_vec, v_vec);
        C_eq = norm(d_vec)^2 + R_circle^2 - L_tie^2;
        
        % 5. Solve the trigonometric equation
        D_mag = sqrt(A^2 + B^2);
        phi = atan2(B, A);
        
        val = -C_eq / D_mag;
        % Prevent mathematical errors if geometry breaks (e.g. rack pulls too far)
        if val < -1; val = -1; end 
        if val > 1; val = 1; end
        
        % There are two mathematical intersections (pushing vs pulling).
        theta1 = phi + acos(val);
        theta2 = phi - acos(val);
        
        % We always want the intersection closest to 0 degrees (the real-world assembly direction)
        if abs(theta1) < abs(theta2)
            theta_final = theta1;
        else
            theta_final = theta2;
        end
    end
    function updateSim(hL, hR, rack, rackZ_adj, tie_adj)
        
        % Apply static tuning offsets from the new UI sliders
        S_base_L = S0_L + [0, 0, rackZ_adj];
        S_base_R = S0_R + [0, 0, rackZ_adj];
        TR_len = TieRod_3D + tie_adj;
        
        % 1. DYNAMIC BACKGROUND PLOT ARRAYS (Creates the full lines on the graphs)
        cam_arr = zeros(size(h_array)); toe_arr_L = zeros(size(h_array)); 
        for i=1:length(h_array)
            % Solve upright geometry for the current bump travel
            [C_d, ~, Pb_d, Wb_d, kp_d, dcam_d, cam_d] = solve_upright(h_array(i));
            
            % Apply basic camber tilt to the unsteered coordinate vectors
            Rx_d = [1 0 0; 0 cosd(dcam_d) -sind(dcam_d); 0 sind(dcam_d) cosd(dcam_d)];
            P_uns_L_d = Pb_d + (Rx_d * vec_P_L')';
            WC_uns_L_d = Wb_d + (Rx_d * vec_W_L')'; 
            S_curr_L_d = S_base_L; % Rack is stationary for bump steer calculation
            
            % Calculate EXACT 3D rotation angle using the new analytical solver
            a_rad_L_d = solve_steer_angle_analytical(P_uns_L_d, C_d, kp_d, S_curr_L_d, TR_len);
            
            % Rotate the wheel center to find the true 2D ground projection angle
            WC_L_final_d = rodrigues_array(WC_uns_L_d, C_d, kp_d, a_rad_L_d);
            sp_vec_L_d = WC_L_final_d - Wb_d;
            
            cam_arr(i) = cam_d - camber_static; 
            toe_arr_L(i) = steer_config * (atan2d(sp_vec_L_d(2), sp_vec_L_d(1)) - 90);
        end
        
        toe_arr_R = -toe_arr_L; % Define right wheel curve
        
        % Update UI graph lines
        set(h_cam_line, 'XData', h_array, 'YData', cam_arr);
        set(h_bs_line, 'XData', h_array, 'YData', toe_arr_R);
        
        min_bs = min(toe_arr_R); max_bs = max(toe_arr_R);
        pad_min_bs = 0.1 * abs(min_bs); pad_max_bs = 0.1 * abs(max_bs);
        if pad_min_bs == 0 && pad_max_bs == 0; pad_min_bs = 0.1; pad_max_bs = 0.1; end
        ylim(ax_bs, [min_bs - pad_min_bs, max_bs + pad_max_bs]);
        
        pad_c = max(0.5, (max(cam_arr) - min(cam_arr)) * 0.2);
        ylim(ax_cam, [min(cam_arr) - pad_c, max(cam_arr) + pad_c]);
        
        % ---------------------------------------------------------------------
        % 2. SOLVE CURRENT STATE (LEFT WHEEL - Applying slider positions)
        % ---------------------------------------------------------------------
        [C_l, D_l, Pb_l, Wb_l, kp_l, d_cam_L, ~] = solve_upright(hL);
        Rx_L = [1 0 0; 0 cosd(d_cam_L) -sind(d_cam_L); 0 sind(d_cam_L) cosd(d_cam_L)];
        
        % Determine where the arm and wheel would sit if the tie rod didn't exist
        P_uns_L = Pb_l + (Rx_L * vec_P_L')';
        WC_uns_L = Wb_l + (Rx_L * vec_W_L')';
        
        % Move the steering rack based on the user slider
        S_curr_L = S_base_L + [0, rack, 0];
        
        % Use the analytical math function to snap the arm to the tie rod
        a_rad_L = solve_steer_angle_analytical(P_uns_L, C_l, kp_l, S_curr_L, TR_len);
        
        % Apply the final rotation to all components
        P_L_final = rodrigues_array(P_uns_L, C_l, kp_l, a_rad_L);
        WC_L_final = rodrigues_array(WC_uns_L, C_l, kp_l, a_rad_L);
        
        % Calculate true 2D horizontal angle from the 3D spindle vector
        sp_vec_L = WC_L_final - Wb_l;
        toe_L_deg = steer_config * (atan2d(sp_vec_L(2), sp_vec_L(1)) - 90);
        
        % ---------------------------------------------------------------------
        % 3. SOLVE CURRENT STATE (RIGHT WHEEL - Applying mirrored logic)
        % ---------------------------------------------------------------------
        [C_r_dum, D_r_dum, Pb_R_dum, Wb_R_dum, ~, d_cam_R, ~] = solve_upright(hR);
        
        % Mirror the Y-coordinates to move everything to the right side of the car
        C_r = [0, -C_r_dum(2), C_r_dum(3)];
        D_r = [0, -D_r_dum(2), D_r_dum(3)];
        Pb_R = [0, -Pb_R_dum(2), Pb_R_dum(3)];
        Wb_R = [0, -Wb_R_dum(2), Wb_R_dum(3)];
        kp_R = (D_r - C_r) / norm(D_r - C_r);
        
        % Apply right-side camber tilt
        Rx_R = [1 0 0; 0 cosd(-d_cam_R) -sind(-d_cam_R); 0 sind(-d_cam_R) cosd(-d_cam_R)];
        P_uns_R = Pb_R + (Rx_R * vec_P_R')';
        WC_uns_R = Wb_R + (Rx_R * vec_W_R')';
        
        % Move the right side of the rack
        S_curr_R = S_base_R + [0, rack, 0];
        
        % Use analytical math to solve for right wheel rotation
        a_rad_R = solve_steer_angle_analytical(P_uns_R, C_r, kp_R, S_curr_R, TR_len);
        
        P_R_final = rodrigues_array(P_uns_R, C_r, kp_R, a_rad_R);
        WC_R_final = rodrigues_array(WC_uns_R, C_r, kp_R, a_rad_R);
        
        % FIXED: Corrected the sign on the Right Wheel projection. 
        % It is correctly offset by +90 degrees and does not need to be artificially flipped.
        sp_vec_R = WC_R_final - Wb_R;
        toe_R_deg = steer_config * (atan2d(sp_vec_R(2), sp_vec_R(1)) + 90);
        
        % ---------------------------------------------------------------------
        % 4. TIRE ROTATIONS (Applying the solved geometry to visually draw the tires)
        % ---------------------------------------------------------------------
        pts_L = [Tire_X_loc(:), Tire_Y_loc(:) + spindle_L, Tire_Z_loc(:)];
        pts_R = [Tire_X_loc(:), Tire_Y_loc(:) - spindle_L, Tire_Z_loc(:)];
        
        % First apply the camber tilt to the tire meshes
        pts_uns_L = Wb_l + (Rx_L * pts_L')';
        pts_uns_R = Wb_R + (Rx_R * pts_R')';
        
        % Then steer the tire meshes by rotating them around the kingpin axes
        pts_st_L = rodrigues_array(pts_uns_L, C_l, kp_l, a_rad_L);
        pts_st_R = rodrigues_array(pts_uns_R, C_r, kp_R, a_rad_R);
        
        % Reshape the data back into visual cylinders for MATLAB to plot
        TLx = reshape(pts_st_L(:,1), size(Tire_X_loc)); TLy = reshape(pts_st_L(:,2), size(Tire_X_loc)); TLz = reshape(pts_st_L(:,3), size(Tire_X_loc));
        TRx = reshape(pts_st_R(:,1), size(Tire_X_loc)); TRy = reshape(pts_st_R(:,2), size(Tire_X_loc)); TRz = reshape(pts_st_R(:,3), size(Tire_X_loc));
        
        % Cap the ends of the tires (Sidewalls)
        pts_C1_L = Wb_l + (Rx_L * [Cap_X_loc(:), Cap1_Y_loc(:) + spindle_L, Cap_Z_loc(:)]')';
        pts_C2_L = Wb_l + (Rx_L * [Cap_X_loc(:), Cap2_Y_loc(:) + spindle_L, Cap_Z_loc(:)]')';
        C1_L_st = rodrigues_array(pts_C1_L, C_l, kp_l, a_rad_L); C2_L_st = rodrigues_array(pts_C2_L, C_l, kp_l, a_rad_L);
        
        pts_C1_R = Wb_R + (Rx_R * [Cap_X_loc(:), Cap1_Y_loc(:) - spindle_L, Cap_Z_loc(:)]')';
        pts_C2_R = Wb_R + (Rx_R * [Cap_X_loc(:), Cap2_Y_loc(:) - spindle_L, Cap_Z_loc(:)]')';
        C1_R_st = rodrigues_array(pts_C1_R, C_r, kp_R, a_rad_R); C2_R_st = rodrigues_array(pts_C2_R, C_r, kp_R, a_rad_R);
        
        % ---------------------------------------------------------------------
        % 5. RENDER UPDATES (Updating the 3D visual lines)
        % ---------------------------------------------------------------------
        set(h_UCA_l, 'XData', [B_f_l(1) D_l(1) B_r_l(1)], 'YData', [B_f_l(2) D_l(2) B_r_l(2)], 'ZData', [B_f_l(3) D_l(3) B_r_l(3)]);
        set(h_LCA_l, 'XData', [A_f_l(1) C_l(1) A_r_l(1)], 'YData', [A_f_l(2) C_l(2) A_r_l(2)], 'ZData', [A_f_l(3) C_l(3) A_r_l(3)]);
        set(h_UCA_r, 'XData', [B_f_r(1) D_r(1) B_r_r(1)], 'YData', [B_f_r(2) D_r(2) B_r_r(2)], 'ZData', [B_f_r(3) D_r(3) B_r_r(3)]);
        set(h_LCA_r, 'XData', [A_f_r(1) C_r(1) A_r_r(1)], 'YData', [A_f_r(2) C_r(2) A_r_r(2)], 'ZData', [A_f_r(3) C_r(3) A_r_r(3)]);
        
        set(h_UpL, 'XData', [D_l(1) C_l(1)], 'YData', [D_l(2) C_l(2)], 'ZData', [D_l(3) C_l(3)]);
        set(h_UpR, 'XData', [D_r(1) C_r(1)], 'YData', [D_r(2) C_r(2)], 'ZData', [D_r(3) C_r(3)]);
        set(h_SpL, 'XData', [Wb_l(1) WC_L_final(1)], 'YData', [Wb_l(2) WC_L_final(2)], 'ZData', [Wb_l(3) WC_L_final(3)]);
        set(h_SpR, 'XData', [Wb_R(1) WC_R_final(1)], 'YData', [Wb_R(2) WC_R_final(2)], 'ZData', [Wb_R(3) WC_R_final(3)]);
        set(h_SaL, 'XData', [Pb_l(1) P_L_final(1)], 'YData', [Pb_l(2) P_L_final(2)], 'ZData', [Pb_l(3) P_L_final(3)]);
        set(h_SaR, 'XData', [Pb_R(1) P_R_final(1)], 'YData', [Pb_R(2) P_R_final(2)], 'ZData', [Pb_R(3) P_R_final(3)]);
        
        set(h_TrL, 'XData', [S_curr_L(1) P_L_final(1)], 'YData', [S_curr_L(2) P_L_final(2)], 'ZData', [S_curr_L(3) P_L_final(3)]);
        set(h_TrR, 'XData', [S_curr_R(1) P_R_final(1)], 'YData', [S_curr_R(2) P_R_final(2)], 'ZData', [S_curr_R(3) P_R_final(3)]);
        set(h_Rack, 'XData', [S_curr_L(1) S_curr_R(1)], 'YData', [S_curr_L(2) S_curr_R(2)], 'ZData', [S_curr_L(3) S_curr_R(3)]);
        
        set(h_TL, 'XData', TLx, 'YData', TLy, 'ZData', TLz);
        set(h_TR, 'XData', TRx, 'YData', TRy, 'ZData', TRz);
        set(h_TL_C1, 'XData', C1_L_st(:,1), 'YData', C1_L_st(:,2), 'ZData', C1_L_st(:,3));
        set(h_TL_C2, 'XData', C2_L_st(:,1), 'YData', C2_L_st(:,2), 'ZData', C2_L_st(:,3));
        set(h_TR_C1, 'XData', C1_R_st(:,1), 'YData', C1_R_st(:,2), 'ZData', C1_R_st(:,3));
        set(h_TR_C2, 'XData', C2_R_st(:,1), 'YData', C2_R_st(:,2), 'ZData', C2_R_st(:,3));
        
        % ---------------------------------------------------------------------
        % 6. Dashboard Text & Plot Readouts
        % ---------------------------------------------------------------------
        lbl_telemetry.Text = sprintf('L Rel Camber: %+.2f°   |   Rack: %+.2f mm   |   R Rel Camber: %+.2f°', d_cam_L, rack, d_cam_R);
        
        set(t_steer_L, 'Position', [WC_L_final(1), WC_L_final(2), WC_L_final(3) + tire_D/2 + 50], 'String', sprintf('Steer: %+.2f°', toe_L_deg));
        set(t_steer_R, 'Position', [WC_R_final(1), WC_R_final(2), WC_R_final(3) + tire_D/2 + 50], 'String', sprintf('Steer: %+.2f°', toe_R_deg));
        
        % Interpolate to find where to place the dot on the graph based on user slider input
        % The left wheel now interpolates along the right wheel's path (toe_arr_R)
        bs_track_L = interp1(h_array, toe_arr_R, hL);
        
        % FIXED: The right suspension is mirrored, so an upward bump movement on the right wheel
        % causes the exact opposite physical steering rotation as the left wheel. 
        bs_track_R = interp1(h_array, toe_arr_R, hR);
        
        set(h_cam_dot_L, 'XData', hL, 'YData', d_cam_L);
        set(h_cam_dot_R, 'XData', hR, 'YData', d_cam_R);
        set(h_bs_dot_L, 'XData', hL, 'YData', bs_track_L);
        set(h_bs_dot_R, 'XData', hR, 'YData', bs_track_R); 
        
        drawnow limitrate; % Render frame
    end
    
    function resetSim()
        sld_L.Value = 0; sld_R.Value = 0; sld_Rack.Value = 0;
        sld_Alpha.Value = 0.95; 
        sld_RackZ.Value = 0; sld_TieRod.Value = 0;
        set([h_TL, h_TR, h_TL_C1, h_TL_C2, h_TR_C1, h_TR_C2], 'FaceAlpha', 0.95);
        updateSim(0, 0, 0, 0, 0);
    end
    
    % Initialize the simulation visually at zero bump and zero steer
    updateSim(0,0,0,0,0);
end