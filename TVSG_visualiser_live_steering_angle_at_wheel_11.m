function steering_spindle_visualizer_fixed()
    % --- Constants & User Geometry (Matched to CAD Sketch) ---
    d = 61.543;              % Rack offset (Y-distance from Kingpin axis)
    L = 351;                 % Tie rod length (Now strictly enforced per CAD)
    R = 61.3;                % Steering arm length (Updated from CAD)
    theta1 = 90 - 83.625;    % Steering arm neutral angle (6.375 deg from longitudinal)
    
    % Track Definitions derived from CAD
    track_width = 1235;      % True automotive track width (Wheel center to wheel center)
    spindle_length = 44.075; % Distance from kingpin to outer wheel center
    kingpin_track = track_width - 2 * spindle_length; % Calculate Kingpin track dynamically
    
    % Tire Dimensions 
    tire_diam = 457.2;       
    tire_width = 177.8;      
    % Base tire shape (centered at 0,0, aligned with vehicle Y axis)
    tire_x_base = [-tire_width/2, tire_width/2, tire_width/2, -tire_width/2];
    tire_y_base = [-tire_diam/2, -tire_diam/2, tire_diam/2, tire_diam/2];
    
    % Base Centerline shape (20% longer than the tire for good visibility)
    cl_length = tire_diam * 1.2;
    cl_x_base = [0, 0];
    cl_y_base = [-cl_length/2, cl_length/2];

    % --- Structural Reference Points ---
    % Kingpins (Fixed Pivot Points)
    lx_kp = -kingpin_track/2;
    rx_kp = kingpin_track/2;
    y_kp = d;
    y_rack = 0;
    
    % Neutral Rack Ends (Derived geometrically from Kingpins)
    % Note: CAD shows steering arms pointing OUTWARD. 
    % Left arm goes further left (-), right arm goes further right (+).
    lx_rack_neutral = lx_kp - R*sind(theta1) + sqrt(L^2 - (d - R*cosd(theta1))^2);
    rx_rack_neutral = rx_kp + R*sind(theta1) - sqrt(L^2 - (d - R*cosd(theta1))^2);
    
    % --- UI Setup & Auto-Fitting ---
    fig = uifigure('Name', 'Ackermann Kinematics Visualizer', 'Position', [100 100 1000 600], 'Color', [0.12 0.12 0.12]);
    ax = uiaxes(fig, 'Position', [50 150 900 400], 'BackgroundColor', [0.08 0.08 0.08], 'XColor', '[0.4 0.4 0.4]', 'YColor', '[0.4 0.4 0.4]');
    grid(ax, 'on');
    axis(ax, 'equal'); 
    hold(ax, 'on');
    
    % Auto-calculate limits with an offset/padding
    padding = 300; % Increased padding so tires fit in the window
    min_X = lx_kp - spindle_length - padding;
    max_X = rx_kp + spindle_length + padding;
    min_Y = y_rack - padding;
    max_Y = y_kp + padding + 50; 
    
    xlim(ax, [min_X, max_X]);
    ylim(ax, [min_Y, max_Y]);
    
    % Pre-allocate plot handles (Cleaned up visuals)
    % Drawing tires first so they sit "under" the spindles visually
    hLeftTire = patch(ax, 'XData', nan, 'YData', nan, 'FaceColor', [0.15 0.15 0.15], 'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 1.5, 'DisplayName', 'Left Tire');
    hRightTire = patch(ax, 'XData', nan, 'YData', nan, 'FaceColor', [0.15 0.15 0.15], 'EdgeColor', [0.5 0.5 0.5], 'LineWidth', 1.5, 'DisplayName', 'Right Tire');
    
    % Centerlines for the tires
    hLeftCL = plot(ax, nan, nan, 'c-.', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    hRightCL = plot(ax, nan, nan, 'm-.', 'LineWidth', 1.2, 'HandleVisibility', 'off');

    hRack = plot(ax, nan, nan, 'Color', [0.8 0.8 0.8], 'LineWidth', 6, 'DisplayName', 'Steering Rack');
    
    hLeftTie = plot(ax, nan, nan, 'Color', [0.3 0.6 1], 'LineWidth', 2.5, 'DisplayName', 'Left Tie Rod');
    hRightTie = plot(ax, nan, nan, 'Color', [1 0.4 0.4], 'LineWidth', 2.5, 'DisplayName', 'Right Tie Rod');
    
    hLKnuckle = plot(ax, nan, nan, 'Color', [0.2 0.8 0.8], 'LineWidth', 3.5, 'DisplayName', 'Left Knuckle/Spindle');
    hRKnuckle = plot(ax, nan, nan, 'Color', [0.8 0.2 0.8], 'LineWidth', 3.5, 'DisplayName', 'Right Knuckle/Spindle');
    
    % Pre-allocate handles for ALL 6 hinge points (Black circles)
    hHinges = plot(ax, nan, nan, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6, 'HandleVisibility', 'off');
    
    % Text labels placed directly on the plot
    tLeftAngle = text(ax, 0, 0, '', 'Color', 'c', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
    tRightAngle = text(ax, 0, 0, '', 'Color', 'm', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

    % --- Clean Continuous Slider ---
    % Removed ticks to make it a clean track
    sld = uislider(fig, 'Position', [250 80 500 3], 'Limits', [-23, 23], 'Value', 0);
    sld.MajorTicks = []; 
    sld.MinorTicks = [];
    
    % Live readout label directly above the slider
    rack_label = uilabel(fig, 'Position', [400 100 200 22], ...
        'Text', 'Rack Displacement: 0.00 mm', ...
        'FontColor', 'w', 'FontSize', 14, 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    
    % Title for wheel angles
    angle_title = title(ax, 'Inner Wheel: 0.00°  |  Outer Wheel: 0.00°', 'Color', 'w', 'FontSize', 14);
    
    % Reset Button
    btnReset = uibutton(fig, 'push', 'Position', [450 30 100 30], 'Text', 'Reset to 0');
    btnReset.ButtonPushedFcn = @(btn,event) resetPlot();
    % Trigger update continuously as you drag
    sld.ValueChangingFcn = @(sld, event) updatePlot(event.Value); 
    
    % Initialize Plot at 0 mm
    updatePlot(0);
    
    function resetPlot()
        sld.Value = 0; % Update slider position
        updatePlot(0); % Update plot and calculations
    end
    function updatePlot(x_input)
        % 1. Calculate strictly horizontal Rack ends
        lx_rack = lx_rack_neutral + x_input;
        rx_rack = rx_rack_neutral + x_input;
        
        % 2. Solve for Steering Arm Angles by enforcing exact Tie Rod Length (L)
        % Equation: (X_outer - X_rack)^2 + (Y_outer - Y_rack)^2 - L^2 = 0
        % Note: sign changed to match OUTWARD pointing steering arms
        fun_L = @(t) (lx_kp - R*sind(t) - lx_rack)^2 + (y_kp - R*cosd(t) - y_rack)^2 - L^2;
        fun_R = @(t) (rx_kp + R*sind(t) - rx_rack)^2 + (y_kp - R*cosd(t) - y_rack)^2 - L^2;
        
        tL = fzero(fun_L, theta1);
        tR = fzero(fun_R, theta1);
        
        % 3. Calculate absolute rotation of the knuckle assembly
        % Since arms point outward, moving the rack right steers the wheel left (CCW)
        phi_L = theta1 - tL; 
        phi_R = tR - theta1; 
        
        % 4. Calculate Tie Rod outer pivot coordinates based on the solved angles
        lx_outer = lx_kp - R*sind(tL);
        ly_outer = y_kp - R*cosd(tL);
        
        rx_outer = rx_kp + R*sind(tR);
        ry_outer = y_kp - R*cosd(tR);
        
        % 5. Calculate Spindle end positions (rotated by phi)
        lx_spindle = lx_kp - spindle_length * cosd(phi_L);
        ly_spindle = y_kp - spindle_length * sind(phi_L);
        
        rx_spindle = rx_kp + spindle_length * cosd(phi_R);
        ry_spindle = y_kp + spindle_length * sind(phi_R); 
        
        % 6. Calculate Tire Coordinates (Apply 2D Rotation Matrix to base shape)
        lx_tire = lx_spindle + tire_x_base * cosd(phi_L) - tire_y_base * sind(phi_L);
        ly_tire = ly_spindle + tire_x_base * sind(phi_L) + tire_y_base * cosd(phi_L);
        
        rx_tire = rx_spindle + tire_x_base * cosd(phi_R) - tire_y_base * sind(phi_R);
        ry_tire = ry_spindle + tire_x_base * sind(phi_R) + tire_y_base * cosd(phi_R);
        
        % 7. Calculate Centerline Coordinates
        lx_cl = lx_spindle + cl_x_base * cosd(phi_L) - cl_y_base * sind(phi_L);
        ly_cl = ly_spindle + cl_x_base * sind(phi_L) + cl_y_base * cosd(phi_L);
        
        rx_cl = rx_spindle + cl_x_base * cosd(phi_R) - cl_y_base * sind(phi_R);
        ry_cl = ry_spindle + cl_x_base * sind(phi_R) + cl_y_base * cosd(phi_R);

        % --- Update Visuals ---
        
        % Tires
        set(hLeftTire, 'XData', lx_tire, 'YData', ly_tire);
        set(hRightTire, 'XData', rx_tire, 'YData', ry_tire);
        
        % Centerlines
        set(hLeftCL, 'XData', lx_cl, 'YData', ly_cl);
        set(hRightCL, 'XData', rx_cl, 'YData', ry_cl);

        % Steering Rack Line (Thick bar)
        set(hRack, 'XData', [lx_rack, rx_rack], 'YData', [y_rack, y_rack]);
        
        % Tie Rods (Rack End -> Outer Pivot)
        % Removed the default marker from the line itself
        set(hLeftTie, 'XData', [lx_rack, lx_outer], 'YData', [y_rack, ly_outer]);
        set(hRightTie, 'XData', [rx_rack, rx_outer], 'YData', [y_rack, ry_outer]);
        
        % Knuckle Assembly (Steering Arm Pivot -> Kingpin -> Spindle End)
        % Drawn as one continuous rigid body line
        set(hLKnuckle, 'XData', [lx_outer, lx_kp, lx_spindle], 'YData', [ly_outer, y_kp, ly_spindle]);
        set(hRKnuckle, 'XData', [rx_outer, rx_kp, rx_spindle], 'YData', [ry_outer, y_kp, ry_spindle]);
        
        % Update Hinge Points (Black circles at all 6 pivot points)
        % 1&2: Rack ends, 3&4: Outer pivots, 5&6: Kingpins
        hinge_X = [lx_rack, rx_rack, lx_outer, rx_outer, lx_kp, rx_kp];
        hinge_Y = [y_rack, y_rack, ly_outer, ry_outer, y_kp, y_kp];
        set(hHinges, 'XData', hinge_X, 'YData', hinge_Y);
        
        % Update text objects floating near the tires
        % Using the 2nd coordinate of the centerline (the 'top' of the line) with a slight offset
        set(tLeftAngle, 'Position', [lx_cl(2), ly_cl(2) + 30, 0], 'String', sprintf('%+.2f°', phi_L));
        set(tRightAngle, 'Position', [rx_cl(2), ry_cl(2) + 30, 0], 'String', sprintf('%+.2f°', phi_R));

        % Update UI Text
        rack_label.Text = sprintf('Rack Displacement: %+.2f mm', x_input);
        angle_title.String = sprintf('Inner Wheel (L): %+.2f°  |  Outer Wheel (R): %+.2f°', phi_L, phi_R);
    end
end