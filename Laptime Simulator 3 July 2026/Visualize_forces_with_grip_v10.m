function interactive_vehicle_viewer_pro
    % clc;
    % close all;
    % =========================================================================
    % LOAD DATA
    % =========================================================================
    out = evalin('base','out');
    % =========================================================================
    % TIME EXTRACTION
    % =========================================================================
    try
        time = squeeze(out.tout);
    catch
        try
            time = squeeze(out.time);
        catch
            time = squeeze(out.x.Time);
        end
    end
    % =========================================================================
    % DATA EXTRACTION
    % =========================================================================
    x_data = squeeze(-out.y.Data);
    y_data = squeeze(out.x.Data);
    yaw_rate  = squeeze(out.Yaw_rate.Data);
    
    % --- Extract Heading and Velocity ---
    try psi = squeeze(out.psi.Data); if length(psi) ~= length(time); psi = zeros(length(time),1); end
    catch, psi = zeros(length(time),1); end
    
    try vel = squeeze(out.Longitudinal_vel.Data); if length(vel) ~= length(time); vel = zeros(length(time),1); end
    catch, vel = zeros(length(time),1); end
    
    % --- Calculate Velocity for Radius of Curvature ---
    dt = diff(time);
    dt(dt == 0) = 1e-6; % Prevent divide-by-zero on duplicate time steps
    vx = [0; diff(x_data) ./ dt];
    vy = [0; diff(y_data) ./ dt];
    V_mag = sqrt(vx.^2 + vy.^2);
    
    % Steering Data
    N = length(time);
    try steer_L = squeeze(out.steer_L.Data); if length(steer_L) ~= N; steer_L = zeros(N,1); end
    catch, steer_L = zeros(N,1); end
    
    try steer_R = squeeze(out.steer_R.Data); if length(steer_R) ~= N; steer_R = zeros(N,1); end
    catch, steer_R = zeros(N,1); end
    
    % --- Tire Frame Forces (for grip calculation) ---
    Fx_fl = squeeze(out.Longitudinal_force_fl.Data);
    Fy_fl = -squeeze(out.Lateral_force_fl.Data);
    Fx_fr = squeeze(out.Longitudinal_force_fr.Data);
    Fy_fr = -squeeze(out.Lateral_force_fr.Data);
    Fx_rl = squeeze(out.Longitudinal_force_rl.Data);
    Fy_rl = -squeeze(out.Lateral_force_rl.Data);
    Fx_rr = squeeze(out.Longitudinal_force_rr.Data);
    Fy_rr = -squeeze(out.Lateral_force_rr.Data);
    
    % Extract Dynamic Peak Limits
    Dx_raw = squeeze(out.Dx.Data);
    Dy_raw = squeeze(out.Dy.Data);
    
    if size(Dx_raw, 1) == 4 && size(Dx_raw, 2) > 4
        Dx_raw = Dx_raw';
        Dy_raw = Dy_raw';
    elseif size(Dx_raw, 2) == 1 
        Dx_raw = repmat(Dx_raw, 1, 4); 
        Dy_raw = repmat(Dy_raw, 1, 4);
    end
    
    % =========================================================================
    % PHYSICS CONVERSIONS 
    % =========================================================================
    mass = 270; % kg 
    g = 9.81;   % m/s^2
    
    Total_Fx = Fx_fl + Fx_fr + Fx_rl + Fx_rr;
    Total_Fy = Fy_fl + Fy_fr + Fy_rl + Fy_rr;
    
    Total_Gx = -(Total_Fx) / (mass * g);
    Total_Gy = -(Total_Fy) / (mass * g);
    
    max_G = max(sqrt(Total_Gx.^2 + Total_Gy.^2));
    gg_lim = ceil(max_G); 
    if gg_lim < 2; gg_lim = 2; end 
    
    % =========================================================================
    % COLOR PALETTE & LAYOUT
    % =========================================================================
    bg_color    = [0.05, 0.05, 0.06];  
    grid_col    = [0.15, 0.15, 0.18];  
    text_col    = [0.90, 0.90, 0.90];  
    cyan_col    = [0.00, 1.00, 1.00];  
    line_sep    = [1.00, 1.00, 1.00];  
    white_col   = [1.00, 1.00, 1.00]; 
    
    body_fill   = [0.15, 0.15, 0.17];  
    aero_fill   = [0.10, 0.10, 0.12];  
    line_col    = [0.40, 0.40, 0.50];  
    
    % =========================================================================
    % MAIN FIGURE
    % =========================================================================
    fig_width = 1600;
    fig_height = 800;
    fig = figure( ...
        'Name', 'FSAE Telemetry Viewer (Pro Dashboard)', ...
        'Color', bg_color, ...
        'Renderer', 'opengl', ...
        'Position', [50 50 fig_width fig_height], ...
        'MenuBar', 'none', ...
        'ToolBar', 'none');
    set(fig, 'GraphicsSmoothing', 'on'); 
    
    % =========================================================================
    % UI SEPARATOR LINES
    % =========================================================================
    annotation(fig, 'line', [0.02 0.98], [0.12 0.12], 'Color', line_sep, 'LineWidth', 1.5);
    annotation(fig, 'line', [0.68 0.68], [0.12 0.95], 'Color', line_sep, 'LineWidth', 1.5);
    annotation(fig, 'line', [0.68 0.98], [0.55 0.55], 'Color', line_sep, 'LineWidth', 1.5);
    
    % =========================================================================
    % AXES 1: MERGED CHASE CAMERA (CAR + TRAJECTORY)
    % =========================================================================
    ax1 = axes('Position', [0.03 0.18 0.63 0.77]);
    hold(ax1, 'on'); grid(ax1, 'on'); axis(ax1, 'equal');
    
    set(ax1, 'Color', bg_color, 'XColor', text_col, 'YColor', text_col, ...
             'GridColor', grid_col, 'GridAlpha', 1, 'LineWidth', 1);
             
    % Plot the full global trajectory behind the car
    plot(ax1, x_data, y_data, '--', 'Color', line_col, 'LineWidth', 1.5);
             
    % Create a master transform to translate and rotate the entire car globally
    car_transform = hgtransform('Parent', ax1);
             
    L = 1.532; % EXACT WHEELBASE AS REQUESTED
    W = 1.20;
    
    % --- SUSPENSION A-ARMS ---
    plot(car_transform, [0.2, 0.54], [0.85, 0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(car_transform, [0.2, 0.54], [0.65, 0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(car_transform, [-0.2, -0.54], [0.85, 0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(car_transform, [-0.2, -0.54], [0.65, 0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(car_transform, [0.2, 0.54], [-0.65, -0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(car_transform, [0.2, 0.54], [-0.85, -0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(car_transform, [-0.2, -0.54], [-0.65, -0.765], 'Color', line_col, 'LineWidth', 1.5);
    plot(car_transform, [-0.2, -0.54], [-0.85, -0.765], 'Color', line_col, 'LineWidth', 1.5);
    
    % --- AERODYNAMICS ---
    fw_x = [-0.65, 0.65, 0.65, -0.65];
    fw_y = [1.15, 1.15, 1.35, 1.35];
    fill(car_transform, fw_x, fw_y, aero_fill, 'EdgeColor', cyan_col, 'LineWidth', 1);
    fill(car_transform, [0.63, 0.67, 0.67, 0.63], [1.10, 1.10, 1.40, 1.40], aero_fill, 'EdgeColor', cyan_col);
    fill(car_transform, [-0.63, -0.67, -0.67, -0.63], [1.10, 1.10, 1.40, 1.40], aero_fill, 'EdgeColor', cyan_col);
    rw_x = [-0.5, 0.5, 0.5, -0.5];
    rw_y = [-1.0, -1.0, -1.2, -1.2];
    fill(car_transform, rw_x, rw_y, aero_fill, 'EdgeColor', cyan_col, 'LineWidth', 1);
    fill(car_transform, [0.48, 0.52, 0.52, 0.48], [-0.95, -0.95, -1.25, -1.25], aero_fill, 'EdgeColor', cyan_col);
    fill(car_transform, [-0.48, -0.52, -0.52, -0.48], [-0.95, -0.95, -1.25, -1.25], aero_fill, 'EdgeColor', cyan_col);
    
    % --- CHASSIS ---
    sp_x = [0.25, 0.55, 0.55, 0.25, -0.25, -0.55, -0.55, -0.25];
    sp_y = [0.30, 0.30, -0.40, -0.70, -0.70, -0.40, 0.30, 0.30];
    fill(car_transform, sp_x, sp_y, body_fill, 'EdgeColor', line_col, 'LineWidth', 1.5);
    
    mono_x = [0, 0.12, 0.2, 0.28, 0.28, 0.2, -0.2, -0.28, -0.28, -0.2, -0.12];
    mono_y = [1.35, 1.15, 0.765, 0.3, -0.2, -0.95, -0.95, -0.2, 0.3, 0.765, 1.15];
    fill(car_transform, mono_x, mono_y, body_fill, 'EdgeColor', text_col, 'LineWidth', 1.5);
    
    % --- COCKPIT ---
    cockpit_x = [0, 0.18, 0.18, -0.18, -0.18];
    cockpit_y = [0.25, 0.1, -0.3, -0.3, 0.1];
    fill(car_transform, cockpit_x, cockpit_y, bg_color, 'EdgeColor', text_col, 'LineWidth', 1);
    plot(car_transform, [-0.1, 0.1], [0.15, 0.15], 'Color', text_col, 'LineWidth', 2);
    
    theta = linspace(0, 2*pi, 30);
    fill(car_transform, 0.08*cos(theta), 0.09*sin(theta), [0.8 0.8 0.8], 'EdgeColor', cyan_col, 'LineWidth', 1.5);
    
    % --- WHEELS & DYNAMIC STEERING TRANSFORMS ---
    % Fixed to properly align left/right to indices [FL, FR, RL, RR]
    wheel_x = [-W/2   W/2  -W/2   W/2]; 
    wheel_y = [ L/2   L/2  -L/2  -L/2];
    t_width = 0.12; t_length = 0.28;
    
    base_tx = [-t_width, -t_width, t_width, t_width];
    base_ty = [-t_length, t_length, t_length, -t_length];
    
    wheel_transforms = gobjects(4,1);
    wheel_fills = gobjects(4,1);
    wheel_texts = gobjects(4,1);
    force_lines = gobjects(4,1);
    
    for i = 1:4
        % Wheels are parented to the car_transform so they move with the chassis
        wheel_transforms(i) = hgtransform('Parent', car_transform);
        
        % The fixed physical tire outline
        plot(wheel_transforms(i), [base_tx base_tx(1)], [base_ty base_ty(1)], 'Color', text_col, 'LineWidth', 1.5);
        
        % The dynamic 2D fill representing grip utilization
        wheel_fills(i) = fill(wheel_transforms(i), [0 0], [0 0], [0 1 0], 'EdgeColor', 'none', 'FaceAlpha', 0.6);
        
        % The force vector
        force_lines(i) = line(wheel_transforms(i), [0 0], [0 0], 'Color', cyan_col, 'LineWidth', 2);
        
        wheel_transforms(i).Matrix = makehgtform('translate', [wheel_x(i), wheel_y(i), 0]);
        
        if wheel_x(i) > 0 
            txt_x = wheel_x(i) + 0.35; 
        else
            txt_x = wheel_x(i) - 0.35; 
        end
        
        % Text parented to car_transform to maintain local offsets during rotation
        wheel_texts(i) = text(txt_x, wheel_y(i), '0%', 'Parent', car_transform, ...
            'Color', [0 1 0], 'FontSize', 10, 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end
    
    title(ax1, 'CHASE CAMERA: TRAJECTORY & DYNAMIC GRIP', 'Color', text_col, 'FontSize', 11);
    xlabel(ax1, 'Global X Position (m)'); ylabel(ax1, 'Global Y Position (m)');
    
    % =========================================================================
    % STATE HUD (TOP RIGHT PANEL)
    % =========================================================================
    state_hud = annotation(fig, 'textbox', [0.70 0.58 0.26 0.35], ...
        'String', '', 'Color', white_col, 'EdgeColor', 'none', ...
        'FontSize', 11, 'FontWeight', 'bold', 'VerticalAlignment', 'top');
    
    % =========================================================================
    % AXES 2: G-G DIAGRAM (BOTTOM RIGHT PANEL)
    % =========================================================================
    ax3_pos = [0.71 0.18 0.25 0.32]; 
    ax3 = axes('Position', ax3_pos);
    hold(ax3, 'on'); grid(ax3, 'on'); 
    axis(ax3, 'equal'); 
    set(ax3, 'Color', bg_color, 'XColor', text_col, 'YColor', text_col, ...
             'GridColor', grid_col, 'GridAlpha', 1, 'XDir', 'reverse', 'LineWidth', 1);
    
    theta_circ = linspace(0, 2*pi, 100);
    for r = 0.5:0.5:gg_lim
        plot(ax3, r*cos(theta_circ), r*sin(theta_circ), '--', 'Color', [0.3 0.3 0.3]);
    end
    
    gg_trace = plot(ax3, Total_Gy, Total_Gx, '.', 'Color', [0.2 0.2 0.2], 'MarkerSize', 2);
    gg_marker = plot(ax3, Total_Gy(1), Total_Gx(1), 'o', ...
        'MarkerEdgeColor', cyan_col, 'MarkerFaceColor', cyan_col, 'MarkerSize', 8);
    
    plot(ax3, [-gg_lim gg_lim], [0 0], '-', 'Color', [0.3 0.3 0.3]);
    plot(ax3, [0 0], [-gg_lim gg_lim], '-', 'Color', [0.3 0.3 0.3]);
    
    gg_lim_padded = gg_lim * 1.15; 
    xlim(ax3, [-gg_lim_padded * 1.4, gg_lim_padded * 1.4]); 
    ylim(ax3, [-gg_lim_padded, gg_lim_padded]);
    
    title(ax3, 'G-G DIAGRAM', 'Color', text_col, 'FontSize', 11);
    xlabel(ax3, 'Lateral (g)'); ylabel(ax3, 'Longitudinal (g)');
    
    % =========================================================================
    % BOTTOM CONTROL PANEL
    % =========================================================================
    timeline_ax = axes('Position', [0.05 0.04 0.65 0.04]);
    hold(timeline_ax, 'on');
    xlim(timeline_ax, [time(1) time(end)]); ylim(timeline_ax, [0 1]);
    timeline_ax.YTick = [];
    set(timeline_ax, 'Color', bg_color, 'XColor', text_col, 'YColor', 'none', ...
        'GridColor', grid_col, 'GridAlpha', 1, 'LineWidth', 1);
    
    plot(timeline_ax, time, 0.5*ones(size(time)), 'Color', grid_col, 'LineWidth', 2);
    timeline_marker = plot(timeline_ax, time(1), 0.5, 'o', ...
        'MarkerFaceColor', cyan_col, 'MarkerEdgeColor', cyan_col, 'MarkerSize', 8);
    xlabel(timeline_ax, 'Simulation Time (s)', 'Color', text_col);
    
    play_button = uicontrol('Style', 'togglebutton', 'String', 'PLAY', ...
        'Units', 'normalized', 'Position', [0.75 0.035 0.10 0.05], ...
        'ForegroundColor', [0 0 0], 'BackgroundColor', [0.9 0.9 0.9], ...
        'FontWeight', 'bold', 'Callback', @play_callback);
        
    speed_menu = uicontrol('Style', 'popupmenu', ...
        'String', {'0.25x Speed', '0.5x Speed', '1.0x Real-Time', '2.0x Speed', '5.0x Speed', '10.0x Speed'}, ...
        'Value', 3, 'Units', 'normalized', 'Position', [0.86 0.035 0.10 0.05], ...
        'ForegroundColor', text_col, 'BackgroundColor', grid_col, ...
        'FontWeight', 'bold');
        
    current_idx = 1;
    update_frame(1);
    
    % =========================================================================
    % KEYBOARD & PLAYBACK FUNCTIONS
    % =========================================================================
    set(fig, 'WindowKeyPressFcn', @key_press);
    
    function key_press(~, event)
        if get(play_button, 'Value'); return; end
        switch event.Key
            case 'rightarrow'
                current_idx = min(current_idx+10, N);
                update_frame(current_idx);
            case 'leftarrow'
                current_idx = max(current_idx-10, 1);
                update_frame(current_idx);
        end
    end
    
    function play_callback(src, ~)
        if get(src, 'Value')
            src.String = 'PAUSE';
            speeds = [0.25, 0.5, 1.0, 2.0, 5.0, 10.0];
            
            sim_time_accumulator = time(current_idx);
            frame_timer = tic;
            
            while get(src, 'Value')
                dt_sys = toc(frame_timer);
                frame_timer = tic; 
                
                multiplier = speeds(get(speed_menu, 'Value'));
                sim_time_accumulator = sim_time_accumulator + (dt_sys * multiplier);
                
                temp_idx = current_idx;
                
                while temp_idx < N && time(temp_idx) < sim_time_accumulator
                    temp_idx = temp_idx + 1;
                end
                
                if temp_idx >= N
                    current_idx = N; update_frame(N);
                    set(src, 'Value', 0); src.String = 'REPLAY';
                    set(gg_trace, 'Color', [0.7 0.7 0.7]);
                    src.Callback = @replay_wrapper;
                    break;
                end
                
                if temp_idx > current_idx
                    update_frame(temp_idx);
                else
                    pause(0.005); 
                end
            end
        else
            src.String = 'PLAY';
        end
    end
    
    function replay_wrapper(src, event)
        set(gg_trace, 'Color', [0.2 0.2 0.2]);
        if current_idx == N
            current_idx = 1; update_frame(1);
        end
        src.Callback = @play_callback; play_callback(src, event);
    end
    
    % =========================================================================
    % RENDER FRAME FUNCTION
    % =========================================================================
    function update_frame(idx)
        current_idx = idx;
        
        timeline_marker.XData = time(idx);
        
        % =====================================================================
        % DYNAMIC CAMERA LOGIC ("Make it big")
        % Set to 3.5 meters to heavily zoom in on the 1.532m wheelbase vehicle
        % =====================================================================
        view_window = 3.5; 
        xlim(ax1, [x_data(idx) - view_window, x_data(idx) + view_window]);
        ylim(ax1, [y_data(idx) - view_window, y_data(idx) + view_window]);
        
        % Rotate AND Translate the overall vehicle along the path
        car_transform.Matrix = makehgtform('translate', [x_data(idx), y_data(idx), 0]) * makehgtform('zrotate', psi(idx));
        
        % --- Radius Calculation ---
        yr = abs(yaw_rate(idx));
        if yr > 0.02 && V_mag(idx) > 0.5 
            R = V_mag(idx) / yr;
            if R > 500
                r_str = 'Straight';
            else
                r_str = sprintf('%.2f m', R);
            end
        else
            r_str = 'Straight';
        end
        
        % Update Combined HUD
        state_hud.String = sprintf([ ...
            'VEHICLE STATE\n', ...
            '=====================\n', ...
            'Time:        %.2f s\n', ...
            'Speed:       %.2f m/s\n', ...
            'Yaw Rate:    %.4f rad/s\n\n', ...
            'GLOBAL POSITION\n', ...
            '=====================\n', ...
            'X Coord:     %.2f m\n', ...
            'Y Coord:     %.2f m\n', ...
            'Turn Radius: %s'], ...
             time(idx), vel(idx), yaw_rate(idx), x_data(idx), y_data(idx), r_str);
        
        % Extract forces in the Tire Frame
        Fx = [Fx_fl(idx) Fx_fr(idx) Fx_rl(idx) Fx_rr(idx)];
        Fy = [Fy_fl(idx) Fy_fr(idx) Fy_rl(idx) Fy_rr(idx)];
        
        Dx = [Dx_raw(idx, 1) Dx_raw(idx, 2) Dx_raw(idx, 3) Dx_raw(idx, 4)];
        Dy = [Dy_raw(idx, 1) Dy_raw(idx, 2) Dy_raw(idx, 3) Dy_raw(idx, 4)];
        
        % Local steering angles mapping standard CCW
        angles = [steer_L(idx), steer_R(idx), 0, 0];
        
        for k = 1:4
            dx_val = max(abs(Dx(k)), 1e-3);
            dy_val = max(abs(Dy(k)), 1e-3);
            
            % 1. Calculate ratios vs the dynamic limits
            ratio_x = Fx(k) / dx_val;
            ratio_y = Fy(k) / dy_val;
            
            % 2. Calculate Combined Eta (Grip Usage)
            eta = sqrt(ratio_x^2 + ratio_y^2);
            eta_clamped = max(0, min(1, eta));
            if eta_clamped < 0.5
                c = [2*eta_clamped, 1, 0];
            else
                c = [1, 2*(1-eta_clamped), 0];
            end
            
            % 3. Fill logic mapped directly to the FIXED tire dimensions
            if eta >= 1.0
                fill_w = t_width;
                fill_h = t_length;
            else
                fill_w = t_width * abs(ratio_y);
                fill_h = t_length * abs(ratio_x);
                % Maintain a small visible center patch at zero force
                fill_w = max(fill_w, 0.15 * t_width);
                fill_h = max(fill_h, 0.15 * t_length);
            end
            
            wheel_fills(k).XData = [-fill_w, -fill_w, fill_w, fill_w];
            wheel_fills(k).YData = [-fill_h, fill_h, fill_h, -fill_h];
            wheel_fills(k).FaceColor = c;
            
            % 4. Plot force vectors mapped so 100% capacity hits the tire edge
            % If Fy == Dy, the vector's X end will be exactly t_width
            force_lines(k).XData = [0, t_width * ratio_y];
            force_lines(k).YData = [0, t_length * ratio_x];
            
            % Update transforms (Force lines/fills rotate with steering, local to chassis)
            wheel_transforms(k).Matrix = makehgtform('translate', [wheel_x(k), wheel_y(k), 0]) * makehgtform('zrotate', angles(k));
            
            if eta >= 1.0
                wheel_texts(k).String = sprintf('SLIP\n%.0f%%', eta*100);
                wheel_texts(k).Color = [1 0.2 0.2]; 
            else
                wheel_texts(k).String = sprintf('%.0f%%', eta*100);
                wheel_texts(k).Color = c; 
            end
        end
        
        gg_marker.XData = Total_Gy(idx);
        gg_marker.YData = Total_Gx(idx);
         
        drawnow limitrate;
    end
end