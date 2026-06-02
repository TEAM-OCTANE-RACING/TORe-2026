function interactive_vehicle_viewer_fast

    clc;
    close all;

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

    x_data = squeeze(-out.x.Data);
    y_data = squeeze(out.y.Data);

    yaw_rate  = squeeze(out.Yaw_rate.Data);

    Fx_fl = squeeze(out.Longitudinal_force_fl.Data);
    Fy_fl = squeeze(out.Lateral_force_fl.Data);

    Fx_fr = squeeze(out.Longitudinal_force_fr.Data);
    Fy_fr = squeeze(out.Lateral_force_fr.Data);

    Fx_rl = squeeze(out.Longitudinal_force_rl.Data);
    Fy_rl = squeeze(out.Lateral_force_rl.Data);

    Fx_rr = squeeze(out.Longitudinal_force_rr.Data);
    Fy_rr = squeeze(out.Lateral_force_rr.Data);

    N = length(time);

    % =========================================================================
    % FIGURE
    % =========================================================================

    fig = figure( ...
        'Name','Ultra Fast Vehicle Viewer', ...
        'Color','w', ...
        'Renderer','opengl', ...
        'Position',[100 100 1500 750]);

    set(fig,'GraphicsSmoothing','off');

    zoom on;
    pan on;

    % =========================================================================
    % TRAJECTORY AXES
    % =========================================================================

    ax1 = subplot(1,2,1);

    hold(ax1,'on');
    grid(ax1,'on');
    axis(ax1,'equal');

    plot(ax1,...
        x_data,...
        y_data,...
        'Color',[0.7 0.7 0.7],...
        'LineWidth',1);

    marker = plot(ax1,...
        x_data(1),...
        y_data(1),...
        'bo',...
        'MarkerFaceColor','b',...
        'MarkerSize',8);

    title(ax1,'Vehicle Trajectory');

    xlabel(ax1,'X Position (m)');
    ylabel(ax1,'Y Position (m)');

    % =========================================================================
    % VEHICLE VIEW AXES
    % =========================================================================

    ax2 = subplot(1,2,2);

    hold(ax2,'on');
    grid(ax2,'on');
    axis(ax2,'equal');

    set(ax2,'XDir','reverse');

    L = 1.53;
    W = 1.20;

    rectangle( ...
        'Position',[-W/2 -L/2 W L], ...
        'EdgeColor','k', ...
        'LineWidth',2);

    wheel_x = [ W/2 -W/2  W/2 -W/2];
    wheel_y = [ L/2  L/2 -L/2 -L/2];

    plot(ax2,...
        wheel_x,...
        wheel_y,...
        'ks',...
        'MarkerFaceColor','k',...
        'MarkerSize',10);

    % =========================================================================
    % FAST FORCE LINES
    % =========================================================================

    force_scale = 0.0015;

    force_lines = gobjects(4,1);

    for i = 1:4

        force_lines(i) = line( ...
            [wheel_x(i) wheel_x(i)], ...
            [wheel_y(i) wheel_y(i)], ...
            'Color','r', ...
            'LineWidth',2);

    end

    % =========================================================================
    % YAW TEXT
    % =========================================================================

    yaw_text = text( ...
        ax2,...
        1,...
        1.7,...
        '',...
        'FontWeight','bold',...
        'FontSize',11,...
        'BackgroundColor','w');

    xlim(ax2,[-2 2]);
    ylim(ax2,[-2 2]);

    title(ax2,'Wheel Forces');

    xlabel(ax2,'Lateral Axis (m)');
    ylabel(ax2,'Longitudinal Axis (m)');

    % =========================================================================
    % TIMELINE AXES
    % =========================================================================

    timeline_ax = axes( ...
        'Position',[0.15 0.05 0.55 0.06]);

    hold(timeline_ax,'on');

    xlim([time(1) time(end)]);
    ylim([0 1]);

    timeline_ax.YTick = [];

    grid(timeline_ax,'on');

    plot(timeline_ax,...
        time,...
        0.5*ones(size(time)),...
        'Color',[0.7 0.7 0.7],...
        'LineWidth',2);

    timeline_marker = plot( ...
        timeline_ax,...
        time(1),...
        0.5,...
        'bo',...
        'MarkerFaceColor','b',...
        'MarkerSize',10);

    xlabel(timeline_ax,'Simulation Time (s)');

    % =========================================================================
    % PLAY BUTTON
    % =========================================================================

    play_button = uicontrol( ...
        'Style','togglebutton', ...
        'String','PLAY', ...
        'Units','normalized', ...
        'Position',[0.75 0.04 0.10 0.05], ...
        'FontWeight','bold', ...
        'FontSize',11, ...
        'Callback',@play_callback);

    % =========================================================================
    % SPEED MENU
    % =========================================================================

    speed_menu = uicontrol( ...
        'Style','popupmenu', ...
        'String',{'1x','2x','5x','10x','20x','50x'}, ...
        'Value',4, ...
        'Units','normalized', ...
        'Position',[0.87 0.04 0.08 0.05]);

    % =========================================================================
    % CURRENT INDEX
    % =========================================================================

    current_idx = 1;

    % =========================================================================
    % INITIAL DRAW
    % =========================================================================

    update_frame(1);

    % =========================================================================
    % KEYBOARD SUPPORT
    % =========================================================================

    set(fig,'WindowKeyPressFcn',@key_press);

    function key_press(~,event)

        switch event.Key

            case 'rightarrow'

                current_idx = min(current_idx+10,N);
                update_frame(current_idx);

            case 'leftarrow'

                current_idx = max(current_idx-10,1);
                update_frame(current_idx);

        end

    end

    % =========================================================================
    % PLAYBACK FUNCTION
    % =========================================================================

    function play_callback(src,~)

        speeds = [1 2 5 10 20 50];

        speed = speeds(get(speed_menu,'Value'));

        while get(src,'Value')

            current_idx = current_idx + speed;

            if current_idx > N

                current_idx = N;

                set(src,'Value',0);

                break;

            end

            update_frame(current_idx);

        end

    end

    % =========================================================================
    % ULTRA FAST UPDATE FUNCTION
    % =========================================================================

    function update_frame(idx)

        current_idx = idx;

        % =========================================================================
        % UPDATE TRAJECTORY MARKER
        % =========================================================================

        marker.XData = x_data(idx);
        marker.YData = y_data(idx);

        % =========================================================================
        % UPDATE TIMELINE MARKER
        % =========================================================================

        timeline_marker.XData = time(idx);

        % =========================================================================
        % FORCE UPDATE
        % =========================================================================

        Fx = [Fx_fl(idx) Fx_fr(idx) Fx_rl(idx) Fx_rr(idx)];
        Fy = [Fy_fl(idx) Fy_fr(idx) Fy_rl(idx) Fy_rr(idx)];

        for k = 1:4

            x0 = wheel_x(k);
            y0 = wheel_y(k);

            force_lines(k).XData = ...
                [x0 x0 + Fy(k)*force_scale];

            force_lines(k).YData = ...
                [y0 y0 + Fx(k)*force_scale];

        end

        % =========================================================================
        % TEXT UPDATE
        % =========================================================================

        yaw_text.String = sprintf( ...
            ['Time : %.2f s\n' ...
             'Yaw Rate : %.2f rad/s'], ...
             time(idx), ...
             yaw_rate(idx));

        % =========================================================================
        % FASTEST POSSIBLE DRAW
        % =========================================================================

        drawnow nocallbacks

    end

end