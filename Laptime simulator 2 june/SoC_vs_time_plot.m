% Create a new figure window and set the background to dark gray/black
fig = figure;
set(fig, 'Color', [0.05 0.05 0.05]); 

% Handle Simulink data extraction safely (works for arrays or timeseries)
time = out.tout;
if isa(out.SoC, 'timeseries')
    soc = out.SoC.Data*100;
else
    soc = out.SoC*100;
end

% Plot with a bright Neon Cyan color (#00FFFF) and a thicker line
plot(time, soc, 'LineWidth', 2.5, 'Color', '#00FFFF'); 

% Customize the axes for the dark theme
ax = gca;
set(ax, 'Color', [0.05 0.05 0.05]);        % Dark axes background
set(ax, 'XColor', 'w', 'YColor', 'w');     % White axes lines and tick marks
set(ax, 'GridColor', 'w', 'GridAlpha', 0.2); % Faint white grid lines

% Add labels and formatting in white
title('State of Charge (SoC) Over Time', 'Color', 'w', 'FontSize', 14);
xlabel('Time (seconds)', 'Color', 'w', 'FontSize', 12);
ylabel('State of Charge (%)', 'Color', 'w', 'FontSize', 12);

% Turn on the grid
grid on;