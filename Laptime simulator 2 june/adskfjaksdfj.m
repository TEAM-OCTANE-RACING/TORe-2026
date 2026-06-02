% 1. Extract the data (assuming it's still coming from your 'out' variable)
x_data = -out.x.Data;
y_data = out.y.Data;

% 2. Create a new figure to avoid overwriting old plots
figure;
hold on; % Keep the plot open to add multiple elements

% 3. Plot the main trajectory line
plot(x_data, y_data, 'b-', 'LineWidth', 2, 'DisplayName', 'Vehicle Path');

% 4. Mark the Start and End points for context
% Using a green circle for Start and a red square for End
plot(x_data(1), y_data(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(x_data(end), y_data(end), 'rs', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'End');

% 5. Format the physical space
axis equal; % CRITICAL: Ensures 1 unit in X looks exactly as long as 1 unit in Y
grid on;    % Adds a grid to help visualize distances

% 6. Add labels and metadata
title('Vehicle Spatial Trajectory');
xlabel('X Coordinate (m)'); % Change 'm' to your actual units if different
ylabel('Y Coordinate (m)');
legend('Location', 'best');

hold off;