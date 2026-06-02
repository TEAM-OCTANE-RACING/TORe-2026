% =========================================================================
% RACING LINE RELAXATION VISUALIZER
% =========================================================================
clear; clc; close all;

% 1. GENERATE DUMMY TRACK DATA (Peanut Shape)
track_file = "C:\Users\vishal bhujbal\OneDrive\Desktop\Modular_vehicle_model\Tracks\Hockenheim.csv";
data = readmatrix(track_file);
xc = data(:,1); yc = data(:,2);
N = length(xc);

% Set track boundaries: 4 meters to the left, 4 meters to the right
wr = data(:,3); 
wl = data(:,4);
Safety_Margin = 0.5;

% Calculate inner and outer walls for plotting
dx = gradient(xc); dy = gradient(yc);
mag = sqrt(dx.^2 + dy.^2);
nx = -dy ./ mag; ny = dx ./ mag; % Normal vectors
x_in = xc + nx .* wl; y_in = yc + ny .* wl;
x_out = xc - nx .* wr; y_out = yc - ny .* wr;

% 2. SET UP THE PLOT
figure('Name', 'Racing Line Relaxation', 'Color', 'w');
hold on; axis equal; grid on;
plot(x_in, y_in, 'k-', 'LineWidth', 2);   % Draw Inside Wall
plot(x_out, y_out, 'k-', 'LineWidth', 2); % Draw Outside Wall
plot(xc, yc, 'k--', 'LineWidth', 1);      % Draw Centerline

% Initialize the racing line as the centerline
x = xc; y = yc;
% Create the red line object that we will animate
racing_line_plot = plot(x, y, 'r-', 'LineWidth', 2.5);
title('Iteration: 0');

% 3. THE RELAXATION LOOP (The "Rubber Band")
Iterations = 100; % Reduced slightly for a clean animation

for iter = 1:Iterations
    % Step A: The Pull - Find the halfway point between neighbors
    xt = 0.5 * ([x(end-1); x(1:end-1)] + [x(2:end); x(2)]);
    yt = 0.5 * ([y(end-1); y(1:end-1)] + [y(2:end); y(2)]);
    
    % Step B: The Wall Check & Push-Back
    for i = 1:N
        % How far did we pull this point away from the center?
        dx_val = xt(i) - xc(i); 
        dy_val = yt(i) - yc(i); 
        dist = sqrt(dx_val^2 + dy_val^2);
        
        % Direction vector of the track at this point
        if i < N
            tx = xc(i+1) - xc(i); ty = yc(i+1) - yc(i); 
        else
            tx = xc(2) - xc(1); ty = yc(2) - yc(1); 
        end
        
        % Are we pulling towards the left wall or right wall?
        if (tx*dy_val - ty*dx_val) > 0
            max_w = wl(i) - Safety_Margin; 
        else
            max_w = wr(i) - Safety_Margin; 
        end
        
        if max_w < 0.1; max_w = 0.1; end % Hard limit to prevent crashing
        
        % If the pulled point hit the wall, push it back exactly to the edge
        if dist > max_w
            scale = max_w / dist; 
            x(i) = xc(i) + dx_val * scale; 
            y(i) = yc(i) + dy_val * scale;
        else
            % If it's safe, keep the pulled point
            x(i) = xt(i); 
            y(i) = yt(i); 
        end
    end
    
    % Step C: Animate!
    % Update the red line's coordinates on the screen
    set(racing_line_plot, 'XData', x, 'YData', y);
    title(sprintf('Relaxing the Line... Iteration: %d', iter));
    drawnow;      % Force MATLAB to draw the frame
    pause(0.05);  % Pause for 50 milliseconds to make it look like a video
end

title('Final Optimized Racing Line');