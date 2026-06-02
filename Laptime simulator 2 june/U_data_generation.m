%% Oval (Pill-Shape) Track Centerline Generator
r = 15.125 / 2; % Radius of the semi-circles
L = 50;         % Length of the straight sections
W = 1.5;        % Track width on each side (m)

N_straight = 50;
N_arc = 60;

% 1. Right Straight (Heading UP from y = 0 to y = L)
x1 = ones(1, N_straight) * r;
y1 = linspace(0, L, N_straight);

% 2. Top Semi-Circle (Counter-clockwise from 0 to pi)
theta_top = linspace(0, pi, N_arc);
x2 = r * cos(theta_top);
y2 = r * sin(theta_top) + L;

% 3. Left Straight (Heading DOWN from y = L to y = 0)
x3 = -ones(1, N_straight) * r;
y3 = linspace(L, 0, N_straight); 

% 4. Bottom Semi-Circle (Counter-clockwise from pi to 2*pi)
theta_bot = linspace(pi, 2*pi, N_arc);
x4 = r * cos(theta_bot);
y4 = r * sin(theta_bot);

% Assemble full centerline path
x = [x1, x2(2:end), x3(2:end), x4(2:end)];
y = [y1, y2(2:end), y3(2:end), y4(2:end)];

% Generate Left and Right track width arrays
% Since the track width is constant, we create arrays of 1.5 matching the length of x
L_track = ones(size(x)) * W;
R_track = ones(size(x)) * W;

% Write to 4-Column CSV
track_data = table(x', y', L_track', R_track', 'VariableNames', {'x', 'y', 'L_track', 'R_track'});
writetable(track_data, 'Oval_data_4col.csv');
disp('CSV written: Oval_data_4col.csv');
disp(['Total points: ', num2str(length(x))]);

%% Plot to verify track geometry and boundaries
% Calculate exact geometric boundaries for visual verification
x_inner = [(r-W)*ones(1,N_straight), (r-W)*cos(theta_top(2:end)), -(r-W)*ones(1,N_straight-1), (r-W)*cos(theta_bot(2:end))];
y_inner = [linspace(0,L,N_straight), (r-W)*sin(theta_top(2:end))+L, linspace(L,0,N_straight-1), (r-W)*sin(theta_bot(2:end))];

x_outer = [(r+W)*ones(1,N_straight), (r+W)*cos(theta_top(2:end)), -(r+W)*ones(1,N_straight-1), (r+W)*cos(theta_bot(2:end))];
y_outer = [linspace(0,L,N_straight), (r+W)*sin(theta_top(2:end))+L, linspace(L,0,N_straight-1), (r+W)*sin(theta_bot(2:end))];

figure('Name', 'Track Geometry', 'Color', 'w');
hold on; axis equal; grid on;

% Plot Track Limits
plot(x_inner, y_inner, 'k--', 'LineWidth', 1, 'DisplayName', 'Inner Boundary');
plot(x_outer, y_outer, 'k-', 'LineWidth', 1.5, 'DisplayName', 'Outer Boundary');

% Plot Centerline
plot(x, y, 'b-.', 'LineWidth', 1.5, 'DisplayName', 'Centerline');

% Mark Start/Finish Line
plot(x(1), y(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', 'Start/Finish');

xlabel('x (m)'); 
ylabel('y (m)');
title('Closed-Loop Oval Track (1.5m Width)');
legend('Location', 'best');