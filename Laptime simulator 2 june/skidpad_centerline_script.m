%% Skidpad Centerline Generator - Formula Student
% Track dimensions from rulebook:
%   Circle diameter = 15.25m → inner edge radius = 7.625m
%   Track width = 3.0m
%   Centerline radius = 7.625 + 1.5 = 9.125m
%   Circle centers at x = ±9.125m from origin (start/finish midpoint)

r_cl = 9.125;           % Centerline radius of each circle (m)
cx_L = -9.125;          % Center of LEFT circle (x)
cx_R =  9.125;          % Center of RIGHT circle (x)
cy   =  0;              % Both circle centers at y = 0

N_straight = 50;        % Points along straight
N_circle   = 360;       % Points per full lap
L_straight = 12;        % Length of entry/exit straights (m)

%% 1. Bottom straight (Entry): 12m straight up to origin (y = 0)
% Runs along x=0, from y = -12m up to y = 0m
x_s1 = zeros(1, N_straight);
y_s1 = linspace(-L_straight, 0, N_straight);

%% 2. Two laps of RIGHT circle (clockwise = negative direction)
% Right circle center: (+9.125, 0)
% Car enters at (0, 0) which is the leftmost point → theta_start = pi
% Turning right means heading clockwise: theta goes from pi → -3*pi
theta_R = linspace(pi, pi - 4*pi, 2*N_circle);
x_R = cx_R + r_cl * cos(theta_R);
y_R = cy   + r_cl * sin(theta_R);

%% 3. Two laps of LEFT circle (counterclockwise = positive direction)
% Left circle center: (-9.125, 0)
% Car crosses back to (0, 0) (rightmost point of left circle) → theta_start = 0
% Turning left means heading counter-clockwise: theta goes from 0 → 4*pi
theta_L = linspace(0, 4*pi, 2*N_circle);
x_L = cx_L + r_cl * cos(theta_L);
y_L = cy   + r_cl * sin(theta_L);

%% 4. Top straight (Exit): 12m straight from origin up to exit
% Runs along x=0, from y = 0m up to y = +12m
x_s2 = zeros(1, N_straight);
y_s2 = linspace(0, L_straight, N_straight);

%% Assemble full path in sequence
% Using (2:end) for subsequent segments avoids generating duplicate (0,0) 
% coordinates where the segments perfectly overlap.
x_all = [x_s1, x_R(2:end), x_L(2:end), x_s2(2:end)];
y_all = [y_s1, y_R(2:end), y_L(2:end), y_s2(2:end)];

%% Write to CSV
skidpad_data = table(x_all', y_all', 'VariableNames', {'x', 'y'});
writetable(skidpad_data, 'skidpad_centerline.csv');
disp('CSV written: skidpad_centerline.csv');
disp(['Total points: ', num2str(length(x_all))]);

%% Plot to verify
figure;
plot(x_all, y_all, 'b-', 'LineWidth', 1.5); hold on;
plot(x_all(1), y_all(1), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
plot(x_all(end), y_all(end), 'rs', 'MarkerSize', 10, 'DisplayName', 'End');

% Mark circle centers
plot(cx_L, cy, 'k+', 'MarkerSize', 12, 'HandleVisibility', 'off');
plot(cx_R, cy, 'k+', 'MarkerSize', 12, 'HandleVisibility', 'off');

% Draw reference circles (inner edge, centerline, outer edge)
theta_ref = linspace(0, 2*pi, 360);
for cx = [cx_L, cx_R]
    plot(cx + 7.625*cos(theta_ref), 7.625*sin(theta_ref), 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    plot(cx + 9.125*cos(theta_ref), 9.125*sin(theta_ref), 'r--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
    plot(cx + 10.625*cos(theta_ref), 10.625*sin(theta_ref), 'k--', 'LineWidth', 0.5, 'HandleVisibility', 'off');
end
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');
title('Skidpad Centerline - Formula Student');
legend('Centerline path', 'Entry', 'Exit', 'Location', 'best');