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

% --- Entry angle where straight meets circle ---
% The straight is at x = 0 (the gap between circles)
% For the left circle centered at (-9.125, 0):
%   x = cx_L + r_cl * cos(theta) = 0  → cos(theta) = 9.125/9.125 = 1 → theta = 0
% So the straight meets the left circle at theta = 0 (rightmost point of left circle)
% And meets the right circle at theta = pi (leftmost point of right circle)

N_straight = 0;        % Points along straight
N_circle   = 360;       % Points per full lap

%% 1. Bottom straight: from entry (below) up to origin (y = 0)
%    The straight runs along x=0, from some y_entry up to y=0
%    Entry point = where car first touches left circle tangent at bottom
%    In FSG rules, car enters from bottom along the straight
%    Straight section: x=0, y goes from -r_cl to 0 (bottom to midpoint)

y_entry = -r_cl;        % Bottom of the straight (-9.125m)
y_mid   =  0;           % Midpoint / origin

x_s1 = zeros(1, N_straight);
y_s1 = linspace(y_entry, y_mid, N_straight);

%% 2. Two laps of LEFT circle (counterclockwise = positive direction)
%    Left circle center: (-9.125, 0)
%    Car enters at (0, 0) which is the rightmost point → theta_start = 0
%    Counterclockwise: theta goes from 0 → 2*pi (one lap), repeated twice

theta_L = linspace(0, 4*pi, 2*N_circle);   % 2 full laps CCW
x_L = cx_L + r_cl * cos(theta_L);
y_L = cy   + r_cl * sin(theta_L);

%% 3. Transition straight: from (0,0) back to (0,0) — just the crossing
%    After 2 laps of left circle, car is back at (0, 0) heading right → crosses to right circle
%    This is a short straight at y=0 from x=0 to x=0 (instantaneous crossing)
%    We add a tiny connecting segment for smoothness (optional, or skip)
%    Actually the path just connects: end of left circle → start of right circle
%    Both meet at origin (0,0), so no extra points needed

%% 4. Two laps of RIGHT circle (clockwise = negative direction)
%    Right circle center: (+9.125, 0)
%    Car enters at (0, 0) which is the leftmost point → theta_start = pi
%    Clockwise: theta goes from pi → pi - 4*pi = -3*pi (two laps CW)

theta_R = linspace(pi, pi - 4*pi, 2*N_circle);   % 2 full laps CW
x_R = cx_R + r_cl * cos(theta_R);
y_R = cy   + r_cl * sin(theta_R);

%% 5. Top straight: from origin (0,0) up to exit (0, +r_cl)
x_s2 = zeros(1, N_straight);
y_s2 = linspace(y_mid, r_cl, N_straight);

%% Assemble full path in sequence
x_all = [x_s1, x_L, x_R, x_s2];
y_all = [y_s1, y_L, y_R, y_s2];

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
plot(cx_L, cy, 'k+', 'MarkerSize', 12);
plot(cx_R, cy, 'k+', 'MarkerSize', 12);

% Draw reference circles (inner edge, centerline, outer edge)
theta_ref = linspace(0, 2*pi, 360);
for cx = [cx_L, cx_R]
    plot(cx + 7.625*cos(theta_ref), 7.625*sin(theta_ref), 'k--', 'LineWidth', 0.5);  % inner
    plot(cx + 9.125*cos(theta_ref), 9.125*sin(theta_ref), 'r--', 'LineWidth', 0.5);  % centerline
    plot(cx + 10.625*cos(theta_ref), 10.625*sin(theta_ref), 'k--', 'LineWidth', 0.5); % outer
end

axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');
title('Skidpad Centerline - Formula Student');
legend('Centerline path', 'Entry', 'Exit', 'Location', 'best');