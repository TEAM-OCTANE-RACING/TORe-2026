% clc;
% clear;
% close all;

%% Vehicle Parameters

cf  = 35238.459;
cr  = 35033.582;

lf  = 1.532 * 0.51;
lr  = 1.532 - lf;

m   = 270;
izz = 47.563;

tr  = 1.235/2;
h   = 0.201;
gr  = 10.05;
rw  = 0.2286;

%% Velocity Range

Vx_array = 1:1:21;      % [1 3 5 ... 21]

Kp_array = zeros(size(Vx_array));
Ki_array = zeros(size(Vx_array));

%% Gain Calculation Loop

for i = 1:length(Vx_array)

    vx = Vx_array(i);

    ku = (lr*m*9.81/(cf*(lf+lr))) - (lf*m*9.81/(cr*(lf+lr)));

    A = [-(cf +cr)/(m*vx), ((-lf*cf) + (lr*cr))/(m*vx) - vx ; ((-lf*cf) + (lr*cr))/(izz*vx), -((lf^2*cf) + (lr^2*cr))/(izz*vx)];
    B = [ cf/(m*vx), 0 ; lf*cf/izz, 1/(izz)];

    a1 = A(1,1);
    a2 = A(1,2);
    a3 = A(2,1);
    a4 = A(2,2);
    b1 = B(1,1);
    b2 = B(1,2);
    b3 = B(2,1);
    b4 = B(2,2);
    
    k1 = b4;
    k2 = b2*a3-a1*b4;
    k3 = -(a1+a4);
    k4 = (a1*a4) - (a2*a3);

    Kp = (40 + 289.89 - k3)/k1;

    Ki = (40*289.89 + 839.89 - k4 - k2*Kp)/k1;

    Kp_array(i) = Kp;
    Ki_array(i) = Ki;

end

%% Display Results

Results = table(Vx_array', Kp_array', Ki_array', ...
    'VariableNames', {'Velocity_mps','Kp','Ki'});

disp(Results);

%% Arrays for Simulink 1-D Lookup Tables

disp('Velocity Breakpoints:')
disp(Vx_array)

disp('Kp Table:')
disp(Kp_array)

disp('Ki Table:')
disp(Ki_array)