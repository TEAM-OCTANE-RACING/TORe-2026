kt= [100000 100000];
rho = 1.225;
g = 9.81;
a= 1;
vmax = 12.5;
ms = 210;
mu = 60;
m = ms + mu;
hcg = 0.21;
h_rm = 0.13;
l= 1.532;
alongmax = 1.2*g;
alatmax = 1.6*g;
t = 1.235;
cd = 3.55; 
zmax = 0.035; %35mm ground clearance
dampingratio = 0.7;
mr = [1.03 0.96]; 
tlld_dist = 0.48;
roll_gradient = 0.8; %deg/g
f_static = [ms*0.45*0.5*g ms*0.55*0.5*g];
m_us = [12 18];
a_long= [0.5*alongmax*m*hcg/l 0.5*alongmax*m*hcg/l];
a_lat =[tlld_dist*alatmax*ms*h_rm/t (1-tlld_dist)*alatmax*ms*h_rm/t];
f_aero = 0.5*rho*cd*a*(vmax)^2;
aero_dist = [f_aero*0.45*0.5 f_aero*0.55*0.5];
f_max = [231.5 202.5] + aero_dist;
naturalfrequency = [2.3 2.1];
kr = (f_static./g).*(2*pi*naturalfrequency).^2;
kw = 1./((1./kr)-(1./kt));
ks = kw./mr.^2;
k_phi = kw * t^2 *0.5;
k_roll = ms*g*h_rm/roll_gradient;
k_roll_nm_rad = k_roll *180/pi;
% k_arb = k_roll_nm_rad - sum(k_phi)
% arb_dist = 0.48;
% 
% arb = [k_arb*arb_dist k_arb*(1-arb_dist)]
k_arb_front = k_roll_nm_rad*tlld_dist - k_phi(1);
k_arb_rear = k_roll_nm_rad*(1-tlld_dist) - k_phi(2);
arb = [k_arb_front k_arb_rear];
natural_roll_gradient = rad2deg(ms*g*h_rm/sum(k_phi));

load_sink = f_static./kw;
dynamic_sink = f_max./kw;
total_travel = load_sink + dynamic_sink;

unsprung_frequency = 1/(2*pi)*sqrt((kw+kt)./m_us);

% *** NEW DERIVED METRICS ***
% 1. Damping Rates (N-s/m) - Based on target damping ratio
c_critical = 2 * sqrt(kw .* (f_static ./ g)); 
c_damping = dampingratio * c_critical;

% 2. Maximum Pitch/Dive Under Braking (approximate degrees)
% Calculates front compression + rear extension over the wheelbase.
% Uses kr (ride rate) instead of kw because tires also compress/extend relative to the ground.
dive_angle_deg = rad2deg(atan((a_long(1)/kr(1) + a_long(2)/kr(2)) / l));

% --- 8. Spring Preload Calculation (Using Damper Dyno Data) ---
% Estimated Damper Dyno Gas Forces at 0 mm/s for DNM BMX Shocks (Newtons)
% Typical DNM shock (10-12mm shaft @ 150-200 PSI IFP pressure) yields ~80N - 120N.
f_gas = [100, 100]; % Updated to typical DNM shock gas force: 100N

% Target shock compression from full extension to static ride height (m)
% Example: We want the shock to sit 20mm compressed at static ride height (droop travel)
target_droop_travel = [0.030, 0.030]; 

% Spring Free Length (m)
free_length = [0.125, 0.125]; % Updated to user value: 125mm

% 1. Static force at the shock absorber (using your MR = Shock/Wheel convention)
f_static_shock = f_static ./ mr;

% 2. The spring only needs to hold up the weight minus what the gas force holds
f_spring_req = f_static_shock - f_gas;

% 3. Total distance the spring must be compressed to achieve this force (m)
spring_comp_total = f_spring_req ./ ks;

% 4. Preload distance on the workbench (Shock fully extended)
% This is how far you must compress the spring collar from the spring's free length
preload_m = spring_comp_total - target_droop_travel;

% 5. Installed Spring Length on the workbench (m)
% What you will measure with calipers when assembling the coilover
installed_length = free_length - preload_m;

% --- 9. Dynamic Clearance Check ---
% Calculate how much clearance is left at maximum load
% dynamic_sink is the max wheel bump travel. If it exceeds zmax, you bottom out.
min_clearance = zmax - dynamic_sink;

% --- 10. Display Output ---
disp('--- SUSPENSION SIZING RESULTS ---');
fprintf('Front Freq:             %.2f Hz | Rear Freq: %.2f Hz\n', naturalfrequency(1), naturalfrequency(2));
fprintf('Front Spring Rate (ks): %.1f N/m (%.2f N/mm)\n', ks(1), ks(1)/1000);
fprintf('Rear Spring Rate (ks):  %.1f N/m (%.2f N/mm)\n', ks(2), ks(2)/1000);
fprintf('Front ARB Required:     %.1f Nm/rad\n', arb(1));
fprintf('Rear ARB Required:      %.1f Nm/rad\n', arb(2));
fprintf('Natural Roll Gradient:  %.2f deg/g (Target: %.2f deg/g)\n', natural_roll_gradient, roll_gradient);
fprintf('Req. Shock Damping:     F: %.1f N-s/m | R: %.1f N-s/m\n', c_damping(1), c_damping(2));
fprintf('Max Dive Angle:         %.2f deg\n', dive_angle_deg);

fprintf('\n--- DYNAMIC CLEARANCE CHECK ---\n');
fprintf('Static Clearance:       %.1f mm\n', zmax*1000);
fprintf('Min Clearance @ Max Gs: F: %.1f mm | R: %.1f mm\n', min_clearance(1)*1000, min_clearance(2)*1000);
if any(min_clearance <= 0)
    disp('>>> WARNING: VEHICLE WILL BOTTOM OUT UNDER MAX LOAD! <<<');
    disp('>>> FIX: Increase ride frequencies (stiffer springs) or raise static ride height. <<<');
end

fprintf('\n--- COILOVER SETUP ---\n');
fprintf('Target Droop Travel:    F: %.1f mm | R: %.1f mm\n', target_droop_travel(1)*1000, target_droop_travel(2)*1000);
fprintf('Required Preload:       F: %.1f mm | R: %.1f mm\n', preload_m(1)*1000, preload_m(2)*1000);
fprintf('Installed Spring Len:   F: %.1f mm | R: %.1f mm\n', installed_length(1)*1000, installed_length(2)*1000);