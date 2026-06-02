% =========================================================================
% HYBRID SENSITIVITY SCRIPT (V10 - LAP TIME + MAX ROLL ANGLE)
% Purpose: OFAT for Springs/ARBs/Camber + Explicit 15-Setup Damper Sweep
% Metrics: Lap Time (t2 - t1) AND Max Roll Angle
% =========================================================================
clc; close all; 

%% --- 0. PRE-FLIGHT WORKSPACE CHECK ---
fprintf('=== RUNNING PRE-FLIGHT DIAGNOSTICS ===\n');
variant_vars = {'Mode_ggv', 'Mode_timeseries', 'Mode_simulink', 'Mode_curvature_vs_steer', 'Mode_sideslip_vs_curvature'};
missing_vars = {};
for i = 1:length(variant_vars)
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', variant_vars{i}))
        missing_vars{end+1} = variant_vars{i};
    end
end
if ~isempty(missing_vars)
    error('Workspace is incomplete. Please run your main vehicle init script first.');
end
fprintf('-> Workspace verified.\n\n');

%% --- 1. INITIALIZATION & BASELINE SETUP ---
fprintf('=== INITIALIZING HYBRID SENSITIVITY ANALYSIS ===\n');
try
    sus_par_base = evalin('base', 'sus_par');
    par_base = evalin('base', 'Par');
catch
    error('Could not find "sus_par" or "Par" in the base workspace.');
end

modelName = 'Vehicle_Model_999_Modular_roll_gradient';
if ~bdIsLoaded(modelName)
    load_system(modelName); 
end

% FORCE DATASHEET BASELINE (DNM3: 16411 Ns/m)
C_f_base = 16411; 
C_r_base = 16411; 
sus_par_base.C_damp_f = C_f_base;
sus_par_base.C_damp_r = C_r_base;

if isfield(sus_par_base, 'Camber_static')
    sus_par_base.Camber_static = deg2rad(sus_par_base.Camber_static);
end
if isfield(par_base, 'Camber_static')
    par_base.Camber_static = deg2rad(par_base.Camber_static);
end

fprintf('=== RUNNING BASELINE SETUP ===\n');
simInBase = Simulink.SimulationInput(modelName);
simInBase = simInBase.setModelParameter('SimulationMode', 'normal');
simInBase = simInBase.setVariable('sus_par', sus_par_base);
simInBase = simInBase.setVariable('Par', par_base);

try
    out_base = sim(simInBase);
    t1_base = getScalarTime(out_base, 't1');
    t2_base = getScalarTime(out_base, 't2');
    base_laptime = t2_base - t1_base;
    base_roll = getMaxRoll(out_base);
catch ME
    fprintf('\n!!! BASELINE SIMULATION FAILED !!!\n');
    UnpackSimulinkError(ME);
    error('Baseline run failed. Check the error log above.');
end
fprintf('-> BASELINE LAP TIME: %.6f s\n', base_laptime);
fprintf('-> BASELINE MAX ROLL: %.6f\n\n', base_roll);

%% --- 2. PHASE 1: CONTINUOUS PARAMETER SWEEP (SPRINGS, ARBS, CAMBER) ---
fprintf('=== STARTING PHASE 1: CONTINUOUS PARAMETERS (±30%%) ===\n');

param_names = {'Ks_f', 'Ks_r', 'K_arb_f', 'K_arb_r', 'Camber_static'};
num_params  = length(param_names);

sweep_vals = cell(num_params, 1);
sweep_vals{1} = [sus_par_base.Ks_f * 0.7,      sus_par_base.Ks_f * 1.3];
sweep_vals{2} = [sus_par_base.Ks_r * 0.7,      sus_par_base.Ks_r * 1.3];
sweep_vals{3} = [sus_par_base.K_arb_f * 0.7,   sus_par_base.K_arb_f * 1.3];
sweep_vals{4} = [sus_par_base.K_arb_r * 0.7,   sus_par_base.K_arb_r * 1.3];
sweep_vals{5} = [-4.0,                         2.0]; % Degrees

pct_change_time_cont = zeros(num_params, 2); 
pct_change_roll_cont = zeros(num_params, 2);

for p = 1:num_params
    p_name = param_names{p};
    for level = 1:2
        sus_par_test = sus_par_base;
        Par_test = par_base;
        test_val = sweep_vals{p}(level);
        
        if strcmp(p_name, 'Camber_static')
            model_test_val = deg2rad(test_val); 
        else
            model_test_val = test_val;
        end
        
        if isfield(sus_par_test, p_name), sus_par_test.(p_name) = model_test_val; end
        if isfield(Par_test, p_name),     Par_test.(p_name)     = model_test_val; end
        
        lvl_str = {'LOW', 'HIGH'};
        fprintf('Testing %s = %.4f (%s)... ', p_name, test_val, lvl_str{level});
        
        simIn = Simulink.SimulationInput(modelName);
        simIn = simIn.setModelParameter('SimulationMode', 'normal');
        simIn = simIn.setVariable('sus_par', sus_par_test);
        simIn = simIn.setVariable('Par', Par_test);
        
        try
            out = sim(simIn);
            laptime = getScalarTime(out, 't2') - getScalarTime(out, 't1');
            maxroll = getMaxRoll(out);
            
            delta_time = ((laptime - base_laptime) / base_laptime) * 100;
            delta_roll = ((maxroll - base_roll) / base_roll) * 100;
            
            pct_change_time_cont(p, level) = delta_time;
            pct_change_roll_cont(p, level) = delta_roll;
            fprintf('Time Δ: %+.4f %% | Roll Δ: %+.4f %%\n', delta_time, delta_roll);
        catch
            fprintf('FAILED\n');
            pct_change_time_cont(p, level) = NaN;
            pct_change_roll_cont(p, level) = NaN;
        end
    end
end

%% --- 3. PHASE 2: EXPLICIT 15-SETUP DAMPER SWEEP ---
fprintf('\n=== STARTING PHASE 2: 15 DISCRETE DAMPER SETUPS ===\n');

configs = struct('name', {}, 'C_damp_f', {}, 'C_damp_r', {});
configs(1)  = struct('name', 'DNM1',   'C_damp_f', 13792.62, 'C_damp_r', 13792.62);
configs(2)  = struct('name', 'DNM2',   'C_damp_f', 14911.00, 'C_damp_r', 14911.00);
configs(3)  = struct('name', 'DNM3',   'C_damp_f', 20530.37, 'C_damp_r', 20530.37);
configs(4)  = struct('name', 'COEPF1', 'C_damp_f', 1281.18,  'C_damp_r', C_r_base);
configs(5)  = struct('name', 'COEPF2', 'C_damp_f', 745.91,   'C_damp_r', C_r_base);
configs(6)  = struct('name', 'COEPF3', 'C_damp_f', 714.34,   'C_damp_r', C_r_base);
configs(7)  = struct('name', 'COEPF4', 'C_damp_f', 1607.26,  'C_damp_r', C_r_base);
configs(8)  = struct('name', 'COEPF5', 'C_damp_f', 1028.65,  'C_damp_r', C_r_base);
configs(9)  = struct('name', 'COEPF6', 'C_damp_f', 714.34,   'C_damp_r', C_r_base);
configs(10) = struct('name', 'COEPR1', 'C_damp_f', C_f_base, 'C_damp_r', 1749.08);
configs(11) = struct('name', 'COEPR2', 'C_damp_f', C_f_base, 'C_damp_r', 1119.41);
configs(12) = struct('name', 'COEPR3', 'C_damp_f', C_f_base, 'C_damp_r', 777.37);
configs(13) = struct('name', 'COEPR4', 'C_damp_f', C_f_base, 'C_damp_r', 791.42);
configs(14) = struct('name', 'COEPR5', 'C_damp_f', C_f_base, 'C_damp_r', 758.89);
configs(15) = struct('name', 'COEPR6', 'C_damp_f', C_f_base, 'C_damp_r', 720.04);

num_dampers = length(configs);
pct_change_time_damp = zeros(num_dampers, 1);
pct_change_roll_damp = zeros(num_dampers, 1);

for i = 1:num_dampers
    fprintf('Run [%02d/15] %-7s (F: %8.2f | R: %8.2f)... ', ...
            i, configs(i).name, configs(i).C_damp_f, configs(i).C_damp_r);
        
    sus_par_test = sus_par_base;
    sus_par_test.C_damp_f = configs(i).C_damp_f;
    sus_par_test.C_damp_r = configs(i).C_damp_r;
    
    simIn = Simulink.SimulationInput(modelName);
    simIn = simIn.setModelParameter('SimulationMode', 'normal');
    simIn = simIn.setVariable('sus_par', sus_par_test);
    simIn = simIn.setVariable('Par', par_base);
    
    try
        out = sim(simIn);
        laptime = getScalarTime(out, 't2') - getScalarTime(out, 't1');
        maxroll = getMaxRoll(out);
        
        delta_time = ((laptime - base_laptime) / base_laptime) * 100;
        delta_roll = ((maxroll - base_roll) / base_roll) * 100;
        
        pct_change_time_damp(i) = delta_time;
        pct_change_roll_damp(i) = delta_roll;
        fprintf('Time Δ: %+.4f %% | Roll Δ: %+.4f %%\n', delta_time, delta_roll);
    catch
        fprintf('FAILED\n');
        pct_change_time_damp(i) = NaN;
        pct_change_roll_damp(i) = NaN;
    end
end
fprintf('\n=== ALL SIMULATIONS COMPLETE ===\n');

%% --- 4. DATA VISUALIZATION (4 HEATMAPS) ---
x_labels_cont = {'Low (-30% / -4°)', 'High (+30% / +2°)'};
y_labels_cont = {'Front Spring', 'Rear Spring', 'Front ARB', 'Rear ARB', 'Static Camber'};
y_labels_damp = {configs.name}';
x_label_damp = {'Performance Delta (%)'};

% Figure 1: LAP TIME (Continuous)
figure('Name', 'Continuous - Lap Time', 'Color', 'w', 'Position', [100 550 600 400]);
h1 = heatmap(x_labels_cont, y_labels_cont, pct_change_time_cont);
h1.Title = sprintf('Lap Time Sensitivity (%% Change)\nBase Lap: %.4f s', base_laptime);
h1.CellLabelFormat = '%+.4f %%'; h1.MissingDataColor = [0.15 0.15 0.15]; h1.Colormap = jet; 
balanceColorLimits(h1, pct_change_time_cont);

% Figure 2: MAX ROLL (Continuous)
figure('Name', 'Continuous - Max Roll Angle', 'Color', 'w', 'Position', [720 550 600 400]);
h2 = heatmap(x_labels_cont, y_labels_cont, pct_change_roll_cont);
h2.Title = sprintf('Max Roll Sensitivity (%% Change)\nBase Roll: %.6f', base_roll);
h2.CellLabelFormat = '%+.4f %%'; h2.MissingDataColor = [0.15 0.15 0.15]; h2.Colormap = turbo; 
balanceColorLimits(h2, pct_change_roll_cont);

% Figure 3: LAP TIME (Dampers)
figure('Name', 'Dampers - Lap Time', 'Color', 'w', 'Position', [100 100 400 350]);
h3 = heatmap(x_label_damp, y_labels_damp, pct_change_time_damp);
h3.Title = 'Damper Setup: Lap Time Δ';
h3.CellLabelFormat = '%+.4f %%'; h3.MissingDataColor = [0.15 0.15 0.15]; h3.Colormap = jet;

% Figure 4: MAX ROLL (Dampers)
figure('Name', 'Dampers - Max Roll Angle', 'Color', 'w', 'Position', [520 100 400 350]);
h4 = heatmap(x_label_damp, y_labels_damp, pct_change_roll_damp);
h4.Title = 'Damper Setup: Max Roll Δ';
h4.CellLabelFormat = '%+.4f %%'; h4.MissingDataColor = [0.15 0.15 0.15]; h4.Colormap = turbo;

% =========================================================================
% LOCAL DIAGNOSTIC & EXTRACTION FUNCTIONS
% =========================================================================
function max_r = getMaxRoll(sim_output)
    % Safely attempts to extract the maximum absolute roll angle
    try 
        r_data = sim_output.roll_angle.Data; 
    catch 
        try 
            r_data = sim_output.get('roll_angle').Data; 
        catch 
            try 
                r_data = sim_output.roll_angle; 
            catch
                r_data = 0; % Fallback if it fails entirely
            end
        end
    end
    max_r = max(abs(double(r_data)));
end

function t_val = getScalarTime(sim_output, var_name)
    try obj = sim_output.(var_name);
    catch, try obj = sim_output.get(var_name); catch, error('Var not logged'); end, end
    if isa(obj, 'timeseries'), t_val = double(obj.Data(end));
    elseif isa(obj, 'Simulink.SimulationData.Signal'), t_val = double(obj.Values.Data(end));
    elseif isstruct(obj) && isfield(obj, 'Data'), t_val = double(obj.Data(end));
    else, t_val = double(obj(end)); end
end

function balanceColorLimits(h, data_matrix)
    max_val = max(abs(data_matrix(~isnan(data_matrix))));
    if max_val > 0, h.ColorLimits = [-max_val, max_val]; end
end

function UnpackSimulinkError(ME)
    fprintf('  Top Level Error: %s\n', ME.message);
    if ~isempty(ME.cause)
        for k = 1:length(ME.cause)
            fprintf('  Root Cause %d: %s\n', k, ME.cause{k}.message);
        end
    end
end