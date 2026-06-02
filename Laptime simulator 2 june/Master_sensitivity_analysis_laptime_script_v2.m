% =========================================================================
% DISCRETE DAMPER SENSITIVITY SCRIPT (V7 - SESSION SAFE & PRE-FLIGHT)
% Purpose: Run 15 explicit datasheet damper configurations
% Safety: Includes Pre-Flight Workspace Check to prevent Variant crashes
% =========================================================================
% clc; close all; % NO clearvars. Your workspace is protected.

%% --- 0. PRE-FLIGHT WORKSPACE CHECK ---
fprintf('=== RUNNING PRE-FLIGHT DIAGNOSTICS ===\n');
% Simulink requires these variables in the base workspace to build the model
variant_vars = {'Mode_ggv', 'Mode_timeseries', 'Mode_simulink', 'Mode_curvature_vs_steer', 'Mode_sideslip_vs_curvature'};
missing_vars = {};
for i = 1:length(variant_vars)
    if ~evalin('base', sprintf('exist(''%s'', ''var'')', variant_vars{i}))
        missing_vars{end+1} = variant_vars{i};
    end
end

if ~isempty(missing_vars)
    fprintf('\n!!! PRE-FLIGHT CHECK FAILED !!!\n');
    fprintf('The following Variant Controls are missing from your workspace:\n');
    for i = 1:length(missing_vars)
        fprintf(' - %s\n', missing_vars{i});
    end
    error('Workspace is incomplete. Please run your main vehicle setup/init script before running this sensitivity analysis.');
end
fprintf('-> Workspace verified. All Variant Controls are present.\n\n');

%% --- 1. INITIALIZATION & DATASHEET BASELINE SETUP ---
fprintf('=== INITIALIZING DISCRETE DAMPER SENSITIVITY ANALYSIS ===\n');

try
    sus_par_base = evalin('base', 'sus_par');
    par_base = evalin('base', 'Par');
catch
    error('Could not find "sus_par" or "Par" in the base workspace. Load baseline parameters first.');
end

modelName = 'Vehicle_Model_999_Modular_roll_gradient';
if ~bdIsLoaded(modelName)
    load_system(modelName); 
end

% FORCE DATASHEET BASELINE (16411 Ns/m)
C_f_base = 16411; 
C_r_base = 16411; 

sus_par_base.C_damp_f = C_f_base;
sus_par_base.C_damp_r = C_r_base;

fprintf('-> Baseline damping explicitly set: %d Ns/m\n', C_f_base);
fprintf('\n=== RUNNING BASELINE SETUP ===\n');

% Safely sandbox the simulation using normal mode to inherit base workspace
simInBase = Simulink.SimulationInput(modelName);
simInBase = simInBase.setModelParameter('SimulationMode', 'normal');
simInBase = simInBase.setVariable('sus_par', sus_par_base);
simInBase = simInBase.setVariable('Par', par_base);

try
    out_base = sim(simInBase);
    t1_base = getScalarTime(out_base, 't1');
    t2_base = getScalarTime(out_base, 't2');
    base_laptime = t2_base - t1_base;
catch ME
    fprintf('\n!!! BASELINE SIMULATION FAILED !!!\n');
    UnpackSimulinkError(ME);
    error('Baseline run failed. Check the error log above.');
end

fprintf('-> BASELINE LAP TIME SUCCESS: %.6f seconds\n\n', base_laptime);

%% --- 2. HARDCODE SPREADSHEET EXPERIMENTAL DATA ---
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

num_runs = length(configs);
raw_laptimes = zeros(num_runs, 1);
pct_deltas   = zeros(num_runs, 1);

%% --- 3. DYNAMIC SIMULATION LOOP ---
fprintf('=== STARTING DISCRETE 15-SET RUNS ===\n');

for i = 1:num_runs
    fprintf('Run [%02d/15] Setup: %-7s | Front C: %8.2f | Rear C: %8.2f\n', ...
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
        t1_run = getScalarTime(out, 't1');
        t2_run = getScalarTime(out, 't2');
        laptime = t2_run - t1_run;
    catch ME
        fprintf(' -> !!! SIMULATION FAILED !!!\n');
        UnpackSimulinkError(ME); 
        laptime = NaN;
    end
    
    raw_laptimes(i) = laptime;
    if isnan(laptime)
        pct_deltas(i) = NaN;
    else
        pct_deltas(i) = ((laptime - base_laptime) / base_laptime) * 100;
        fprintf(' -> Resulting Lap Time: %.6f s | Delta: %+.6f %%\n', laptime, pct_deltas(i));
    end
end
fprintf('\n=== DISCRETE PERFORMANCE SWEEP COMPLETE ===\n');

%% --- 4. DATA ARCHIVING & PLOTTING ---
y_labels = {configs.name}';
results_table = table(y_labels, raw_laptimes, pct_deltas, ...
    'VariableNames', {'Damper_Setup', 'Laptime_Seconds', 'Percent_Delta'});
assignin('base', 'Damper_Sweep_Results', results_table);

figure('Name', 'Damper Variant Performance Heatmap', 'Color', 'w', 'Position', [300 100 650 700]);
display_matrix = [raw_laptimes, pct_deltas];
x_labels = {'Raw Lap Time (s)', 'Performance Delta (%)'};
h = heatmap(x_labels, y_labels, display_matrix);
h.Title = sprintf('Damper Setup Comparison\n(Base: %.4f s)', base_laptime);
h.XLabel = 'Performance Metrics';
h.YLabel = 'Damper Config';
h.CellLabelFormat = '%.4f'; 
h.MissingDataLabel = 'Solver Crash';
h.MissingDataColor = [0.15 0.15 0.15]; 
h.Colormap = turbo;

% =========================================================================
% LOCAL DIAGNOSTIC FUNCTIONS
% =========================================================================
function UnpackSimulinkError(ME)
    fprintf('  Top Level Error: %s\n', ME.message);
    if ~isempty(ME.cause)
        for k = 1:length(ME.cause)
            cause = ME.cause{k};
            fprintf('  Root Cause %d: %s\n', k, cause.message);
        end
    end
    fprintf('------------------------------------------------------\n');
end

function t_val = getScalarTime(sim_output, var_name)
    try
        obj = sim_output.(var_name);
    catch
        try
            obj = sim_output.get(var_name);
        catch
            error('Variable "%s" not logged.', var_name);
        end
    end
    
    if isa(obj, 'timeseries')
        t_val = double(obj.Data(end));
    elseif isa(obj, 'Simulink.SimulationData.Signal')
        t_val = double(obj.Values.Data(end));
    elseif isstruct(obj) && isfield(obj, 'Data')
        t_val = double(obj.Data(end));
    elseif isnumeric(obj)
        t_val = double(obj(end));
    else
        t_val = double(obj);
    end
end