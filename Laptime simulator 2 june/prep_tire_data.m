% =========================================================================
%  DATA PREPARATION SCRIPT FOR SIMULINK
%  1. Checks for specific .tir file (Errors if missing)
%  2. Reads and Prints ALL values for verification
%  3. Creates 'TireData' struct in Base Workspace
% =========================================================================
% 1. CONFIGURATION
filename = 'C:\Disk_E\TOR_controls\Modular_vehicle_model_23_May\mrf.tir';  %

% 2. FILE CHECK 
if ~isfile(filename)
    error('CRITICAL: Tire file "%s" not found in current directory.', filename);
end

fprintf('Reading tire data from: %s\n', filename);

% 3. READ DATA
[Tir, Scales] = read_tir_file(filename);
Scales.("LMUX") = 0.7;
Scales.("LMUY") = 0.7;

% 4. PRINT VALUES TO VERIFY
fprintf('\n%s\n', repmat('=', 1, 60));
fprintf('              PARSED TIRE PARAMETERS\n');
fprintf('%s\n', repmat('-', 1, 60));

% Print Parameters
fields = fieldnames(Tir);
for i = 1:length(fields)
    key = fields{i};
    val = Tir.(key);
    fprintf('%-25s : %10.4f\n', key, val);
end

fprintf('\n%s\n', repmat('-', 1, 60));
fprintf('              SCALING FACTORS (LAMBDAS)\n');
fprintf('%s\n', repmat('-', 1, 60));

% Print Scaling Factors 
fields = fieldnames(Scales);
for i = 1:length(fields)
    key = fields{i};
    val = Scales.(key);
    fprintf('%-25s : %10.4f\n', key, val);
end
fprintf('%s\n\n', repmat('=', 1, 60));

% 5. PACKAGE INTO 'TireData' STRUCT FOR SIMULINK
TireData = Tir;
TireData.Scales = Scales;

% Pre-calculate Physical Constants
TireData.Fz0 = Tir.FNOMIN * Scales.LFZ0;
TireData.R0  = Tir.UNLOADED_RADIUS;
TireData.VERTICAL_STIFFNESS = 90000;

% Final Safety Check
if TireData.R0 == 0
    warning('UNLOADED_RADIUS is 0. Division by zero may occur in Mz calc.');
    TireData.R0 = 0.228; % Fallback
    fprintf('>> Patched UNLOADED_RADIUS to %.2fm\n', TireData.R0);
end

% 6. EXPORT TO WORKSPACE
assignin('base', 'TireData', TireData);

fprintf('SUCCESS: "TireData" struct created in Base Workspace.\n');
fprintf('Nominal Load: %.0f N\n', TireData.Fz0);
fprintf('Ready for Simulink.\n');


%% ========================================================================
%  HELPER FUNCTION: READ .TIR FILE
%  ========================================================================
function [Tir, Scales] = read_tir_file(fname)
    fid = fopen(fname, 'r');
    Tir = struct(); Scales = struct();
   
    % This ensures the struct has all fields even if the file is sparse
    d_vals = {'UNLOADED_RADIUS',0, 'FNOMIN',0, ...
              'PCX1',0, 'PDX1',0, 'PDX2',0, 'PDX3',0, 'PKX1',0, 'PKX2',0, 'PKX3',0, 'PEX1',0, 'PEX2',0, 'PEX3',0, 'PEX4',0, 'PHX1',0, 'PHX2',0, 'PVX1',0, 'PVX2',0, ...
              'PCY1',0, 'PDY1',0, 'PDY2',0, 'PDY3',0, 'PKY1',0, 'PKY2',0, 'PKY3',0, 'PEY1',0, 'PEY2',0, 'PEY3',0, 'PEY4',0, 'PHY1',0, 'PHY2',0, 'PVY1',0, 'PVY2',0, ...
              'QCZ1',0, 'QDZ1',0, 'QDZ2',0, 'QDZ3',0, 'QBZ1',0, 'QBZ2',0, 'QBZ3',0, 'QBZ4',0, 'QEZ1',0, 'QEZ2',0, 'QEZ3',0, 'QDZ6',0, 'QDZ7',0};
    for i=1:2:length(d_vals), Tir.(d_vals{i}) = d_vals{i+1}; end
    
    % Initialize SCALES (Defaults to 1.0)
    s_names = {'LFZ0','LCX','LMUX','LEX','LKX','LHX','LVX','LCY','LMUY','LEY','LKY','LHY','LVY','LTR','LRES'};
    for i=1:length(s_names), Scales.(s_names{i}) = 1.0;
    end
    while ~feof(fid)
        line = fgetl(fid);
        % Skip comments or empty lines
        if isempty(line) || startsWith(strtrim(line), '!') || startsWith(strtrim(line), '$'), continue; end
        
        if contains(line, '=')
            parts = split(line, '=');
            key = upper(strtrim(parts{1})); 
            val = strtrim(parts{2});
            
            % Remove trailing comments (e.g. "1.5 $ comment")
            if contains(val, '$'), vsp = split(val, '$'); val = vsp{1}; end
            if contains(val, '!'), vsp = split(val, '!'); val = vsp{1}; end
            
            num = str2double(val);
            if ~isnan(num)
                % Sort into Scales (L-params) or Data (everything else)
                if startsWith(key, 'L') && length(key)<=5
                    Scales.(key) = num;
                else
                    Tir.(key) = num;
                end
            end
        end
    end
    
    fclose(fid);
end