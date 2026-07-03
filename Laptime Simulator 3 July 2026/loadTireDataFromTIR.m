function TireData = loadTireDataFromTIR(filename, doPrint, saveToBase)
% loadTireDataFromTIR
% ---------------------------------------------------------
% Reads a Pacejka .tir file and creates TireData struct
% with scaling + derived constants. Optionally prints
% parsed values and exports TireData to base workspace.
%
% INPUTS:
%   filename   (string/char) : full path to .tir file
%   doPrint    (logical)     : true -> print all values
%   saveToBase (logical)     : true -> assignin('base','TireData',TireData)
%
% OUTPUT:
%   TireData (struct)
% ---------------------------------------------------------

    % Defaults
    if nargin < 2 || isempty(doPrint)
        doPrint = true;
    end
    if nargin < 3 || isempty(saveToBase)
        saveToBase = true;
    end

    % Convert input to string
    filename = string(filename);

    %% 1) FILE CHECK
    if ~isfile(filename)
        error('CRITICAL: Tire file "%s" not found.', filename);
    end

    if doPrint
        fprintf('Reading tire data from: %s\n', filename);
    end

    %% 2) READ DATA
    [Tir, Scales] = read_tir_file(filename);
    Scales.("LMUX") = 0.7;
    Scales.("LMUY") = 0.7;

    %% 3) PRINT VALUES TO VERIFY
    if doPrint
        fprintf('\n%s\n', repmat('=', 1, 60));
        fprintf('              PARSED TIRE PARAMETERS\n');
        fprintf('%s\n', repmat('-', 1, 60));

        fields = fieldnames(Tir);
        for i = 1:length(fields)
            key = fields{i};
            val = Tir.(key);
            fprintf('%-25s : %10.4f\n', key, val);
        end

        fprintf('\n%s\n', repmat('-', 1, 60));
        fprintf('              SCALING FACTORS (LAMBDAS)\n');
        fprintf('%s\n', repmat('-', 1, 60));

        fields = fieldnames(Scales);
        for i = 1:length(fields)
            key = fields{i};
            val = Scales.(key);
            fprintf('%-25s : %10.4f\n', key, val);
        end
        fprintf('%s\n\n', repmat('=', 1, 60));
    end
    % Patch Vertical Stiffness if not provided in TIR
    if ~isfield(Tir,'VERTICAL_STIFFNESS') || Tir.VERTICAL_STIFFNESS == 0
        Tir.VERTICAL_STIFFNESS = 90000;   % <-- choose your value (N/m)
        fprintf('>> Vertical stiffness not found. Using default = %.0f N/m\n', Tir.VERTICAL_STIFFNESS);
    end


    %% 4) PACKAGE INTO 'TireData' STRUCT
    TireData = Tir;
    TireData.Scales = Scales;

    % Derived constants
    TireData.Fz0 = Tir.FNOMIN * Scales.LFZ0;
    TireData.R0  = Tir.UNLOADED_RADIUS;

    % Safety fallback
    if TireData.R0 == 0
        warning('UNLOADED_RADIUS is 0. Division by zero may occur in Mz calc.');
        TireData.R0 = 0.228;
        if doPrint
            fprintf('>> Patched UNLOADED_RADIUS to %.3f m\n', TireData.R0);
        end
    end

    %% 5) EXPORT TO BASE WORKSPACE (optional)
    if saveToBase
        assignin('base', 'TireData', TireData);
        if doPrint
            fprintf('SUCCESS: "TireData" struct created in Base Workspace.\n');
            fprintf('Nominal Load: %.0f N\n', TireData.Fz0);
            fprintf('Ready for Simulink.\n');
        end
    end

end

%% ========================================================================
%  HELPER FUNCTION: READ .TIR FILE
% ========================================================================
function [Tir, Scales] = read_tir_file(fname)

    fid = fopen(fname, 'r');
    if fid < 0
        error('Could not open file: %s', fname);
    end

    Tir = struct();
    Scales = struct();

    % Defaults for tire parameters (ensures fields exist)
    d_vals = {'UNLOADED_RADIUS',0, 'FNOMIN',0, ...
              'PCX1',0, 'PDX1',0, 'PDX2',0, 'PDX3',0, 'PKX1',0, 'PKX2',0, 'PKX3',0, 'PEX1',0, 'PEX2',0, 'PEX3',0, 'PEX4',0, 'PHX1',0, 'PHX2',0, 'PVX1',0, 'PVX2',0, ...
              'PCY1',0, 'PDY1',0, 'PDY2',0, 'PDY3',0, 'PKY1',0, 'PKY2',0, 'PKY3',0, 'PEY1',0, 'PEY2',0, 'PEY3',0, 'PEY4',0, 'PHY1',0, 'PHY2',0, 'PVY1',0, 'PVY2',0, ...
              'QCZ1',0, 'QDZ1',0, 'QDZ2',0, 'QDZ3',0, 'QBZ1',0, 'QBZ2',0, 'QBZ3',0, 'QBZ4',0, 'QEZ1',0, 'QEZ2',0, 'QEZ3',0, 'QDZ6',0, 'QDZ7',0};

    for i = 1:2:length(d_vals)
        Tir.(d_vals{i}) = d_vals{i+1};
    end

    % Default scaling factors
    s_names = {'LFZ0','LCX','LMUX','LEX','LKX','LHX','LVX','LCY','LMUY','LEY','LKY','LHY','LVY','LTR','LRES'};
    for i = 1:length(s_names)
        Scales.(s_names{i}) = 1.0;
    end

    % Parse file line-by-line
    while ~feof(fid)
        line = fgetl(fid);

        % Skip invalid
        if ~ischar(line) || isempty(line)
            continue;
        end

        line_trim = strtrim(line);

        % Skip comments
        if startsWith(line_trim, '!') || startsWith(line_trim, '$')
            continue;
        end

        if contains(line_trim, '=')
            parts = split(line_trim, '=');
            key = upper(strtrim(parts{1}));
            val = strtrim(parts{2});

            % Remove trailing comments
            if contains(val, '$'), val = split(val, '$'); val = strtrim(val{1}); end
            if contains(val, '!'), val = split(val, '!'); val = strtrim(val{1}); end

            num = str2double(val);
            if ~isnan(num)
                if startsWith(key, 'L') && length(key) <= 5
                    Scales.(key) = num;
                else
                    Tir.(key) = num;
                end
            end
        end
    end

    fclose(fid);
end
