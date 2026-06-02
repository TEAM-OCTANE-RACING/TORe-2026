function laptime = calculate_laptime(track_file, ggv_file)
% CALCULATE_LAPTIME Returns only the total lap time in seconds.
% Inputs:
%   track_file - String path to the track .csv file
%   ggv_file   - String path to the GGV .mat file

   if ~exist(track_file, 'file')
        error('Track file "%s" not found.', track_file);
    end
    
    if ~exist(ggv_file, 'file')
        potential_files = {'GGV_Final_Data.mat', 'GGV_Sensor_Data.mat', 'GGV_Safe_Data.mat', 'GGV_Map.mat'};
        found = false;
        for k = 1:length(potential_files)
            if exist(potential_files{k}, 'file')
                ggv_file = potential_files{k};
                found = true;
                break;
            end
        end
        if ~found
            error('No GGV files found. Please ensure the 3D GGV Script has been run.');
        end
    end
    
    loaded_data = load(ggv_file, 'results'); 
    if ~isfield(loaded_data, 'results')
        error('File "%s" loaded, but it lacks the "results" structure.', ggv_file);
    end
    ggv_raw = loaded_data.results;


    % 2. Optimize Racing Line (Extract distance 's' and curvature 'k')
    [path_s, path_k, ~, ~] = Optimize_Racing_Line(track_file);

    % 3. Pre-process the vehicle's performance envelope
    GGV_Env = Process_GGV_Envelope(ggv_raw);

    % 4. Run the solver to get the velocity profile and time
    Results = Solve_3Pass(path_s, path_k, GGV_Env);

    % 5. Output the final lap time
    laptime = Results.laptime;
end

% =========================================================================
% REQUIRED HELPER FUNCTIONS (The core math)
% =========================================================================

function [s_final, k_final, x_final, y_final] = Optimize_Racing_Line(filename)
    data = readmatrix(filename);
    xc = data(:,1); yc = data(:,2); wr = data(:,3); wl = data(:,4);
    
    if norm([xc(1)-xc(end), yc(1)-yc(end)]) > 0.1
        xc(end+1)=xc(1); yc(end+1)=yc(1); wl(end+1)=wl(1); wr(end+1)=wr(1);
    end
    
    N = length(xc); x = xc; y = yc; 
    Iterations = 200; Safety_Margin = 0.5;
    
    for iter = 1:Iterations
        xt = 0.5 * ([x(end-1); x(1:end-1)] + [x(2:end); x(2)]);
        yt = 0.5 * ([y(end-1); y(1:end-1)] + [y(2:end); y(2)]);
        
        for i = 1:N
            dx = xt(i) - xc(i); dy = yt(i) - yc(i); dist = sqrt(dx^2 + dy^2);
            if i < N, tx = xc(i+1)-xc(i); ty = yc(i+1)-yc(i); else, tx = xc(2)-xc(1); ty = yc(2)-yc(1); end
            
            if (tx*dy - ty*dx) > 0, max_w = wl(i)-Safety_Margin; else, max_w = wr(i)-Safety_Margin; end
            if max_w < 0.1, max_w = 0.1; end
            
            if dist > max_w
                scale = max_w / dist; 
                x(i) = xc(i) + dx * scale; y(i) = yc(i) + dy * scale;
            else
                x(i) = xt(i); y(i) = yt(i); 
            end
        end
    end
    
    dx = gradient(x); dy = gradient(y); ddx = gradient(dx); ddy = gradient(dy);
    k_raw = (dx .* ddy - dy .* ddx) ./ ((dx.^2 + dy.^2).^1.5);
    k_final = smoothdata(k_raw, 'gaussian', 15); 
    
    ds = sqrt(dx.^2 + dy.^2); 
    s_final = [0; cumsum(ds(2:end))];
    x_final = x; y_final = y;
end

function Env = Process_GGV_Envelope(results)
    Env.v = results.v(:); 
    Env.ay_max = abs(results.ay_max(:)) * 9.81;
    Env.ax_acc = results.ax_max(:) * 9.81;
    Env.ax_brk = abs(results.ax_brk(:)) * 9.81; 
    
    [Env.v, idx] = sort(Env.v);
    Env.ay_max = Env.ay_max(idx);
    Env.ax_acc = Env.ax_acc(idx);
    Env.ax_brk = Env.ax_brk(idx);
end

function R = Solve_3Pass(s, k, Env)
    N = length(s);
    v_apex = zeros(N, 1);
    v_acc = zeros(N, 1); ax_acc_log = zeros(N, 1);
    v_brk = zeros(N, 1); ax_brk_log = zeros(N, 1);
    
    % --- PASS 1: APEX SPEED ---
    for i = 1:N
        curv = abs(k(i));
        if curv < 1e-4
            v_apex(i) = max(Env.v); 
        else
            v_g = 20; 
            for iter=1:5
                ay_lim = interp1(Env.v, Env.ay_max, v_g, 'linear', 'extrap');
                v_new = sqrt(ay_lim / curv);
                v_g = 0.6*v_g + 0.4*v_new; 
            end
            v_apex(i) = v_g;
        end
    end
    
    % --- PASS 2: ACCELERATION (Forward) ---
    v_acc(1) = 0;      
    for i = 1:N-1
        ds = s(i+1) - s(i); 
        v_curr = v_acc(i);
        ay_req = v_curr^2 * abs(k(i));
        
        ay_max_v = interp1(Env.v, Env.ay_max, v_curr, 'linear', 'extrap');
        ax_max_v = interp1(Env.v, Env.ax_acc, v_curr, 'linear', 'extrap');
        
        if ay_req >= ay_max_v
            ax_avail = 0; 
        else
            ax_avail = ax_max_v * sqrt(1 - (ay_req/ay_max_v)^2);
        end
        ax_acc_log(i) = ax_avail; 
        
        v_next = sqrt(v_curr^2 + 2 * ax_avail * ds);
        v_acc(i+1) = min(v_next, v_apex(i+1));
    end
    
    % --- PASS 3: BRAKING (Backward) ---
    v_brk(N) = v_apex(N);
    for i = N:-1:2
        ds = s(i) - s(i-1); 
        v_curr = v_brk(i);
        ay_req = v_curr^2 * abs(k(i));
        
        ay_max_v = interp1(Env.v, Env.ay_max, v_curr, 'linear', 'extrap');
        ax_brk_v = abs(interp1(Env.v, Env.ax_brk, v_curr, 'linear', 'extrap'));
        
        if ay_req >= ay_max_v
            ax_avail = 0;
        else
            ax_avail = ax_brk_v * sqrt(1 - (ay_req/ay_max_v)^2);
        end
        ax_brk_log(i) = -ax_avail; 
        
        v_prev = sqrt(v_curr^2 + 2 * ax_avail * ds);
        v_brk(i-1) = min(v_prev, v_apex(i-1));
    end
    
    % --- FINAL COMBINATION ---
    v_final = zeros(N, 1);
    
    for i = 1:N
        v_final(i) = min([v_acc(i), v_brk(i), v_apex(i)]);
    end
    
    % Lap time calculation
    R.laptime = sum(diff(s) ./ ((v_final(1:end-1) + v_final(2:end))/2));
end


track_file = 'C:\Users\vishal bhujbal\OneDrive\Desktop\Modular_vehicle_model\skidpad_centerline.csv';
laptime = calculate_laptime(track_file,ggv_file);
disp(laptime)