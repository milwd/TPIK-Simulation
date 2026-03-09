classdef SimulationLogger < handle
    properties
        % Time and Indexing
        t
        maxLoops
        counter = 0
        
        % Robot State
        ql, qdotl        % Left Arm
        qr, qdotr        % Right Arm
        
        % Task Data (Cell arrays)
        xdotbar_task     
        activation_task  
        
        % Performance Metrics
        ee_dist_error    
        
        % --- NEW: Cooperative Analysis Data ---
        coop_v_ref      % Desired Object Velocity
        coop_v_solo_L   % Left Arm Independent Velocity
        coop_v_solo_R   % Right Arm Independent Velocity
        coop_v_final_L  % Left Arm Cooperative Velocity
        coop_v_final_R  % Right Arm Cooperative Velocity
        
        % References
        robot
        action_manager 
        n_actions
    end
    
    methods
        function obj = SimulationLogger(maxLoops, robotModel, actionManager)
            obj.maxLoops = maxLoops;
            obj.robot = robotModel;
            obj.action_manager = actionManager;
            
            % Pre-allocate arrays
            obj.t = nan(1, maxLoops);
            obj.ql = nan(7, maxLoops);
            obj.qdotl = nan(7, maxLoops);
            obj.qr = nan(7, maxLoops);
            obj.qdotr = nan(7, maxLoops);
            obj.ee_dist_error = nan(1, maxLoops);
            
            % New Coop Arrays
            obj.coop_v_ref = nan(6, maxLoops);
            obj.coop_v_solo_L = nan(6, maxLoops);
            obj.coop_v_solo_R = nan(6, maxLoops);
            obj.coop_v_final_L = nan(6, maxLoops);
            obj.coop_v_final_R = nan(6, maxLoops);
            
            % Setup Task Storage
            obj.n_actions = length(actionManager.actions);
            max_tasks = 0;
            for i = 1:obj.n_actions
                max_tasks = max(max_tasks, length(actionManager.actions{i}));
            end
            obj.xdotbar_task = cell(obj.n_actions, max_tasks, maxLoops);
            obj.activation_task = cell(obj.n_actions, max_tasks, maxLoops);
        end
        
        function update(obj, t, loop, actionManager)
            obj.counter = loop;
            obj.t(loop) = t;
            
            % 1. Store Joint State
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;
            
            % 2. Store Task Specifics
            curr_act_idx = actionManager.currentAction;
            if curr_act_idx <= obj.n_actions
                tasks = actionManager.actions{curr_act_idx};
                for j = 1:length(tasks)
                    task = tasks{j};
                    obj.xdotbar_task{curr_act_idx, j, loop} = task.xdotbar;
                    if ~isempty(task.A)
                        obj.activation_task{curr_act_idx, j, loop} = diag(task.A);
                    else
                        obj.activation_task{curr_act_idx, j, loop} = 0;
                    end
                end
            end
            
            % 3. Store Constraint Metric (Rigid Body Check)
            pL = obj.robot.left_arm.wTt(1:3, 4);
            pR = obj.robot.right_arm.wTt(1:3, 4);
            obj.ee_dist_error(loop) = norm(pL - pR);
            
            % 4. --- NEW: Capture Cooperative Solver Internals ---
            if isprop(actionManager, 'last_solver_data') && actionManager.last_solver_data.is_coop
                data = actionManager.last_solver_data;
                obj.coop_v_ref(:, loop)     = data.v_ref;
                obj.coop_v_solo_L(:, loop)  = data.v_solo_L;
                obj.coop_v_solo_R(:, loop)  = data.v_solo_R;
                obj.coop_v_final_L(:, loop) = data.v_coop_L;
                obj.coop_v_final_R(:, loop) = data.v_coop_R;
            end
        end
        
        function plotCoopAnalysis(obj)
            % Plots comparison of Independent vs Cooperative velocities
            % and the Rigid Body Constraint maintenance.
            
            % Downsample logic with NaN check
            step_size = 10;
            raw_idx = 1:step_size:obj.counter;
            valid_mask = ~isnan(obj.coop_v_ref(1, raw_idx)); % Check if coop data exists
            valid_idx = raw_idx(valid_mask);
            
            if isempty(valid_idx)
                fprintf('No cooperative task data found to plot.\n');
                return;
            end
            
            t_vec = obj.t(valid_idx);
            
            % --- FIGURE 1: VELOCITY COMPARISON ---
            
            figure('Name', 'Cooperative Analysis: Velocities', 'Color', 'w');
            tiledlayout(2,1);
            
            % Left Arm Comparison
            nexttile; hold on; grid on;
            v_ref_norm = vecnorm(obj.coop_v_ref(:, valid_idx));
            v_solo_L_norm = vecnorm(obj.coop_v_solo_L(:, valid_idx));
            v_final_L_norm = vecnorm(obj.coop_v_final_L(:, valid_idx));
            
            plot(t_vec, v_ref_norm, 'k--', 'LineWidth', 2, 'DisplayName', 'Desired Object Vel (v_{ref})');
            plot(t_vec, v_solo_L_norm, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Non-Cooperative (v_{solo})');
            plot(t_vec, v_final_L_norm, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Cooperative (v_{final})');
            ylabel('Vel Norm [m/s]'); title('Left Arm: Independent vs Cooperative');
            legend('Location', 'best');
            
            % Right Arm Comparison
            nexttile; hold on; grid on;
            v_solo_R_norm = vecnorm(obj.coop_v_solo_R(:, valid_idx));
            v_final_R_norm = vecnorm(obj.coop_v_final_R(:, valid_idx));
            
            plot(t_vec, v_ref_norm, 'k--', 'LineWidth', 2, 'DisplayName', 'Desired Object Vel');
            plot(t_vec, v_solo_R_norm, 'r:', 'LineWidth', 1.5, 'DisplayName', 'Non-Cooperative');
            plot(t_vec, v_final_R_norm, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Cooperative');
            ylabel('Vel Norm [m/s]'); title('Right Arm: Independent vs Cooperative');
            xlabel('Time [s]');
            
            % --- FIGURE 2: CONSTRAINT SATISFACTION ---
            
            figure('Name', 'Cooperative Analysis: Constraint', 'Color', 'w');
            
            % We look at the distance error specifically during the cooperative phase
            dist_err = obj.ee_dist_error(valid_idx);
            initial_dist = dist_err(1);
            deviation = (dist_err - initial_dist) * 1000; % Convert to mm
            
            subplot(2,1,1);
            plot(t_vec, dist_err, 'LineWidth', 2);
            ylabel('Distance [m]'); title('Distance between End-Effectors');
            grid on;
            
            subplot(2,1,2);
            plot(t_vec, deviation, 'r', 'LineWidth', 1.5);
            ylabel('Drift [mm]'); title('Deviation from Initial Grasp Distance');
            xlabel('Time [s]'); grid on;
            
            fprintf('Max grasp deviation during cooperation: %.4f mm\n', max(abs(deviation)));
        end
        
        function plotTaskPerformance(obj)
            % Aggregates Action 1 and Action 2 performance into a single figure
            % and adds vertical lines at transition times.
            
            valid_idx = 1:obj.counter; 
            max_t = obj.t(obj.counter);
            
            % 1. Detect Switch Times
            % Find indices where action_idx changes
            % We look at the 'action_idx' stored in the manager or inferred from data
            % Since we didn't explicitly log 'action_idx' in the properties above, 
            % we can infer it or rely on the user passing the switch time.
            % BETTER: Let's detect it from the activation data. 
            % When Action 2 starts, Action 1 tasks fade out.
            
            switch_times = [];
            % Simple heuristic: If we have multiple actions, find when Action 2 becomes active
            if obj.n_actions > 1
                % Check the first task of Action 2
                raw_act_a2 = squeeze(obj.activation_task(2, 1, valid_idx));
                for k = 1:length(raw_act_a2)
                    if ~isempty(raw_act_a2{k}) && norm(raw_act_a2{k}) > 0.01
                        switch_times = [switch_times, obj.t(k)];
                        break; 
                    end
                end
            end
            
            % 2. Setup the Figure
            figure('Name', 'Global Mission Performance', 'Color', 'w');
            
            % We will plot: 
            % Row 1: Action 1 Tasks (Vel Norm)
            % Row 2: Action 1 Tasks (Activation)
            % Row 3: Action 2 Tasks (Vel Norm)
            % Row 4: Action 2 Tasks (Activation)
            
            % Calculate total rows needed based on how many tasks exist
            n_tasks_a1 = length(obj.action_manager.actions{1});
            n_tasks_a2 = length(obj.action_manager.actions{2});
            
            % Let's make a grid: 2 Columns (Action 1 Left, Action 2 Right)
            % Rows = Max(n_tasks_a1, n_tasks_a2) * 2 (Vel + Act)
            
            tiledlayout(2, 2, 'TileSpacing', 'compact'); 
            % Layout: 
            % Tile 1: Action 1 Velocities (All tasks superimposed)
            % Tile 2: Action 2 Velocities (All tasks superimposed)
            % Tile 3: Action 1 Activations
            % Tile 4: Action 2 Activations
            
            % --- PLOT ACTION 1 (Left Column) ---
            nexttile; hold on; grid on; title('Action 1: Reach Tasks (Error)');
            names_a1 = {};
            for k = 1:n_tasks_a1
                [vec_x, ~] = obj.extract_data(1, k, valid_idx);
                if ~isempty(vec_x)
                    plot(obj.t(valid_idx), vecnorm(vec_x), 'LineWidth', 1.5);
                    tsk = obj.action_manager.actions{1}{k};
                    names_a1{end+1} = tsk.task_name;
                end
            end
            if ~isempty(switch_times), xline(switch_times(1), '--k', 'Phase Switch'); end
            ylabel('||x_{dot}||'); xlim([0 max_t]); legend(names_a1, 'Location', 'best');

            % --- PLOT ACTION 2 (Right Column) ---
            nexttile; hold on; grid on; title('Action 2: Coop Tasks (Error)');
            names_a2 = {};
            for k = 1:n_tasks_a2
                [vec_x, ~] = obj.extract_data(2, k, valid_idx);
                if ~isempty(vec_x)
                    plot(obj.t(valid_idx), vecnorm(vec_x), 'LineWidth', 1.5);
                    tsk = obj.action_manager.actions{2}{k};
                    names_a2{end+1} = tsk.task_name;
                end
            end
            if ~isempty(switch_times), xline(switch_times(1), '--k', 'Phase Switch'); end
            xlim([0 max_t]); legend(names_a2, 'Location', 'best');
            
            % --- PLOT ACTIVATIONS A1 (Bottom Left) ---
            nexttile; hold on; grid on; title('Action 1: Activation Levels');
            for k = 1:n_tasks_a1
                [~, vec_a] = obj.extract_data(1, k, valid_idx);
                if ~isempty(vec_a)
                    % Take mean of diagonal for simpler plot
                    plot(obj.t(valid_idx), mean(vec_a,1), 'LineWidth', 1.5);
                end
            end
            if ~isempty(switch_times), xline(switch_times(1), '--k'); end
            ylabel('Activation (0-1)'); xlabel('Time [s]'); ylim([-0.1 1.1]); xlim([0 max_t]);
            
            % --- PLOT ACTIVATIONS A2 (Bottom Right) ---
            nexttile; hold on; grid on; title('Action 2: Activation Levels');
            for k = 1:n_tasks_a2
                [~, vec_a] = obj.extract_data(2, k, valid_idx);
                if ~isempty(vec_a)
                    plot(obj.t(valid_idx), mean(vec_a,1), 'LineWidth', 1.5);
                end
            end
            if ~isempty(switch_times), xline(switch_times(1), '--k'); end
            xlabel('Time [s]'); ylim([-0.1 1.1]); xlim([0 max_t]);
        end
        
        % Helper to extract clean matrices from cells
        function [mat_x, mat_a] = extract_data(obj, act_idx, task_idx, valid_idx)
            raw_x = squeeze(obj.xdotbar_task(act_idx, task_idx, valid_idx));
            raw_a = squeeze(obj.activation_task(act_idx, task_idx, valid_idx));
            
            % Scan dimensions
            dim_x = 0; dim_a = 0;
            for i=1:length(raw_x), if ~isempty(raw_x{i}), dim_x=max(dim_x, length(raw_x{i})); end; end
            for i=1:length(raw_a), if ~isempty(raw_a{i}), dim_a=max(dim_a, length(raw_a{i})); end; end
            
            if dim_x == 0, mat_x = []; mat_a = []; return; end
            if dim_a == 0, dim_a = 1; end
            
            mat_x = zeros(dim_x, length(valid_idx));
            mat_a = zeros(dim_a, length(valid_idx));
            
            for i = 1:length(valid_idx)
                if ~isempty(raw_x{i}), mat_x(:,i) = raw_x{i}; end
                if ~isempty(raw_a{i}), mat_a(:,i) = raw_a{i}; end
            end
        end
        
        function plotAll(obj)
            valid_idx = 1:obj.counter; 
            max_t = obj.t(obj.counter);
            
            %% 1. JOINT STATES (Global)
            figure('Name', 'System Joint States', 'Color', 'w');
            tiledlayout(2,2, 'TileSpacing', 'compact');
            
            nexttile; plot(obj.t(valid_idx), obj.ql(:, valid_idx)'); title('Left Position'); grid on; xlim([0 max_t]);
            nexttile; plot(obj.t(valid_idx), obj.qr(:, valid_idx)'); title('Right Position'); grid on; xlim([0 max_t]);
            nexttile; plot(obj.t(valid_idx), obj.qdotl(:, valid_idx)'); title('Left Velocity'); grid on; xlim([0 max_t]);
            nexttile; plot(obj.t(valid_idx), obj.qdotr(:, valid_idx)'); title('Right Velocity'); grid on; xlim([0 max_t]);
            
            %% 2. TASK PERFORMANCE (Per Action)
            % Iterate over ALL actions automatically
            for act_i = 1:obj.n_actions
                
                tasks = obj.action_manager.actions{act_i};
                n_tasks = length(tasks);
                
                % Check if this action actually ran (has data)
                has_data = false;
                for t_idx = 1:length(valid_idx)
                    if ~isempty(obj.xdotbar_task{act_i, 1, t_idx})
                        has_data = true; break;
                    end
                end
                
                if ~has_data
                    fprintf('Skipping plot for Action %d (No data logged).\n', act_i);
                    continue;
                end
                
                figure('Name', sprintf('Action %d: %s', act_i, 'Performance'), 'Color', 'w');
                tiledlayout(n_tasks, 2, 'TileSpacing', 'compact');
                
                for t_k = 1:n_tasks
                    task_name = tasks{t_k}.task_name;
                    if isempty(task_name), task_name = sprintf('Task %d', t_k); end
                    
                    % --- ROBUST DATA EXTRACTION (Fixes the 1x1 error) ---
                    raw_xdot = squeeze(obj.xdotbar_task(act_i, t_k, valid_idx));
                    raw_act  = squeeze(obj.activation_task(act_i, t_k, valid_idx));
                    
                    % 1. Find Max Dimension in history
                    dim_x = 0; dim_a = 0;
                    for scan = 1:length(raw_xdot)
                        if ~isempty(raw_xdot{scan}), dim_x = max(dim_x, length(raw_xdot{scan})); end
                        if ~isempty(raw_act{scan}),  dim_a = max(dim_a, length(raw_act{scan})); end
                    end
                    if dim_x == 0, dim_x = 1; end
                    if dim_a == 0, dim_a = 1; end
                    
                    % 2. Build Matrices
                    mat_x = nan(dim_x, length(valid_idx));
                    mat_a = nan(dim_a, length(valid_idx));
                    
                    for k = 1:length(valid_idx)
                        % Fill Xdot
                        val_x = raw_xdot{k};
                        if isempty(val_x)
                             % Keep NaN
                        elseif length(val_x) == dim_x
                            mat_x(:, k) = val_x;
                        else
                            % Mismatch (e.g. task off), fill zeros
                             mat_x(:, k) = zeros(dim_x, 1);
                        end
                        
                        % Fill Activation
                        val_a = raw_act{k};
                        if isempty(val_a)
                            % Keep NaN
                        elseif length(val_a) == dim_a
                            mat_a(:, k) = val_a;
                        elseif length(val_a) == 1 && val_a == 0
                            % Expansion: Scalar 0 -> Vector of 0s
                            mat_a(:, k) = zeros(dim_a, 1);
                        end
                    end
                    
                    % Plot Velocity Norm
                    nexttile; 
                    plot(obj.t(valid_idx), vecnorm(mat_x), 'LineWidth', 2);
                    grid on; title([task_name ' Error Norm']); xlim([0 max_t]);
                    
                    % Plot Activation
                    nexttile; 
                    plot(obj.t(valid_idx), mat_a', 'LineWidth', 1.5);
                    grid on; title([task_name ' Activation']); ylim([-0.1 1.1]); xlim([0 max_t]);
                end
            end
        end
        
        function plotCoopCartesian(obj)
            % Plots trajectory of Left Tool, Right Tool, and Object
            
            % Robust Downsampling
            step_size = 20;
            if obj.counter < 1
                fprintf('No data logged to plot.\n'); return;
            end
            
            if obj.counter < step_size
                raw_idx = 1:obj.counter;
            else
                raw_idx = 1:step_size:obj.counter;
            end
            
            % --- CRITICAL FIX: Filter out NaNs ---
            % Check if columns in ql or qr have any NaNs and exclude those indices
            valid_mask = ~any(isnan(obj.ql(:, raw_idx)), 1) & ~any(isnan(obj.qr(:, raw_idx)), 1);
            valid_idx = raw_idx(valid_mask);
            
            if isempty(valid_idx)
                fprintf('plotCoopCartesian: No valid (non-NaN) joint states found.\n');
                return;
            end
            
            figure('Name', 'Cooperative Cartesian Trajectory', 'Color', 'w');
            view(3); grid on; hold on; axis equal;
            xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
            
            pL = []; pR = []; pCenter = [];
            
            fprintf('Reconstructing Cartesian Trajectories (%d points)...\n', length(valid_idx));
            
            % Reconstruction Loop
            for i = 1:length(valid_idx)
                idx = valid_idx(i);
                
                % Update Robot State safely
                obj.robot.left_arm.q = obj.ql(:, idx);
                obj.robot.left_arm.update_transform();
                
                obj.robot.right_arm.q = obj.qr(:, idx);
                obj.robot.right_arm.update_transform();
                
                tL = obj.robot.left_arm.wTt(1:3, 4);
                tR = obj.robot.right_arm.wTt(1:3, 4);
                
                pL(:, end+1) = tL;
                pR(:, end+1) = tR;
                pCenter(:, end+1) = (tL + tR) / 2; 
            end
            
            % Plot Paths
            if ~isempty(pL)
                plot3(pL(1,:), pL(2,:), pL(3,:), 'b--', 'LineWidth', 1, 'DisplayName', 'Left Tool');
                plot3(pR(1,:), pR(2,:), pR(3,:), 'r--', 'LineWidth', 1, 'DisplayName', 'Right Tool');
                plot3(pCenter(1,:), pCenter(2,:), pCenter(3,:), 'k-', 'LineWidth', 2, 'DisplayName', 'Object Center');
                
                % Plot Start/End
                plot3(pCenter(1,1), pCenter(2,1), pCenter(3,1), 'go', 'MarkerFaceColor','g', 'DisplayName', 'Start');
                plot3(pCenter(1,end), pCenter(2,end), pCenter(3,end), 'ks', 'MarkerFaceColor','k', 'DisplayName', 'Current');
            end
            
            % Plot Goal if available
            try
                found_goal = false;
                for a_idx = 1:length(obj.action_manager.actions)
                    action_stack = obj.action_manager.actions{a_idx};
                    for k = 1:length(action_stack)
                        if isprop(action_stack{k}, 'wTog')
                            goal = action_stack{k}.wTog(1:3, 4);
                            plot3(goal(1), goal(2), goal(3), 'm*', 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Goal');
                            found_goal = true;
                            break;
                        end
                    end
                    if found_goal, break; end
                end
            catch
                % Ignore
            end
            
            legend('show', 'Location', 'best');
            title('Bimanual Object Transport');
        end

        function plot3DTraj(obj)
            % 3D Stick Figure Plot
            
            % Check if we have valid data at the current counter
            idx = obj.counter;
            if idx < 1
                 fprintf('plot3DTraj: No data logged.\n'); return; 
            end
            
            % --- CRITICAL FIX: Ensure current index is not NaN ---
            % If the last step is NaN (e.g. simulation crash), backtrack until valid
            while idx > 0 && (any(isnan(obj.ql(:, idx))) || any(isnan(obj.qr(:, idx))))
                idx = idx - 1;
            end
            
            if idx == 0
                fprintf('plot3DTraj: No valid joint states found in history.\n');
                return;
            end
            
            figure('Name', '3D Robot Pose', 'Color', 'w');
            view(3); grid on; hold on; axis equal;
            xlabel('X'); ylabel('Y'); zlabel('Z');
            
            % Update Robot to this valid state
            obj.robot.left_arm.q = obj.ql(:, idx);
            obj.robot.left_arm.update_transform();
            
            obj.robot.right_arm.q = obj.qr(:, idx);
            obj.robot.right_arm.update_transform();
            
            % Get bases
            bL = obj.robot.left_arm.wTb(1:3, 4);
            bR = obj.robot.right_arm.wTb(1:3, 4);
            
            % Get End Effectors
            eL = obj.robot.left_arm.wTt(1:3, 4);
            eR = obj.robot.right_arm.wTt(1:3, 4);
            
            % Plot Base to EE vectors
            plot3([bL(1) eL(1)], [bL(2) eL(2)], [bL(3) eL(3)], 'b-o', 'LineWidth', 3, 'DisplayName', 'Left Arm');
            plot3([bR(1) eR(1)], [bR(2) eR(2)], [bR(3) eR(3)], 'r-o', 'LineWidth', 3, 'DisplayName', 'Right Arm');
            
            % Plot Base Origins
            plot3(bL(1), bL(2), bL(3), 'ks', 'MarkerFaceColor', 'k', 'DisplayName', 'Base');
            plot3(bR(1), bR(2), bR(3), 'ks', 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');

            legend show;
            title(sprintf('Robot Pose at t=%.2f', obj.t(idx)));
        end
    end
end

function idxs = step_down(max_val, step)
    if max_val < step
        idxs = 1:max_val;
    else
        idxs = 1:step:max_val;
    end
end