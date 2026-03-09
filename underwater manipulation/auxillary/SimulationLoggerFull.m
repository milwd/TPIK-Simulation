classdef SimulationLoggerFull < handle
    properties
        % --- Robot State ---
        t            % time vector [1 x N]
        q            % joint positions [7 x N]
        q_dot        % joint velocities [7 x N]
        eta          % vehicle pose [6 x N]
        v_nu         % vehicle velocity [6 x N]
        
        % --- Mission State ---
        action_idx   % Action Index [1 x N]
        
        % --- Task Data Storage ---
        % Map: Key = 'TaskID' (char), Value = struct('xdotbar', [], 'activation', [])
        task_data    
        
        robot        % Handle to robot model
    end

    methods
        function obj = SimulationLoggerFull(maxLoops, robotModel)
            obj.robot = robotModel;
            
            % Preallocate Robot State
            obj.t = nan(1, maxLoops);
            obj.q = nan(7, maxLoops);
            obj.q_dot = nan(7, maxLoops);
            obj.eta = nan(6, maxLoops);
            obj.v_nu = nan(6, maxLoops);
            obj.action_idx = nan(1, maxLoops);
            
            % Initialize Map for Dynamic Task Logging
            obj.task_data = containers.Map();
        end

        function update(obj, t, loop, actionManager)
            % 1. Log Standard Robot Data
            obj.t(loop) = t;
            obj.q(:, loop) = obj.robot.q;
            obj.q_dot(:, loop) = obj.robot.q_dot;
            obj.eta(:, loop) = obj.robot.eta;
            obj.v_nu(:, loop) = obj.robot.v_nu;
            obj.action_idx(loop) = actionManager.currentAction;
            
            % 2. Log Tasks dynamically based on what is active
            
            % Get the list of currently running tasks
            activeTasks = actionManager.activeStack;
            
            % Get their IDs from the transition plan
            % Fallback: if plan is empty/malformed, use current action definition
            if isfield(actionManager.transitionPlan, 'id') && ~isempty(actionManager.transitionPlan)
                currentIDs = {actionManager.transitionPlan.id};
            else
                currentIDs = actionManager.actions_ids{actionManager.currentAction};
            end
            
            % Loop through every active task
            for i = 1:length(activeTasks)
                taskObj = activeTasks{i};
                
                % Ensure ID is a valid character array for the Map Key
                taskID = char(string(currentIDs{i})); 
                
                % If this is the first time we see this Task ID, initialize its storage
                if ~isKey(obj.task_data, taskID)
                    s = struct();
                    % Preallocate with NaNs so inactive periods show as breaks
                    s.xdotbar = nan(6, length(obj.t)); 
                    s.activation = nan(6, length(obj.t));
                    obj.task_data(taskID) = s;
                end
                
                % Retrieve struct, fill data, save back
                s = obj.task_data(taskID);
                
                % Handle 6D or smaller tasks
                ref = taskObj.xdotbar;
                act = diag(taskObj.A); % Only log the diagonal of Activation
                
                dim_r = length(ref);
                dim_a = length(act);
                
                if dim_r > 0, s.xdotbar(1:dim_r, loop) = ref; end
                if dim_a > 0, s.activation(1:dim_a, loop) = act; end
                
                obj.task_data(taskID) = s;
            end
        end

        function plotAll(obj)
            % Plot 1: Standard Robot State
            figure('Name', 'Robot State', 'NumberTitle', 'off');
            
            subplot(5,1,1);
            plot(obj.t, obj.eta(1:3, :)', 'LineWidth', 1.5);
            title('Vehicle Position (\eta)');
            ylabel('[m]'); 
            legend({'x','y','z'}, 'Location', 'best');
            grid on;

            subplot(5,1,2);
            plot(obj.t, obj.eta(4:6, :)', 'LineWidth', 1.5);
            title('Vehicle Orientation (\eta)');
            ylabel('[rad]'); 
            legend({'roll','pitch','yaw'}, 'Location', 'best');
            grid on;
            
            subplot(5,1,3);
            plot(obj.t, obj.v_nu(1:3, :)', 'LineWidth', 1.5);
            title('Vehicle Linear Velocity (\nu)');
            ylabel('[m/s]');
            grid on;

            subplot(5,1,4);
            plot(obj.t, obj.v_nu(4:6, :)', 'LineWidth', 1.5);
            title('Vehicle Angular Velocity (\nu)');
            ylabel('[rad/s]');
            grid on;
            
            subplot(5,1,5);
            % Use stairs for discrete action steps
            stairs(obj.t, obj.action_idx, 'LineWidth', 2, 'Color', 'k');
            title('Mission Phase');
            xlabel('Time [s]'); ylabel('Action Index');
            ylim([0 max(obj.action_idx)+1]);
            grid on;
            
            % --- MIXED PLOTS FOR TASKS ---
            
            % Get all unique Task IDs encountered
            allTaskIDs = keys(obj.task_data);
            colors = lines(length(allTaskIDs)); % Generate distinct colors
            
            % Plot 2: Global Activation Timeline (The "Mix Plot")
            figure('Name', 'Task Activations (Overview)', 'NumberTitle', 'off');
            hold on;
            for k = 1:length(allTaskIDs)
                id = allTaskIDs{k};
                data = obj.task_data(id);
                % We take the mean activation (or max) to represent "Task Activity" 
                % just to show when it turned on/off in a single line.
                avg_act = nanmean(data.activation, 1); 
                plot(obj.t, avg_act, 'LineWidth', 2, 'DisplayName', id, 'Color', colors(k,:));
            end
            hold off;
            title('Task Activation Timeline (0=Inactive, 1=Active)');
            xlabel('Time [s]'); ylabel('Activation Level');
            legend('show', 'Location', 'best');
            grid on;
            ylim([-0.1 1.1]);
            
            % Plot 3: Detailed Reference Velocities (One Subplot per Task)
            figure('Name', 'Task Reference Velocities', 'NumberTitle', 'off');
            numTasks = length(allTaskIDs);
            
            for k = 1:numTasks
                id = allTaskIDs{k};
                data = obj.task_data(id);
                
                subplot(numTasks, 1, k);
                plot(obj.t, data.xdotbar', 'LineWidth', 1.5);
                title(['Reference Velocity: ' id]);
                ylabel('[m/s] / [rad/s]');
                if k == numTasks, xlabel('Time [s]'); end
                grid on;
                
                % Add a dynamic legend based on how many dimensions (rows) have data
                % Check which rows are not all NaNs
                active_rows = find(sum(~isnan(data.xdotbar), 2) > 0);
                if ~isempty(active_rows)
                    % Create generic labels like "dim 1", "dim 2" or specific if known
                    labels = arrayfun(@(x) sprintf('dim %d', x), active_rows, 'UniformOutput', false);
                    legend(labels, 'Location', 'eastoutside');
                end
            end
        end
    end
end