classdef ActionManager_T < handle
    properties
        actions = {}      
        actions_ids = {}  
        currentAction = 1 
        
        % --- Transition State Variables ---
        activeStack = {}       
        transitionPlan = []    
        inTransition = false   
        t_switch = 0           
        t_max = 8.0            
        
        completion_threshold = 0.001; 
    end

    methods
        function addAction(obj, taskStack, taskStackID)
            obj.actions{end+1} = taskStack;
            obj.actions_ids{end+1} = taskStackID;
            if length(obj.actions) == 1
                obj.activeStack = taskStack;
                obj.transitionPlan = struct('id', taskStackID, 'type', num2cell(zeros(1, length(taskStack))));
            end
        end

        function setCurrentAction(obj, newActionIndex, currentTime)
            if newActionIndex == obj.currentAction, return; end
            
            % Get IDs
            oldIDs = obj.actions_ids{obj.currentAction};
            newIDs = obj.actions_ids{newActionIndex};
            
            % Generate Plan
            plan = generatePriorityList(oldIDs, newIDs);
            
            % Build Unified Stack
            unifiedStack = cell(1, length(plan));
            oldStack = obj.actions{obj.currentAction};
            newStack = obj.actions{newActionIndex};
            
            for i = 1:length(plan)
                taskID = plan(i).id;
                taskType = plan(i).type;
                foundTask = [];
                
                % --- ROBUST FIND LOGIC ---
                % Use string() conversion to match "VP" vs 'VP' reliably
                if taskType == -1
                    % Look in OLD stack
                    idx = find(string(oldIDs) == string(taskID), 1);
                    if ~isempty(idx), foundTask = oldStack{idx}; end
                else
                    % Look in NEW stack (Constant or Increasing)
                    idx = find(string(newIDs) == string(taskID), 1);
                    if ~isempty(idx), foundTask = newStack{idx}; end
                end
                
                % ERROR TRAP
                if isempty(foundTask)
                    error('ActionManager: Could not find task with ID "%s" in the source stack. Check your Task IDs in RobustMain.', taskID);
                end
                
                unifiedStack{i} = foundTask;
            end
            
            % Update State
            obj.activeStack = unifiedStack;
            obj.transitionPlan = plan;
            obj.currentAction = newActionIndex;
            obj.inTransition = true;
            obj.t_switch = currentTime;
            
            fprintf('[ActionManager] Auto-Switching to Action %d at t=%.2f\n', newActionIndex, currentTime);
        end

        % NOTE: Output order swapped to [qdot, v_nu] to match RobustMain
        function [qdot, v_nu, act] = computeICAT(obj, robot, currentTime) 
            
            % --- A. Handle Transition Timing ---
            dt = 0;
            if obj.inTransition
                dt = currentTime - obj.t_switch;
                if dt >= obj.t_max
                    obj.inTransition = false;
                    obj.activeStack = obj.actions{obj.currentAction};
                    obj.transitionPlan = struct('type', num2cell(zeros(1, length(obj.activeStack))));
                    fprintf('[ActionManager] Transition Complete at t=%.2f\n', currentTime);
                end
                %{
                if dt >= obj.t_max
                    obj.inTransition = false;
                    obj.activeStack = obj.actions{obj.currentAction};
                    % Retrieve the IDs for the current action so the logger can see them
                    currentIDs = obj.actions_ids{obj.currentAction};                    
                    % Create a plan struct that HAS the 'id' field
                    obj.transitionPlan = struct('id', currentIDs, ...
                                                'type', num2cell(zeros(1, length(obj.activeStack))));
                    fprintf('[ActionManager] Transition Complete at t=%.2f\n', currentTime);
                end
                %}
            end

            tasks = obj.activeStack;
            
            % --- B. Update Tasks & Apply Activation ---
            for i = 1:length(tasks)
                % The Error happened here because tasks{i} was empty
                if isempty(tasks{i})
                     error('ActionManager: Active stack contains empty tasks at index %d', i);
                end

                tasks{i}.updateReference(robot);
                tasks{i}.updateJacobian(robot);
                tasks{i}.updateActivation(robot); 
                
                alpha = 1.0;
                if obj.inTransition && i <= length(obj.transitionPlan)
                    type = obj.transitionPlan(i).type;
                    if type == 1 
                        alpha = IncreasingBellShapedFunction(0, obj.t_max, 0, 1, dt);
                    elseif type == -1
                        alpha = DecreasingBellShapedFunction(0, obj.t_max, 0, 1, dt);
                    end
                end
                
                if i==1, act = alpha; end 
                tasks{i}.A = tasks{i}.A * alpha; 
            end

            % --- C. AUTO-SEQUENCER ---
            if ~obj.inTransition && obj.currentAction < length(obj.actions)
                % Check convergence of the primary task (Index 1) of the CURRENT action
                currentDef = obj.actions{obj.currentAction};
                primaryTask = currentDef{1}; 
                
                % Use Norm of reference velocity as error metric
                err_metric = norm(primaryTask.xdotbar);
                
                if err_metric < obj.completion_threshold
                     fprintf('[ActionManager] Task Converged (Error=%.4f). Triggering next action.\n', err_metric);
                     obj.setCurrentAction(obj.currentAction + 1, currentTime);
                end
            end

            % --- D. Execute ICAT Solver ---
            ydotbar = zeros(13,1);
            Qp = eye(13);
            for i = 1:length(tasks)
                if norm(tasks{i}.A, 'fro') > 1e-6
                    [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, Qp, ydotbar, tasks{i}.xdotbar, 1e-4, 0.01, 10);
                end
            end
            [~, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 1e-4, 0.01, 10);

            qdot = ydotbar(1:7);
            v_nu = ydotbar(8:13);
        end
    end
end