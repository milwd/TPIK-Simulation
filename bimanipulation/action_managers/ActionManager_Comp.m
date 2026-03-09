classdef ActionManager_Comp < handle
    properties
        actions = {}      
        actions_ids = {}  
        currentAction = 1 
        
        % --- Transition State Variables ---
        activeStack = {}       
        transitionPlan = []    
        inTransition = false   
        t_switch = 0           
        t_max = 0.01            
        
        mu0 = 0.5;
        completion_threshold = 1e-3; 
        
        % --- NEW: Logging Buffer for Internal Solver States ---
        % This allows the Logger to see what happened inside the math
        last_solver_data = struct('is_coop', false, ...
                                  'v_ref', zeros(6,1), ...
                                  'v_solo_L', zeros(6,1), ...
                                  'v_solo_R', zeros(6,1), ...
                                  'v_coop_L', zeros(6,1), ...
                                  'v_coop_R', zeros(6,1));
    end
    methods
        function obj = ActionManager_Comp()
            obj.actions = {};
            obj.actions_ids = {};
            obj.currentAction = 1;
        end
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
            if newActionIndex > length(obj.actions), return; end
            
            % Get IDs
            oldIDs = obj.actions_ids{obj.currentAction};
            newIDs = obj.actions_ids{newActionIndex};
            
            % Generate Transition Plan
            plan = generatePriorityList(oldIDs, newIDs);
            
            % Build Unified Stack
            unifiedStack = cell(1, length(plan));
            oldStack = obj.actions{obj.currentAction};
            newStack = obj.actions{newActionIndex};
            
            for i = 1:length(plan)
                taskID = plan(i).id;
                taskType = plan(i).type;
                foundTask = [];
                
                % ROBUST FIND LOGIC
                if taskType == -1 || taskType == 0
                    idx = find(string(oldIDs) == string(taskID), 1);
                    if ~isempty(idx), foundTask = oldStack{idx}; end
                end
                
                if isempty(foundTask) && (taskType == 1 || taskType == 0)
                    idx = find(string(newIDs) == string(taskID), 1);
                    if ~isempty(idx), foundTask = newStack{idx}; end
                end
                
                if isempty(foundTask)
                    error('ActionManager: Could not find task with ID "%s"', taskID);
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
        
        function [qdot] = computeICAT(obj, bm_system, currentTime)
            
            % --- Reset Log Data for this step ---
            obj.last_solver_data.is_coop = false;
            obj.last_solver_data.v_ref = zeros(6,1); 

            % --- A. Handle Transition Timing ---
            dt_trans = 0;
            if obj.inTransition
                dt_trans = currentTime - obj.t_switch;
                if dt_trans >= obj.t_max
                    obj.inTransition = false;
                    obj.activeStack = obj.actions{obj.currentAction};
                    currentIDs = obj.actions_ids{obj.currentAction};
                    obj.transitionPlan = struct('id', currentIDs, ...
                                                'type', num2cell(zeros(1, length(obj.activeStack))));
                    fprintf('[ActionManager] Transition Complete at t=%.2f\n', currentTime);
                end
            end
            tasks = obj.activeStack;
            
            % --- B. Update Tasks & Apply Activation ---
            for i = 1:length(tasks)
                tasks{i}.updateReference(bm_system);
                tasks{i}.updateJacobian(bm_system);
                tasks{i}.updateActivation(bm_system);
                
                % Apply Transition Gains
                alpha = 1.0;
                if obj.inTransition && i <= length(obj.transitionPlan)
                    type = obj.transitionPlan(i).type;
                    if type == 1 
                        alpha = IncreasingBellShapedFunction(0, obj.t_max, 0, 1, dt_trans);
                    elseif type == -1 
                        alpha = DecreasingBellShapedFunction(0, obj.t_max, 0, 1, dt_trans);
                    end
                end
                
                tasks{i}.A = tasks{i}.A * alpha; 
            end

            % --- CHECK IF COOPERATIVE TASK EXISTS ---
            isCoopIndex = 0;
            for i = 1:length(tasks)
                if isprop(tasks{i}, 'isCooperative') && tasks{i}.isCooperative
                    isCoopIndex = i;
                    break;
                end
            end

            if isCoopIndex > 0
                % =========================================================
                % COOPERATIVE DISTRIBUTED SOLVER
                % =========================================================
                task_mission = tasks{isCoopIndex};
                v_ref_global = task_mission.v_obj_ref_6d; 
                J_L = task_mission.J(1:6, 1:7);
                J_R = task_mission.J(7:12, 8:14);
    
                % PHASE 1: INDEPENDENT "SOLO" ESTIMATION 
                [qdot_L_solo] = obj.solve_solo_tpik(bm_system.left_arm, J_L, v_ref_global);
                xdot_L_solo = J_L * qdot_L_solo;
                H_L = J_L * pinv(J_L);
    
                [qdot_R_solo] = obj.solve_solo_tpik(bm_system.right_arm, J_R, v_ref_global);
                xdot_R_solo = J_R * qdot_R_solo;
                H_R = J_R * pinv(J_R);
    
                % PHASE 2: NEGOTIATION & CONSENSUS 
                mu_L = obj.mu0 + norm(v_ref_global - xdot_L_solo);
                mu_R = obj.mu0 + norm(v_ref_global - xdot_R_solo);
                xdot_hat = (mu_L * xdot_L_solo + mu_R * xdot_R_solo) / (mu_L + mu_R);
                
                % PHASE 3: PROJECTION 
                C = [H_L, -H_R];
                xdot_ab = [xdot_hat; xdot_hat];
                Hab = blkdiag(H_L, H_R);
                xdot_ab_feasible = Hab * xdot_ab - Hab * pinv(C) * C * xdot_ab;
                xdot_L = xdot_ab_feasible(1:6);
                xdot_R = xdot_ab_feasible(7:12);
    
                % PHASE 4: COORDINATED EXECUTION 
                qdot_L_final = obj.solve_coordinated_tpik(bm_system.left_arm, J_L, xdot_L);
                qdot_R_final = obj.solve_coordinated_tpik(bm_system.right_arm, J_R, xdot_R);
                qdot = [qdot_L_final; qdot_R_final];
    
                % --- SAVE DATA FOR LOGGING ---
                obj.last_solver_data.is_coop = true;
                obj.last_solver_data.v_ref = v_ref_global;
                obj.last_solver_data.v_solo_L = xdot_L_solo;
                obj.last_solver_data.v_solo_R = xdot_R_solo;
                obj.last_solver_data.v_coop_L = xdot_L;
                obj.last_solver_data.v_coop_R = xdot_R;

                % --- AUTO SWITCHING (COOP) ---
                if ~obj.inTransition && obj.currentAction < length(obj.actions)
                    err_metric = norm(v_ref_global); 
                    if err_metric < obj.completion_threshold 
                         fprintf('[ActionManager] Coop Task Converged (xdot=%.4f). Next Action.\n', err_metric);
                         obj.setCurrentAction(obj.currentAction + 1, currentTime);
                    end
                end

            else
                % =========================================================
                % STANDARD ICAT SOLVER
                % =========================================================
                if ~obj.inTransition && obj.currentAction < length(obj.actions)
                    primaryTask = tasks{end}; 
                    err_metric = norm(primaryTask.xdotbar);
                    if err_metric < obj.completion_threshold 
                         fprintf('[ActionManager] Task Converged (xdot=%.4f). Next Action.\n', err_metric);
                         obj.setCurrentAction(obj.currentAction + 1, currentTime);
                    end
                end
                
                ydotbar = zeros(14,1);
                Qp = eye(14);
                
                for i = 1:length(tasks)
                    if norm(tasks{i}.A, 'fro') > 1e-6
                        [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
                                                   Qp, ydotbar, tasks{i}.xdotbar, ...
                                                   1e-4, 0.01, 10);
                    end
                end
                [~, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 1e-4, 0.01, 10);
                qdot = ydotbar;

                idx_log = 0;
                for k = 1:length(tasks)
                    if isa(tasks{k}, 'TaskCooperationDistributed')
                        idx_log = k; break;
                    end
                end

                if idx_log > 0
                    t_log = tasks{idx_log};
                    J_L = t_log.J(1:6, 1:7);
                    J_R = t_log.J(7:12, 8:14);
                    
                    % 1. Desired Object Velocity (Reference)
                    % If iscoop=false, v_obj_ref_6d might be empty or raw xdotbar average
                    if norm(t_log.v_obj_ref_6d) > 1e-6
                         v_ref = t_log.v_obj_ref_6d;
                    else
                         % Fallback: Average of the left/right commands
                         v_ref = 0.5 * (t_log.xdotbar(1:6) + t_log.xdotbar(7:12));
                    end
                    
                    % 2. "Solo" Velocity
                    % In standard ICAT, "Solo" is just the Task Reference (what it asked for)
                    v_solo_L = t_log.xdotbar(1:6);
                    v_solo_R = t_log.xdotbar(7:12);
                    
                    % 3. "Final" Velocity
                    % The actual velocity achieved by the robot after all constraints
                    v_final_L = J_L * qdot(1:7);
                    v_final_R = J_R * qdot(8:14);
                    
                    % 4. Populate Data
                    obj.last_solver_data.is_coop = true; % Force true so Logger plots it!
                    obj.last_solver_data.v_ref = v_ref;
                    obj.last_solver_data.v_solo_L = v_solo_L;
                    obj.last_solver_data.v_solo_R = v_solo_R;
                    obj.last_solver_data.v_coop_L = v_final_L;
                    obj.last_solver_data.v_coop_R = v_final_R;
                end
            end
        end
        
        function qdot = solve_solo_tpik(obj, arm, J_tool, v_ref)
            n = 7; 
            ydotbar = zeros(n,1);
            Qp = eye(n);
            [Qp, ydotbar] = iCAT_task(eye(6), J_tool, Qp, ydotbar, v_ref, 1e-4, 0.01, 10);
            qdot = ydotbar;
        end
        
        function qdot = solve_coordinated_tpik(obj, arm, J_tool, v_coop)
            n = 7;
            ydotbar = zeros(n,1);
            Qp = eye(n);
            [Qp, ydotbar] = iCAT_task(eye(6), J_tool, Qp, ydotbar, v_coop, 1e-4, 0.01, 10);
            qdot = ydotbar;
        end
    end
end