classdef ActionManager_TDistributed < handle
    properties
        actions = {}      
        actions_ids = {}  
        currentAction = 1 
        
        % Parameters for the weighting logic
        mu0 = 0.5; % Base weight
        completion_threshold = 0.0001;
    end

    methods
        function obj = ActionManager_TDistributed()
            obj.actions = {};
            obj.actions_ids = {};
            obj.currentAction = 1;
        end

        function addAction(obj, taskStack, taskStackID)
            obj.actions{end+1} = taskStack;
            obj.actions_ids{end+1} = taskStackID;
        end

        function setCurrentAction(obj, newActionIndex)
            if newActionIndex <= length(obj.actions)
                obj.currentAction = newActionIndex;
            end
        end
        
        function [qdot] = computeICAT(obj, bm_sim, t)
            tasks = obj.actions{obj.currentAction};
            for i = 1:length(tasks)
                tasks{i}.updateReference(bm_sim);
                tasks{i}.updateJacobian(bm_sim);
            end

            % Identify the Cooperation Task (Main Mission) and Safety Tasks
            task_mission = [];
            for i = 1:length(tasks)
                if isprop(tasks{i}, 'v_obj_ref_6d')
                    task_mission = tasks{i};
                    isCooperative = true;
                    break;
                end
            end
            
            if isempty(task_mission)
                error('Cooperative Task not found in current action!');
            end
            %{
            if isCooperative
                qdot = obj.solveDistributed(tasks, bm_sim);
            else
                qdot = obj.solveStandard(tasks, bm_sim);
            end
            %}
            
            v_ref_global = task_mission.v_obj_ref_6d; 
            J_L = task_mission.J(1:6, 1:7);
            J_R = task_mission.J(7:12, 8:14);

            % =========================================================
            % PHASE 1: INDEPENDENT "SOLO" ESTIMATION 
            % =========================================================
            
            % solve for Left arm 
            [qdot_L_solo] = obj.solve_solo_tpik(bm_sim.left_arm, J_L, v_ref_global);
            xdot_L_solo = J_L * qdot_L_solo;
            H_L = J_L * pinv(J_L);

            % solve for Right arm
            [qdot_R_solo] = obj.solve_solo_tpik(bm_sim.right_arm, J_R, v_ref_global);
            xdot_R_solo = J_R * qdot_R_solo;
            H_R = J_R * pinv(J_R);

            % =========================================================
            % PHASE 2: NEGOTIATION & CONSENSUS 
            % =========================================================
            
            % Compute weights
            mu_L = obj.mu0 + norm(v_ref_global - xdot_L_solo);
            mu_R = obj.mu0 + norm(v_ref_global - xdot_R_solo);
            % Weighted Average Velocity
            xdot_hat = (mu_L * xdot_L_solo + mu_R * xdot_R_solo) / (mu_L + mu_R);
            
            % =========================================================
            % PHASE 3: PROJECTION 
            % =========================================================
            
            % Constraint Matrix C 
            C = [H_L, -H_R];
            
            xdot_ab = [xdot_hat; xdot_hat];
            Hab = blkdiag(H_L, H_R);

            xdot_ab_feasible = Hab * xdot_ab - Hab * pinv(C) * C * xdot_ab;
            
            xdot_L = xdot_ab_feasible(1:6);
            xdot_R = xdot_ab_feasible(7:12);

            % =========================================================
            % PHASE 4: COORDINATED EXECUTION 
            % =========================================================
            
            % Now both agents must track 'xdot_coop' with HIGHEST priority.
            % Hierarchy: 
            % 1. Strict Cooperative Velocity Tracking (xdot_coop)
            % 2. Joint Limits / Safety (Projected into Nullspace of 1)
            % 3. Posture / Secondary tasks
            
            % solve final for Left 
            qdot_L_final = obj.solve_coordinated_tpik(bm_sim.left_arm, J_L, xdot_L);
            
            % solve final for Right 
            qdot_R_final = obj.solve_coordinated_tpik(bm_sim.right_arm, J_R, xdot_R);
            
            qdot = [qdot_L_final; qdot_R_final];

            err_metric = norm(task_mission.e);  % used mostly for debug
            if err_metric < obj.completion_threshold 
                 fprintf('[ActionManager] Task Converged (Error=%.4f). Triggering next action.\n', err_metric);
                 obj.setCurrentAction(obj.currentAction + 1, currentTime);
            end
        end
        
        function qdot = solve_solo_tpik(obj, arm, J_tool, v_ref)
            n = 7; 
            ydotbar = zeros(n,1);
            Qp = eye(n);
            
            % 1. Joint Limits (Safety)
            
            % 2. Object Reference
            [Qp, ydotbar] = iCAT_task(eye(6), J_tool, Qp, ydotbar, v_ref, 1e-4, 0.01, 10);
            
            % 3. Damping
            %[~, ydotbar] = iCAT_task(eye(n), eye(n), Qp, ydotbar, zeros(n,1), 1e-4, 0.01, 10);
            
            qdot = ydotbar;
        end
        
        function qdot = solve_coordinated_tpik(obj, arm, J_tool, v_coop)
            % The final control command.
            % Hierarchy: 1. COOP VELOCITY (Strict), 2. Safety/JL, 3. Damping
            
            n = 7;
            ydotbar = zeros(n,1);
            Qp = eye(n);
            
            % 1. Cooperative Velocity Constraint (Highest Priority)
            [Qp, ydotbar] = iCAT_task(eye(6), J_tool, Qp, ydotbar, v_coop, 1e-4, 0.01, 10);
            
            % 2. Joint Limits / Safety
            % (Insert JL task logic here, projecting onto Qp)
            
            % 3. Damping / Posture
            %[~, ydotbar] = iCAT_task(eye(n), eye(n), Qp, ydotbar, zeros(n,1), 1e-4, 0.01, 10);
            
            qdot = ydotbar;
        end
    end
end