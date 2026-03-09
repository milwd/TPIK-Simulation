classdef ActionManager < handle
    properties
        actions = {}      % cell array of actions (each action = stack of tasks)
        currentAction = 1 % index of currently active action
    end

    methods

        function obj = ActionManager()
        end

        function addAction(obj, taskStack)
            % Append actions in a structure
            obj.actions{end+1} = taskStack;
        end

        function [ydotbar] = computeICAT(obj,bm_system)
            tasks = obj.actions{obj.currentAction};
            %% 1. Update references, Jacobians, activations
            for i = 1:length(tasks)
                tasks{i}.updateReference(bm_system);
                tasks{i}.updateJacobian(bm_system);
                tasks{i}.updateActivation(bm_system);
            end
            
            %% 2. Perform ICAT (task-priority inverse kinematics) for the current Action
            ydotbar = zeros(14,1);
            Qp = eye(14);
            for i = 1:length(tasks)
                 %% TRANSITION FROM PREVIOUS ACTION SET TO NEXT ACTION SET
                [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
                                           Qp, ydotbar, tasks{i}.xdotbar, ...
                                           1e-4, 0.01, 10);
            end
            %% 3. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 1e-4, 0.01, 10);
        end

        function setCurrentAction(obj, actionIndex)
            % Switch to a different action
            if actionIndex >= 1 && actionIndex <= length(obj.actions)
                obj.currentAction = actionIndex;
            else
                error('Action index out of range');
            end
        end
    end
end