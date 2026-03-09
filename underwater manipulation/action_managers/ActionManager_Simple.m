classdef ActionManager < handle
    properties
        actions = {}      % cell array of actions (each action = stack of tasks)
        currentAction = 1 % index of currently active action
    end

    methods
        function addAction(obj, taskStack)
            % taskStack: cell array of tasks that define an action
            obj.actions{end+1} = taskStack;
        end

        function [v_nu, qdot] = computeICAT(obj, robot)
            % Get current action
            tasks = obj.actions{obj.currentAction};

            % 1. Update references, Jacobians, activations
            for i = 1:length(tasks)
                tasks{i}.updateReference(robot);
                tasks{i}.updateJacobian(robot);
                tasks{i}.updateActivation(robot);
            end

            % 2. Perform ICAT (task-priority inverse kinematics)
            ydotbar = zeros(13,1);
            Qp = eye(13);
            for i = 1:length(tasks)
                [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
                                           Qp, ydotbar, tasks{i}.xdotbar, ...
                                           1e-4, 0.01, 10);
            end

            % 3. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 1e-4, 0.01, 10);

            % 4. Split velocities for vehicle and arm
            qdot = ydotbar(1:7);
            v_nu = ydotbar(8:13); % projected on the vehicle frame
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