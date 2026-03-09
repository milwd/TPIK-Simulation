classdef TaskToolPosition < handle   
    % TaskToolGoal: Tool position control for a specific target
    % Allows defining different targets for different actions.
    
    properties
        ID          % 'L' or 'R'
        task_name
        target_T    % Target Transformation (4x4)
        J           % Jacobian
        xdotbar     % Reference velocity
        A           % Activation matrix
        e
        v_obj_ref_6d
    end

    methods
        function obj = TaskToolPosition(robot_ID, taskID, target_T)
            obj.ID = robot_ID;
            obj.task_name = taskID;
            obj.target_T = target_T; % Store the specific target for this task instance
            obj.A = eye(6);
            obj.xdotbar = zeros(6,1);
        end

        function updateReference(obj, robot_system)
            % Select correct arm
            if(obj.ID=='L')
                robot = robot_system.left_arm;
            elseif(obj.ID=='R')
                robot = robot_system.right_arm;    
            end
            
            % Compute Error between Specific Target (target_T) and Current Tool (wTt)
            [v_ang, v_lin] = CartError(obj.target_T, robot.wTt);
            
            % Save to robot for logging/debugging if needed
            robot.dist_to_goal = v_lin;
            robot.rot_to_goal = v_ang;

            obj.e = [v_ang; v_lin];
            
            % Proportional Control
            obj.xdotbar = 1.0 * [v_ang; v_lin];
            
            % Saturation
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3);
            obj.v_obj_ref_6d = obj.xdotbar;
        end
        
        function updateJacobian(obj, robot_system)
            if(obj.ID=='L')
                robot = robot_system.left_arm;
            elseif(obj.ID=='R')
                robot = robot_system.right_arm;    
            end
            
            tool_jacobian = robot.wJt;
            
            % Construct 14-DOF Jacobian
            if obj.ID=='L'
                obj.J = [tool_jacobian, zeros(6, 7)];
            elseif obj.ID=='R'
                obj.J = [zeros(6, 7), tool_jacobian];
            end
        end

        function updateActivation(obj, ~)
            obj.A = eye(6);
        end
    end
end