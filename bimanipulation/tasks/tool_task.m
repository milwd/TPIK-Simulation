classdef tool_task < Task   
    %Tool position control for a single arm
    properties

    end

    methods
        function obj=tool_task(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;
        end
        function updateReference(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
         [v_ang, v_lin] = CartError(robot.wTg , robot.wTt);
         robot.dist_to_goal=v_lin;
         robot.rot_to_goal=v_ang;
         obj.xdotbar = 1.0 * [v_ang; v_lin];
         % limit the requested velocities...
         obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3);
         obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3);
        end
        
        function updateJacobian(obj,robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            tool_jacobian=robot.wJt;
            
            if obj.ID=='L'
                obj.J=[tool_jacobian, zeros(6, 7)];
            elseif obj.ID=='R'
                obj.J=[zeros(6, 7), tool_jacobian];
            end
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end