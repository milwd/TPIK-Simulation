classdef TaskVehiclePositionA < Task   
    properties
        target
    end

    methods
        function obj = TaskVehiclePositionA(target)
            if nargin > 0
                obj.target = target;
            else
                obj.target = zeros(6,1);
            end
        end

        function updateReference(obj, robot)
            wT_des = [rotation(obj.target(4), obj.target(5), obj.target(6)), obj.target(1:3); 0 0 0 1];

            [v_ang, v_lin] = CartError(wT_des, robot.wTv);
            obj.xdotbar = [v_ang; v_lin];

            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.5);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.5);
        end
        
        function updateJacobian(obj, robot)
            J_ang = [zeros(3, 7), zeros(3, 3), robot.wTv(1:3, 1:3)];
            J_lin = [zeros(3, 7), robot.wTv(1:3, 1:3), zeros(3, 3)];
            
            obj.J = [J_ang; 
                     J_lin];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end