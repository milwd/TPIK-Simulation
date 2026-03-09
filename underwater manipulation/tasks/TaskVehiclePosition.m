classdef TaskVehiclePosition < Task   
    properties

    end

    methods
        function updateReference(obj, robot)
            %[ang, lin] = CartError(robot.wTg , robot.wTv);
            %obj.xdotbar = 1 * [ang; lin];
            obj.xdotbar = [0; 0; 0; robot.wTgv(1:3, 4) - robot.wTv(1:3, 4);];
            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.5);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.5);
        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(3, 13);
                    zeros(3, 7) robot.wTv(1:3, 1:3) zeros(3, 3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = [zeros(3) zeros(3); 
                    zeros(3) eye(3)];
        end
    end
end