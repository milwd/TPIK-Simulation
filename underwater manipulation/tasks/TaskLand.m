classdef TaskLand < Task   
    properties

    end

    methods
        
        function updateReference(obj, robot)
            if ~isempty(robot.altitude)
                obj.xdotbar = 0.5 * robot.altitude;
            else
                obj.xdotbar = 0;
            end
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.5);
        end

        function updateJacobian(obj, robot)
            obj.J = [zeros(1, 7) -robot.wTv(3, 1:3) zeros(1, 3)];
        end
        
        function updateActivation(obj, robot)
            obj.A = 1;
        end
    end
end