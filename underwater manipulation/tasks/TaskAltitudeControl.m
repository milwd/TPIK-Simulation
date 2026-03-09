classdef TaskAltitudeControl < Task   
    properties

    end


    methods

        function updateReference(obj, robot)
            if ~isempty(robot.altitude)
                alt = robot.altitude;
            else
                alt = 0;
            end
            z_min = 2;
            if alt < z_min
                obj.xdotbar = Saturate(1.0 * (z_min - alt), 0.5);
            else
                obj.xdotbar = 0;
            end
        end

        function updateJacobian(obj, robot)
            obj.J = [zeros(1,7) robot.wTv(3,1:3) zeros(1, 3)];
        end
        
        function updateActivation(obj, robot)
            if ~isempty(robot.altitude)
                act = DecreasingBellShapedFunction(2, 3, 0, 1, robot.altitude);
            else
                act = 0;
            end
            obj.A = act;
        end
    end
end