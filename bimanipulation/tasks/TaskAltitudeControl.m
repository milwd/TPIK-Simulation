classdef TaskAltitudeControl < Task   
    properties
        altitude
        h_star
        minAltitude
        delta_h
        e
        v_obj_ref_6d
    end

    methods
        function obj = TaskAltitudeControl(robot_ID, taskID, minAltitude, delta_h)
            % Set default values for optional arguments
            if nargin < 3
                minAltitude = 0.4;  % default minimum altitude
            end
            if nargin < 4
                delta_h = 1e-3;      % default increment
            end
        
            obj.ID = robot_ID;
            obj.task_name = taskID;
            obj.minAltitude = minAltitude;
            obj.delta_h = delta_h;
        
            obj.h_star = obj.minAltitude + obj.delta_h;
        end
        function updateReference(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end

            obj.altitude = robot.wTe(3,4);
            obj.e = obj.h_star - obj.altitude;
            obj.xdotbar = 1.0 * (obj.h_star - obj.altitude);
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);
            obj.v_obj_ref_6d = obj.xdotbar;
        end

        function updateJacobian(obj, robot_system)
            if(obj.ID=='L')
                robot = robot_system.left_arm;
                obj.J = [robot.wJt(6, :), zeros(1,7)];
            elseif(obj.ID=='R')
                robot = robot_system.right_arm; 
                obj.J = [zeros(1,7), robot.wJt(6, :)];
            end
        end
        
        function updateActivation(obj, robot_system)
            if(obj.ID=='L')
                robot = robot_system.left_arm;
            elseif(obj.ID=='R')
                robot = robot_system.right_arm;    
            end

            obj.altitude = robot.wTe(3,4);
            obj.A = DecreasingBellShapedFunction(obj.minAltitude, ...
                                                    obj.h_star, ...
                                                    0, ...
                                                    1, ...
                                                    obj.altitude);
        end
    end
end