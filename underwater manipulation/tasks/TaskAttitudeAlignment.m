classdef TaskAttitudeAlignment < Task   
    properties

    end


    methods
        function updateReference(obj, robot)
            lin = [0 0 0]';
            ang = [0, 0, robot.eta(6)]' - robot.eta(4:6);
            obj.xdotbar = 1 * [ang; lin];
            
            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.5);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.5);
        end
        function updateJacobian(obj, robot)
            s_phi = sin(robot.eta(4)); c_phi = cos(robot.eta(4));
            t_theta = tan(robot.eta(5)); c_theta = cos(robot.eta(5));
            
            if abs(c_theta) < 0.01
                T = eye(3); 
            else
                T = [1,  s_phi * t_theta,  c_phi * t_theta;
                     0,  c_phi,           -s_phi;
                     0,  s_phi / c_theta,  c_phi / c_theta];
            end
            obj.J = [zeros(3, 10), T;
                    zeros(3, 13)];
        end
        
        function updateActivation(obj, robot)
            misalignment = norm(robot.wTv(1:3, 3));

            act_pos = IncreasingBellShapedFunction(0, 0.2, 0, 1, misalignment);
            act_neg = DecreasingBellShapedFunction(-0.2, 0, 0, 1, misalignment);
            act = act_pos + act_neg;
            
            obj.A = diag([act, act, 0, 0, 0, 0]);
        end
    end
end