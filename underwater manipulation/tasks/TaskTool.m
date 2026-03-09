classdef TaskTool < Task   
    properties

    end


    methods
        function updateReference(obj, robot)
            [v_ang, v_lin] = CartError(robot.vTg , robot.vTt);
            obj.xdotbar = 0.2 * [v_ang; v_lin];
            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.2);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.2);
        end
        function updateJacobian(obj, robot)
            bJe = RobustJacobian(robot.q);
            Ste = [eye(3) zeros(3);  -skew(robot.vTe(1:3,1:3)*robot.eTt(1:3,4)) eye(3)];
            Jt_a  = Ste * [robot.vTb(1:3,1:3) zeros(3,3); zeros(3,3) robot.vTb(1:3,1:3)] * bJe;
            Jt_v = [zeros(3) eye(3); eye(3) -skew(robot.vTt(1:3,4))];
            obj.J = [Jt_a Jt_v];
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end