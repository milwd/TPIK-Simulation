classdef TaskAttitudeDirect < Task
    properties
        nodule_position  
        kp
        task_name
    end

    methods
        function obj = TaskAttitudeDirect(nodule_pos)
            obj.nodule_position = nodule_pos(:);
            obj.kp = 1.0;
            obj.task_name = "AtDX";
        end

        function updateReference(obj, robot)
            p_v = robot.eta(1:3);
            d = obj.nodule_position - p_v;
            d(3) = 0;
            if norm(d) < 1e-6
                obj.xdotbar = zeros(6,1);
                return;
            end
            d_hat = d / norm(d);
            wRv = robot.wTv(1:3,1:3);
            x_v = wRv * [1; 0; 0];
            e_ang = cross(x_v, d_hat);
            v_ang = obj.kp * e_ang;
            v_lin = zeros(3,1);

            obj.xdotbar = [v_ang; v_lin];
            limit = 0.5;
            if norm(obj.xdotbar(1:3)) > limit
                obj.xdotbar(1:3) = obj.xdotbar(1:3) ...
                    * limit / norm(obj.xdotbar(1:3));
            end
        end

        function updateJacobian(obj, robot)
            wRv = robot.wTv(1:3,1:3);

            obj.J = [zeros(3,7), zeros(3,3), wRv;
                     zeros(3,13)];
        end

        function updateActivation(obj, ~)
            obj.A = diag([1 1 1 0 0 0]);
        end
    end
end