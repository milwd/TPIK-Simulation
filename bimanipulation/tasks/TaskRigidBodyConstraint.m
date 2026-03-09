classdef TaskRigidBodyConstraint < Task
    properties
        % Fixed relative transforms from Tool to Object (constant during grasp)
        l_T_o
        r_T_o

        % Optional: allow enabling/disabling the constraint
        isActive = true;
        iscoop

        % Debug
        e
    end

    methods
        function obj = TaskRigidBodyConstraint(l_T_o, r_T_o, isActive, iscoop)
            % Rigid-body grasp constraint task
            % Enforces:  J_obj_L*qdot_L  -  J_obj_R*qdot_R = 0

            obj.task_name = "RigidBodyConstraint";
            obj.iscoop = true;

            obj.l_T_o = l_T_o;
            obj.r_T_o = r_T_o;

            if nargin >= 3
                obj.isActive = isActive;
            end

            obj.A = eye(6);
            obj.xdotbar = zeros(6,1);
            obj.e = zeros(6,1);
        end

        function updateReference(obj, ~)
            % Equality constraint -> reference always zero
            obj.xdotbar = zeros(6,1);
            obj.e = zeros(6,1);
        end

        function updateJacobian(obj, bm_system)
            % ------------------------------------------------------------
            % Build object-point Jacobians for each arm:
            %   [w; v_obj] = W(r_tool->obj) * [w; v_tool]
            %
            % Using same convention as your code:
            %   twist = [omega; v]
            % and same shift matrix you used:
            %   v_obj = v_tool + omega x (p_obj - p_tool)
            %         = v_tool - skew(r)*omega   (since omega x r = -r x omega)
            % so:
            %   [omega; v_obj] = [ I   0
            %                    -S(r) I ] [omega; v_tool]
            % ------------------------------------------------------------

            % ---------- LEFT ARM ----------
            wTt_L = bm_system.left_arm.wTt;
            wTo_L = wTt_L * obj.l_T_o;

            p_tool_L = wTt_L(1:3,4);
            p_obj_L  = wTo_L(1:3,4);

            r_L_world = p_obj_L - p_tool_L;  % world vector tool -> object point

            W_L = [eye(3),           zeros(3);
                   -skew(r_L_world), eye(3)];

            % wJt is assumed 6x7 mapping qdot_L -> [omega; v_tool]
            J_obj_L = W_L * bm_system.left_arm.wJt; % 6x7


            % ---------- RIGHT ARM ----------
            wTt_R = bm_system.right_arm.wTt;
            wTo_R = wTt_R * obj.r_T_o;

            p_tool_R = wTt_R(1:3,4);
            p_obj_R  = wTo_R(1:3,4);

            r_R_world = p_obj_R - p_tool_R;

            W_R = [eye(3),           zeros(3);
                   -skew(r_R_world), eye(3)];

            J_obj_R = W_R * bm_system.right_arm.wJt; % 6x7


            % ---------- RIGID BODY CONSTRAINT ----------
            % Jkc * [qdot_L; qdot_R] = 0
            obj.J = [J_obj_L, -J_obj_R]; % 6x14
        end

        function updateActivation(obj, ~)
            if obj.isActive
                obj.A = eye(6);
            else
                obj.A = zeros(6);
            end
        end
    end
end
