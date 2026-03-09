classdef TaskCooperationDistributed < Task
    properties
        % Fixed relative transformations from Tools to Object
        l_T_o
        r_T_o

        % Object Goal Pose
        wTog
        e

        % Expose the raw 6D reference velocity
        v_obj_ref_6d
        isCooperative

        % Algorithm params
        mu0  = 0.05;     % > 0
        vmax = 0.30;     % saturation limit
        eps_fallback = 1e-4;
    end

    methods
        function obj = TaskCooperationDistributed(wTog, arm1, arm2, l_T_o, r_T_o, iscoop)
            obj.l_T_o = l_T_o;
            obj.r_T_o = r_T_o;
            obj.wTog  = wTog;

            obj.task_name = "Coop";
            obj.A = eye(12);
            obj.xdotbar = zeros(12,1);
            obj.v_obj_ref_6d = zeros(6,1);
            obj.isCooperative = iscoop;
        end

        function updateReference(obj, bm_system)
            % ============================================================
            % Algorithm 1 CooperativeTPIK(Am)
            % ============================================================

            % --- Line 1: xdotbar_t <- GetDesiredObjectVelocity() ---
            wTo_curr_L = bm_system.left_arm.wTt  * obj.l_T_o;
            [v_ang_L, v_lin_L] = CartError(obj.wTog, wTo_curr_L);
            v_ref_L = [v_ang_L; v_lin_L];
            v_ref_L = Saturate(v_ref_L, obj.vmax);

            wTo_curr_R = bm_system.right_arm.wTt * obj.r_T_o;
            [v_ang_R, v_lin_R] = CartError(obj.wTog, wTo_curr_R);
            v_ref_R = [v_ang_R; v_lin_R];
            v_ref_R = Saturate(v_ref_R, obj.vmax);

            xdot_bar = 0.5 * (v_ref_L + v_ref_R);    % desired object velocity
            xdot_bar = Saturate(xdot_bar, obj.vmax);

            % Keep an error metric similar to your old code (manager-friendly)
            if norm(v_ref_L) > norm(v_ref_R)
                obj.e = v_ref_L;
            else
                obj.e = v_ref_R;
            end

            % If cooperation is OFF -> behave like your old code
            if ~obj.isCooperative
                obj.v_obj_ref_6d = xdot_bar;
                obj.xdotbar = [v_ref_L; v_ref_R];
                return;
            end

            % --- Lines 3-4: xdot_t,i and H_i = J_i * J_i# ---
            [J_obj_L, H_L] = obj.getObjectJacobianAndH(bm_system, "L");
            [J_obj_R, H_R] = obj.getObjectJacobianAndH(bm_system, "R");

            xdot_i = H_L * xdot_bar;   % = J J# xdot_bar
            xdot_j = H_R * xdot_bar;

            % --- Lines 7-9: weighted compromise xdot_hat ---
            mu_i = obj.mu0 + norm(xdot_bar - xdot_i);
            mu_j = obj.mu0 + norm(xdot_bar - xdot_j);

            xdot_hat = (mu_i * xdot_i + mu_j * xdot_j) / (mu_i + mu_j);

            % --- Lines 10-11: compatibility projection ---
            C = H_L - H_R;
            P = eye(6) - pinv(C) * C;

            H_ij = 0.5 * (H_L + H_R);

            xdot_tilde = H_ij * (P * xdot_hat);
            xdot_tilde = Saturate(xdot_tilde, obj.vmax);

            % ============================================================
            % IMPORTANT PRACTICAL FIX:
            % If projection kills the motion (xdot_tilde ~ 0) while still far
            % from goal (xdot_bar not small), do NOT go to zero.
            % ============================================================
            if norm(xdot_tilde) < obj.eps_fallback && norm(xdot_bar) > obj.eps_fallback
                % fallback to a safe cooperative motion (still consistent)
                xdot_tilde = H_ij * xdot_hat;
                xdot_tilde = Saturate(xdot_tilde, obj.vmax);
            end

            % --- Output cooperative reference (same velocity for both arms) ---
            obj.v_obj_ref_6d = xdot_tilde;
            obj.xdotbar = [xdot_tilde; xdot_tilde];
        end

        function updateJacobian(obj, bm_system)
            % Stacked Jacobian for tracking [x_L; x_R]
            J_obj_L = obj.getObjectJacobianAndH(bm_system, "L");
            J_obj_R = obj.getObjectJacobianAndH(bm_system, "R");

            obj.J = [J_obj_L, zeros(6,7);
                     zeros(6,7), J_obj_R];
        end

        function updateActivation(obj, ~)
            obj.A = eye(12);
        end
    end

    methods (Access = private)
        function [J_obj, H] = getObjectJacobianAndH(obj, bm_system, whichArm)
            if whichArm == "L"
                wTt = bm_system.left_arm.wTt;
                wTo = wTt * obj.l_T_o;
                wJt = bm_system.left_arm.wJt;
            else
                wTt = bm_system.right_arm.wTt;
                wTo = wTt * obj.r_T_o;
                wJt = bm_system.right_arm.wJt;
            end

            p_tool = wTt(1:3,4);
            p_obj  = wTo(1:3,4);
            r_world = p_obj - p_tool;

            % twist ordering [omega; v]
            W = [eye(3),           zeros(3);
                 -skew(r_world),   eye(3)];

            J_obj = W * wJt;          % 6x7
            H = J_obj * pinv(J_obj);  % 6x6
        end
    end
end
