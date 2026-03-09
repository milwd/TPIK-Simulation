classdef TaskCooperation < Task
    properties
        % Fixed relative transformations from Tools to Object
        l_T_o 
        r_T_o 
        
        % Object Goal Pose
        wTog 
        e                   % used for debug only
        v_obj_ref_6d
    end

    methods
        function obj = TaskCooperation(wTog, arm1, arm2, l_T_o, r_T_o)
            % Constructor: 
            % wTog:  Goal Pose of the Object (4x4)
            % arm1/2: Robot objects (for Jacobian access)
            % l_T_o: Constant transform from Left Tool to Object (computed from grasp target)
            % r_T_o: Constant transform from Right Tool to Object
            
            obj.l_T_o = l_T_o;
            obj.r_T_o = r_T_o;
            obj.wTog = wTog;
            
            % Initialize standard task properties
            obj.task_name = "Coop";
            obj.A = eye(12);
            obj.xdotbar = zeros(12,1);
            obj.e = zeros(6);
        end
        
        function updateReference(obj, bm_system)
            % 1. left arm feedback 
            wTo_curr_L = bm_system.left_arm.wTt * obj.l_T_o;
            [v_ang_L, v_lin_L] = CartError(obj.wTog, wTo_curr_L);
            
            % 2. right arm feedback 
            wTo_curr_R = bm_system.right_arm.wTt * obj.r_T_o;
            [v_ang_R, v_lin_R] = CartError(obj.wTog, wTo_curr_R);
            
            % 3. deine independent velocities
            v_ref_L = [v_ang_L; v_lin_L];
            v_ref_R = [v_ang_R; v_lin_R];
            
            % Saturate
            v_ref_L = Saturate(v_ref_L, 0.3);
            v_ref_R = Saturate(v_ref_R, 0.3);
                        
            % 4. global ref (helps for unity)
            obj.v_obj_ref_6d = 0.5 * (v_ref_L + v_ref_R);
            
            obj.xdotbar = [v_ref_L; v_ref_R];
            if norm(v_ref_L) > norm(v_ref_R)  % used for debug only
                obj.e = v_ref_L;
            else
                obj.e = v_ref_R;
            end
        end

        function updateJacobian(obj, bm_system)
            % left Arm 
            % tool to object center (in world coordinates)
            r_L_world = bm_system.left_arm.wTt(1:3,1:3) * obj.l_T_o(1:3, 4);
            W_L = [eye(3),         zeros(3);
                   -skew(r_L_world), eye(3)]; % additional rigid_body jacobian from the object
            J_obj_L = W_L * bm_system.left_arm.wJt;  

            % right arm 
            r_R_world = bm_system.right_arm.wTt(1:3,1:3) * obj.r_T_o(1:3, 4);
            
            W_R = [eye(3),         zeros(3);
                   -skew(r_R_world), eye(3)];
               
            J_obj_R = W_R * bm_system.right_arm.wJt;
            
            % stack jacobians (should be 12x14)
            obj.J = [J_obj_L,     zeros(6,7);
                     zeros(6,7),  J_obj_R];
        end

        function updateActivation(obj, ~)
            obj.A = eye(12); 
        end
    end
end