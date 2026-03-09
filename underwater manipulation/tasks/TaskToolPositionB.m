classdef TaskToolPositionB < Task   
    properties
        target_pose
        task_name
    end

    methods
        function obj = TaskToolPositionB(target_matrix)
            if nargin > 0
                obj.target_pose = target_matrix;
            else
                obj.target_pose = eye(4);
            end
            % Standard initialization
            obj.task_name = "TP";
        end

        function updateReference(obj, robot)
            % 1. Compute Standard Global Error (Desired - Current)
            [v_ang, v_lin] = CartError(robot.vTw * obj.target_pose, robot.vTt);
            v_global_error = [v_ang; v_lin];
            
            % 2. COMPUTE VEHICLE DISTURBANCE (The "Theory" Part)
            % We need to know how much the tool is moving JUST because the vehicle is moving.
            % J_veh (6x6) is the part of the full Jacobian corresponding to the vehicle.
            % Note: Check your indices. Usually Arm=1:7, Vehicle=8:13
            
            % Get full Jacobian first (we need the vehicle part)
            J_full = obj.computeFullJacobian(robot);
            J_veh  = J_full(:, 8:13); 
            
            % Calculate velocity induced by the vehicle's ACTUAL measured velocity
            % robot.v_nu is the measured vehicle velocity [nu_1 ... nu_6]'
            v_disturbance = J_veh * robot.v_nu; 
            
            % 3. COMPENSATION
            % We want: J_arm * qdot_arm = v_global_error - v_disturbance
            % So we set the reference to be the compensated difference.
            obj.xdotbar = 0.5 * (v_global_error - v_disturbance);
            
            % Saturate limits
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.2);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.2);
        end

        function updateJacobian(obj, robot)
            % 1. Compute the Full Jacobian
            J_full = obj.computeFullJacobian(robot);
            
            % 2. MASK THE VEHICLE (Decoupling)
            % We zero out the vehicle columns (8:13).
            % This tells the solver: "You cannot use the vehicle to solve THIS task. 
            % You must find a solution using only the Arm (1:7)."
            % Since we already subtracted the vehicle's motion in updateReference,
            % the Arm will now move to EXACTLY cancel out the vehicle's error + reach the goal.
            
            J_arm_only = J_full;
            J_arm_only(:, 8:13) = 0; 
            
            obj.J = J_arm_only;
        end
        
        function updateActivation(obj, ~)
            obj.A = eye(6);
        end

        % --- Helper to keep code clean ---
        function J = computeFullJacobian(~, robot)
            % Replicating your standard Jacobian logic here
            % Arm Jacobian (Base to Tool)
            bJe = RobustJacobian(robot.q); 
            
            % Transform Arm Jacobian to World Frame (via Vehicle)
            % We need the transformation from Vehicle Base to World
            % S_v = [R_v 0; 0 R_v] (roughly speaking, depending on your conventions)
            
            % Jacobian of Tool with respect to Arm Joints (projected to world)
            % Ste transforms twists from End-Effector to Tool
            Ste = [eye(3) zeros(3);  -skew(robot.vTe(1:3,1:3)*robot.eTt(1:3,4)) eye(3)];
            
            % Map Arm Jacobian to Vehicle Frame
            Jt_arm_local = Ste * [robot.vTb(1:3,1:3) zeros(3,3); zeros(3,3) robot.vTb(1:3,1:3)] * bJe;
            
            % Jacobian of Tool with respect to Vehicle Motion
            % This is the Adjoint transform from Vehicle frame to Tool frame
            % Jt_veh = Ad(vTt)^-1 ... simplified:
            
            % Lever arm from Vehicle Center to Tool
            p_vt = robot.vTt(1:3, 4); 
            
            % Interaction Matrix (Vehicle velocity -> Tool velocity)
            % Assuming robot.v_nu is expressed in the VEHICLE frame
            Jt_veh_local = [eye(3)      zeros(3); 
                            -skew(p_vt) eye(3)];
                        
            % Now rotate everything into the World Frame for the solver
            % (Assuming the solver expects World Frame velocities)
            R_v = robot.wTv(1:3,1:3);
            W_R = [R_v zeros(3); zeros(3) R_v];
            
            J_arm_world = W_R * Jt_arm_local;
            J_veh_world = W_R * Jt_veh_local;
            
            J = [J_arm_world, J_veh_world];
        end
    end
end