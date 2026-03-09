classdef TaskJointLimits < Task
    properties
        jl_min   % Vector of min limits
        jl_max   % Vector of max limits
        armId    % 'L' or 'R' (Renamed from ID to avoid conflict with superclass)
        buffer   % Safety buffer in radians (e.g. 0.1 rad)
        
        % For ActionManager compatibility
        v_obj_ref_6d 
        e = inf;
    end
    
    methods
        function obj = TaskJointLimits(armCode, taskID, limits_max, limits_min)
            % Constructor: Pass BOTH min and max limits
            % If limits_min is missing, it tries to assume symmetry (dangerous for Panda)
            
            obj.armId = armCode;
            obj.task_name = taskID;
            
            if nargin < 4
                 % Fallback for Panda if only one vector is provided
                 % (Assuming limits_max is actually the arm object or similar, but better to be explicit)
                 warning('TaskJointLimits: Only symmetric limits provided. This will fail for Panda Joint 4.');
                 obj.jl_max = limits_max;
                 obj.jl_min = -limits_max;
            else
                 obj.jl_max = limits_max;
                 obj.jl_min = limits_min;
            end
            
            obj.buffer = 0.15; % Activate task 0.15 rad (approx 8.5 deg) before limit
            
            % Initialize
            obj.A = zeros(7);
            obj.J = zeros(7, 14); 
            obj.xdotbar = zeros(7,1);
            obj.v_obj_ref_6d = zeros(6,1);
        end
        
        function updateReference(obj, robot_system)
            % 1. Get Robot State
            if obj.armId == 'L'
                q = robot_system.left_arm.q;
            else
                q = robot_system.right_arm.q;
            end
            
            obj.xdotbar = zeros(7,1);
            
            % 2. Check Upper Limits
            % If q is closer to max than buffer, push negative
            dist_upper = obj.jl_max - q;
            
            penetration_upper = obj.buffer - dist_upper; 
            
            % 3. Check Lower Limits
            % If q is closer to min than buffer, push positive
            dist_lower = q - obj.jl_min;
            penetration_lower = obj.buffer - dist_lower;
            
            % 4. Compute Velocity
            gain = 1.0; 
            
            for i = 1:7
                if penetration_upper(i) > 0
                    % We are too close to MAX -> Push Negative
                    obj.xdotbar(i) = -gain * penetration_upper(i);
                elseif penetration_lower(i) > 0
                    % We are too close to MIN -> Push Positive
                    obj.xdotbar(i) =  gain * penetration_lower(i);
                else
                    obj.xdotbar(i) = 0;
                end
            end
            
            % Saturate for safety (max 0.5 rad/s correction)
            obj.xdotbar = Saturate(obj.xdotbar, 0.5);
            %obj.e = penetration_upper;
        end
        
        function updateJacobian(obj, ~)
            % Select the correct columns for Left (1:7) or Right (8:14)
            if obj.armId == 'L'
                obj.J = [eye(7), zeros(7,7)];
            else
                obj.J = [zeros(7,7), eye(7)];
            end
        end
        
        function updateActivation(obj, robot_system)
            if obj.armId == 'L'
                q = robot_system.left_arm.q;
            else
                q = robot_system.right_arm.q;
            end
            
            a = zeros(7,1);
            
            for i = 1:7
                % Calculate normalized distance into the buffer zone
                % 0 = at buffer edge (safe)
                % 1 = at hard limit (unsafe)
                
                % Check Upper
                if q(i) > (obj.jl_max(i) - obj.buffer)
                    a(i) = IncreasingBellShapedFunction(0, obj.buffer, 0, 1, q(i) - (obj.jl_max(i) - obj.buffer));
                
                % Check Lower
                elseif q(i) < (obj.jl_min(i) + obj.buffer)
                     % Distance from buffer edge
                     dist_into_buffer = (obj.jl_min(i) + obj.buffer) - q(i);
                     a(i) = IncreasingBellShapedFunction(0, obj.buffer, 0, 1, dist_into_buffer);
                end
            end
            
            obj.A = diag(a);
        end
    end
end