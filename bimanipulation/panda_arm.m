classdef panda_arm < handle
    % Franka Emika Panda Kinematic Model

    properties
        %% Rigid Body Tree
        robot_model
        %% --- State variables ---
        q
        qdot
        %% --- Geometry ---
        wTb
        %% --- Limits ---
        jlmin
        jlmax
        %% ---Transformations ---
        bTe
        wTe
        eTt
        wTt
        %% ---Goals---
        wTg
        wTog
        wTo
        robot_type
        tTo
        wJt
        wJo
        %% Sensor variables
        alt
        dist_to_goal
        rot_to_goal
    end

    methods
        function obj = panda_arm(model, wTb, q)
            % Constructor
            obj.robot_model = model;
            obj.wTb = wTb;

            if nargin > 2
                obj.q = q; % initialize joint positions if given
            else 
                obj.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
            end
            
            %Initialize Default State
            obj.qdot = [0 0 0 0 0 0 0]';
            
            %Get current transformation from world frame to end effector
            %Frame
            obj.bTe = getTransform(obj.robot_model.franka,[obj.q',0,0],'panda_link7');
            obj.wTe = obj.wTb * obj.bTe;

            %Default Limits
            obj.jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
            obj.jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

            % FIXED END EFFECTOR
            theta = -44.9949;       % FIXED ANGLE BETWEEN EE AND TOOL 
            tool_length = 0.2104;   % FIXED DISTANCE BETWEEN EE AND TOOL

            % TO DO: Define trasnformation matrix from ee to tool, and
            % transformation from world frame to tool
            obj.eTt = [rotation(0, 0, deg2rad(theta)), [0, 0, tool_length]'; 
                       0, 0, 0, 1];
            obj.wTt = obj.wTe * obj.eTt;
          
        end

        function setGoal(obj,obj_position,obj_orientation,arm_dist_offset,arm_rot_offset)
            % Set goal positions and orientations for arm 
            obj.wTo=[[obj_orientation obj_position]; 0 0 0 1];
            obj.wTg=[[arm_rot_offset arm_dist_offset]; 0 0 0 1];
        end
        
        function set_obj_goal(obj,wTog)
            % Set goal positions and orientations for the object
            obj.wTog = wTog;
        end


        function update_transform(obj)
            % Compute forward kinematics of the robot

            obj.bTe = getTransform(obj.robot_model.franka,[obj.q',0,0],'panda_link7');
            obj.wTe = obj.wTb*obj.bTe;
            obj.wTt = obj.wTe * obj.eTt;
            
        end

        function update_jacobian(obj)
            % Compute Differential kinematics from the base frame to the
            % Tool Frame
            bJe = geometricJacobian(obj.robot_model.franka, [obj.q',0,0], 'panda_link7');%DO NOT EDIT
            Ste = [eye(3) zeros(3); -skew(obj.wTe(1:3,1:3)*obj.eTt(1:3,4)) eye(3)];
            obj.wJt = Ste * [obj.wTb(1:3,1:3) zeros(3,3); zeros(3,3) obj.wTb(1:3,1:3)] * bJe(:, 1:7);
        end
    end
end