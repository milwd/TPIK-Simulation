classdef UvmsModel < handle
    %UVMSMODEL Vehicle–Manipulator System Kinematic Model
    %
    % This class represents the kinematic state and geometry of an
    % Underwater Vehicle–Manipulator System (UVMS).
    % It provides:
    %   - storage of joint and vehicle states
    %   - forward kinematics computation
    %   - Jacobian computation
    %   - goal configuration definition
    %
    % The class does not implement any control or simulation logic.
    %
    % Example:
    %   model = UvmsModel('Robust');
    %   model.updateKinematics();

    properties
        %% --- State variables ---
        q           % [7x1] manipulator joint positions
        q_dot       % [7x1] manipulator joint velocities
        eta         % [6x1] vehicle pose [x y z r p y]
        v_nu        % [6x1] vehicle velocites (linear and angular) proj. on the vehicle frame
        altitude

        %% --- Geometry ---
        vTb         % fixed transform from vehicle to manipulator base
        eTt         % transform from end-effector to tool point

        %% --- Limits ---
        jlmin       % joint lower limits
        jlmax       % joint upper limits

        %% --- Transformations ---
        wTv         % vehicle wrt world
        vTw         % world wrt vehicle
        vTe         % end-effector wrt vehicle
        vTt         % tool wrt vehicle
        wTt         % tool wrt world
        wTg         % goal (tool) wrt wolrd
        wTgv        % goal (vehicle) wrt world
        bTe         % end-effector wrt arm base
        vTg         % goal (tool) wrt vehicle

        %% --- Goals ---
        wRg         % desired orientation of tool in world
        goalPosition% desired position of tool in world
        wRgv        % desired vehicle orientation in world
        vehicleGoalPosition % desired vehicle position in world
    end

    methods
        function obj = UvmsModel(robotType)
            % Constructor
            %   robotType - string: 'DexROV' or 'Robust'
            %
            % Initializes geometry, default state, and goal transforms.

            if nargin < 1
                robotType = 'Robust';
            end

            % Define the geometry of the manipulator mounting
            switch robotType
                case 'DexROV'
                    obj.vTb = [rotation(pi, 0, pi) [0.167; 0; -0.43]; 0 0 0 1];
                otherwise % 'Robust'
                    obj.vTb = [rotation(0, 0, pi) [0.85; 0; -0.42]; 0 0 0 1];
            end

            % Initialize default state
            obj.q      = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853 -pi/2]';
            obj.eta    = [12.5 38.5 -39 -1 -0.06 2.5]';

            % Default limits
            obj.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
            obj.jlmax  = [ 2.9; 1.65; 2.9; 0.01; 2.9; 1.25; 2.8];

            % Initialize transformations
            obj.wTv = eye(4);
            obj.vTw = eye(4);
            obj.vTe = eye(4);
            obj.vTt = eye(4);
            obj.wTt = eye(4);
            obj.eTt = eye(4); % tool coincides with end-effector by default

            % Initialize goal placeholders
            obj.wTg = eye(4);
            obj.wTgv = eye(4);

            updateTransformations(obj);
        end

        function setGoal(obj, toolPosition, toolOrientation, vehiclePosition, vehicleOrientation)
            % Set goal positions and orientations for tool and vehicle
            %
            % Inputs:
            %   toolPosition      [3x1] desired tool position in world
            %   toolOrientation   [3x1] desired RPY angles (rad)
            %   vehiclePosition   [3x1] desired vehicle position in world
            %   vehicleOrientation[3x1] desired RPY angles (rad)

            obj.wRg = rotation(toolOrientation(1), toolOrientation(2), toolOrientation(3));
            obj.goalPosition = toolPosition;
            obj.wTg = [obj.wRg obj.goalPosition; 0 0 0 1];

            obj.wRgv = rotation(vehicleOrientation(1), vehicleOrientation(2), vehicleOrientation(3));
            obj.vehicleGoalPosition = vehiclePosition;
            obj.wTgv = [obj.wRgv obj.vehicleGoalPosition; 0 0 0 1];
        end

        function updateTransformations(obj)
            % Compute forward kinematics of the UVMS
            %
            % Updates all transformations from current state.
            obj.wTv = [rotation(obj.eta(4), obj.eta(5), obj.eta(6)) obj.eta(1:3); 0 0 0 1];
            obj.vTw = inv(obj.wTv);
            obj.vTg = obj.vTw * obj.wTg;

            obj.bTe = RobustEndEffectorTransform(obj.q);
            obj.vTe = obj.vTb * obj.bTe;
            obj.vTt = obj.vTe * obj.eTt;
            obj.wTt = obj.wTv * obj.vTt;
        end
    end
end