classdef UvmsSim < handle
    %UVMSIM Simulator for UVMS (vehicle + manipulator)
    % - Integrates velocities to update the robot state
    % - Updates transforms via the robot model
    % - Handles logging and plotting (optional)

    properties
        robot           % instance of UvmsModel (holds all state)
        dt              % simulation timestep
        time            % current simulation time
        maxSteps        % maximum number of simulation steps
        loopCounter     % loop counter for logging
    end

    methods
        function obj = UvmsSim(dt, robotModel, endTime)
            obj.dt = dt;
            obj.robot = robotModel;
            obj.time = 0;
            obj.loopCounter = 1;
            obj.maxSteps = ceil(endTime/dt);
        end

        function step(obj, v_nu, q_dot)
            % update the state in the robot object
            obj.robot.q_dot = q_dot;
            obj.robot.v_nu = v_nu;

            % Integrate vehicle pose
            obj.robot.eta = integrate_vehicle(obj.robot.eta, v_nu, obj.dt);

            % Integrate manipulator joints
            obj.robot.q = obj.robot.q + q_dot * obj.dt;

            % Update all transformations
            obj.robot.updateTransformations();

            % increment time and loop counter
            obj.time = obj.time + obj.dt;
            obj.loopCounter = obj.loopCounter + 1;
        end
    end
end