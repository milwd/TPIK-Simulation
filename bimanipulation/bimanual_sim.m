classdef bimanual_sim < handle
    %Bimanual Simulator for two Franka Emika Manipulators
    % - Integrates velocities to update the robot state
    % - Updates transforms via the robot model

    properties
        left_arm      %Instace of arm model
        right_arm     %Instace of arm model
        dt            %Simulation time step
        time          %Current Simulation time
        maxSteps      %Maximum number of simulation steps
        loopCounter   %Loop counter for logging
    end

    methods
        function obj = bimanual_sim(dt,arm_model1,arm_model2,endTime)
            obj.dt = dt;
            obj.left_arm = arm_model1;
            obj.right_arm = arm_model2;
            obj.time = 0;
            obj.loopCounter = 1;
            obj.maxSteps = ceil(endTime/dt);
        end

        function sim(obj,qdot)
            % update the state in the robot object
            obj.left_arm.qdot = qdot(1:7);
            obj.right_arm.qdot = qdot(8:14);

            % Integrate manipulator joints
            obj.left_arm.q = obj.left_arm.q + obj.left_arm.qdot*obj.dt;    
            obj.right_arm.q = obj.right_arm.q + obj.right_arm.qdot*obj.dt;

            % increment time and loop counter
            obj.time = obj.time + obj.dt;
            obj.loopCounter = obj.loopCounter + 1;
        end

        function update_full_kinematics(obj)
            %Compute forward and differential kinematics for both arms,
            %considering the current mission phase
            obj.left_arm.update_transform()
            obj.right_arm.update_transform()
            obj.left_arm.update_jacobian()
            obj.right_arm.update_jacobian()
        end
    end
end