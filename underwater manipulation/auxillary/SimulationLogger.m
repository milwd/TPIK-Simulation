classdef SimulationLogger < handle
    properties
        t            % time vector
        q            % joint positions
        q_dot        % joint velocities
        eta          % vehicle pose
        v_nu         % vehicle velocity
        a            % task activations (diagonal only)
        xdotbar_task % reference velocities for tasks (cell array)
        robot        % robot model
        task_set     % set of tasks
    end

    methods
        function obj = SimulationLogger(maxLoops, robotModel, task_set)
            obj.robot = robotModel;
            obj.task_set = task_set;

            obj.t = zeros(1, maxLoops);
            obj.q = zeros(7, maxLoops);
            obj.q_dot = zeros(7, maxLoops);
            obj.eta = zeros(6, maxLoops);
            obj.v_nu = zeros(6, maxLoops);

            % Store the diagonal of each activation matrix
            maxDiagSize = max(cellfun(@(t) size(t.A,1), task_set));
            obj.a = zeros(maxDiagSize, maxLoops, length(task_set));

            % Initialize cell array to store task reference velocities
            obj.xdotbar_task = cell(length(task_set), maxLoops);
        end

        function update(obj, t, loop)
            % Store robot state
            obj.t(loop) = t;
            obj.q(:, loop) = obj.robot.q;
            obj.q_dot(:, loop) = obj.robot.q_dot;
            obj.eta(:, loop) = obj.robot.eta;
            obj.v_nu(:, loop) = obj.robot.v_nu;

            % Store task activations (diagonal only) and reference velocities
            for i = 1:length(obj.task_set)
                diagA = diag(obj.task_set{i}.A);           % extract diagonal
                obj.a(1:length(diagA), loop, i) = diagA;
                obj.xdotbar_task{i, loop} = obj.task_set{i}.xdotbar;
            end
        end

        function plotAll(obj)
            % Example plotting for robot state
            figure(1);
            subplot(2,1,1);
            plot(obj.t, obj.q, 'LineWidth', 1);
            legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
            subplot(2,1,2);
            plot(obj.t, obj.q_dot, 'LineWidth', 1);
            legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');

            figure(2);
            subplot(2,1,1);
            plot(obj.t, obj.eta, 'LineWidth', 1);
            legend('x','y','z','roll','pitch','yaw');
            subplot(2,1,2);
            plot(obj.t, obj.v_nu, 'LineWidth', 1);
            legend('xdot','ydot','zdot','omega_x','omega_y','omega_z');

            % Optional: plot task activations
            figure(3);
            for i = 1:size(obj.a,3)
                subplot(size(obj.a,3),1,i);
                plot(obj.t, squeeze(obj.a(:, :, i))', 'LineWidth', 1);
                title(['Task ', num2str(i), ' Activations (diagonal)']);
            end
        end
    end
end