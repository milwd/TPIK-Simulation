% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./tasks');
addpath('./action_managers');
addpath('./auxillary');
addpath('./robust_robot');
clc; clear; close all;

dt       = 0.1;
endTime  = 20;

robotModel = UvmsModel();          
sim = UvmsSim(dt, robotModel, endTime);
unity = UnityInterface("127.0.0.1"); 

% define goals
vehicle_stop = [10.7 37.4 -38 0 -0.06 0.5]';
nodule_pos   = [12.20 37.3 -39.9]';

% Define tasks and actions
actionManager = ActionManager_TAVC();

task_vehicleposition    = TaskVehiclePositionA(vehicle_stop);
task_altitudecontrol    = TaskAltitudeControl();

task_set_SafeNavigation    = {task_altitudecontrol, task_vehicleposition};
task_set_id_SafeNavigation = {"Alt", "VP"};
actionManager.addAction(task_set_SafeNavigation, task_set_id_SafeNavigation);   % action 1

task_land               = TaskLand();
task_attitudealignment  = TaskAttitudeAlignment();
task_attitudedirect     = TaskAttitudeDirect(nodule_pos);

task_set_Land    = {task_attitudedirect, task_attitudealignment, task_land};
task_set_id_Land = {"AtD", "Att", "L"};

actionManager.addAction(task_set_Land, task_set_id_Land);                       % action 2 

task_tool               = TaskToolPositionB([rotation(pi, 0, 0), nodule_pos; 0 0 0 1] );
task_set_Manipulate     = {task_tool};
task_set_id_Manipulate  = {"TP"};

actionManager.addAction(task_set_Manipulate, task_set_id_Manipulate);           % action 3

logger = SimulationLoggerFull(ceil(endTime/dt)+1, robotModel);

for step = 1:sim.maxSteps
    robotModel.altitude = unity.receiveAltitude(robotModel);

    [q_dot, v_nu, act] = actionManager.computeICAT(robotModel, step);
    
    sim.step(v_nu, q_dot);

    unity.send(robotModel);

    logger.update(sim.time, sim.loopCounter, actionManager);

    if mod(sim.loopCounter, round(1 / sim.dt)) == 0
        fprintf('x=%.2f - y=%.2f - z=%.2f - a=%.2f\n', robotModel.eta(1), robotModel.eta(2), robotModel.eta(3), act);
        fprintf('roll = %.2f - pitch = %.2f \n', robotModel.eta(4), robotModel.eta(5));
        fprintf('t = %.2f s\n', sim.time);
        fprintf('alt = %.2f m\n', robotModel.altitude);
    end

    SlowdownToRealtime(dt);
end

logger.plotAll();

delete(unity);