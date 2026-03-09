
% Add paths
addpath('./simulation_scripts');
addpath('./tools')
addpath('./icat')
addpath('./tasks')
addpath('./action_managers')
clc; clear; close all; 

% Simulation Parameters
dt = 0.005;
end_time = 20;

% Simulation Setup
real_robot = false;
robot_udp = UDP_interface(real_robot);
model = load("panda.mat");

% Initialize Arms (Home Position)
arm1 = panda_arm(model, eye(4));                                                
arm2 = panda_arm(model, [rotation(0, 0, pi), [1.06, -0.01, 0]'; 0, 0, 0, 1]);   

% Initialize Bimanual Simulator
bm_sim = bimanual_sim(dt, arm1, arm2, end_time);

%% --- GEOMETRY & TARGETS ---

% 1. Define Object Initial Pose
obj_length = 0.06;
w_obj_pos = [0.5 0 0.59]';
w_obj_ori = rotation(0, 0, 0);
wTo_obj_init = [w_obj_ori w_obj_pos; 0 0 0 1];

% 2. Define Grasp Targets (Symmetric Tilt)
% Base rotation: Gripper pointing DOWN (X-axis rotation of 180)
R_base = rotation(pi, 0, 0);

% TILT LOGIC:
% Left Arm: Rotate +30 deg around Y to point inward (towards +X)
% Right Arm: Rotate -30 deg around Y to point inward (towards -X)
R_L_grasp = R_base * rotation(0, deg2rad(20), 0);
R_R_grasp = R_base * rotation(0, deg2rad(-20), 0);

% Grasp Positions: +/- 6cm from center along X
pL_grasp = w_obj_pos + [-obj_length/2; 0; 0]; 
pR_grasp = w_obj_pos + [ obj_length/2; 0; 0]; 

% Construct Grasp Transforms
T_L_grasp = [R_L_grasp pL_grasp; 0 0 0 1];
T_R_grasp = [R_R_grasp pR_grasp; 0 0 0 1];

% 3. Define Object Goal (For Action 2)
wOg_pos = [0.60, 0.40, 0.48]';
%wOg_pos = [0.7, 0.8, 0.17]';
wTog = [rotation(0,0,0) wOg_pos; 0 0 0 1];

% 4. Compute Rigid Offsets for Cooperation
% These determine how the arms hold the object during Phase 2
l_T_o = T_L_grasp \ wTo_obj_init; 
r_T_o = T_R_grasp \ wTo_obj_init;

%% --- TASK DEFINITIONS ---

left_jointLimits_task    = TaskJointLimits("L", "LJL", arm1.jlmax, arm1.jlmin);
right_jointLimits_task   = TaskJointLimits("R", "RJL", arm2.jlmax, arm2.jlmin);
left_altitudeLimit_task  = TaskAltitudeControl("L", "LAC", 0.15); 
right_altitudeLimit_task = TaskAltitudeControl("R", "RAC", 0.15);

% --- Phase 1: Reach Grasp Points ---
task_L_reach = TaskToolPosition("L", "ReachL", T_L_grasp);
task_R_reach = TaskToolPosition("R", "ReachR", T_R_grasp);

phase1_stack = {left_jointLimits_task, right_jointLimits_task, ...
                left_altitudeLimit_task, right_altitudeLimit_task, ...
                task_L_reach, task_R_reach};
phase1_ids   = { 'LJL', 'RJL', 'LAC', 'RAC', 'ReachL', 'ReachR'};

% --- Phase 2: Cooperative Move ---
coop_task = TaskCooperationDistributed(wTog, arm1, arm2, l_T_o, r_T_o, true);
rigid_body_task = TaskRigidBodyConstraint(l_T_o, r_T_o, true, true);

phase_coop_safety_stack = {rigid_body_task, ...
                left_jointLimits_task, right_jointLimits_task, ...
                left_altitudeLimit_task, right_altitudeLimit_task, ...
                coop_task, ...
                           };

phase_coop_safety_ids   = {'RBC', 'LJL', 'RJL', 'LAC', 'RAC', 'Coop'};

% --- Phase 3: Stop ---
% Assignment: "velocities set to zero, every action deactivated except min altitude"
phase3_stack = {left_altitudeLimit_task, right_altitudeLimit_task};
phase3_ids   = {'LAC', 'RAC'};

%% --- ACTION MANAGER ---
%% --- ACTION MANAGER SETUP ---
actionManager = ActionManager_Comp();

actionManager.addAction(phase1_stack, phase1_ids);
actionManager.addAction(phase_coop_safety_stack, phase_coop_safety_ids); 
actionManager.addAction(phase3_stack, phase3_ids);


logger = SimulationLogger(ceil(end_time/dt)+1, bm_sim, actionManager);

%% --- MAIN LOOP ---
fprintf('Starting 3-Phase Mission: Approach -> Grasp -> Carry\n');

% Flag to ensure we only print phase changes once
last_phase = 0; 

for t = 0:dt:end_time
    % 1. Receive UDP / Update Robot State
    [ql,qr] = robot_udp.udp_receive(t);
    if real_robot
        bm_sim.left_arm.q  = ql;
        bm_sim.right_arm.q = qr;
    end
    
    % 2. Kinematics
    bm_sim.update_full_kinematics();
    
    % 3. COMPUTE CONTROL (HYBRID ARCHITECTURE)
    % We use 'actionManager' as the Master State Machine
    currentPhase = actionManager.currentAction;
    
    [q_dot] = actionManager.computeICAT(bm_sim, t);
    
    % 4. Integration
    bm_sim.sim(q_dot);
    
    % 5. Send & Log
    robot_udp.send(t, bm_sim);
    
    %logger.update(bm_sim.time, bm_sim.loopCounter);
    logger.update(t, bm_sim.loopCounter, actionManager);

    if mod(bm_sim.loopCounter, 50) == 0
        switch currentPhase
        case 1 
            % --- PHASE 1: INDEPENDENT APPROACH ---
            fprintf('[Main] Phase 1: Independent Approach\n'); last_phase = 1;
        case 2
            fprintf('[Main] Phase 2: Distributed Cooperation\n'); last_phase = 2;
        case 3
            fprintf('[Main] Phase 3: Holding Altitude\n'); last_phase = 3;
        otherwise
            q_dot = zeros(14,1);
        end

        ee_dif = arm1.wTt(1:3, 4) - arm2.wTt(1:3, 4);
        fprintf('\t\t\t\t\t ee-dif x: %.2f y: %.2f z: %.2f | norm: %.3f\n', ...
                ee_dif(1), ee_dif(2), ee_dif(3), norm(ee_dif));

        curr_stack = actionManager.actions{currentPhase};
        if ~isempty(curr_stack)
            err_val = norm(curr_stack{2}.xdotbar);
        else
            err_val = 0;
        end
        manager_name = "STD ";
        
        fprintf('T: %.2f | Phase: %d | Mode: %s | Err: %.4f\n', ...
            t, currentPhase, manager_name, err_val);
    end
    
    SlowdownToRealtime(dt);
end

% Plot final results
% 1. Plot Reference Velocity and Activation for ALL actions automatically
logger.plotAll();

% 2. Plot the Cartesian Path of the Object and Hands towards the Goal
logger.plotCoopCartesian();

% 3. Plot final 3D Pose
logger.plot3DTraj();

logger.plotCoopAnalysis();
