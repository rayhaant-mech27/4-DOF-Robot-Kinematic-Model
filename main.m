% =========================================================================
%  MAIN.M  —  4-DOF Robotic Manipulator: Kinematic Modelling
%  Run this file to execute the full project in sequence.
%
%  Project structure:
%    Phase 1 : setup_robot.m          — Define D-H parameters
%    Phase 2 : forward_kinematics.m   — Compute & visualise FK
%    Phase 3 : inverse_kinematics.m   — Compute IK
%    Phase 4 : workspace_viz.m        — Monte Carlo workspace
%    Phase 5 : trajectory.m           — Joint & Cartesian trajectory
%    Phase 6 : validation.m           — Compare FK vs manual math
%
%  Requirements: Peter Corke's Robotics Toolbox for MATLAB
%  Install via: Home > Add-Ons > Get Add-Ons > search "Robotics Toolbox"
% =========================================================================

clc;          % Clear the command window
clear;        % Clear all variables from workspace
close all;    % Close any open figure windows

fprintf('==============================================\n');
fprintf('  4-DOF Robotic Manipulator — Kinematic Model\n');
fprintf('==============================================\n\n');

% ---- Phase 1: Build the robot model using D-H parameters ----------------
fprintf('[Phase 1] Setting up robot model...\n');
robot = setup_robot();       % Returns a SerialLink robot object
fprintf('          Robot "%s" created with %d links.\n\n', robot.name, robot.n);

% ---- Phase 2: Forward Kinematics ----------------------------------------
fprintf('[Phase 2] Running forward kinematics...\n');
forward_kinematics(robot);
fprintf('          Done. Figure 1 shows the robot pose.\n\n');

% ---- Phase 3: Inverse Kinematics ----------------------------------------
fprintf('[Phase 3] Running inverse kinematics...\n');
inverse_kinematics(robot);
fprintf('          Done. Figure 2 shows IK solution.\n\n');

% ---- Phase 4: Workspace Visualisation -----------------------------------
fprintf('[Phase 4] Generating workspace (takes ~10 seconds)...\n');
workspace_viz(robot);
fprintf('          Done. Figure 3 shows the reachable workspace.\n\n');

% ---- Phase 5: Trajectory Planning ----------------------------------------
fprintf('[Phase 5] Planning and animating trajectory...\n');
trajectory(robot);
fprintf('          Done. Figure 4 shows joint trajectories.\n\n');

% ---- Phase 6: Validation -------------------------------------------------
fprintf('[Phase 6] Validating FK against manual D-H math...\n');
validation(robot);
fprintf('          Done. See command window for error report.\n\n');

fprintf('==============================================\n');
fprintf('  All phases complete!\n');
fprintf('==============================================\n');
