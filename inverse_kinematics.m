% =========================================================================
%  INVERSE_KINEMATICS.M  —  Phase 3: Inverse Kinematics
%
%  Inverse Kinematics (IK) answers the opposite question to FK:
%    "Given a desired end-effector position/orientation — what joint
%     angles q1, q2, q3, q4 do I need?"
%
%  This is harder than FK because:
%    - Multiple solutions may exist (elbow-up vs elbow-down)
%    - No solution may exist (target outside workspace)
%    - We need an initial guess to start the solver
%
%  We use robot.ikine() (numerical IK) and robot.ikunc() (unconstrained).
%  We also provide a mask vector to handle the redundancy in a 4-DOF arm
%  (a 4-DOF arm cannot control all 6 DOF of the end-effector).
% =========================================================================

function inverse_kinematics(robot)

    fprintf('\n  --- Inverse Kinematics Results ---\n');

    % =====================================================================
    %  The mask vector tells the solver which components of the pose to
    %  match. For a 4-DOF arm in 3D space:
    %    [1 1 1 0 0 0] = match x, y, z position only (ignore orientation)
    %    [1 1 1 1 0 0] = also match orientation about Z
    % =====================================================================
    mask = [1, 1, 1, 0, 0, 0];   % Solve for position only

    % Starting guess for the solver (important — bad guess = no solution)
    q0 = [0, 0, 0, 0];

    % =====================================================================
    %  Target 1: A specific XYZ position we want to reach
    % =====================================================================
    % Build a target transformation matrix using transl() — translation only
    target1_pos = [0.6, 0.2, 0.4];    % x, y, z in metres
    T_target1   = transl(target1_pos);

    fprintf('\n  Target 1: Position [%.2f, %.2f, %.2f] m\n', target1_pos);

    q_ik1 = robot.ikine(T_target1, q0, 'mask', mask);

    % Verify: run FK on the IK solution and check position
    T_check1 = robot.fkine(q_ik1);
    error1   = norm(T_check1.t - target1_pos');   % Euclidean distance error

    fprintf('  IK solution (degrees): q = [%.2f, %.2f, %.2f, %.2f]\n', ...
            rad2deg(q_ik1(1)), rad2deg(q_ik1(2)), ...
            rad2deg(q_ik1(3)), rad2deg(q_ik1(4)));
    fprintf('  FK verification position: [%.4f, %.4f, %.4f]\n', ...
            T_check1.t(1), T_check1.t(2), T_check1.t(3));
    fprintf('  Position error: %.6f m  (%s)\n', error1, ...
            ternary(error1 < 1e-4, 'PASS', 'FAIL — try adjusting q0'));

    % =====================================================================
    %  Target 2: Another position
    % =====================================================================
    target2_pos = [0.3, 0.4, 0.3];
    T_target2   = transl(target2_pos);

    fprintf('\n  Target 2: Position [%.2f, %.2f, %.2f] m\n', target2_pos);

    q_ik2 = robot.ikine(T_target2, q0, 'mask', mask);
    T_check2 = robot.fkine(q_ik2);
    error2   = norm(T_check2.t - target2_pos');

    fprintf('  IK solution (degrees): q = [%.2f, %.2f, %.2f, %.2f]\n', ...
            rad2deg(q_ik2(1)), rad2deg(q_ik2(2)), ...
            rad2deg(q_ik2(3)), rad2deg(q_ik2(4)));
    fprintf('  Position error: %.6f m  (%s)\n', error2, ...
            ternary(error2 < 1e-4, 'PASS', 'FAIL'));

    % =====================================================================
    %  Target 3: Near the boundary of the workspace
    % =====================================================================
    target3_pos = [0.8, 0.1, 0.5];
    T_target3   = transl(target3_pos);

    fprintf('\n  Target 3 (near boundary): Position [%.2f, %.2f, %.2f] m\n', target3_pos);

    q_ik3 = robot.ikine(T_target3, q0, 'mask', mask);
    T_check3 = robot.fkine(q_ik3);
    error3   = norm(T_check3.t - target3_pos');

    fprintf('  Position error: %.6f m  (%s)\n', error3, ...
            ternary(error3 < 1e-3, 'PASS', 'Outside workspace or poor convergence'));

    % =====================================================================
    %  Visualise the IK solutions
    % =====================================================================
    figure(3);
    clf;

    subplot(1, 3, 1);
    robot.plot(q_ik1);
    title(sprintf('IK Target 1\n[%.2f, %.2f, %.2f]', target1_pos), 'FontSize', 9);

    subplot(1, 3, 2);
    robot.plot(q_ik2);
    title(sprintf('IK Target 2\n[%.2f, %.2f, %.2f]', target2_pos), 'FontSize', 9);

    subplot(1, 3, 3);
    robot.plot(q_ik3);
    title(sprintf('IK Target 3\n[%.2f, %.2f, %.2f]', target3_pos), 'FontSize', 9);

    sgtitle('Inverse Kinematics — 3 Target Positions', 'FontSize', 12);

end

% ---- Helper: ternary operator (replaces if/else in fprintf calls) -------
function out = ternary(cond, a, b)
    if cond
        out = a;
    else
        out = b;
    end
end
