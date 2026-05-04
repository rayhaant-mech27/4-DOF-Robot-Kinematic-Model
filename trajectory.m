% =========================================================================
%  TRAJECTORY.M  —  Phase 5: Trajectory Planning
%
%  A trajectory is a smooth path between two configurations over time.
%  We plan both:
%
%    1. Joint-space trajectory  (jtraj)
%       - Interpolates joint angles directly using a quintic polynomial
%       - Smooth, guaranteed to stay within joint limits
%       - Path in Cartesian space is curved and hard to predict
%
%    2. Cartesian-space trajectory  (ctraj)
%       - Interpolates end-effector position as a straight line in space
%       - Useful when a straight-line tool path is needed
%       - Requires IK at every step; can fail near singularities
%
% =========================================================================

function trajectory(robot)

    % =====================================================================
    %  Define start and end configurations (in joint space)
    % =====================================================================
    q_start = [0,       0,      0,      0     ];   % Home position
    q_end   = [pi/3,    pi/4,  -pi/4,   pi/6  ];   % A target pose

    n_steps = 100;    % Number of time steps (100 = smooth animation)

    % =====================================================================
    %  PART 1 — Joint-space trajectory using jtraj()
    % =====================================================================
    %  jtraj(q0, qf, n) returns an n×4 matrix where each row is a joint
    %  angle configuration at that time step.  Also returns velocity (qd)
    %  and acceleration (qdd) profiles.
    % =====================================================================

    [q_traj, qd_traj, qdd_traj] = jtraj(q_start, q_end, n_steps);

    % --- Compute end-effector Cartesian path for this trajectory ---------
    cart_path = zeros(n_steps, 3);
    for i = 1:n_steps
        T = robot.fkine(q_traj(i, :));
        cart_path(i, :) = T.t';
    end

    % =====================================================================
    %  PART 2 — Cartesian-space trajectory using ctraj()
    % =====================================================================
    %  ctraj() interpolates between two SE3 poses in Cartesian space.
    % =====================================================================

    T_start = robot.fkine(q_start);   % Start pose as SE3 object
    T_end   = robot.fkine(q_end);     % End pose as SE3 object

    % Generate linearly interpolated Cartesian path
    T_cart_traj = ctraj(T_start, T_end, n_steps);

    % Solve IK at each step to get joint angles for Cartesian trajectory
    mask   = [1, 1, 1, 0, 0, 0];
    q_ctraj = zeros(n_steps, 4);
    q_prev  = q_start;    % Use previous solution as the initial guess
    for i = 1:n_steps
        q_ctraj(i,:) = robot.ikine(T_cart_traj(i), q_prev, 'mask', mask);
        q_prev = q_ctraj(i,:);   % Warm-start for next step
    end

    % =====================================================================
    %  Figure 5a — Joint angle profiles over time
    % =====================================================================
    figure(5);
    clf;

    t = linspace(0, 1, n_steps);   % Normalised time (0 to 1)

    subplot(3, 1, 1);
    plot(t, rad2deg(q_traj));
    title('Joint Angles over Time (joint-space trajectory)');
    xlabel('Normalised time'); ylabel('Angle (degrees)');
    legend('q1', 'q2', 'q3', 'q4', 'Location', 'best');
    grid on;

    subplot(3, 1, 2);
    plot(t, rad2deg(qd_traj));
    title('Joint Velocities over Time');
    xlabel('Normalised time'); ylabel('Velocity (deg/s)');
    legend('dq1', 'dq2', 'dq3', 'dq4', 'Location', 'best');
    grid on;

    subplot(3, 1, 3);
    plot(t, rad2deg(qdd_traj));
    title('Joint Accelerations over Time');
    xlabel('Normalised time'); ylabel('Acceleration (deg/s²)');
    legend('ddq1', 'ddq2', 'ddq3', 'ddq4', 'Location', 'best');
    grid on;

    sgtitle('Joint-Space Trajectory (jtraj)', 'FontSize', 12);

    % =====================================================================
    %  Figure 5b — Cartesian end-effector path
    % =====================================================================
    figure(6);
    clf;

    subplot(1, 2, 1);
    plot3(cart_path(:,1), cart_path(:,2), cart_path(:,3), 'b-', 'LineWidth', 2);
    hold on;
    plot3(cart_path(1,1),   cart_path(1,2),   cart_path(1,3),   'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot3(cart_path(end,1), cart_path(end,2), cart_path(end,3), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
    legend('Path', 'Start', 'End');
    title('End-Effector Path (joint-space trajectory)');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    grid on; axis equal;

    subplot(1, 2, 2);
    % Cartesian path from ctraj
    cart_ctraj = zeros(n_steps, 3);
    for i = 1:n_steps
        cart_ctraj(i,:) = T_cart_traj(i).t';
    end
    plot3(cart_ctraj(:,1), cart_ctraj(:,2), cart_ctraj(:,3), 'r-', 'LineWidth', 2);
    hold on;
    plot3(cart_ctraj(1,1),   cart_ctraj(1,2),   cart_ctraj(1,3),   'go', 'MarkerSize', 10, 'LineWidth', 2);
    plot3(cart_ctraj(end,1), cart_ctraj(end,2), cart_ctraj(end,3), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
    legend('Path', 'Start', 'End');
    title('End-Effector Path (Cartesian trajectory — straight line)');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    grid on; axis equal;

    sgtitle('Trajectory Comparison: Joint-space vs Cartesian', 'FontSize', 12);

    % =====================================================================
    %  Animate the joint-space trajectory
    % =====================================================================
    fprintf('\n    Animating joint-space trajectory (close figure to continue)...\n');

    figure(7);
    robot.plot(q_traj, 'loop', 'fps', 30);
    title('Trajectory Animation');

end
