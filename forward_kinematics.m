% =========================================================================
%  FORWARD_KINEMATICS.M  —  Phase 2: Forward Kinematics
%
%  Forward Kinematics (FK) answers:
%    "Given joint angles q1, q2, q3, q4 — where is the end-effector?"
%
%  The answer is a 4×4 Homogeneous Transformation Matrix (HTM):
%
%       T = [ R  |  p ]
%           [ 0  |  1 ]
%
%  where R (3×3) is the end-effector orientation and p (3×1) is its
%  position in the base (world) frame.
%
%  We use robot.fkine(q) from the Robotics Toolbox.
% =========================================================================

function forward_kinematics(robot)

    fprintf('\n  --- Forward Kinematics Results ---\n');

    % =====================================================================
    %  Test Case 1: Home position  (all joints at 0 degrees)
    % =====================================================================
    q1 = [0, 0, 0, 0];
    T1 = robot.fkine(q1);   % Returns an SE3 object

    fprintf('\n  Configuration 1: q = [0°, 0°, 0°, 0°]\n');
    fprintf('  End-effector position: x=%.4f  y=%.4f  z=%.4f  (metres)\n', ...
            T1.t(1), T1.t(2), T1.t(3));
    fprintf('  Transformation matrix T:\n');
    disp(T1.T);   % .T gives the raw 4×4 matrix

    % =====================================================================
    %  Test Case 2: Arm reaching forward (elbow at 90°)
    % =====================================================================
    q2 = [0, pi/4, pi/4, 0];    % 45° at joint 2 and 3
    T2 = robot.fkine(q2);

    fprintf('\n  Configuration 2: q = [0°, 45°, 45°, 0°]\n');
    fprintf('  End-effector position: x=%.4f  y=%.4f  z=%.4f  (metres)\n', ...
            T2.t(1), T2.t(2), T2.t(3));

    % =====================================================================
    %  Test Case 3: Arm fully extended upward
    % =====================================================================
    q3 = [pi/2, 0, 0, 0];     % 90° shoulder rotation
    T3 = robot.fkine(q3);

    fprintf('\n  Configuration 3: q = [90°, 0°, 0°, 0°]\n');
    fprintf('  End-effector position: x=%.4f  y=%.4f  z=%.4f  (metres)\n', ...
            T3.t(1), T3.t(2), T3.t(3));

    % =====================================================================
    %  Test Case 4: All joints at 45 degrees
    % =====================================================================
    q4 = [pi/4, pi/4, pi/4, pi/4];
    T4 = robot.fkine(q4);

    fprintf('\n  Configuration 4: q = [45°, 45°, 45°, 45°]\n');
    fprintf('  End-effector position: x=%.4f  y=%.4f  z=%.4f  (metres)\n', ...
            T4.t(1), T4.t(2), T4.t(3));

    % =====================================================================
    %  Visualise all configurations side by side
    % =====================================================================
    figure(2);
    clf;    % Clear any previous figure content

    configs = {q1, q2, q3, q4};
    labels  = {'Home [0,0,0,0]', '[0,45°,45°,0]', '[90°,0,0,0]', '[45°,45°,45°,45°]'};

    for i = 1:4
        subplot(2, 2, i);
        robot.plot(configs{i});
        title(labels{i}, 'FontSize', 9);
        xlabel('X'); ylabel('Y'); zlabel('Z');
    end

    sgtitle('Forward Kinematics — 4 Configurations', 'FontSize', 12);

    % =====================================================================
    %  Summary table of end-effector positions
    % =====================================================================
    fprintf('\n  --- End-Effector Position Summary ---\n');
    fprintf('  %-26s  %8s  %8s  %8s\n', 'Configuration', 'X (m)', 'Y (m)', 'Z (m)');
    fprintf('  %s\n', repmat('-', 1, 56));

    Ts = {T1, T2, T3, T4};
    for i = 1:4
        fprintf('  %-26s  %8.4f  %8.4f  %8.4f\n', ...
                labels{i}, Ts{i}.t(1), Ts{i}.t(2), Ts{i}.t(3));
    end
    fprintf('\n');

end
