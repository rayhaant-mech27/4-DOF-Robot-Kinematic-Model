% =========================================================================
%  SETUP_ROBOT.M  —  Phase 1: D-H Parameter Model
%
%  Denavit-Hartenberg (D-H) convention assigns 4 parameters to each joint:
%
%    a     (link length)   — distance between Z-axes along X-axis
%    alpha (link twist)    — angle between Z-axes about X-axis
%    d     (link offset)   — distance between X-axes along Z-axis
%    theta (joint angle)   — angle between X-axes about Z-axis  ← this is
%                            the variable (q) for a revolute joint
%
%  Our 4-DOF arm geometry (all units in metres, angles in radians):
%
%   Link |  a (m)  | alpha (deg) |  d (m)  | theta  | Joint type
%   -----|---------|-------------|---------|--------|------------
%     1  |  0.000  |     90      |  0.500  |   q1   | Revolute
%     2  |  0.400  |      0      |  0.000  |   q2   | Revolute
%     3  |  0.350  |      0      |  0.000  |   q3   | Revolute
%     4  |  0.200  |      0      |  0.000  |   q4   | Revolute
%
%  Imagine a shoulder joint (Link 1) rising vertically, then three
%  arm segments (Links 2–4) folding in the same plane.
% =========================================================================

function robot = setup_robot()

    % --- Define each link using the Link() constructor -------------------
    %
    %   Link([theta, d, a, alpha])   — standard D-H convention
    %   The variable joint angle (q) is added automatically at runtime.
    %
    %   We add joint limits [min max] in radians for each link so that
    %   the inverse kinematics solver stays within a physical range.

    L1 = Link([0,  0.500,  0.000,  pi/2], 'standard');
    L1.qlim = [-pi,  pi];      % ±180° for the shoulder

    L2 = Link([0,  0.000,  0.400,  0    ], 'standard');
    L2.qlim = [-pi/2,  pi/2]; % ±90° for upper arm

    L3 = Link([0,  0.000,  0.350,  0    ], 'standard');
    L3.qlim = [-pi/2,  pi/2]; % ±90° for forearm

    L4 = Link([0,  0.000,  0.200,  0    ], 'standard');
    L4.qlim = [-pi,  pi];      % ±180° for the wrist

    % --- Assemble into a SerialLink robot --------------------------------
    robot = SerialLink([L1, L2, L3, L4], 'name', '4-DOF Arm');

    % --- Print the D-H table to the command window -----------------------
    fprintf('\n  D-H Parameter Table:\n');
    fprintf('  %-6s %-10s %-14s %-10s %-10s\n', ...
            'Link', 'a (m)', 'alpha (rad)', 'd (m)', 'theta');
    fprintf('  %s\n', repmat('-', 1, 54));

    dh = [0,     pi/2,  0.5,  0;
          0.4,   0,     0,    0;
          0.35,  0,     0,    0;
          0.2,   0,     0,    0];

    for i = 1:4
        fprintf('  L%-5d %-10.3f %-14.4f %-10.3f %-10s\n', ...
                i, dh(i,1), dh(i,2), dh(i,3), 'q_i');
    end
    fprintf('\n');

    % --- Show the robot in its zero (home) configuration -----------------
    %   q = [0, 0, 0, 0] means every joint angle is 0
    q_home = [0, 0, 0, 0];

    figure(1);
    robot.plot(q_home);
    title('Robot at Home Configuration (q = [0, 0, 0, 0])');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

end
