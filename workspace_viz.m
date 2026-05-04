% =========================================================================
%  WORKSPACE_VIZ.M  —  Phase 4: Workspace Visualisation
%
%  The workspace is the set of ALL positions the end-effector can reach.
%
%  Method: Monte Carlo Sampling
%    - Generate thousands of random joint-angle combinations
%    - Run FK on each combination
%    - Plot the resulting end-effector positions as a point cloud
%
%  This gives us both:
%    - Reachable workspace  (the full cloud)
%    - Dexterous workspace  (the dense region reachable in many ways)
%
%  We also show cross-section slices (YZ and XZ planes) so it's easier
%  to understand the 3D shape.
% =========================================================================

function workspace_viz(robot)

    % =====================================================================
    %  Parameters
    % =====================================================================
    N = 50000;    % Number of random configurations to sample
                  % Increase for denser cloud, decrease if it's too slow

    % =====================================================================
    %  Generate random joint angles within each link's limits
    % =====================================================================
    n_joints = robot.n;            % Number of joints (4)
    q_rand   = zeros(N, n_joints); % Pre-allocate for speed

    for j = 1:n_joints
        lo = robot.links(j).qlim(1);    % Lower joint limit
        hi = robot.links(j).qlim(2);    % Upper joint limit
        % Uniform random values between [lo, hi]
        q_rand(:, j) = lo + (hi - lo) .* rand(N, 1);
    end

    % =====================================================================
    %  Compute FK for every random configuration
    % =====================================================================
    positions = zeros(N, 3);    % Each row = [x, y, z] of end-effector

    fprintf('    Sampling %d random configurations... ', N);
    for i = 1:N
        T = robot.fkine(q_rand(i, :));
        positions(i, :) = T.t';
    end
    fprintf('done.\n');

    % =====================================================================
    %  Figure 4a: 3D Workspace Point Cloud
    % =====================================================================
    figure(4);
    clf;

    subplot(2, 2, [1 2]);   % Wide top panel for the 3D view

    % Colour the points by height (z value) — gives a nice depth effect
    scatter3(positions(:,1), positions(:,2), positions(:,3), ...
             1, positions(:,3), 'filled');   % '1' = dot size

    colormap(jet);
    colorbar;
    title('3D Reachable Workspace (Monte Carlo, 50k samples)');
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    axis equal;
    grid on;
    view(45, 30);   % Camera angle

    % =====================================================================
    %  Figure 4b: XZ cross-section (side view, Y ≈ 0)
    % =====================================================================
    subplot(2, 2, 3);

    % Filter points near the XZ plane (Y close to 0)
    near_xz = abs(positions(:,2)) < 0.05;
    scatter(positions(near_xz, 1), positions(near_xz, 3), ...
            1, 'b', 'filled');
    title('XZ Cross-section (Y ≈ 0)');
    xlabel('X (m)'); ylabel('Z (m)');
    axis equal; grid on;

    % =====================================================================
    %  Figure 4c: XY cross-section (top view, Z ≈ fixed height)
    % =====================================================================
    subplot(2, 2, 4);

    % Filter points at a mid-height
    mid_z    = median(positions(:,3));
    near_xy  = abs(positions(:,3) - mid_z) < 0.05;
    scatter(positions(near_xy, 1), positions(near_xy, 2), ...
            1, 'r', 'filled');
    title(sprintf('XY Cross-section (Z ≈ %.2f m)', mid_z));
    xlabel('X (m)'); ylabel('Y (m)');
    axis equal; grid on;

    sgtitle('Workspace Visualisation — 4-DOF Arm', 'FontSize', 12);

    % =====================================================================
    %  Print workspace statistics
    % =====================================================================
    fprintf('\n  --- Workspace Statistics ---\n');
    fprintf('  X range: [%.3f, %.3f] m\n', min(positions(:,1)), max(positions(:,1)));
    fprintf('  Y range: [%.3f, %.3f] m\n', min(positions(:,2)), max(positions(:,2)));
    fprintf('  Z range: [%.3f, %.3f] m\n', min(positions(:,3)), max(positions(:,3)));

    % Maximum reach = farthest point from origin
    distances   = sqrt(sum(positions.^2, 2));
    max_reach   = max(distances);
    fprintf('  Maximum reach from origin: %.4f m\n', max_reach);
    fprintf('  Theoretical max reach: %.4f m  (a2 + a3 + a4 = 0.4+0.35+0.2)\n', 0.95);

end
