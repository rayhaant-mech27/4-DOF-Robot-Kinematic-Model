% =========================================================================
%  VALIDATION.M  —  Phase 6: Validate FK Against Manual D-H Math
%
%  We manually compute the transformation matrices using the D-H formula:
%
%    T_i = Rot_z(theta_i) * Trans_z(d_i) * Trans_x(a_i) * Rot_x(alpha_i)
%
%  Then multiply them:  T_total = T1 * T2 * T3 * T4
%
%  And compare with what robot.fkine() gives us.
%  If the error is ~0 (< 1e-10), our D-H model is correctly implemented.
%
%  This is the validation step: simulated result == theoretical result.
% =========================================================================

function validation(robot)

    fprintf('\n  --- Validation: Manual D-H vs robot.fkine() ---\n\n');

    % =====================================================================
    %  Test configurations to validate
    % =====================================================================
    test_configs = {
        [0,       0,      0,     0     ],   'Home [0,0,0,0]';
        [pi/4,    pi/4,   pi/4,  pi/4  ],   '45° all joints';
        [pi/2,    0,      0,     0     ],   '90° joint 1 only';
        [0,       pi/3,  -pi/6,  pi/4  ],   'Mixed angles';
        [-pi/4,   pi/6,   pi/3, -pi/3  ],   'Negative angles';
    };

    n_tests     = size(test_configs, 1);
    max_errors  = zeros(n_tests, 1);   % Store max element-wise error

    % D-H Parameters (must match setup_robot.m exactly)
    %         a      alpha   d     (theta is the variable)
    dh = [0,     pi/2,  0.5;   % Link 1
          0.4,   0,     0;     % Link 2
          0.35,  0,     0;     % Link 3
          0.2,   0,     0];    % Link 4

    fprintf('  %-26s  %-16s  %s\n', 'Configuration', 'Max Error', 'Status');
    fprintf('  %s\n', repmat('-', 1, 60));

    for k = 1:n_tests
        q     = test_configs{k, 1};
        label = test_configs{k, 2};

        % --- Toolbox FK ---------------------------------------------------
        T_toolbox = robot.fkine(q);
        T_tool_mat = T_toolbox.T;   % Raw 4×4 matrix

        % --- Manual D-H computation ---------------------------------------
        T_manual = eye(4);    % Start with identity (base frame)

        for i = 1:4
            a_i     = dh(i, 1);
            alpha_i = dh(i, 2);
            d_i     = dh(i, 3);
            theta_i = q(i);

            % Standard D-H transformation for link i:
            %  T_i = Rz(theta) * Tz(d) * Tx(a) * Rx(alpha)
            T_i = dh_transform(theta_i, d_i, a_i, alpha_i);

            T_manual = T_manual * T_i;   % Chain multiply
        end

        % --- Compare -------------------------------------------------------
        error_mat  = abs(T_tool_mat - T_manual);
        max_err    = max(error_mat(:));
        max_errors(k) = max_err;

        status = 'PASS';
        if max_err > 1e-10
            status = 'WARN (check D-H params)';
        end

        fprintf('  %-26s  %-16.2e  %s\n', label, max_err, status);

        % Print detailed comparison for the first test case
        if k == 1
            fprintf('\n  [Detailed comparison for "%s"]\n', label);
            fprintf('  Toolbox FK matrix:\n'); disp(T_tool_mat);
            fprintf('  Manual D-H matrix:\n'); disp(T_manual);
            fprintf('  Difference matrix:\n'); disp(error_mat);
        end
    end

    % =====================================================================
    %  Summary
    % =====================================================================
    fprintf('\n  Overall max error across all test cases: %.2e\n', max(max_errors));
    if max(max_errors) < 1e-10
        fprintf('  VALIDATION PASSED — D-H model is correct.\n');
    else
        fprintf('  VALIDATION WARNING — Check D-H parameters in setup_robot.m\n');
    end

    % =====================================================================
    %  Figure: Error bar chart across test cases
    % =====================================================================
    figure(8);
    clf;
    bar(max_errors, 'FaceColor', [0.2 0.6 0.9]);
    set(gca, 'XTickLabel', test_configs(:, 2), 'XTickLabelRotation', 15);
    ylabel('Max element-wise error');
    title('Validation: FK Toolbox vs Manual D-H Computation');
    yscale log;    % Log scale so near-zero errors are visible
    grid on;
    yline(1e-10, 'r--', 'Pass threshold (1e-10)', 'LabelHorizontalAlignment', 'left');

end

% =========================================================================
%  DH_TRANSFORM — Build the 4×4 D-H transformation matrix for one link
%
%  Standard form:
%    T = [ cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a·cos(θ) ]
%        [ sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a·sin(θ) ]
%        [   0        sin(α)          cos(α)           d     ]
%        [   0          0               0               1    ]
% =========================================================================
function T = dh_transform(theta, d, a, alpha)
    ct = cos(theta);  st = sin(theta);
    ca = cos(alpha);  sa = sin(alpha);

    T = [ ct,  -st*ca,   st*sa,   a*ct;
          st,   ct*ca,  -ct*sa,   a*st;
           0,      sa,      ca,      d;
           0,       0,       0,      1];
end
