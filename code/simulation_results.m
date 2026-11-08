function result = simulation_results(t, x, J, cmg, ref, gains)
    N = numel(t);
    result.t = t;
    result.theta_hat = x(:, 8:10);
    result.theta_tilde = diag(J)' - result.theta_hat;
    result.attitude_error = zeros(N, 1);
    result.rate_error = zeros(N, 1);
    result.desired_rate = zeros(N, 1);
    result.tau_cmd = zeros(N, 3);
    result.delta_dot = zeros(N, 4);
    result.tau_error = zeros(N, 1);
    result.sigma_min = zeros(N, 1);
    H_I = zeros(N, 3);
    V = zeros(N, 1);
    g = zeros(N, 1);

    for ii = 1:N
        state = x(ii, :)';
        [x_dot, data] = attitude_dynamics(t(ii), state, J, cmg, ref, gains);
        epsilon_e = data.q_e(1:3);
        eta_e = data.q_e(4);
        r = data.r;
        theta_tilde = result.theta_tilde(ii, :)';

        result.attitude_error(ii) = 2 * atan2(norm(epsilon_e), abs(eta_e));
        result.rate_error(ii) = norm(data.omega_e);
        result.desired_rate(ii) = norm(data.omega_d);
        result.tau_cmd(ii, :) = data.tau_cmd';
        result.delta_dot(ii, :) = x_dot(11:14)';
        result.tau_error(ii) = norm(data.tau_cmg - data.tau_cmd);
        result.sigma_min(ii) = data.sigma_min;

        q = state(1:4);
        R_IB = quat2rotm_body(q / norm(q));
        omega = state(5:7);
        H_I(ii, :) = (R_IB' * (J * omega + data.h))';
        V(ii) = 2 * gains.k_e * (1 - eta_e) + 0.5 * r' * J * r ...
                + 0.5 * theta_tilde' * (gains.Gamma \ theta_tilde);
        g(ii) = r' * gains.K * r + gains.k_e * gains.alpha * (epsilon_e' * epsilon_e);
    end

    result.V = V;
    result.end_time = t(end);
    result.peak_angle = max(result.attitude_error);
    result.peak_rate_error = max(result.rate_error);
    result.final_angle = result.attitude_error(end);
    result.final_rate_error = result.rate_error(end);
    result.final_inertia_error = norm(result.theta_tilde(end, :));
    result.final_theta_hat = result.theta_hat(end, :)';
    above = find(rad2deg(result.attitude_error) > 1e-3, 1, 'last');
    if isempty(above)
        result.settling_time = 0;
    else
        result.settling_time = t(above);
    end
    result.peak_torque = max(vecnorm(result.tau_cmd, 2, 2));
    result.peak_gimbal_rate = max(abs(result.delta_dot(:)));
    result.min_sigma = min(result.sigma_min);
    result.peak_torque_error = max(result.tau_error);
    q_norms = [vecnorm(x(:, 1:4), 2, 2), vecnorm(x(:, 15:18), 2, 2)];
    result.max_q_norm_error = max(abs(q_norms - 1), [], 'all');
    result.max_momentum_drift = max(vecnorm(H_I - H_I(1, :), 2, 2));
    result.max_lyapunov_residual = max(abs(V + cumtrapz(t, g) - V(1)));
end
