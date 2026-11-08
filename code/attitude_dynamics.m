function [x_dot, data] = attitude_dynamics(t, x, J, cmg, ref, gains)
    q = x(1:4);
    omega = x(5:7);
    theta_hat = x(8:10);
    delta = x(11:14);
    q_d = x(15:18);

    [omega_d, omega_d_dot] = reference_motion(t, ref);
    [tau_cmd, theta_hat_dot, q_e, omega_e, r] = controller( ...
        q, omega, q_d, omega_d, omega_d_dot, theta_hat, gains);
    [h, A] = cmg_model(delta, cmg);
    sigma_min = min(svd(A));
    if sigma_min <= cmg.a_min
        error('CMG Jacobian reached the singularity threshold at %.3f s.', t);
    end
    delta_dot = steering_algorithm(tau_cmd, omega, h, A, cmg.h0);
    tau_cmg = -cmg.h0 * A * delta_dot - cross(omega, h);

    q_dot = quaternion_kinematics(q, omega);
    q_d_dot = quaternion_kinematics(q_d, omega_d);
    omega_dot = J \ (tau_cmg - cross(omega, J * omega));
    x_dot = [q_dot; omega_dot; theta_hat_dot; delta_dot; q_d_dot];

    if nargout > 1
        data.q_e = q_e;
        data.omega_e = omega_e;
        data.omega_d = omega_d;
        data.r = r;
        data.h = h;
        data.sigma_min = sigma_min;
        data.tau_cmd = tau_cmd;
        data.tau_cmg = tau_cmg;
    end
end
