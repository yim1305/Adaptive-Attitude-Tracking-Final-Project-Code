function [tau_cmd, theta_hat_dot, q_e, omega_e, r, Y, a] = controller( ...
    q, omega, q_d, omega_d, omega_d_dot, theta_hat, gains)
    epsilon = q(1:3);
    eta = q(4);
    epsilon_d = q_d(1:3);
    eta_d = q_d(4);

    epsilon_e = eta_d * epsilon - eta * epsilon_d + cross(epsilon, epsilon_d);
    eta_e = eta_d * eta + epsilon_d' * epsilon;
    q_e = [epsilon_e; eta_e];
    q_e = q_e / norm(q_e);
    epsilon_e = q_e(1:3);
    eta_e = q_e(4);

    C_e = quat2rotm_body(q_e);
    omega_d_B = C_e * omega_d;
    omega_e = omega - omega_d_B;
    epsilon_e_dot = 0.5 * (eta_e * eye(3) + skew(epsilon_e)) * omega_e;
    r = omega_e + gains.alpha * epsilon_e;
    a = cross(omega_e, omega_d_B) - C_e * omega_d_dot + gains.alpha * epsilon_e_dot;

    omega1 = omega(1);
    omega2 = omega(2);
    omega3 = omega(3);
    Y = [           a(1),  omega2 * omega3, -omega2 * omega3;
         -omega1 * omega3,            a(2),  omega1 * omega3;
          omega1 * omega2, -omega1 * omega2,            a(3)];

    tau_cmd = -Y * theta_hat - gains.K * r - gains.k_e * epsilon_e;
    theta_hat_dot = gains.Gamma * Y' * r;
end
