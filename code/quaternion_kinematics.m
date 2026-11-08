function q_dot = quaternion_kinematics(q, omega)
    epsilon = q(1:3);
    eta = q(4);
    epsilon_dot = 0.5 * (eta * omega + cross(epsilon, omega));
    eta_dot = -0.5 * epsilon' * omega;
    q_dot = [epsilon_dot; eta_dot];
end
