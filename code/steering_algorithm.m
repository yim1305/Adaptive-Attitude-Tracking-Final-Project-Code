function delta_dot = steering_algorithm(tau_cmd, omega, h, A, h0)
    delta_dot = -pinv(A) * (tau_cmd + cross(omega, h)) / h0;
end
