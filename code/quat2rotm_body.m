function R = quat2rotm_body(q)
    epsilon = q(1:3);
    eta = q(4);
    R = (eta^2 - epsilon' * epsilon) * eye(3) + 2 * (epsilon * epsilon') - 2 * eta * skew(epsilon);
end
