function [omega_d, omega_d_dot] = reference_motion(t, ref)
    if ref.periodic
        frequency = ref.frequency * [1; 2; 3];
        omega_d = ref.amplitude * sin(frequency * t);
        omega_d_dot = ref.amplitude * frequency .* cos(frequency * t);
        return;
    end

    s = min(max(t / ref.slew_time, 0), 1);
    angle_dot = ref.angle * 30 * s^2 * (1 - s)^2 / ref.slew_time;
    angle_ddot = ref.angle * 60 * s * (1 - s) * (1 - 2 * s) / ref.slew_time^2;

    omega_d = ref.axis * angle_dot;
    omega_d_dot = ref.axis * angle_ddot;
end
