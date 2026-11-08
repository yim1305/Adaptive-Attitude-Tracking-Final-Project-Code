function q = axis_angle_quat(axis, angle)
    axis = axis / norm(axis);
    q = [axis * sin(angle / 2); cos(angle / 2)];
end
