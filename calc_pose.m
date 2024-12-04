function [res] = calc_pose(joint_angles_sym, current_joint_angle_vector, transformation_matrix_sym)
    transformation_matrix_numeric = subs(transformation_matrix_sym, joint_angles_sym, current_joint_angle_vector);
    transformation_matrix_numeric = double(vpa(transformation_matrix_numeric,12));
    assert(isequal(size(transformation_matrix_numeric), [4, 4]))

    tip_xyz_coords_numeric = transformation_matrix_numeric(1:3, 4);
    tip_xyz_coords_numeric = tip_xyz_coords_numeric';
    rotm_numeric = transformation_matrix_numeric(1:3, 1:3);
    tip_quat_numeric = rotm2quat(rotm_numeric);
    tip_pos_numeric = cat(2, tip_xyz_coords_numeric, tip_quat_numeric);
    res = tip_pos_numeric;
end