function [res] = calculate_angle_no_ee(joint_angles_sym, transformation_matrix_no_ee_sym, previous_joint_angle_vector, waypoint)

joint_angles_sym = joint_angles_sym(1:3);
previous_joint_angle_vector = previous_joint_angle_vector(1:3);
waypoint = waypoint(1:3);

num_joints = size(joint_angles_sym, 2);
num_pose_variables = size(waypoint, 2);

assert(isequal(size(joint_angles_sym), [1,num_joints]))
assert(isequal(size(transformation_matrix_no_ee_sym), [4, 4]))
assert(isequal(size(previous_joint_angle_vector), [1,num_joints]))
assert(isequal(size(waypoint), [1,num_pose_variables]))

current_joint_angle_vector = previous_joint_angle_vector;
angle_constraints = [
    -110, 160;
    -35, 70;
    -120, 60;
    -180, 145;
    -200, 30;
    -360, 360;
];
angle_constraints_in_radians = deg2rad(angle_constraints);
angle_constraints_in_radians = angle_constraints_in_radians(1:3, :);
assert(isequal(size(angle_constraints_in_radians), [num_joints, 2]))
tip_pos_numeric = calc_pose(joint_angles_sym, current_joint_angle_vector, transformation_matrix_no_ee_sym);
tip_pos_numeric = tip_pos_numeric(1:num_pose_variables);
assert(isequal(size(tip_pos_numeric), [1,num_pose_variables]))

dist_to_waypoint_vector = waypoint-tip_pos_numeric;
assert(isequal(size(dist_to_waypoint_vector), [1,num_pose_variables]))
dist_to_waypoint_scalar = norm(dist_to_waypoint_vector);
counter = 0;

while dist_to_waypoint_scalar > 1e-12
    
    if counter >= 10
        error("newton raphson failed to converge");
    end
    
    epsilon = 1e-6;
    jacobian_numeric = my_jacobian(joint_angles_sym, current_joint_angle_vector, transformation_matrix_no_ee_sym, epsilon);
    jacobian_numeric = jacobian_numeric(1:3, :);
    assert(isequal(size(jacobian_numeric), [num_pose_variables,num_joints]))
    jacobian_condition_num = cond(jacobian_numeric);
    if jacobian_condition_num > 1e6
        disp('Warning: Jacobian is ill-conditioned!');
    end
    jacobian_numeric_inverse = pinv(jacobian_numeric);
    assert(isequal(size(jacobian_numeric_inverse), [num_joints,num_pose_variables]))

    delta_joint_angles = jacobian_numeric_inverse * dist_to_waypoint_vector';
    delta_joint_angles = delta_joint_angles';
    assert(isequal(size(delta_joint_angles), [1,num_joints]))
    delta_joint_angles_norm = norm(delta_joint_angles);
    current_joint_angle_vector = previous_joint_angle_vector + delta_joint_angles;
    assert(isequal(size(current_joint_angle_vector), [1,num_joints]))
    for i = 1:size(current_joint_angle_vector,2)
        check_upper_constraint = min(current_joint_angle_vector(i), angle_constraints_in_radians(i,2));
        check_upper_and_lower_constraint = max(check_upper_constraint,angle_constraints_in_radians(i,1));
        assert(isequal(size(check_upper_and_lower_constraint), [1,1]))
        current_joint_angle_vector(i) = check_upper_and_lower_constraint;
    end
    assert(isequal(size(current_joint_angle_vector), [1,num_joints]))
    
    tip_pos_numeric = calc_pose(joint_angles_sym, current_joint_angle_vector, transformation_matrix_no_ee_sym);
    tip_pos_numeric = tip_pos_numeric(1:num_pose_variables);
    assert(isequal(size(tip_pos_numeric), [1,num_pose_variables]))
    dist_to_waypoint_vector = waypoint-tip_pos_numeric;
    assert(isequal(size(dist_to_waypoint_vector), [1,num_pose_variables]))
    dist_to_waypoint_scalar = norm(dist_to_waypoint_vector);     
    
    previous_joint_angle_vector = current_joint_angle_vector;
    assert(isequal(size(previous_joint_angle_vector), [1,num_joints]))
    counter = counter + 1;
    fprintf('dist_to_waypoint_scalar = %d \t delta_joint_angles_norm = %d\n', dist_to_waypoint_scalar, delta_joint_angles_norm);
end

fprintf('iterations taken for Newton-Raphson to converge: %d', counter);
res = current_joint_angle_vector;
assert(isequal(size(res), [1,num_joints]))

end