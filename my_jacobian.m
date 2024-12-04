function [res] = my_jacobian(joint_angles_sym, current_joint_angle_vector, transformation_matrix_sym, epsilon)
    f = @(x) calc_pose(joint_angles_sym, x, transformation_matrix_sym);
    n = size(joint_angles_sym, 2);  % Number of variables
    m = 7;  % Number of functions
    J = zeros(m, n);

    for j = 1:n
        % Perturb the j-th variable and compute the finite difference
        delta = zeros(1, n);
        delta(j) = epsilon;
        res_perturbed = f(current_joint_angle_vector + delta);
        res_normal = f(current_joint_angle_vector);
        for i = 1:m
            J(i, j) = (res_perturbed(i) - res_normal(i)) / epsilon;
        end
    end

    res = J;
end