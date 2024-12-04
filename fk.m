function [tip_pos] = fk(T, sym_thetas, joint_angles)
T_val = subs(T, sym_thetas, joint_angles);
T_val = double(vpa(T_val,12));
tip_pos = T_val * [0 0 0 1]';
tip_pos = tip_pos(1:3) .* 1000;
end