function plot_obj_trajectory_task_2(obj_positions, name)
    global file_format;
    X = obj_positions(:, 1);
    Y = obj_positions(:, 2);
    Z = obj_positions(:, 3);
    figure;
    hold on;
    plot3(X, Y, Z, 'b', 'LineWidth', 2);
    xlabel('X-axis [mm]');
    ylabel('Y-axis [mm]');
    zlabel('Z-axis [mm]');
    title(name);
    grid on;
    hold off;
    % campos([300, 300, 300]);
    % camtarget([0, 0, 0]);
    % camup([0, 0, 1]); % Set the up vector along the Z-axis
    % camzoom(1.0);
    saveFigureAsPDF(gcf, name + file_format);
end