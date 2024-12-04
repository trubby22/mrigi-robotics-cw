function plot_obj_trajectory_task_4(obj_positions, name)
    global file_format;
    % global gcf;
    X = obj_positions(:, 1);
    Y = obj_positions(:, 2);
    Z = obj_positions(:, 3);
    figure;
    hold on;
    plot(Y, Z, 'b', 'LineWidth', 2);
    line_width = 10;
    xlabel('Y-axis [mm]');
    ylabel('Z-axis [mm]');
    % zlabel('Z-axis [mm]');
    title(name);
    grid on;
    hold off;
    saveFigureAsPDF(gcf, name + file_format);
end