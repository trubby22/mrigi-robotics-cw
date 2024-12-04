function plot_obj_trajectory_task_3(obj_positions, plot_markers, name)
    global file_format;
    % global gcf;
    X = obj_positions(:, 1);
    Y = obj_positions(:, 2);
    Z = obj_positions(:, 3);
    figure;
    hold on;
    plot(X, Z, 'b', 'LineWidth', 2);
    line_width = 10;
    if plot_markers
        plot(150, 330, 'ro', 'MarkerSize', 1, 'LineWidth', line_width);
        plot(130, 20, 'co', 'MarkerSize', 1, 'LineWidth', line_width);
        plot(285, 95, 'mo', 'MarkerSize', 1, 'LineWidth', line_width);
    end
    xlabel('X-axis [mm]');
    ylabel('Z-axis [mm]');
    title(name);
    if plot_markers
        legend('CoppeliaSim trajectory', 'P_0', 'P_1', 'P_2');
    end
    grid on;
    hold off;
    saveFigureAsPDF(gcf, name + file_format);
end