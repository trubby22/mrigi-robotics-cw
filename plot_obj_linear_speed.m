function plot_obj_linear_speed(obj_speed, name)
    global file_format;
    % global gcf;
    figure;
    plot(obj_speed, '-');
    xlabel('time steps [au]');
    ylabel('linear speed [mm / s]');
    title(name);
    saveFigureAsPDF(gcf, name + file_format);
end