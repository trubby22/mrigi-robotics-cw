function plot_obj_angular_speed(obj_speed, name)
    global file_format;
    % global gcf;
    figure;
    plot(obj_speed, '-');
    xlabel('time steps [au]');
    ylabel('angular speed [deg / s]');
    title(name);
    saveFigureAsPDF(gcf, name + file_format);
end