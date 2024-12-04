function make_figure_power_point_size(gcf)
    set(gcf, 'PaperUnits', 'inches');  % Set units to inches
    set(gcf, 'PaperSize', [13.33, 7.5]); % Set the paper size (16:9 aspect ratio)
    set(gcf, 'PaperPosition', [0, 0, 13.33, 7.5]); % Fill the entire paper
end