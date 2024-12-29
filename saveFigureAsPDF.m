function saveFigureAsPDF(figHandle, filename)
    if false
        % Apply specific settings for the given figure
        set(figHandle, 'PaperUnits', 'inches');
        set(figHandle, 'PaperSize', [13.33, 7.5]); % Standard PowerPoint size
        set(figHandle, 'PaperPosition', [0, 0, 13.33, 7.5]);
        
        % Save the figure as a PDF
        saveas(figHandle, filename);
    end
end