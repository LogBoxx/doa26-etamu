% Name - DoA_demo_StaticData.m
% Author - Alden Edwards (Mostly Google Gemini)
% Date - 03/06/2026
% Notes - How does this project help the Jews?


% 1. Configuration & Loading
filename = "C:\Users\alden\DoAzimuthents\CapStone\2026 DoA Capstone Azimith DoA Compare_2-27-2026.xlsx";
opts = detectImportOptions(filename);
opts.VariableNamingRule = 'preserve';
data = readtable(filename, opts);

% Plotting Constants
FONT_SIZE = 10;
MARKER_SIZE = 40;
COLOR_PRIMARY = [0, 0.4470, 0.7410]; % Blue
REF_COLOR = [0.5, 0.5, 0.5];         % Grey

%% 2. Generate ULA Figures (2x2 Grid)
% ULA Azimuth is 0-180, Elevation is 0-90
ula_scenarios = {'Outside', 'Inside'};

for i = 1:length(ula_scenarios)
    scenario = ula_scenarios{i};
    figure('Name', ['ULA Results - ' scenario], 'Color', 'w', ...
           'Units', 'normalized', 'Position', [0.1, 0.1, 0.45, 0.7]);
    
    subplot(2, 2, 1);
    plot_doa_data(data, 'ULA', 'MUSIC', 'AZ', scenario, FONT_SIZE, MARKER_SIZE, COLOR_PRIMARY, REF_COLOR);
    
    subplot(2, 2, 2);
    plot_doa_data(data, 'ULA', 'ESPRIT', 'AZ', scenario, FONT_SIZE, MARKER_SIZE, COLOR_PRIMARY, REF_COLOR);
    
    subplot(2, 2, 3);
    plot_doa_data(data, 'ULA', 'MUSIC', 'EL', scenario, FONT_SIZE, MARKER_SIZE, COLOR_PRIMARY, REF_COLOR);
    
    subplot(2, 2, 4);
    plot_doa_data(data, 'ULA', 'ESPRIT', 'EL', scenario, FONT_SIZE, MARKER_SIZE, COLOR_PRIMARY, REF_COLOR);
    
    sgtitle(['ULA DoA Performance: ', scenario], 'FontSize', FONT_SIZE+4, 'FontWeight', 'bold');
end

%% 3. Generate UCA Figure (1x2 Grid) - Phase Wrapped
% Single figure with Outdoor Azimuth on Left, Indoor Azimuth on Right
% Bounds set to [-180, 180]
figure('Name', 'UCA Azimuth Results (Wrapped)', 'Color', 'w', ...
       'Units', 'normalized', 'Position', [0.2, 0.3, 0.6, 0.4]);

% Subplot 1: UCA Outdoor Azimuth
subplot(1, 2, 1);
plot_doa_data(data, 'UCA', 'MUSIC', 'AZ', 'Outside', FONT_SIZE, MARKER_SIZE, COLOR_PRIMARY, REF_COLOR);

% Subplot 2: UCA Indoor Azimuth
subplot(1, 2, 2);
plot_doa_data(data, 'UCA', 'MUSIC', 'AZ', 'Inside', FONT_SIZE, MARKER_SIZE, COLOR_PRIMARY, REF_COLOR);

sgtitle('UCA Azimuth Performance Comparison (Normalized \pm180°)', 'FontSize', FONT_SIZE+4, 'FontWeight', 'bold');

%% Helper Function for Standardized Plotting
function plot_doa_data(data, array, algo, dim, inout, fs, ms, cp, rc)
    % Filter data for specific case
    idx = strcmp(data.Array, array) & ...
          strcmp(data.Algorithm, algo) & ...
          strcmp(data.('AZ/EL'), dim) & ...
          strcmp(data.('In/Out'), inout);
    
    subTable = data(idx, :);
    hold on;
    
    % --- Determine Axis Limits and Apply Wrapping ---
    if strcmp(dim, 'AZ')
        trueCol = 'True AZ DOA (°)';
        dimLabel = 'Azimuth';
        
        if strcmp(array, 'UCA')
            % UCA Azimuth: Normalize 0-360 to [-180, 180]
            axLim = [-180, 180];
            if ~isempty(subTable)
                trueVal = mod(subTable.(trueCol) + 180, 360) - 180;
                estVal  = mod(subTable.('Mean Est. DOA (°)') + 180, 360) - 180;
            end
        else
            % ULA Azimuth: Standard 0-180
            axLim = [-90, 180];
            if ~isempty(subTable)
                trueVal = subTable.(trueCol);
                estVal  = subTable.('Mean Est. DOA (°)');
            end
        end
    else
        % Elevation: Standard 0-90
        trueCol = 'True EL DOA (°)';
        dimLabel = 'Elevation';
        axLim = [-45, 90];
        if ~isempty(subTable)
            trueVal = subTable.(trueCol);
            estVal  = subTable.('Mean Est. DOA (°)');
        end
    end
    
    % Plot 1:1 Reference Line (Ideal)
    plot(axLim, axLim, '--', 'Color', rc, 'LineWidth', 1, 'HandleVisibility', 'off');
    
    if ~isempty(subTable)
        % Scatter Plot
        scatter(trueVal, estVal, ms, 'filled', 'MarkerFaceColor', cp, 'MarkerEdgeColor', 'k');
    else
        % Placeholder if no data exists
        text(mean(axLim), mean(axLim), 'No Data Found', ...
            'HorizontalAlignment', 'center', 'FontSize', fs-2, 'Color', [0.7 0.7 0.7]);
    end
    
    % Formatting
    title(sprintf('%s %s %s (%s)', array, algo, dimLabel, inout), 'FontSize', fs);
    xlabel(['True ', dimLabel, ' (°)'], 'FontSize', fs-1);
    ylabel(['Est. ', dimLabel, ' (°)'], 'FontSize', fs-1);
    xlim(axLim); ylim(axLim);
    grid on; box on;
    
    % Standardize Ticks
    if strcmp(array, 'UCA') && strcmp(dim, 'AZ')
        set(gca, 'XTick', -180:90:180, 'YTick', -180:90:180);
    else
        set(gca, 'XTick', linspace(axLim(1), axLim(2), 5));
    end
    set(gca, 'FontSize', fs-1);
end