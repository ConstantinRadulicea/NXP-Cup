function plot_wheel_speed(fig_handle, time_now, rpm_left, rpm_right)
    persistent h_ax h_line_left h_line_right h_initialized ...
               rpm_data_time rpm_data_left rpm_data_right ...
               h_text_legend

    % Check if any graphics object is missing or deleted
    if isempty(h_initialized) || ...
       ~isvalid(fig_handle) || ...
       isempty(h_ax) || ~isgraphics(h_ax) || ...
       isempty(h_line_left) || ~isgraphics(h_line_left) || ...
       isempty(h_line_right) || ~isgraphics(h_line_right)

        % Reinitialize everything
        figure(fig_handle); clf(fig_handle);

        h_ax = axes('Parent', fig_handle);
        hold(h_ax, 'on');
        grid(h_ax, 'on');
        xlabel(h_ax, 'Time [s]');
        ylabel(h_ax, 'm/s');
        title(h_ax, 'Left & Right Wheel speed [m/s]');

        h_line_left = animatedline(h_ax, 'Color', 'b', 'LineWidth', 1.5);
        h_line_right = animatedline(h_ax, 'Color', 'r', 'LineWidth', 1.5);

        % Manual legend
        h_text_legend(1) = text(h_ax, 0.01, 0.95, '● Left Wheel', ...
                                'Units', 'normalized', 'Color', 'b', ...
                                'FontSize', 10, 'FontWeight', 'bold');
        h_text_legend(2) = text(h_ax, 0.01, 0.90, '● Right Wheel', ...
                                'Units', 'normalized', 'Color', 'r', ...
                                'FontSize', 10, 'FontWeight', 'bold');

        rpm_data_time = [];
        rpm_data_left = [];
        rpm_data_right = [];

        h_initialized = true;
    end

    % Append new data
    rpm_data_time(end+1) = time_now;
    rpm_data_left(end+1) = rpm_left;
    rpm_data_right(end+1) = rpm_right;

    % Keep only last 10 seconds
    time_window = 10;
    valid_idx = rpm_data_time >= (time_now - time_window);

    rpm_data_time = rpm_data_time(valid_idx);
    rpm_data_left = rpm_data_left(valid_idx);
    rpm_data_right = rpm_data_right(valid_idx);

    % Update animated lines
    clearpoints(h_line_left);
    clearpoints(h_line_right);
    addpoints(h_line_left, rpm_data_time, rpm_data_left);
    addpoints(h_line_right, rpm_data_time, rpm_data_right);

    % Update axis limits
    min_rpm = min([rpm_data_left, rpm_data_right], [], 'omitnan');
    max_rpm = max([rpm_data_left, rpm_data_right], [], 'omitnan');
    if isempty(min_rpm), min_rpm = 0; end
    if isempty(max_rpm), max_rpm = 100; end
    ylim(h_ax, [floor(min_rpm - 1), ceil(max_rpm + 1)]);
    xlim(h_ax, [max(0, time_now - time_window), time_now]);

    drawnow limitrate;
end
