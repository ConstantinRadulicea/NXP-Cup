function update_car_view_scene(fig_handle, ...
                      middle_line, left_vector, right_vector, ...
                      left_vector_old, right_vector_old, ...
                      left_finish_line, right_finish_line, ...
                      car_position, lookahead_point_position, ...
                      steeringWheelAngle, ...
                      carAcceleration, ...
                      frontObstacleDistance, ...
                      lookAheadDistance, ...
                      carSpeedRaw, ...
                      g_finish_line_detected, ...
                      g_finish_line_detected_now, ...
                      g_loop_time_ms, ...
                      x_limits, y_limits)

    persistent h_initialized h_lines h_points h_labels axes_handle h_texts

    if isempty(h_initialized) || ~ishandle(h_lines(1)) || ~isgraphics(h_lines(1))
        figure(fig_handle); clf(fig_handle);
        axes_handle = axes('Parent', fig_handle);
        axis(axes_handle, 'equal');
        axis(axes_handle, [x_limits, y_limits]);
        grid(axes_handle, 'on'); hold(axes_handle, 'on');

        xlabel(axes_handle, 'X [cm]');
        ylabel(axes_handle, 'Y [cm]');
        title(axes_handle, 'Car View Scene');

        % Define line specs
        line_specs = {
            {'k-', 2}, {'b-', 2}, {'r-', 2}, ...
            {'b--', 1}, {'r--', 1}, ...
            {'g-', 2}, {'g-', 2}
        };

        % Create line handles
        for i = 1:7
            h_lines(i) = plot(axes_handle, [0 0], [0 0], ...
                              line_specs{i}{1}, 'LineWidth', line_specs{i}{2});
        end

        % Points: car and lookahead
        h_points(1) = plot(axes_handle, 0, 0, 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
        h_points(2) = plot(axes_handle, 0, 0, 'co', 'MarkerSize', 8, 'MarkerFaceColor', 'c');

        % Labels (legend-like descriptions)
        h_labels = gobjects(1, 7);
        label_names = {
            'Middle Line', 'Left Vector', 'Right Vector', ...
            'Left Vector (Old)', 'Right Vector (Old)', ...
            'Left Finish Line', 'Right Finish Line'
        };
        for i = 1:7
            h_labels(i) = text(axes_handle, x_limits(2) - 30, y_limits(2) - 5 * i, ...
                               label_names{i}, 'Color', get(h_lines(i), 'Color'));
        end

        % Dynamic info texts
        xmin = x_limits(1);
        ymax = y_limits(2);
        h_texts = gobjects(1, 8);
        for i = 1:8
            h_texts(i) = text(axes_handle, xmin, ymax - (i * 4), '', ...
                              'FontSize', 9, 'Color', 'k', ...
                              'VerticalAlignment', 'top');
        end

        h_initialized = true;
    end

    % Keep axis limits fixed
    axis(axes_handle, [x_limits, y_limits]);

    % Vectors to update
    vectors = {
        middle_line, left_vector, right_vector, ...
        left_vector_old, right_vector_old, ...
        left_finish_line, right_finish_line
    };

    for i = 1:7
        set(h_lines(i), 'XData', [vectors{i}(1,1), vectors{i}(2,1)], ...
                        'YData', [vectors{i}(1,2), vectors{i}(2,2)]);
    end

    % Update car and lookahead points
    set(h_points(1), 'XData', car_position(1), 'YData', car_position(2));
    set(h_points(2), 'XData', lookahead_point_position(1), 'YData', lookahead_point_position(2));

    % Update text labels with current values
    set(h_texts(1), 'String', sprintf('Steering Angle: %.2f°', steeringWheelAngle * (180/pi)));
    set(h_texts(2), 'String', sprintf('Acceleration: %.2f%%', carAcceleration * 100));
    set(h_texts(3), 'String', sprintf('Obstacle Distance: %.3f m', frontObstacleDistance));
    set(h_texts(4), 'String', sprintf('Lookahead Distance: %.2f cm', lookAheadDistance));
    set(h_texts(5), 'String', sprintf('Car Speed: %.2f m/s', carSpeedRaw));
    set(h_texts(6), 'String', sprintf('Finish Line Detected: %d', g_finish_line_detected));
    set(h_texts(7), 'String', sprintf('Finish Line Now: %d', g_finish_line_detected_now));
    set(h_texts(8), 'String', sprintf('Loop Time: %d ms', g_loop_time_ms));

    % Update title with dynamic info (optional)
    title(axes_handle, sprintf('Speed: %.2f m/s | Loop Time: %d ms', carSpeedRaw, g_loop_time_ms));

    drawnow;
end
