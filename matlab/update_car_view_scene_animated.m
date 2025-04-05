function update_car_view_scene_animated(fig_handle, ...
                      middle_line, left_vector, right_vector, ...
                      left_vector_old, right_vector_old, ...
                      left_finish_line, right_finish_line, ...
                      car_position, lookahead_point_position, ...
                      x_limits, y_limits)

    persistent h_initialized h_lines h_points ax

    axis_limits = [x_limits,  y_limits];
  % Reset everything if the figure or axes are gone
    if isempty(h_initialized) || ~isvalid(fig_handle) || isempty(h_lines) || ~all(isvalid(h_lines))
        clf(fig_handle);
        ax = axes('Parent', fig_handle);
        axis(ax, 'equal');
        axis(ax, axis_limits);
        grid(ax, 'on'); hold(ax, 'on');

        % Create animated lines
        h_lines = gobjects(1, 7);  % Preallocate for safety
        h_lines(1) = animatedline('Parent', ax, 'Color', 'k', 'LineWidth', 2);
        h_lines(2) = animatedline('Parent', ax, 'Color', 'b', 'LineWidth', 2);
        h_lines(3) = animatedline('Parent', ax, 'Color', 'r', 'LineWidth', 2);
        h_lines(4) = animatedline('Parent', ax, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 1);
        h_lines(5) = animatedline('Parent', ax, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1);
        h_lines(6) = animatedline('Parent', ax, 'Color', 'g', 'LineWidth', 2);
        h_lines(7) = animatedline('Parent', ax, 'Color', 'g', 'LineWidth', 2);

        % Create car and lookahead points
        h_points(1) = plot(ax, 0, 0, 'mo', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
        h_points(2) = plot(ax, 0, 0, 'co', 'MarkerSize', 8, 'MarkerFaceColor', 'c');

        legend(h_lines, {'Middle Line', 'Left Vector', 'Right Vector', ...
                         'Left Vector (Old)', 'Right Vector (Old)', ...
                         'Left Finish Line', 'Right Finish Line'});

        h_initialized = true;
    end

    % Line vector format: [x1 y1 x2 y2]
    vectors = {middle_line, left_vector, right_vector, ...
               left_vector_old, right_vector_old, ...
               left_finish_line, right_finish_line};

    for i = 1:7
        if isvalid(h_lines(i))
            clearpoints(h_lines(i));
            addpoints(h_lines(i), [vectors{i}(1,1), vectors{i}(2,1)], [vectors{i}(1,2), vectors{i}(2,2)]);
        end
    end

    % Update point markers
    set(h_points(1), 'XData', car_position(1), 'YData', car_position(2));
    set(h_points(2), 'XData', lookahead_point_position(1), 'YData', lookahead_point_position(2));

    % Reset axis each time
    axis(ax, axis_limits);

    drawnow limitrate;
end