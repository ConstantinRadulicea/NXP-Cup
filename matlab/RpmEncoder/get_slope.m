
start_interval = 1;
stop_interval = 46000;

x_values = raw_throttle(start_interval:stop_interval);
y_values = adjusted_rpm(start_interval:stop_interval);

x_steps = 90:1:max(x_values);
average_values = [];
for i=x_steps
    positions = x_values == i;
    values_at_positions = y_values(positions);
    values_at_positions = values_at_positions(20:end-20);
    average_values =[average_values mean(values_at_positions)];
end
p = polyfit(x_steps, average_values, 2);
y_fit = polyval(p, x_steps);

hold on;
% plot(x_steps,average_values, 'Color', 'Magenta', 'DisplayName','Slope')
% plot(x_steps,y_fit, 'Color', 'Magenta', 'DisplayName','Slope')
plot(timestamp,polyval(p, raw_throttle), 'Color', 'Magenta', 'DisplayName','simulated')
hold off;

% slope = (y_fit(end) - y_fit(1)) / (x_steps(end) - x_steps(1))
for i=1:length(average_values)
    fprintf("%f, ", average_values(i));
    if mod(i, 10) == 0
        fprintf("\n");
    end
end