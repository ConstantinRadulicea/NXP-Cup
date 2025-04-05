% Initialization
figure;
axis([-10 10 -10 10]);
hold on;
grid on;

% Create 6 line objects (placeholders)
lines = gobjects(1, 6);
for i = 1:6
    lines(i) = plot([0, 0], [0, 0], 'LineWidth', 2); % initial dummy segment
end

% Main loop - 60 Hz update rate
dt = 1/60;
for t = 1:600 % run for 10 seconds (600 frames at 60 Hz)
    for i = 1:6
        % Example: rotate 6 segments around origin
        angle = (t + i*60) * pi/180;
        x1 = 0;
        y1 = 0;
        x2 = 5 * cos(angle);
        y2 = 5 * sin(angle);
        
        % Update line coordinates
        set(lines(i), 'XData', [x1 x2], 'YData', [y1 y2]);
    end
    
    drawnow;
    pause(dt); % maintain 60 Hz
end
