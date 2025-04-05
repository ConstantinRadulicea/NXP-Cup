fig = figure('Name', 'Car View Scene');

for t = 1:100
    theta = t * 0.05;
    
    middle_line        = [0, 0; 10*cos(theta), 10*sin(theta)];
    left_vector        = [0, 0; 10*cos(theta + 0.1), 10*sin(theta + 0.1)];
    right_vector       = [0, 0; 10*cos(theta - 0.1), 10*sin(theta - 0.1)];
    left_vector_old    = [0, 0; 10*cos(theta + 0.2), 10*sin(theta + 0.2)];
    right_vector_old   = [0, 0; 10*cos(theta - 0.2), 10*sin(theta - 0.2)];
    left_finish_line   = [5, -5; 5, 5];
    right_finish_line  = [-5, -5; -5, 5];
    
    car_position = [2*cos(theta), 2*sin(theta)];
    lookahead_point_position = [5*cos(theta), 5*sin(theta)];
    
    update_car_view_scene(fig, ...
        middle_line, left_vector, right_vector, ...
        left_vector_old, right_vector_old, ...
        left_finish_line, right_finish_line, ...
        car_position, lookahead_point_position);
    
    pause(1/60); % ~60Hz
end
