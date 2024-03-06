recordTerminator = sprintf('\r\n');
fieldTerminator = ';';

lane_width_vector_unit_real = 60.0;
lookahead_min_distance_cm = 16.0;
lookahead_max_distance_cm = 30.0;
emergency_break_distance_cm = 30.0;
min_speed = 96.0;
max_speed = 105.0;
black_color_treshold = 0.2;
car_length_cm = 17.5;
enable_car_engine = 1.0;
enable_car_steering_wheel = 1.0;
emergency_brake_min_speed = 94;
emergency_brake_distance_from_obstacle_cm = 9;

values = [lane_width_vector_unit_real lookahead_min_distance_cm...
    lookahead_max_distance_cm emergency_break_distance_cm...
    min_speed max_speed black_color_treshold car_length_cm...
    enable_car_engine...
    enable_car_steering_wheel...
    emergency_brake_min_speed...
    emergency_brake_distance_from_obstacle_cm];

outputString = '';

for i = 1:length(values)
    outputString = [outputString sprintf('%.2f%c', values(i), fieldTerminator)];
end

if ~isempty(outputString)
    outputString = outputString(1:end-1);
end

outputString = [outputString recordTerminator];

server.write(outputString)


