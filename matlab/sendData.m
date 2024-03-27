recordTerminator = sprintf('\r\n');
fieldTerminator = ';';


enable_car_engine = 1.0;
enable_car_steering_wheel = 1.0;
enable_emergency_brake = 1.0;
enable_pixy_vector_approximation = 0.0;
enable_distance_sensor1 = 1.0;
enable_distance_sensor2 = 1.0;

lane_width_vector_unit_real = 60.0;
black_color_treshold = 0.2;
car_length_cm = 17.5;
lookahead_min_distance_cm = 22.0;                       % 22
lookahead_max_distance_cm = 45.0;                       % 40
min_speed = 97.0;
max_speed = 122.0;                                      % 115 merge si 120
emergency_break_distance_cm = 85;                     % 75
emergency_brake_min_speed = 93.0;
emergency_brake_distance_from_obstacle_cm = 60.0;       % 14

values = [lane_width_vector_unit_real lookahead_min_distance_cm...
    lookahead_max_distance_cm emergency_break_distance_cm...
    min_speed max_speed black_color_treshold car_length_cm...
    enable_car_engine...
    enable_car_steering_wheel...
    emergency_brake_min_speed...
    emergency_brake_distance_from_obstacle_cm...
    enable_emergency_brake...
    enable_pixy_vector_approximation...
    enable_distance_sensor1...
    enable_distance_sensor2];

outputString = '';

for i = 1:length(values)
    outputString = [outputString sprintf('%.2f%c', values(i), fieldTerminator)];
end

if ~isempty(outputString)
    outputString = outputString(1:end-1);
end

outputString = [outputString recordTerminator];

server.write(outputString)


