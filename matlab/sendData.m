recordTerminator = sprintf('\r\n');
fieldTerminator = ';';

lane_width_vector_unit_real = 60.0;
lookahead_min_distance_cm = 16.0;
lookahead_max_distance_cm = 30.0;
emergency_break_distance_cm = 60.0;
min_speed = 96.0;
max_speed = 112.0;
black_color_treshold = 0.2;
car_length_cm = 17.5;

values = [lane_width_vector_unit_real lookahead_min_distance_cm...
    lookahead_max_distance_cm emergency_break_distance_cm...
    min_speed max_speed black_color_treshold car_length_cm];

outputString = '';

for i = 1:length(values)
    outputString = [outputString sprintf('%.2f%c', values(i), fieldTerminator)];
end

if ~isempty(outputString)
    outputString = outputString(1:end-1);
end

outputString = [outputString recordTerminator];

server.write(outputString)


