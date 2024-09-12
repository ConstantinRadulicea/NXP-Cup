function read_callback_serialport(src, ~)
    persistent startTime
    
    if isempty(startTime)
        startTime = tic;
        return
    end
    raw_data = src.readline();
    raw_data_chars = char(raw_data);

    if(raw_data_chars(1) == '%')
         disp(raw_data)
%         src.UserData.Debug = [raw_data; src.UserData.Debug];
%         src.UserData.Debug = src.UserData.Debug(1:min(end, 50), 1);
        return;
    end

    % if(toc(startTime) < 1)
    %     return;
    % end
    % startTime = tic;

    raw_data = split(raw_data, ";");
    leftVectorOld_str = split(raw_data(1,1), ",");
    rightVectorOld_str = split(raw_data(2,1), ",");
    leftVector_str = split(raw_data(3,1), ",");
    rightVector_str = split(raw_data(4,1), ",");
    leftLine_str = split(raw_data(5,1), ",");
    rightLine_str = split(raw_data(6,1), ",");
    middleLaneLine_str = split(raw_data(7,1), ",");
    carPosition_str = split(raw_data(8,1), ",");
    newWayPointPosition_str = split(raw_data(9,1), ",");
    steeringWheelAngle = str2double(raw_data(10,1));
    carAcceleration = str2double(raw_data(11,1));
    frontObstacleDistance = str2double(raw_data(12,1));
    lookAheadDistance = str2double(raw_data(13,1));
    carSpeedRaw = str2double(raw_data(14,1));
    g_finish_line_detected = str2double(raw_data(15,1));
    finish_line_left_segment_str = split(raw_data(16,1), ",");
    finish_line_right_segment_str = split(raw_data(17,1), ",");
    g_finish_line_detected_now = str2double(raw_data(18,1));
    g_loop_time_ms = str2double(raw_data(19,1));

    left_wheel_raw_rpm = str2double(raw_data(20,1));
    left_wheel_adjusted_rpm = str2double(raw_data(21,1));
    right_wheel_raw_rpm = str2double(raw_data(22,1));
    right_wheel_adjusted_rpm = str2double(raw_data(23,1));
    left_wheel_speed_request_raw = str2double(raw_data(24,1));
    right_wheel_speed_request_raw = str2double(raw_data(25,1));
    
    sample_batch_size = length(src.UserData.wheels_rpm.left.raw_rpm) + 1;
    sample_batch_max_size = 1500;
    sample_batch_size = min(sample_batch_size, sample_batch_max_size);

    src.UserData.wheels_rpm.left.raw_rpm(end+1) = left_wheel_raw_rpm;
    src.UserData.wheels_rpm.left.raw_rpm = src.UserData.wheels_rpm.left.raw_rpm(end - sample_batch_size + 1:end);
    src.UserData.wheels_rpm.left.adjusted_rpm(end+1) = left_wheel_adjusted_rpm;
    src.UserData.wheels_rpm.left.adjusted_rpm = src.UserData.wheels_rpm.left.adjusted_rpm(end - sample_batch_size + 1:end);
    src.UserData.wheels_rpm.right.raw_rpm(end+1) = right_wheel_raw_rpm;
    src.UserData.wheels_rpm.right.raw_rpm = src.UserData.wheels_rpm.right.raw_rpm(end - sample_batch_size + 1:end);
    src.UserData.wheels_rpm.right.adjusted_rpm(end+1) = right_wheel_adjusted_rpm;
    src.UserData.wheels_rpm.right.adjusted_rpm = src.UserData.wheels_rpm.right.adjusted_rpm(end - sample_batch_size + 1:end);
    src.UserData.left_wheel_speed_request_raw(end+1) = left_wheel_speed_request_raw;
    src.UserData.left_wheel_speed_request_raw = src.UserData.left_wheel_speed_request_raw(end - sample_batch_size + 1:end);
    src.UserData.right_wheel_speed_request_raw(end+1) = right_wheel_speed_request_raw;
    src.UserData.right_wheel_speed_request_raw = src.UserData.right_wheel_speed_request_raw(end - sample_batch_size + 1:end);
    
    if(toc(startTime) < 0.5)
        return;
    end
    startTime = tic;

    y_array = 1:1:length(src.UserData.wheels_rpm.left.raw_rpm);
    set(0, 'CurrentFigure', src.UserData.wheelRpm_figure)
    % plot(y_array, src.UserData.wheels_rpm.left.raw_rpm, 'DisplayName', 'LeftWheel_RawRpm');
    % hold on;
    plot(y_array, src.UserData.wheels_rpm.left.adjusted_rpm, 'DisplayName', 'LeftWheel_AdjustedRpm');
    hold on;
    % plot(y_array, src.UserData.wheels_rpm.right.raw_rpm, 'DisplayName', 'RightWheel_RawRpm');
    % hold on;
    plot(y_array, src.UserData.wheels_rpm.right.adjusted_rpm, 'DisplayName', 'RightWheel_AdjustedRpm');
    hold off;
    legend show;

    set(0, 'CurrentFigure', src.UserData.wheelSpeedRequestRaw_figure)
    plot(y_array, src.UserData.left_wheel_speed_request_raw, 'DisplayName', 'LeftWheel speed request RAW');
    hold on;
    plot(y_array, src.UserData.right_wheel_speed_request_raw, 'DisplayName', 'RightWheel speed request RAW');
    hold off;
    legend show;

    leftVectorOld = str2double(leftVectorOld_str(:, 1))';
    rightVectorOld = str2double(rightVectorOld_str(:, 1))';
    leftVector = str2double(leftVector_str(:, 1))';
    rightVector = str2double(rightVector_str(:, 1))';
    leftLine = str2double(leftLine_str(:, 1))';
    rightLine = str2double(rightLine_str(:, 1))';
    middleLaneLine = str2double(middleLaneLine_str(:, 1))';
    carPosition = str2double(carPosition_str(:, 1))';
    newWayPointPosition = str2double(newWayPointPosition_str(:, 1))';
    finish_line_left_segment = str2double(finish_line_left_segment_str(:, 1))';
    finish_line_right_segment = str2double(finish_line_right_segment_str(:, 1))';

%     f_leftLine = @(x,y) x.*leftLine(1) + y.*leftLine(2) + leftLine(3);
%     f_rightLine = @(x,y) x.*rightLine(1) + y.*rightLine(2) + rightLine(3);
%     f_middleLaneLine = @(x,y) x.*middleLaneLine(1) + y.*middleLaneLine(2) + middleLaneLine(3);
    
    xmin = 0;
    xmax = 80;
    ymin = -50;
    ymax = 80;

%     [x1, y1] = plotLineABC(leftLine, xmin, xmax, xmin, xmax);
%     [x2, y2] = plotLineABC(rightLine, xmin, xmax, xmin, xmax);
    
    [x3, y3] = plotLineABC(middleLaneLine, xmin, xmax, ymin, ymax);

%     plot([leftVectorOld(1) leftVectorOld(3)], [leftVectorOld(2) leftVectorOld(4)], "--o");
%     hold on;
%     plot([rightVectorOld(1) rightVectorOld(3)], [rightVectorOld(2) rightVectorOld(4)], "--o");
%     hold on;
%     plot([leftVector(1) leftVector(3)], [leftVector(2) leftVector(4)], [rightVector(1) rightVector(3)], [rightVector(2) rightVector(4)], x3, y3, carPosition(1), carPosition(2), "^", newWayPointPosition(1), newWayPointPosition(2), "*");
 % figure(1, 'Name','Camera view');
 set(0, 'CurrentFigure', src.UserData.cameraView_figure)
plot([leftVector(1) leftVector(3)], [leftVector(2) leftVector(4)], ...
        [rightVector(1) rightVector(3)], [rightVector(2) rightVector(4)], ...
        x3, y3, carPosition(1), carPosition(2), "^", ...
        newWayPointPosition(1), newWayPointPosition(2), "*", ...
        [leftVectorOld(1) leftVectorOld(3)], [leftVectorOld(2) leftVectorOld(4)], "--o", ...
        [rightVectorOld(1) rightVectorOld(3)], [rightVectorOld(2) rightVectorOld(4)], "--o", ...
        [finish_line_left_segment(1) finish_line_left_segment(3)], [finish_line_left_segment(2) finish_line_left_segment(4)], "-.s", ...
        [finish_line_right_segment(1) finish_line_right_segment(3)], [finish_line_right_segment(2) finish_line_right_segment(4)], "-.s");
    text(leftVector(1), leftVector(2), "1");
    text(leftVector(3), leftVector(4), "2");
    text(rightVector(1), rightVector(2), "1");
    text(rightVector(3), rightVector(4), "2");
    myText = sprintf('SteeringAngle: %.2f°', steeringWheelAngle * (180/pi));
    text(xmin, ymax-3, myText);
    myText = sprintf("Gas: %.2f%%", carAcceleration * 100);
    text(xmin, ymax-7, myText);
    myText = sprintf("Obstacle distance [m]: %.3f", frontObstacleDistance);
    text(xmin, ymax-11, myText);
    myText = sprintf("LookAheadDistance[cm]: %.2f", lookAheadDistance);
    text(xmin, ymax-15, myText);
    myText = sprintf("g_car_speed[raw]: %.2f", carSpeedRaw);
    text(xmin, ymax-19, myText);
    myText = sprintf("FinishLine[1/0]: %d", g_finish_line_detected);
    text(xmin, ymax-23, myText);
    myText = sprintf("FinishLineNow[1/0]: %d", g_finish_line_detected_now);
    text(xmin, ymax-27, myText);
    myText = sprintf("LoopTime[ms]: %d", g_loop_time_ms);
    text(xmin, ymax-31, myText);

    
    xlim([xmin xmax])
    ylim([ymin ymax])

end