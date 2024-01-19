function read_callback_simulate_process(src, ~)
    time_2 = toc(src.UserData.time_1);
    
    raw_data = readline(src);
    raw_data_chars = convertStringsToChars(raw_data);
    if(raw_data_chars(1) == '%')
        return;
    end
    raw_data = split(raw_data, ";");
    leftLine_str = split(raw_data(1,1), ",");
    rightLine_str = split(raw_data(2,1), ",");
    middleLaneLine_str = split(raw_data(3,1), ",");
    carPosition_str = split(raw_data(4,1), ",");
    newWayPointPosition_str = split(raw_data(5,1), ",");
    steeringWheelAngle = str2double(raw_data(6,1));

    leftLine = str2double(leftLine_str(:, 1))';
    rightLine = str2double(rightLine_str(:, 1))';
    middleLaneLine = str2double(middleLaneLine_str(:, 1))';
    carPosition = str2double(carPosition_str(:, 1))';
    newWayPointPosition = str2double(newWayPointPosition_str(:, 1))';

%     f_leftLine = @(x,y) x.*leftLine(1) + y.*leftLine(2) + leftLine(3);
%     f_rightLine = @(x,y) x.*rightLine(1) + y.*rightLine(2) + rightLine(3);
%     f_middleLaneLine = @(x,y) x.*middleLaneLine(1) + y.*middleLaneLine(2) + middleLaneLine(3);
    
    xmin = 0;
    xmax = 80;

    [x1, y1] = plotLineABC(leftLine, xmin, xmax, xmin, xmax);
    [x2, y2] = plotLineABC(rightLine, xmin, xmax, xmin, xmax);
    [x3, y3] = plotLineABC(middleLaneLine, xmin, xmax, xmin, xmax);

    plot(x1, y1, x2, y2, x3, y3, carPosition(1), carPosition(2), "^", newWayPointPosition(1), newWayPointPosition(2), "*");

    xlim([xmin xmax])
    ylim([xmin xmax])

end