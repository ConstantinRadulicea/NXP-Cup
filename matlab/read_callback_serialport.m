function read_callback_serialport(src, ~)
    
    raw_data = readline(src);
    raw_data_chars = convertStringsToChars(raw_data);
    if(raw_data_chars(1) == '%')
         raw_data
%         src.UserData.Debug = [raw_data; src.UserData.Debug];
%         src.UserData.Debug = src.UserData.Debug(1:min(end, 50), 1);
        return;
    end
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
    

    leftVectorOld = str2double(leftVectorOld_str(:, 1))';
    rightVectorOld = str2double(rightVectorOld_str(:, 1))';
    leftVector = str2double(leftVector_str(:, 1))';
    rightVector = str2double(rightVector_str(:, 1))';
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

%     [x1, y1] = plotLineABC(leftLine, xmin, xmax, xmin, xmax);
%     [x2, y2] = plotLineABC(rightLine, xmin, xmax, xmin, xmax);
    
    [x3, y3] = plotLineABC(middleLaneLine, xmin, xmax, xmin, xmax);

%     plot(x1, y1, x2, y2, x3, y3, carPosition(1), carPosition(2), "^", newWayPointPosition(1), newWayPointPosition(2), "*");
    plot([leftVectorOld(1) leftVectorOld(3)], [leftVectorOld(2) leftVectorOld(4)], "--or");
    hold on;
    plot([rightVectorOld(1) rightVectorOld(3)], [rightVectorOld(2) rightVectorOld(4)], "--or");
    hold on;
    plot([leftVector(1) leftVector(3)], [leftVector(2) leftVector(4)], [rightVector(1) rightVector(3)], [rightVector(2) rightVector(4)], x3, y3, carPosition(1), carPosition(2), "^", newWayPointPosition(1), newWayPointPosition(2), "*");
    hold off;
    myText = sprintf('SteeringAngle: %.2f°', steeringWheelAngle * (180/pi));
    text(xmin, xmax-2, myText);
    myText = sprintf("Gas: %.2f%%", carAcceleration * 100);
    text(xmin, xmax-5, myText);
    xlim([xmin xmax])
    ylim([xmin xmax])

end