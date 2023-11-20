function read_callback_simulate_process(src, ~)
    time_2 = toc(src.UserData.time_1);
    
    raw_data = readline(src);
    raw_data_chars = convertStringsToChars(raw_data);
    if(raw_data_chars(1) == '%')
        return;
    end
    raw_data = split(raw_data, ",");
    raw_data_1 = split(raw_data(1,1), ";");
    raw_data_2 = split(raw_data(2,1), ";")
    data = str2double(raw_data_1(:, 1));
    data2 = str2double(raw_data_2(:, 1));
    
    temp_data_2 = {transpose(data)};
    src.UserData.storedData.pixels = [src.UserData.storedData.pixels; temp_data_2];
    src.UserData.storedData.time = [src.UserData.storedData.time time_2];
    %to_store = src.UserData.storedData;
    %save('test.mat','to_store');
    x = 1:length(data);
    x2 = 1:length(data2);
    plot(x', data);
    hold on;
    plot(data2(1:2), 2, 'o', 'MarkerEdgeColor','b');
    hold on;
    plot(data2(3:4), 2, 'o', 'MarkerEdgeColor','r');
    hold off;
end