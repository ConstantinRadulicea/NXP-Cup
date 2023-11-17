clear;
clc;

data = load('test.mat');

length(data.to_store.pixels)
a = 1;
coef = 1/4;
b = [coef coef coef coef];


for i = 1:length(data.to_store.pixels)
    y = cell2mat(data.to_store.pixels(i));
    y2 = movmean(y,4);
    x = 1:length(y);
    
    y3 = ones(1, length(y));
    y3 = y3 .* 180;

    plot(x,y3)
      hold on;
    plot(x,y2,'-')
      hold on;
      plot(x, y);
      hold off;
    pause(0.1);
end
