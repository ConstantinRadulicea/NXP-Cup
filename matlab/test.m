clc;
clear;
close all;

leftLine = [-10 2 3];
xmin = 0;
xmax = 100;

plotLineABC(leftLine, xmin, xmax, xmin, xmax);



xlim([xmin xmax])
ylim([xmin xmax])


theNumber = 10;
myText = sprintf('The number is %.2f', theNumber);
text(10, 10, myText);