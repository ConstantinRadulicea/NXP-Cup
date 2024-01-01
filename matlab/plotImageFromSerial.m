clc;
clear;
close all;
% delete(arduinoObj)

frameWidth = 310;
frameHeight = 190;

arduinoObj = serialport("COM6",115200);
flush(arduinoObj);


img = zeros(frameHeight, frameWidth, 3);
img = uint8(img);
count = 0;
figure
imshow(img)

for y = 0:(frameHeight-1)
    for x = 0:(frameWidth-1)
        arduinoObj.write(x, "uint16");
        arduinoObj.write(y, "uint16");
        
        R = read(arduinoObj,1,"uint8");
        G = read(arduinoObj,1,"uint8");
        B = read(arduinoObj,1,"uint8");

        img(y+1, x+1, 1) = R;
        img(y+1, x+1, 2) = G;
        img(y+1, x+1, 3) = B;
        imshow(img)
        count = count + 1
    end
end









