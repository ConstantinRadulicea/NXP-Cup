clc;
clear;
close all;
% delete(arduinoObj)

arduinoObj = serialport("COM6",115200);
arduinoObj.UserData.time_1 = tic;
arduinoObj.UserData.storedData = [];
arduinoObj.UserData.storedData.time = [];
arduinoObj.UserData.storedData.pixels = [];
configureTerminator(arduinoObj,"CR/LF");
flush(arduinoObj);
configureCallback(arduinoObj,"terminator",@read_callback_simulate_process);



