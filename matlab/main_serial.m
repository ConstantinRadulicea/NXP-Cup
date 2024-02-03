% delete(arduinoObj)
clc;
clear;
close all;


arduinoObj = serialport("COM4",115200);
arduinoObj.UserData.Debug = ["ciao"];
configureTerminator(arduinoObj,"CR/LF");
flush(arduinoObj);
configureCallback(arduinoObj,"terminator",@read_callback_serialport);

