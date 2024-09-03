clc;
clear;
close all;

server = tcpclient(car_ip_address,car_port, "ConnectionChangedFcn",@newClientCallBack)
server.UserData.lastFlushed = 1;
server.UserData.figureHandle = figure;
server.UserData.wheelRpm_figure = figure('Name','Wheels Rpm');



server.UserData.wheels_rpm.left.raw_rpm = [];
server.UserData.wheels_rpm.left.adjusted_rpm = [];
server.UserData.wheels_rpm.right.raw_rpm = [];
server.UserData.wheels_rpm.right.adjusted_rpm = [];

server.configureTerminator("CR/LF");
server.configureCallback("terminator", @read_callback_serialport);


