close all;
clc;
clear;

car_ip_address = '192.168.124.223';
% car_ip_address = '192.168.124.128';

car_port = 6789;

server = tcpclient(car_ip_address,car_port)
server.UserData.lastFlushed = 1;
server.UserData.cameraView_figure = figure('Name','cameraView');
% server.UserData.wheelRpm_figure = figure('Name','Wheels Rpm');
% server.UserData.wheelSpeedRequestRaw_figure = figure('Name','Wheels speed request Raw');
server.UserData.wheelSpeedRequest_figure = figure('Name','Wheels speed request');
server.UserData.gyro_figure = figure('Name','Gyro');



server.UserData.wheels_rpm.left.raw_rpm = [];
server.UserData.wheels_rpm.left.adjusted_rpm = [];
server.UserData.wheels_rpm.right.raw_rpm = [];
server.UserData.wheels_rpm.right.adjusted_rpm = [];
server.UserData.wheels_rpm.right.adjusted_rpm = [];


server.UserData.left_wheel_speed_request_raw = [];
server.UserData.right_wheel_speed_request_raw = [];

server.configureTerminator("CR/LF");
server.configureCallback("terminator", @read_callback_serialport);


