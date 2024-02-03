clc;
clear;
close all;

server = tcpserver("localhost",6789,"ConnectionChangedFcn",@newClientCallBack)
server.flush("input");
server.flush("output");
server.configureTerminator("CR/LF");
server.configureCallback("terminator", @read_callback_serialport)