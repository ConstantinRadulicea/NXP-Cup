function newClientCallBack(src,~)
if src.Connected
   disp("This message is sent by the server after accepting the client connection request.")
else
   disp("Client has disconnected.")
end
end

