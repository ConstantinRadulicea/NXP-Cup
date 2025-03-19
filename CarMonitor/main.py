import socket
from time import sleep

from matplotlib import pyplot as plt

import ParseData
import PlotData

# === Live Update Loop ===
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots(figsize=(6, 6))

record = ""
parsed_data = {}

def process_data(raw_packet):
    parsed_data = {}
    try:
        parsed_data = ParseData.process_raw_data(raw_packet)

    except Exception as e:
        parsed_data = {}
        print("corrupt_data: ", e)
    else:
        try:
            PlotData.update_plot(parsed_data, ax)
            plt.pause(0.5)  # Update every 0.5 seconds
        except Exception as e:
            print("plot_error: ", e)

    print("Received packet:", parsed_data)


def tcp_client(host: str, port: int):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        try:
            client_socket.connect((host, port))
            print(f"Connected to {host}:{port}")
            buffer = ""

            while True:
                data = client_socket.recv(1024).decode('utf-8')
                if not data:
                    break
                buffer += data

                while "\r\n" in buffer:
                    packet, buffer = buffer.split("\r\n", 1)
                    process_data(packet)

        except ConnectionError as e:
            print("Connection error:", e)
        finally:
            print("Disconnected from server")


if __name__ == "__main__":

    #tcp_client("127.0.0.1", 6789)  # Change IP and port as needed
    packet = "8,0,13,88;59,0,68,83;8,0,13,88;59,0,68,83;-17.600,1.000,140.800;-9.222,1.000,544.111;-12.110,1.000,405.056;39.500,0.000;36.484,36.794;0.035;1.000;1.321;38.426;2.000;0;32,20,22,21;0,0,0,0;1;16.000;43.436;0.000;28.826;0.000;0.000;0.000;0.000;0.000;0.000;0.000;0.000"
    while True:
        process_data(packet)
