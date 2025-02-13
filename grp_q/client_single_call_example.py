import socket
import threading
import time
import random
import sys
import signal

# Robot 2 (Client)
def robot2_client_onetime(server_ip):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((server_ip, 5000))
    print("Connected to DDBoat 17 (Server)")

    # def signal_handler(sig, frame):
    #     print("Closing client connection gracefully...")
    #     client_socket.close()
    #     sys.exit(0)
    
    # signal.signal(signal.SIGINT, signal_handler)

    data = ""
    try:
        while True:
            data = client_socket.recv(1024).decode()
            if not data:
                break
            print("Received GPS Position from DDBoat 17: {}".format(data))
            break
    except Exception as e:
        print("Client error: {}".format(e))
    finally:
        client_socket.close()
    return data

if __name__ == "__main__":
    # IP address of the server (e.g. 172.20.25.217 for DDBoat 17)
    gps_pos = robot2_client_onetime("172.20.25.216")
    #gps_pos = robot2_client_onetime("172.20.25.221") # test ddboat
    print ("GPS Position",gps_pos)


