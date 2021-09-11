# coding=utf-8
import socket
import sys
import datetime

LISTEN_ON_PORT  = 3333

def main():
    
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Bind the socket to the port
    server_address = ('0.0.0.0', LISTEN_ON_PORT)
    print('starting up on {} port {}'.format(*server_address))
    sock.bind(server_address)

    # 打开文件
    now_time=datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    print(now_time)
    file_name = "data-"+now_time+".csv"
    fo = open(file_name, "ab+")
    fo.close()

    

    # # 关闭文件
    # fo.close()
    try:
        while True:
            print('\nwaiting to receive message')
            data, address = sock.recvfrom(4096)

            print('received {} bytes from {}'.format(
                len(data), address))
            print(data)
            
            if data:    
                sent = sock.sendto(data, address)
                print('sent {} bytes back to {}'.format(
                    sent, address))
                fo = open(file_name, "ab+")
                fo.write( data )
                fo.close()
            
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
        fo.close()

if __name__ == "__main__":
    # execute only if run as a script
    main()