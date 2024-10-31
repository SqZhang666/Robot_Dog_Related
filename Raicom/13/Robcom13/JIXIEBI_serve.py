import socket
import threading
import time
import os
import subprocess
import ActionGroupControl as AGC


def handle_client(conn, addr):
    while True:
        data = conn.recv(1024)
        if not data:
            print('client', addr, 'log_out')
            break
        message = data.decode()
        print('recv_fromClient:', message)

        if message == 'circle1:machineArm':
            print("operation...")
            AGC.runAction('small_yice')
            #time.sleep(10)
            response = 'Circle1_END!'

        elif message == 'circle2:machineArm_first':
            print("operation...")
            AGC.runAction('small_yice')
            #time.sleep(10)
            response = 'Circle2_END'

        elif message == 'circle2:machineArm_second':
            print("operation...")
            AGC.runAction('big_tongce')
            #time.sleep(10)
            response = 'Circle2_END'

        elif message == 'circle3:machineArm_first':
            print("operation...")
            AGC.runAction('small_tongce')
            #time.sleep(13)
            response = 'Circle3_END'

        elif message == 'circle3:machineArm_second':
            print("operation...")
            AGC.runAction('big_yice')
            #time.sleep(10)
            response = 'Circle3_END'

        elif message == 'circle4:machineArm':
            print("operation...")
            AGC.runAction('big_yice')
            #time.sleep(10)
            response = 'Circle4_END'

        else:
            print('What CAN I Say')
            response = 'You Are Out'

        conn.sendall(response.encode())
        print('SEND_END_to_Client')

    conn.close()


HOST = '192.168.123.20'
PORT = 5003

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))

    s.listen()
    print("Listenning...")

    while True:
        conn, addr = s.accept()

        threading.Thread(target=handle_client, args=(conn, addr)).start()