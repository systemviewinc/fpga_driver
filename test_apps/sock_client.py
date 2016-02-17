import socket
import threading
import time

def tx_data(port, size, s, data_len):
   # tx = bytearray(size)
    tx = bytearray(size)
    txed = 0
    while (txed < data_len):
        sent = s.send(tx) 
        print 'Just Sent: ', sent
        txed = txed + sent
     #   time.sleep(2) # delays for seconds
    print 'Total Sent: ', txed
    return

def rx_data(s, size, data_len):
    rcvd = 0
    while (rcvd < data_len):
        data = s.recv(size) 
        print 'Got back: ', len(data)
        rcvd = rcvd + len(data)
    s.close() 
    print 'Total Received: ', rcvd
    return

def setup_threads():
    host = '10.12.4.250'
    port = 2020
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
    s.connect((host,port)) 
    t = threading.Thread(target=tx_data, args=(2020, 16384, s, 32768))
    r = threading.Thread(target=rx_data, args=(s, 16384, 32768))
    t.start()
    r.start()
    t.join()
    print 'TX Thread Finished\n'
    r.join()
    print 'RX Thread Finished\n'
    return

def main():
    #tasks = [1]
    #for task in tasks:
    setup_threads()
    return

if __name__ == "__main__":
    main()
