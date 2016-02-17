import socket
import threading
import time
import sys
from PyQt4 import QtGui, QtCore


#def tx_data(port, size, s, data_len):
#    tx = bytearray(size)
#    txed = 0
#    while (txed < data_len):
#        sent = s.send(tx) 
#        print 'Just Sent: ', sent
#        txed = txed + sent
#        time.sleep(2) # delays for seconds
#    print 'Total Sent: ', txed
#    return

#def rx_data(s, size, data_len):
#    rcvd = 0
#    while (rcvd < data_len):
#        data = s.recv(512) 
#        print 'Got back: ', len(data)
#        rcvd = rcvd + len(data)
#    s.close() 
#    print 'Total Received: ', rcvd
#    return

#def setup_threads():
#    host = '10.12.4.250'
#    port = 2020
#    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
#    s.connect((host,port)) 
#    t = threading.Thread(target=tx_data, args=(2020, 16384, s, 32768))
#    r = threading.Thread(target=rx_data, args=(s, 16384, 32768))
#    t.start()
#    r.start()
#    t.join()
#    print 'TX Thread Finished\n'
#    r.join()
#    print 'RX Thread Finished\n'
#    return

class SystemViewGUI(QtGui.QWidget):
        
    def __init__(self):
        super(SystemViewGUI, self).__init__()
        self.initUI()
                                            
    def initUI(self):

        thread_1  = QtGui.QLabel('Thread 1')
        thread_2  = QtGui.QLabel('Thread 2')
        tx_sent   = QtGui.QLabel('TX Bytes Sent')
        rx_rcv    = QtGui.QLabel('RX Bytes Received')

        qbtn = QtGui.QPushButton('Begin', self)
        qbtn.clicked.connect(self.setup_threads_gui)
        qbtn.resize(qbtn.sizeHint())
        #qbtn.move(50, 50)
        qbtn_2 = QtGui.QPushButton('Begin', self)
        qbtn_2.clicked.connect(self.setup_threads_gui)
        qbtn_2.resize(qbtn_2.sizeHint())
        #qbtn.move(50, 50)
    
        self.tx_1 = QtGui.QLineEdit(self)
        self.rx_1 = QtGui.QLineEdit(self)
        self.tx_2 = QtGui.QLineEdit(self)
        self.rx_2 = QtGui.QLineEdit(self)
        
        grid = QtGui.QGridLayout()
        grid.setSpacing(10)

        grid.addWidget(thread_1, 1, 0)
        grid.addWidget(tx_sent,  1, 1)
        grid.addWidget(rx_rcv,  1, 2)
        grid.addWidget(qbtn, 2, 0)
        grid.addWidget(self.tx_1, 2, 1)
        grid.addWidget(self.rx_1, 2, 2)

        grid.addWidget(thread_2, 5, 0)
        grid.addWidget(tx_sent,  5, 1)
        grid.addWidget(rx_rcv,  5, 2)
        grid.addWidget(qbtn_2, 6, 0)
        grid.addWidget(self.tx_2, 6, 1)
        grid.addWidget(self.rx_2, 6, 2)
       
        self.setLayout(grid) 

        self.resize(500, 450)
        self.center()
        self.setWindowTitle('System View')
        self.setWindowIcon(QtGui.QIcon('logo-v2.png'))        
        self.show()
    
    def center(self):
                    
        qr = self.frameGeometry()
        cp = QtGui.QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        
   # def buttonClicked(self):

    def setup_threads_gui(self):
        host = '10.12.4.250'
        port = 2020
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        s.connect((host,port)) 
        t = threading.Thread(target=self.tx_data_gui, args=(2020, 16384, s, 32768))
        r = threading.Thread(target=self.rx_data_gui, args=(s, 16384, 32768))
        t.start()
        r.start()
        t.join()
        print 'TX Thread Finished\n'
        r.join()
        print 'RX Thread Finished\n'

    def tx_data_gui(self, port, size, s, data_len):
        tx = bytearray(size)
        txed = 0
        while (txed < data_len):
            sent = s.send(tx) 
            print 'Just Sent: ', sent
            txed = txed + sent
            time.sleep(2) # delays for seconds
        print 'Total Sent: ', txed
        self.tx_1.setText(str(txed))
        return

    def rx_data_gui(self, s, size, data_len):
        rcvd = 0
        while (rcvd < data_len):
            data = s.recv(512) 
            print 'Got back: ', len(data)
            rcvd = rcvd + len(data)
        s.close() 
        print 'Total Received: ', rcvd
        self.rx_1.setText(str(rcvd))
        return

def main():
    #tasks = [1]
    #for task in tasks:
    app = QtGui.QApplication(sys.argv)
    ex = SystemViewGUI()
    sys.exit(app.exec_())
   
   # setup_threads()
    return

if __name__ == "__main__":
    main()
