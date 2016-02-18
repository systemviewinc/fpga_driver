import socket
import threading
import time
import sys
from PyQt4 import QtGui, QtCore
from multiprocessing.pool import ThreadPool


class Socket_session():

    def __init__(self, port):
        self.port = port;
   
    def setup_threads(self):
        
        host = '10.12.4.250'
        port = self.port
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) 
        s.connect((host, port)) 
        self.t = threading.Thread(target=self.tx_data, args=(port, 16384, s, 32768))
        self.r = threading.Thread(target=self.rx_data, args=(s, 16384, 32768))
        self.t.start()
        self.r.start()
        #pool = ThreadPool(processes=2)
        #self.tx = pool.apply_async(self.tx_data, (port, 16384, s, 32768,))
        #self.rx = pool.apply_async(self.rx_data, (s, 16384, 32768,))
       
    def end_threads(self):

        #self.tx_result = self.tx.get()
        self.t.join()
        print 'TX Thread Finished\n'
        self.r.join()
        #self.rx_result = self.rx.get()
        print 'RX Thread Finished\n'
   
    def tx_data(self, port, size, s, data_len):
        tx = bytearray(size)
        txed = 0
        while (txed < data_len):
            sent = s.send(tx) 
            print 'Just Sent: ', sent
            txed = txed + sent
            time.sleep(2) # delays for seconds
        print 'Total Sent: ', txed
        self.tx_result = txed
        #self.tx_1.setText(str(txed))
        #return txed
        return

    def rx_data(self, s, size, data_len):
        rcvd = 0
        while (rcvd < data_len):
            data = s.recv(512) 
            print 'Got back: ', len(data)
            rcvd = rcvd + len(data)
        s.close() 
        print 'Total Received: ', rcvd
        self.rx_result = rcvd
        #self.rx_1.setText(str(rcvd))
        #return rcvd
        return

class SystemViewGUI(QtGui.QWidget):
        
    def __init__(self):
        super(SystemViewGUI, self).__init__()
        self.initUI()
                                            
    def initUI(self):

        thread_1  = QtGui.QLabel('Thread 1')
        thread_2  = QtGui.QLabel('Thread 2')
        thread_3  = QtGui.QLabel('Thread 3')
        thread_4  = QtGui.QLabel('Thread 4')
        tx_sent   = QtGui.QLabel('TX Bytes Sent')
        rx_rcv    = QtGui.QLabel('RX Bytes Received')

        qbtn = QtGui.QPushButton('Begin', self)
        qbtn.clicked.connect(self.buttonClicked)
        qbtn.resize(qbtn.sizeHint())
        
        qbtn_2 = QtGui.QPushButton('Begin', self)
        qbtn_2.clicked.connect(self.buttonClicked)
        qbtn_2.resize(qbtn_2.sizeHint())
        
        qbtn_3 = QtGui.QPushButton('Begin', self)
        qbtn_3.clicked.connect(self.buttonClicked)
        qbtn_3.resize(qbtn_3.sizeHint())
       
        qbtn_4 = QtGui.QPushButton('Begin', self)
        qbtn_4.clicked.connect(self.buttonClicked)
        qbtn_4.resize(qbtn_4.sizeHint())
    
        self.tx_1 = QtGui.QLineEdit(self)
        self.rx_1 = QtGui.QLineEdit(self)
        self.tx_2 = QtGui.QLineEdit(self)
        self.rx_2 = QtGui.QLineEdit(self)
        self.tx_3 = QtGui.QLineEdit(self)
        self.rx_3 = QtGui.QLineEdit(self)
        self.tx_4 = QtGui.QLineEdit(self)
        self.rx_4 = QtGui.QLineEdit(self)
        
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
       
        grid.addWidget(thread_3, 9, 0)
        grid.addWidget(tx_sent,  9, 1)
        grid.addWidget(rx_rcv,  9, 2)
        grid.addWidget(qbtn_3, 10, 0)
        grid.addWidget(self.tx_3, 10, 1)
        grid.addWidget(self.rx_3, 10, 2)

        grid.addWidget(thread_4, 13, 0)
        grid.addWidget(tx_sent,  13, 1)
        grid.addWidget(rx_rcv,  13, 2)
        grid.addWidget(qbtn_4, 14, 0)
        grid.addWidget(self.tx_4, 14, 1)
        grid.addWidget(self.rx_4, 14, 2)
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
        
    def buttonClicked(self):

        sender = self.sender()
        print  sender.text(), ' was pressed'
        #self.setup_threads_gui()
        thread_1 = Socket_session(2020)
        thread_2 = Socket_session(2030)
        thread_1.setup_threads()
        thread_2.setup_threads()
       
       
        thread_1.end_threads()
        thread_2.end_threads()
        
        self.tx_1.setText(str(thread_1.tx_result))
        self.rx_1.setText(str(thread_1.rx_result))
        self.tx_2.setText(str(thread_2.tx_result))
        self.rx_2.setText(str(thread_2.rx_result))


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
