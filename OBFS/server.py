import socket
from threading import Thread
from queue import Queue

def receive_message(q, sock):
    print("Listening for messages...")
    while True:
        data, addr = sock.recvfrom(1024)
        msg = data.decode('utf-8')
        q.put(msg)
        if msg == "close":
            sock.close()
            break

def get_data(q): ## this function is to automatically get data from the queue and display it
    print("Listening for queue...")
    while True:
        if not q.empty():
            msg = q.get()
            print(msg)
            if msg == "close":
                break

class ADS_receive:
    def __init__(self, ip, port):
        self.q = Queue()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((ip, port))
        self.con = Thread(target=receive_message, args=(self.q, self.sock))
        self.ptr = Thread(target=get_data, args=(self.q,))
        
    def startConnection(self):
        # print("Starting threads...")
        self.con.start()
        self.ptr.start()
        self.con.join()
        self.ptr.join()
        
    def getTop(self):
        m=999999
        try:
            m=self.q.get()
        except:
            print("The queue is full")
        return m


# The following is to execute theis server independently
# ads = ADS_receive("127.0.0.1", 5005)
# ads.startConnection()