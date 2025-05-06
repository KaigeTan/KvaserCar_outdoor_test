import threading

class SharedObj:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = None
    
    def set_data(self, elem):
        with self.lock:
            self.data = elem
        
    def get_data(self):
        elem = None
        with self.lock:
            elem = self.data
        return elem