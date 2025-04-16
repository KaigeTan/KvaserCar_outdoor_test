import socket

class OBFS_send:
    
    def __init__ (self, ip, port):
        self.UDP_IP=ip
        self.UDP_PORT=port
        self.socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
        print("UDP target IP: %s" % ip)
        print("UDP target port: %s" % port)
    def sendMessage(self, msg):
        self.socket.sendto(msg.encode(), (self.UDP_IP, self.UDP_PORT))
        
    def closeSocket (self): 
        self. socket.close()



# If we do not close the socket at the server end, the socket port can not be used for future deployments.
# Thus we need solution like the following to close the server socket externally.
# please use the following along with this code.

##############
# client=OBFS_send("127.0.0.1", 5005)
# client.sendMessage("close")
    