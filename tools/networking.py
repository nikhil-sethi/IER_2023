import socket
import struct

def createUDPLink(addr, is_server = None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if is_server == True:   # server
        sock.bind(addr)
    elif is_server == False:   # client. Need to be explicit
        sock.connect(addr)
    else:
        pass    # do nothing when is_server = None. This isn't stupidity. Need this for multicast. 
    return sock

def createMulticastLink(mcast_addr, bind_addr, is_server = True):
    sock = createUDPLink(mcast_addr)
    if is_server:
        membership = socket.inet_aton(mcast_addr[0]) + socket.inet_aton(bind_addr[0])

        # group = socket.inet_aton(self.multicast_addr)
        # mreq = struct.pack("4sl", group, socket.INADDR_ANY)
        sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(bind_addr[0]))
        
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, membership)
        # self.recv_sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

        sock.bind(bind_addr)
    else:   # client
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            # sock.settimeout(.2)

        ttl = struct.pack('b', 1)### = b'x01'###time to live
        # configuring socket in os in multicast mode
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, ttl) # affects multicast diagram
        sock.connect(mcast_addr)
    return sock