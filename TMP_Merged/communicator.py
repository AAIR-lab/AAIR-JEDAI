import socket

class Communicator:
    
    MSG_START = "msg_start"
    MSG_END = "msg_end"
    
    
    @staticmethod    
    def send(s, msg):
        
        cmd = "%s%s%s" % (Communicator.MSG_START, 
                          msg, Communicator.MSG_END)
        s.sendall(cmd.encode("utf-8"))
    
    @staticmethod    
    def recv(s):
        
        try:
            msg = s.recv(4096).decode("utf-8")
            if msg.startswith(Communicator.MSG_START):
                
                while not Communicator.MSG_END in msg:
                    
                    tmp_msg = s.recv(4096).decode("utf-8")
                    if tmp_msg == "":
                    
                        # This implies the connection is DONE!
                        return None
                    
                    msg += tmp_msg
            
                assert msg.endswith(Communicator.MSG_END)
        except socket.error as e:
            
            msg = ""
        
        if msg == "":
            
            # We will never receive blank msgs unless
            # the connection is severed.
            return None
        else:
            
            return msg[0 + len(Communicator.MSG_START):
                       len(msg) - len(Communicator.MSG_END)]