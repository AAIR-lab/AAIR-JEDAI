import argparse
import socket
import threading
from communicator import Communicator
import networkx as nx
import pickle

class MockJEDAIServer(threading.Thread):
    
    def __init__(self, port, ip_address):
                
        super(MockJEDAIServer, self).__init__()
                
        self.port = port
        self.ip_address = ip_address
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    
    def process_user_cmd(self, conn, cmd):
        
        if cmd == "":
            return
        if cmd == "terminate":
            
            Communicator.send(conn, cmd)
        elif cmd == "close":
            
            pass
        elif cmd.startswith("plan"):
            
            filepath = cmd.split(" ")[1]
            graph = nx.nx_pydot.read_dot(filepath)
            
            msg = "plan%s" % (pickle.dumps(graph))
            Communicator.send(conn, msg)
        elif cmd.startswith("legacy_plan"):
            
            # +1 for the space.
            plan = cmd[len("legacy_plan") + 1:]
            msg = "legacy_plan%s" % (plan)
            Communicator.send(conn, msg)
        elif cmd.startswith("run_plan"):
            
            Communicator.send(conn, cmd)
        else:
            
            assert False
    
    def run(self):
        
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.ip_address, self.port))
        self.socket.listen(1)
        
        conn = None
        while conn is None:
            try:
                conn, _ = self.socket.accept()
            except socket.error as e:
                
                pass
        
        while True:
            
            cmd = raw_input("Enter a command: ")
            self.process_user_cmd(conn, cmd)
            
            if cmd == "terminate":
                
                break
        
        conn.close()
        print("MockJEDAIServer done")
        
if __name__ == "__main__":
    
    # Some mock commands.
    # plan /tmp/graph.gv
    # legacy_plan move starting_point fetch table_red,move table_red fetch counter
    
    parser = argparse.ArgumentParser(description="A mock JEDAI server")
    parser.add_argument("--port", required=True, type=int,
                        help="The port number for the server.")
    parser.add_argument("--ip-address", type=str, default="localhost",
                        help="The ip address to bind to.")
    
    args = parser.parse_args()
    server = MockJEDAIServer(args.port, args.ip_address)
    server.start()
    
    # Wait for the thread to finish.
    server.join()