import threading
import socket
import pickle
import argparse
from communicator import Communicator
from src.Planner.FFPlanner import FFPlanner
from collections import namedtuple

USE_MOCK_TMP = False

if USE_MOCK_TMP:
    class TMP:
        
        def __init__(self):
            
            pass
        
        def execute(self):
            
            pass
else:
    from TMP import TMP
    
TMPResult = namedtuple("TMPResult", ["success", "policy_tree"], 
                       verbose=False)

class TMPThread(threading.Thread):
    
    def __init__(self, args, event):
        
        super(TMPThread, self).__init__()
        self.args = args
        self.stopped = False
        self.event = event
        self.tmp = TMP(args=args)
        self.state = "waiting_for_cmds"
        self.tmp_state = None
    def run(self):
        
        while not self.stopped:
            
            self.state = "waiting_for_cmds"
            self.event.wait()
            
            if self.state == "execute":
                success, policy_tree = self.tmp.execute()
                self.tmp_state = TMPResult(success, policy_tree)
            elif self.state == "run_plan":
                
                self.tmp.run_plan(self.tmp_state.policy_tree)
            
    def set_state(self, state):
        
        assert self.state == "waiting_for_cmds"
        self.state = state        
    
    def stop(self):
        
        self.stopped = True
        
    def setup_plan(self, plan):
        
        self.tmp.set_plan(plan)

class JEDAIClient:
    
    def __init__(self, args):
        
        assert args.port is not None
        assert args.ip_address is not None
        self.args = args
        self.event = threading.Event()
        self.tmp_thread = TMPThread(args, self.event)
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.state = "waiting_for_cmds"
        
        
    def terminate_cmd(self):
        
        self.tmp_thread.stop()
        self.event.set()
        
    def plan_cmd(self, cmd):
        
        plan = pickle.loads(cmd[len("plan"):])
        self.tmp_thread.setup_plan(plan)
        self.tmp_thread.set_state("execute")
        self.event.set()
        
    def legacy_plan_cmd(self, cmd):
        
        plan = cmd[len("legacy_plan"):]
        plan = FFPlanner.convert_legacy_plan_to_graph(plan)
        self.tmp_thread.setup_plan(plan)
        self.tmp_thread.set_state("execute")
        self.state = "check_tmp"
        self.event.set()
        
    def run_plan_cmd(self):
        
        self.tmp_thread.set_state("run_plan")
        self.state = "check_tmp_execution"
        self.event.set()
        
    def signal_cmd(self, cmd):

        if cmd is not None:        
            print("Received cmd: ", cmd)

        if cmd is None:
            
            return False
        elif cmd.startswith("terminate"):
            
            self.terminate_cmd()
            return True
        elif cmd.startswith("plan"):
            
            self.plan_cmd(cmd)
            return False
        elif cmd.startswith("legacy_plan"):
            
            self.legacy_plan_cmd(cmd)
            return False
        elif cmd.startswith("run_plan"):
            self.run_plan_cmd()
            return False
        else:
            
            return False
    
    def run(self):
        
        done = False
        self.socket.connect((self.args.ip_address, self.args.port))
        self.tmp_thread.start()
        while not done:
            
            if self.state == "waiting_for_cmds":
                cmd = Communicator.recv(self.socket)
                done = self.signal_cmd(cmd)
            elif self.state == "check_tmp":
                
                if self.tmp_thread.state == "waiting_for_cmds":
                
                    Communicator.send(self.socket, "refined")
                    self.state = "waiting_for_cmds"
            elif self.state == "check_tmp_execution":
                
                if self.tmp_thread.state == "waiting_for_cmds":
                    Communicator.send(self.socket, "run_complete")
                    self.state = "waiting_for_cmds"
            else:
                
                assert False
            pass
        
        self.socket.close()
        self.tmp_thread.join()
        print("JEDAIClient done")
        
if __name__ == "__main__":
    
    
    parser = argparse.ArgumentParser(description="A mock JEDAI server")
    parser.add_argument("--port", required=True, type=int,
                        help="The port number for the server.")
    parser.add_argument("--ip-address", type=str, default="localhost",
                        help="The ip address to bind to.")
    
    args = parser.parse_args()
    client = JEDAIClient(args)
    client.start()
    
    # Wait for the thread to finish.
    client.join()