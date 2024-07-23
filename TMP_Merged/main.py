#!/usr/bin/env python
import argparse
import random
import numpy as np
import time
from TMP import TMP
from jedai_client import JEDAIClient
from jedai_server import MockJEDAIServer

def run_local_tmp(args):
    
    assert args.port is None

    start_time = time.time()
    print(start_time)
    util.blockprint()
    tmp = TMP(args=args, policy_file=args.policy_file)
    tmp.execute()
    util.enablePrint()
    print("--- %s seconds ---" % (time.time() - start_time))


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--assume-refinable", default=False, action="store_true",
                        help="If set, assume a motion plan is always possible (skip running the motion planner)")
    parser.add_argument("--store-policy-tree", default=False, action="store_true",
                        help="Store the policy tree")
    parser.add_argument("--store-simulated-executions", default=False, action="store_true",
                        help="Store the simulated executions in the policy tree")
    parser.add_argument("--store-image", default=False, action="store_true",
                        help="Store images of low-level states in the policy tree")
    parser.add_argument("--show-viewer", default=False, action="store_true",
                        help="Show the viewer")
    parser.add_argument("--set-camera", default=False, action="store_true",
                        help="Set the initial camera view.")
    parser.add_argument("--domain", default=None, required=True, type=str,
                        choices=["KevaDeterministic", "KevaStochastic", "Cafeworld",
                                 "CafeworldDeterministic"],
                        help="The domain to run against")
    parser.add_argument("--output-dir", default=None, type=str,
                        help="The output dir where to store the result files")
    parser.add_argument("--problem-file", default=None, type=str,
                        help="The problem file")
    parser.add_argument("--domain-file", default=None, type=str,
                        help="The domain file")
    parser.add_argument("--ll-file", default=None, type=str,
                        help="Initial ll config file")
    parser.add_argument("--run-trajectory", default=False, action="store_true",
                        help="Run the trajectory after computation.")
    parser.add_argument("--policy-file", default=None,
                        help="Refine against this policy file")
    parser.add_argument("--env-path", default=None,
                        help="The path to the environment file")
    parser.add_argument("--port", default=None, type=int,
                        help="The port number to listen on")
    parser.add_argument("--ip-address", default=None, type=str,
                        help="The ip address to connect to.")
    parser.add_argument("--use-mock-server", default=False, action="store_true",
                        help="Use a dummy server instead of jedai.")
    
    args = parser.parse_args()
    
    from core_paths import *
    domain_dir = TEST_DIR + "/" + args.domain + "/"
    sys.path.insert(0, domain_dir)
    
    # Now get all the imports into the main module.
    # We need to do this since this is a dirty hack
    # to first configure Config.py correctly and then
    # import the necessary globals.
    import Config
    import src.util as util
    
    Config.SHOW_VIEWER = args.show_viewer
    Config.RUN_TRAJ = args.run_trajectory
    
    if args.env_path is not None:

        Config.OPENRAVE_ENV_XML = args.env_path
    
    if args.port is None:
        
        run_local_tmp(args)
        
    else:
        # Start a mock server that mimics a JEDAI server.
        if args.use_mock_server:
            mock_jedai_server = MockJEDAIServer(args.port, args.ip_address)
            mock_jedai_server.start()
            assert mock_jedai_server.is_alive()
        
        jedai_client = JEDAIClient(args)
        jedai_client.run()
        
        # Cleanup the server appropriately.
        if args.use_mock_server:
            mock_jedai_server.join()
            print("Joined MockJEDAIServer")
