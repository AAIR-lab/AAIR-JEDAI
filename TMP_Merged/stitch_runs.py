#!/usr/bin/env python
import os,sys
import copy
import time
from multiprocessing import Process, Manager
import sys
from StringIO import StringIO
import IPython



class StitchedRuns:
	@staticmethod
	def update_run_track_file(run_number,domain_dir):
		# from Config import DOMAIN_DIR
		with open(domain_dir+'run_count.txt',"w") as f:
			f.write(str(run_number))
			f.close()

	# function to run test on each domain
	@staticmethod
	def individual_run(run_num,mgr_dct):
		import TMP
		tmp = TMP.TMP()
		# result = tmp.execute()
		# with Capturing() as output:
		# 	try:
		result = tmp.execute()
		# 	except Exception as e:
		# 		print(e)
		# log_file = 'results/'+str(run_num)+'_run.txt'
		# with open(log_file,'wb') as f:
			# output = "\n".join(output)
			# f.write(output)
			# f.close()
		# mgr_dct[run_num] = [result,  log_file]
		print("Running test for run "+str(run_num)+" Completed. Result: "+str(result))
		# print("Log file: "+log_file)
		return mgr_dct
	
	@staticmethod
	def stitch_runs_sequential(num_runs, domain_dir):

		if not os.path.exists('results/'):
			os.mkdir('results/')
		executor_function = StitchedRuns.individual_run
		manager = Manager()
		mgr_dct = manager.dict()
		for run_number in range(1,num_runs+1):
			StitchedRuns.update_run_track_file(run_number,domain_dir)
			p = Process(target=executor_function, args=(run_number,mgr_dct))
			p.start()
			p.join()
			file_name = domain_dir+'refined_tree_'+str(run_number)+'.pkl'
			old_file_name = domain_dir+'refined_tree.pkl'
			os.rename(old_file_name,file_name)
			print("########Iteration "+str(run_number)+" completed##########")
		return mgr_dct

if __name__ == '__main__':
	# from Config import NUMBER_OF_RUNS
	# import Config
	mgr_dct = StitchedRuns.stitch_runs_sequential(1, 'test_domains/KevaLooped/')
	print(mgr_dct)
