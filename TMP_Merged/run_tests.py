#!/usr/bin/env python
import os,sys
import copy
import time
from multiprocessing import Process, Manager
import sys
from StringIO import StringIO


def run_tests(options,test_executor_function):
	manager = Manager()
	mgr_dct = manager.dict()
	proc_list = []
	# Spawn process for each domain
	for opt in options:
		p = Process(target=test_executor_function, args=(opt,mgr_dct))
		p.start()
		proc_list.append(p)
		# Give some time for resetting config file
		time.sleep(2)
	for p in proc_list:
		p.join()
	return mgr_dct

# Class used in capturing stdout from each domain
class Capturing(list):
    def __enter__(self):
        self._stdout = sys.stdout
        sys.stdout = self._stringio = StringIO()
        return self
    def __exit__(self, *args):
        self.extend(self._stringio.getvalue().splitlines())
        del self._stringio
        sys.stdout = self._stdout


class TestOnDomains:
	@staticmethod
	def set_conf(domain_name):
		sedstr = "sed -i \"s|# DOMAIN = \'Testing\'|DOMAIN = \'"+domain_name+"\' # TestScript|g\" Config.py"
		os.system(sedstr)

	@staticmethod
	def reset_conf(domain_name):
		sedstr = "sed -i \"s|DOMAIN = \'"+domain_name+"\' # TestScript|# DOMAIN = \'Testing\'|g\" Config.py"
		os.system(sedstr)
	
	# function to run test on each domain
	@staticmethod
	def run_test_on_individual_domain(DOMAIN,mgr_dct):
		start = time.time()
		TestOnDomains.set_conf(DOMAIN)
		import TMP
		TestOnDomains.reset_conf(DOMAIN)
		tmp = TMP.TMP()
		result = False
		with Capturing() as output:
			try:
				result = tmp.execute()
			except Exception as e:
				print(e)
		log_file = 'results/'+DOMAIN+'_run.txt'
		with open(log_file,'wb') as f:
			output = "\n".join(output)
			f.write(output)
		end  = time.time()
		mgr_dct[DOMAIN] = [result, end - start, log_file]
		print("Running test for "+DOMAIN+" Completed. Result: "+str(result))

	
	@staticmethod
	def run_domain_tests(domain_names = ['DelicateCan','Keva']):
		if not os.path.exists('results/'):
			os.mkdir('results/')
		log_file = 'results/test_results.csv'
		with open(log_file,'wb') as f:
			lines = ['Logs:']
			output = "\n".join(lines)
			f.write(output)
		options = domain_names
		test_executor_function = TestOnDomains.run_test_on_individual_domain
		mgr_dct = run_tests(options=domain_names, test_executor_function=test_executor_function)
		log_file = 'results/test_results.csv'
		with open(log_file,'wb') as f:
			lines = []
			lines.append("Domain;Result;Runtime;Log File")
			for i in mgr_dct.keys():
				result, time_taken, log_file = mgr_dct[i]
				lines.append(i+";"+str(result)+";"+str(time_taken)+";"+log_file)
			output = "\n".join(lines)
			f.write(output)
		print("Results:")
		print(mgr_dct)
		print("Results saved in directory results/")
		return mgr_dct

if __name__ == '__main__':
	mgr_dct = TestOnDomains.run_domain_tests(['DelicateCan','Keva'])
