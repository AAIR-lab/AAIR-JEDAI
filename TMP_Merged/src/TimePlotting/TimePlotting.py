import matplotlib.pyplot as plt
import csv
import time
import Config

class TimePlotter():
    def __init__(self,pr_graph):
        self.pr_graph = pr_graph
        self.no_sequences = None
        self.no_total_nodes = 0
        self.percent_nodes_refined = [0]
        self.prob_refined = [0]
        self.start_time = time.time()
        self.times = [0]
        with open(Config.PROJ_DIR+"ICRA_Results/Keva/nodes_" + Config.RESULTS_FILE, "a") as f:
            f.write("\n")
        with open(Config.PROJ_DIR+"ICRA_Results/Keva/prob_" + Config.RESULTS_FILE, "a") as f:
            f.write("\n")
        with open(Config.PROJ_DIR+"ICRA_Results/Keva/time_" + Config.RESULTS_FILE, "a") as f:
            f.write("\n")
        with open(Config.PROJ_DIR + "ICRA_Results/Keva/action_fail_" + Config.RESULTS_FILE, "a") as f:
            f.write("\n")
        self.init()

    def init(self):
        root_node = self.pr_graph.get_root_node()
        self.no_sequences = root_node.lqueue.qsize()
        queue = root_node.lqueue.queue
        for seq in queue:
            hlas = seq[1]
            self.no_total_nodes += hlas.length

    def update(self):
        t = time.time()
        min_size_pr_node = None
        num_min_sequences = float("inf")
        root_node = self.pr_graph.get_root_node()
        queue = [root_node]
        while len(queue) > 0:
            node = queue.pop(0)
            if node.lqueue is not None:
                if node.lqueue.qsize() < num_min_sequences:
                    num_min_sequences = node.lqueue.qsize()
                    min_size_pr_node = node
        min_sequences = min_size_pr_node.lqueue.queue
        remaining_nodes = 0
        remaining_probs = 0
        for seq in min_sequences:
            hlas = seq[1]
            remaining_nodes += hlas.length
            # remaining_probs += hlas.action_list[-1].prob
            edge = min_size_pr_node.hl_plan_tree.get_edge(hlas.action_list[-2], hlas.action_list[-1])
            if edge.ll_plan is None:
                remaining_probs += edge.prob
        percent_nodes_refined = ((self.no_total_nodes - remaining_nodes) / float(self.no_total_nodes)) * 100
        self.percent_nodes_refined.append(percent_nodes_refined)
        self.prob_refined.append((1 - remaining_probs))
        actions_exec = min_size_pr_node.action_execute
        failed_actions = min_size_pr_node.restart
        self.times.append(t - self.start_time)
        with open(Config.PROJ_DIR+"ICRA_Results/Keva/nodes_" + Config.RESULTS_FILE, "a") as f:
            f.write(str(percent_nodes_refined)+",")
        with open(Config.PROJ_DIR+"ICRA_Results/Keva/prob_" + Config.RESULTS_FILE, "a") as f:
            f.write(str(1-remaining_probs) + ",")
        with open(Config.PROJ_DIR+"ICRA_Results/Keva/time_" + Config.RESULTS_FILE, "a") as f:
            f.write(str(t - self.start_time) + ",")
        with open(Config.PROJ_DIR + "ICRA_Results/Keva/action_fail_" + Config.RESULTS_FILE, "a") as f:
            f.write(str(t - self.start_time)+","+str(actions_exec)+","+str(failed_actions)+";")

    def generate_plot(self):
        # with open("nodes_"+Config.RESULTS_FILE,"a") as f:
        #     writer = csv.writer(f)
        #     writer.writerow(self.percent_nodes_refined)
        # with open("prob_"+Config.RESULTS_FILE,"a") as f:
        #     writer = csv.writer(f)
        #     writer.writerow(self.prob_refined)
        # with open("time_"+Config.RESULTS_FILE,"a") as f:
        #     writer = csv.writer(f)
        #     writer.writerow(self.times)
        pass
        # f = plt.figure()
        # plt.plot(self.percent_nodes_refined,self.prob_refined)
        # plt.xlabel("Percentage of nodes refined")
        # plt.ylabel("Probability mass refined")
        # plt.savefig("result.png")



