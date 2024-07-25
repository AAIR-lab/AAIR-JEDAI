from abc import ABCMeta, abstractmethod
class MotionPlannerBase:
	__metaclass__ = ABCMeta

	@abstractmethod
	def get_mp_trajectory_to_goal(goal_transform):
		pass



