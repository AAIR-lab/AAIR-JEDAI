from abc import ABCMeta, abstractmethod
class MotionPlannerBase:
	__metaclass__ = ABCMeta

	@abstractmethod
	def getPlanTrajectoryToGoal(goal_transform):
		pass



